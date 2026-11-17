"""Live IR-SIM external-state frontend for the existing M-TARE 2D planner.

M-TARE owns abstract motion, observations, communication and completion. IR-SIM
owns the live external-state world and its robot/obstacle rendering. No IR-SIM
behavior, LiDAR, collision response or goal-arrival rule drives the planner.
"""
from __future__ import annotations

import argparse
from dataclasses import fields
import hashlib
import json
import logging
import math
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent
PROJECT = ROOT.parent
sys.path.insert(0, str(PROJECT))

import numpy as np
import yaml

from mtare_2d.completion import COMPLETION_REASONS
from mtare_2d.config import Config
from mtare_2d.recording import Journal
from mtare_2d.simulation import Simulation
from mtare_2d.viewer import Viewer


def scene_config(sim: Simulation) -> dict:
    """Export the SAME generated scene, without sampling a new IR-SIM map."""
    cfg = sim.config
    return {
        "world": {
            "width": cfg.width, "height": cfg.height,
            "step_time": 1.0, "sample_time": 1.0, "step_mode": "external",
            "control_mode": "auto", "collision_mode": "unobstructed",
            "plot": {"figure_pixels": [1400, 700], "show_title": False},
        },
        "gui": {"keyboard": {"backend": "mpl"}},
        "robot": [
            {"number": 1, "name": f"mtare_robot_{r.robot_id}",
             "kinematics": {"name": "omni"},
             "shape": {"name": "circle", "radius": 0.5},
             "state": [*r.position, 0.0], "goal": [-1000.0, -1000.0, 0.0],
             "color": Viewer.COLORS[r.robot_id % len(Viewer.COLORS)],
             "plot": {"show_goal": False, "show_arrow": False,
                      "show_text": True, "show_trajectory": False}}
            for r in sim.robots
        ],
        "obstacle": [
            {"number": 1, "name": f"obstacle_{i}", "state": [c.x, c.y, 0.0],
             "shape": {"name": "circle", "radius": c.radius}, "color": "#8055a0"}
            for i, c in enumerate(sim.world.circles)
        ],
    }


class IRSimBridge:
    def __init__(self, sim: Simulation, journal: Journal | None, *, headless=False,
                 paused=False, read_only=False):
        import irsim
        import matplotlib.pyplot as plt

        self.sim, self.journal, self.plt = sim, journal, plt
        self.headless, self.paused = headless, paused
        self.finished = read_only
        self.closed = self.failed = False
        self.view_robot = 0
        self.steps = 0
        self.max_pose_error = 0.0
        self.overlays = []
        self.recorder = None
        directory = sim.directory if journal is None else journal.directory
        self.scene_path = directory / "irsim_scene.yaml"
        if not read_only:
            self.scene_path.write_text(yaml.safe_dump(scene_config(sim), sort_keys=False))
        if not self.scene_path.exists():
            raise FileNotFoundError("This saved run has no exported IR-SIM scene.")
        self.env = irsim.make(str(self.scene_path), display=not headless,
                              step_mode="external", log_level="ERROR", seed=sim.config.seed)
        # This version's renderer exposes its axes through EnvPlot. The adapter
        # is pinned to the tested upstream revision; no upstream source is edited.
        plot = self.env._env_plot
        self.figure, self.world_axis = plot.fig, plot.ax
        if self.env.keyboard is not None:
            keyboard = self.env.keyboard
            keyboard.close()
            for name in ("_mpl_press_cid", "_mpl_release_cid", "_mpl_enter_cid",
                         "_mpl_leave_cid", "_mpl_close_cid"):
                connection = getattr(keyboard, name, None)
                if connection is not None:
                    self.figure.canvas.mpl_disconnect(connection)
            self.env.keyboard = None
        # Our timer owns event dispatch; EnvBase.render must not nest plt.pause.
        self.env.display = False
        self.figure.set_size_inches(14, 7)
        self.world_axis.set_position([0.055, 0.22, 0.40, 0.66])
        self.known_axis = self.figure.add_axes([0.565, 0.22, 0.40, 0.66])
        self.figure.canvas.manager.set_window_title("M-TARE 2D in IR-SIM")
        self.caption = self.figure.text(0.5, 0.09, "", ha="center", fontsize=9)
        self.figure.text(0.5, 0.045,
                         "Space: pause/resume | N: one cycle | 0/1/2: robot | A: team | Q: save/close",
                         ha="center", fontsize=10)
        self.figure.canvas.mpl_connect("key_press_event", self.key)
        self.figure.canvas.mpl_connect("close_event", self.close)
        self.timer = self.figure.canvas.new_timer(interval=max(20, round(sim.config.frame_delay * 1000)))
        self.timer.add_callback(self.timer_tick)
        self.sync_states()
        self.render()

    def sync_states(self, previous=None):
        for model, robot in zip(self.env.robot_list, self.sim.robots, strict=True):
            old = robot.position if previous is None else previous[robot.robot_id]
            dx, dy = robot.position[0] - old[0], robot.position[1] - old[1]
            heading = math.atan2(dy, dx) if dx or dy else float(model.state[2, 0])
            model.set_state([*robot.position, heading])
            model.set_velocity([dx / self.env.step_time, dy / self.env.step_time])
        if previous is None:
            self.env.refresh()
        else:
            self.env.step()
            self.steps += 1
        errors = [float(np.linalg.norm(model.state[:2, 0] - robot.position))
                  for model, robot in zip(self.env.robot_list, self.sim.robots, strict=True)]
        self.max_pose_error = max(self.max_pose_error, *errors)
        if self.max_pose_error > 1e-9:
            raise AssertionError("IR-SIM poses diverged from the supplied M-TARE states")

    def render(self):
        from matplotlib.colors import ListedColormap
        from matplotlib.patches import Circle

        cfg = self.sim.config
        for artist in self.overlays:
            artist.remove()
        self.overlays.clear()
        self.env.render(interval=0)
        union = self.sim.union_map()
        self.overlays.append(self.world_axis.imshow(
            union == 0, origin="lower", extent=[0, cfg.width, 0, cfg.height],
            cmap=ListedColormap(["#fbfcfd", "#bfe5f6"]), vmin=0, vmax=1,
            interpolation="nearest", zorder=-1))
        self.world_axis.set_title("IR-SIM world + team observations", fontsize=11)
        self.known_axis.clear()
        selected = None if self.view_robot is None else self.sim.robots[self.view_robot]
        known = union if selected is None else selected.known
        cells = np.zeros_like(known)
        cells[known == 0] = 1
        cells[known == 1] = 2
        self.known_axis.imshow(cells, origin="lower", extent=[0, cfg.width, 0, cfg.height],
                              cmap=ListedColormap(["#e0e5eb", "#bfe5f6", "#8055a0"]),
                              vmin=0, vmax=2, interpolation="nearest")
        self.known_axis.set_title("Team observation union" if selected is None else
                                  f"Robot {selected.robot_id}'s private knowledge", fontsize=11)
        for axis in (self.world_axis, self.known_axis):
            axis.set_xlim(0, cfg.width)
            axis.set_ylim(0, cfg.height)
            axis.set_aspect("equal")
            axis.set_xlabel("x (m)")
            axis.set_ylabel("y (m)")
        for robot in self.sim.robots:
            color = Viewer.COLORS[robot.robot_id % len(Viewer.COLORS)]
            trace = np.asarray(robot.trajectory)
            self.overlays.extend(self.world_axis.plot(trace[:, 0], trace[:, 1], color=color, linewidth=1.6))
            own = selected is None or selected.robot_id == robot.robot_id
            position = robot.position if own else selected.peers[robot.robot_id].position
            if own:
                self.known_axis.plot(trace[:, 0], trace[:, 1], color=color, linewidth=1.6)
                Viewer._draw_local(self, self.known_axis, robot, color)
            self.known_axis.scatter(*position, color=color, edgecolors="white", s=65, zorder=7)
            self.known_axis.text(position[0] + 1, position[1] + 1,
                                 str(robot.robot_id) + ("" if own else " cached"), color=color)
        if selected is not None:
            if selected.proposal:
                for robot_id, route in enumerate(selected.proposal.routes):
                    Viewer._draw_route(self, self.known_axis, route, Viewer.COLORS[robot_id])
            if cfg.comms_range is not None:
                for axis in (self.world_axis, self.known_axis):
                    ring = Circle(selected.position, cfg.comms_range, fill=False, linestyle="--",
                                  edgecolor=Viewer.COLORS[selected.robot_id], linewidth=1.5, zorder=6)
                    axis.add_patch(ring)
                    if axis is self.world_axis:
                        self.overlays.append(ring)
        state = "COMPLETE" if self.finished and self.sim.reason in COMPLETION_REASONS else \
                "STOPPED" if self.finished else "PAUSED" if self.paused else "RUNNING"
        stats = self.sim.stats()
        comms = "full" if cfg.comms_range is None else f"{cfg.comms_range:g} m"
        self.figure.suptitle(f"M-TARE 2D in IR-SIM | {state} | {comms} comms | cycle {self.sim.tick} | "
                             f"observed {100 * stats['explored_ratio']:.2f}%")
        self.caption.set_text(f"Global VRP calls: {stats['vrp_calls']} | "
                              f"Local plans: {stats['local_planning_calls']} | "
                              f"Distance: {stats['team_distance']:.1f} m | Selected robot: {self.view_robot}")
        self.figure.canvas.draw_idle()

    def advance(self, render=True):
        if self.finished or self.sim.done:
            return
        previous = [r.position for r in self.sim.robots]
        stats = self.sim.step()
        self.sync_states(previous)
        if self.sim.tick % 20 == 0:
            logging.info("Cycle %d: %.2f%% observed; %.1f m travelled", self.sim.tick,
                         100 * stats['explored_ratio'], stats['team_distance'])
        if self.sim.done:
            self.finish("finished" if self.sim.reason in COMPLETION_REASONS else "stopped")
        else:
            if render or self.recorder is not None:
                self.render()
            if self.recorder is not None:
                self.recorder.capture(self)

    def finish(self, status):
        if self.finished:
            return
        self.finished = True
        self.failed = status == "error"
        self.timer.stop()
        self.sim.close()
        if not self.closed:
            self.render()
        if self.recorder is not None:
            self.recorder.capture(self)
            self.recorder.close()
        self.figure.savefig(self.journal.directory / "final.png", dpi=120, bbox_inches="tight")
        import irsim
        metadata = {
            "simulator": "IR-SIM", "version": irsim.__version__, "step_mode": self.env.step_mode,
            "planner_project": str(PROJECT), "external_steps": self.steps,
            "max_pose_error": self.max_pose_error,
            "coverage_source": "existing M-TARE abstract observation model",
            "motion_source": "existing M-TARE abstract executor",
            "collision_response": "disabled; existing grid collision checks retained; robot radius is visual",
            "original_source_hashes": {
                p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                for p in (PROJECT / "mtare_2d").glob("*.py")
            },
        }
        (self.journal.directory / "irsim_integration.json").write_text(json.dumps(metadata, indent=2) + "\n")
        self.journal.finish(self.sim, status)
        logging.info("Run %s: %s; %.2f%% observed.", status, self.sim.reason,
                     100 * self.sim.stats()['explored_ratio'])

    def timer_tick(self):
        if self.finished:
            return False
        if not self.paused:
            try:
                self.advance()
            except Exception as exc:
                logging.exception("IR-SIM run failed")
                self.sim.reason = str(exc)
                self.finish("error")
                return False
        return True

    def key(self, event):
        key = event.key
        if key in ("q", "escape"):
            if not self.finished:
                self.sim.reason = "user_stop"
                self.finish("stopped")
            self.plt.close(self.figure)
        elif key in (" ", "space") and not self.finished:
            self.paused = not self.paused
        elif key in ("n", "right") and not self.finished:
            self.paused = True
            self.advance()
        elif key == "a":
            self.view_robot = None
        elif key and key.isdigit() and int(key) < len(self.sim.robots):
            self.view_robot = int(key)
        if not self.closed:
            self.render()

    def close(self, event):
        self.closed = True
        if not self.finished:
            self.sim.reason = "window_closed"
            self.finish("stopped")

    def show(self):
        if not self.finished:
            self.timer.start()
        self.plt.show()


def show_saved_results(directory):
    import matplotlib
    matplotlib.use("TkAgg")
    from mtare_2d.results_viewer import SavedRun
    viewer = IRSimBridge(SavedRun(directory), None, read_only=True)
    try:
        viewer.show()
    finally:
        viewer.env.end(ending_time=0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robots", type=int, default=3)
    parser.add_argument("--range", dest="comms_range", type=lambda s: None if s == "full" else float(s), default=None)
    parser.add_argument("--seed", type=int, default=755)
    parser.add_argument("--solver-ms", type=int, default=80)
    parser.add_argument("--completion-min-coverage", type=float, default=0.91,
                        help="Minimum team observed fraction for idle completion (default: 0.91).")
    parser.add_argument("--max-cycles", type=int, default=1200)
    parser.add_argument("--planning-mode", choices=("parallel", "sequential"), default="parallel")
    parser.add_argument("--frame-delay", type=float, default=0.15)
    parser.add_argument("--paused", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--no-open", action="store_true")
    parser.add_argument("--rerun", action="store_true",
                        help="Record every simulation cycle to Rerun and open it after a headless run.")
    parser.add_argument("--output", type=Path, default=ROOT)
    parser.add_argument("--view", nargs="?", const="latest", metavar="RUN",
                        help="Open an exported IR-SIM result without rerunning the planner.")
    args = parser.parse_args()
    if args.paused and args.headless:
        parser.error("--paused requires the live window")
    values = {key: value for key, value in vars(args).items() if key in {f.name for f in fields(Config)}}
    try:
        config = Config(**values)
    except ValueError as exc:
        parser.error(str(exc))
    os.environ.setdefault("MPLCONFIGDIR", str(ROOT / ".mplconfig"))
    os.environ["OPENBLAS_NUM_THREADS"] = os.environ["OMP_NUM_THREADS"] = "1"
    import matplotlib
    matplotlib.use("Agg" if args.headless else "TkAgg")
    if args.view is not None:
        directory = (Path((args.output / "latest_run.txt").read_text().strip())
                     if args.view == "latest" else Path(args.view))
        show_saved_results(directory)
        return 0
    journal = Journal(args.output, config)
    logging.basicConfig(level=logging.INFO, format="%(message)s", force=True, handlers=[
        logging.StreamHandler(), logging.FileHandler(journal.directory / "console.log")])
    logging.info("M-TARE 2D in IR-SIM: %d robots, %s communication", config.robots,
                 "full" if config.comms_range is None else f"{config.comms_range:g} m")
    sim, bridge = None, None
    try:
        sim = Simulation(config, journal)
        journal.initialize(sim)
        bridge = IRSimBridge(sim, journal, headless=args.headless, paused=args.paused)
        if args.rerun:
            from .rerun_recorder import RerunRecorder
            bridge.recorder = RerunRecorder(journal.directory, config)
            bridge.recorder.capture(bridge)
        if args.headless:
            while not sim.done:
                bridge.advance(render=False)
            if not args.no_open and (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
                bridge.plt.close(bridge.figure)
                if args.rerun:
                    from .rerun_recorder import open_recording
                    open_recording(journal.directory / "simulation.rrd")
                else:
                    show_saved_results(journal.directory)
        else:
            bridge.show()
    except KeyboardInterrupt:
        if sim is not None:
            sim.reason = "keyboard_interrupt"
        if bridge is not None:
            bridge.finish("stopped")
    except Exception as exc:
        logging.exception("Run failed")
        if sim is not None:
            sim.reason = str(exc)
            if bridge is not None:
                bridge.finish("error")
            elif not journal.finished:
                journal.finish(sim, "error")
        return 1
    finally:
        if sim is not None:
            sim.close()
        if bridge is not None:
            if bridge.recorder is not None:
                bridge.recorder.close()
                (ROOT / "latest_rerun.txt").write_text(str(bridge.recorder.path.resolve()) + "\n")
            bridge.env.end(ending_time=0)
    return int(bool(bridge and bridge.failed))


if __name__ == "__main__":
    raise SystemExit(main())
