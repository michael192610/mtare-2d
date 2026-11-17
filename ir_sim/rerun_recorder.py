"""Optional Rerun recording: one inspectable sample per logical simulation cycle."""
from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys

import numpy as np


class RerunRecorder:
    def __init__(self, directory: Path, config):
        import rerun as rr
        import rerun.blueprint as rrb

        self.rr = rr
        self.directory = directory
        self.path = directory / "simulation.rrd"
        if self.path.exists():
            raise FileExistsError(self.path)
        self.recording = rr.RecordingStream("mtare_2d_irsim", recording_id=directory.name)
        self.recording.__enter__()
        private_views = [rrb.Spatial2DView(origin=f"/robots/robot_{i}",
                                          name=f"Robot {i}: private map",
                                          visual_bounds=rrb.VisualBounds2D(
                                              x_range=[0, config.width / config.cell_size],
                                              y_range=[0, config.height / config.cell_size]))
                         for i in range(config.robots)]
        blueprint = rrb.Blueprint(
            rrb.Vertical(
                rrb.Horizontal(
                    rrb.Spatial2DView(origin="/irsim", name="IR-SIM simulation frame"),
                    rrb.Tabs(*private_views, name="Private robot maps"),
                    column_shares=[3, 2]),
                rrb.Horizontal(
                    rrb.TimeSeriesView(origin="/metrics", contents=["/metrics/coverage_percent"],
                                       name="Team observed (%)"),
                    rrb.TextDocumentView(origin="/status", name="Planner state")),
                row_shares=[4, 1]),
            rrb.TimePanel(timeline="cycle", play_state=rrb.components.PlayState.Paused, fps=5),
            auto_layout=False, auto_views=False)
        self.recording.save(self.path, default_blueprint=blueprint)
        self.cycles = set()
        self.closed = False
        self.palette = np.array([[224, 229, 235], [191, 229, 246], [128, 85, 160]], dtype=np.uint8)

    @staticmethod
    def map_pixels(sim, positions):
        points = np.asarray(positions, dtype=float).reshape(-1, 2).copy()
        points[:, 0] /= sim.config.cell_size
        points[:, 1] = (sim.config.height - points[:, 1]) / sim.config.cell_size
        return points

    def capture(self, bridge):
        if self.closed:
            return
        rr, recording, sim = self.rr, self.recording, bridge.sim
        recording.set_time("cycle", sequence=sim.tick)
        bridge.figure.canvas.draw()
        rgb = np.asarray(bridge.figure.canvas.buffer_rgba())[:, :, :3].copy()
        recording.log("irsim/frame", rr.Image(rgb).compress(jpeg_quality=85))
        from mtare_2d.viewer import Viewer
        colors = [tuple(bytes.fromhex(c.lstrip("#"))) for c in Viewer.COLORS]
        for robot in sim.robots:
            root = f"robots/robot_{robot.robot_id}"
            cells = np.zeros_like(robot.known)
            cells[robot.known == 0] = 1
            cells[robot.known == 1] = 2
            recording.log(f"{root}/map", rr.Image(self.palette[np.flipud(cells)]))
            positions = [robot.position if i == robot.robot_id else robot.peers[i].position
                         for i in range(sim.config.robots)]
            labels = [str(i) if i == robot.robot_id else f"{i}*"
                      for i in range(sim.config.robots)]
            recording.log(f"{root}/robots", rr.Points2D(self.map_pixels(sim, positions),
                                                       colors=colors[:sim.config.robots],
                                                       radii=.4, labels=labels, show_labels=True,
                                                       draw_order=30))
            recording.log(f"{root}/trajectory", rr.LineStrips2D(
                [self.map_pixels(sim, robot.trajectory)], colors=colors[robot.robot_id],
                radii=.08, draw_order=10))
            routes = ([self.map_pixels(sim, [sim.world.xy(n) for n in row])
                       for row in robot.proposal.routes] if robot.proposal else [])
            recording.log(f"{root}/global_routes", rr.LineStrips2D(
                routes, colors=colors[:len(routes)], radii=.06, draw_order=15))
            local = [self.map_pixels(sim, [robot.position, *[sim.world.xy(n) for n in robot.path]])] \
                    if robot.path else []
            recording.log(f"{root}/local_path", rr.LineStrips2D(
                local, colors=colors[robot.robot_id], radii=.1, draw_order=20))
            ring = []
            if sim.config.comms_range is not None:
                angles = np.linspace(0, 2 * np.pi, 97)
                points = np.asarray(robot.position) + sim.config.comms_range * np.column_stack(
                    (np.cos(angles), np.sin(angles)))
                ring = [self.map_pixels(sim, points)]
            recording.log(f"{root}/comms_range", rr.LineStrips2D(
                ring, colors=colors[robot.robot_id], radii=.06, draw_order=5))
        stats = sim.stats()
        for name, value in {"coverage_percent": 100 * stats["explored_ratio"],
                            "distance_m": stats["team_distance"],
                            "global_calls": stats["vrp_calls"]}.items():
            recording.log(f"metrics/{name}", rr.Scalars(value))
        status = {
            "cycle": sim.tick, "reason": sim.reason, "done": sim.done,
            "communication_range_m": sim.config.comms_range,
            "private_map_coordinates": f"image cells; each cell is {sim.config.cell_size:g} m",
            "private_map_labels": "* marks a cached teammate position",
            "robots": [{"id": r.robot_id, "position_m": r.position,
                        "global_regions": r.global_exploring_regions,
                        "assigned_tasks": r.selected_tasks,
                        "selected_local_viewpoints": len(r.local_viewpoints),
                        "local_complete": r.local_coverage_complete,
                        "finished": r.exploration_finished} for r in sim.robots],
        }
        recording.log("status/state", rr.TextDocument(json.dumps(status, indent=2)))
        self.cycles.add(sim.tick)

    def close(self):
        if self.closed:
            return
        self.recording.__exit__(None, None, None)
        self.closed = True
        (self.directory / "rerun_recording.json").write_text(json.dumps({
            "sdk_version": self.rr.__version__, "file": self.path.name,
            "timeline": "cycle", "frame_count": len(self.cycles),
            "first_cycle": min(self.cycles) if self.cycles else None,
            "last_cycle": max(self.cycles) if self.cycles else None,
            "granularity": "initial state and every logical cycle; not individual grid substeps",
            "contents": "IR-SIM frames, all private maps/cached poses/routes, ranges, metrics and planner state",
        }, indent=2) + "\n")


def open_recording(path: Path, *, wait=True):
    executable = Path(sys.executable).with_name("rerun")
    if wait:
        return subprocess.run([str(executable), "--new", "--bind", "127.0.0.1", str(path)],
                              check=False).returncode
    return subprocess.Popen([str(executable), "--new", "--bind", "127.0.0.1", str(path)])


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Open an optional M-TARE Rerun recording")
    parser.add_argument("run", nargs="?", type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    pointer = root / "latest_rerun.txt"
    if args.run is None and not pointer.exists():
        parser.error("No Rerun recording yet. Run the simulation with --rerun first.")
    directory = args.run or Path(pointer.read_text().strip())
    path = directory if directory.suffix == ".rrd" else directory / "simulation.rrd"
    if not path.exists():
        parser.error("No Rerun recording found. Run the simulation with --rerun first.")
    raise SystemExit(open_recording(path))
