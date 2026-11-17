"""Two-panel Matplotlib viewer adapted from the EM demonstration layout."""
import logging

import numpy as np


class Viewer:
    COLORS = ["#d74b40", "#21976e", "#e5a000", "#8666bb", "#247abd", "#bd589e", "#6d7669", "#553830"]

    def __init__(self, simulation, journal, headless=False, paused=False, save_frames=True, read_only=False):
        import matplotlib.pyplot as plt
        self.plt = plt
        self.sim = simulation
        self.journal = journal
        self.headless = headless
        self.paused = paused
        self.save_frames = save_frames
        self.finished = read_only
        self.failed = getattr(simulation, "saved_status", None) == "error"
        self.closed = False
        self.view_robot = 0
        self.figure, self.axes = plt.subplots(1, 2, figsize=(14, 7))
        self.figure.subplots_adjust(left=0.055, right=0.98, bottom=0.21, top=0.86, wspace=0.17)
        self.figure.canvas.manager.set_window_title("M-TARE 2D — 3-robot VRP exploration"
                                                     if simulation.config.robots == 3 else "M-TARE 2D — VRP exploration")
        self.caption = self.figure.text(0.5, 0.092, "", ha="center", fontsize=9)
        self.state_caption = self.figure.text(0.5, 0.072, "", ha="center", fontsize=8)
        controls = ("0/1/2: robot map    A: team view    Q: close    Saved result — no simulation advances"
                    if read_only else
                    "Space: pause/resume    N: one cycle    0/1/2: robot map    A: team view    Q: stop/save")
        self.figure.text(0.5, 0.052, controls,
                         ha="center", fontsize=10)
        self.figure.text(0.5, 0.022, "Light blue: observed free    Solid: executed    Dashed: global VRP order    Dotted + circles: local coverage",
                         ha="center", fontsize=10)
        self.figure.canvas.mpl_connect("key_press_event", self.key)
        self.figure.canvas.mpl_connect("close_event", self.close)
        self.timer = self.figure.canvas.new_timer(interval=max(20, round(simulation.config.frame_delay*1000)))
        self.timer.add_callback(self.timer_tick)
        self.render()

    def _draw_route(self, axis, route, color):
        if len(route) < 3:
            return
        points = np.array([self.sim.world.xy(cell) for cell in route])
        axis.plot(points[:, 0], points[:, 1], "--", color=color, alpha=0.6, linewidth=1.2)
        axis.scatter(points[1:-1, 0], points[1:-1, 1], s=10, facecolors="none", edgecolors=color, zorder=4)

    def _draw_local(self, axis, robot, color):
        path = getattr(robot, "path", [])
        if path:
            points = np.array([robot.position, *[self.sim.world.xy(c) for c in path]])
            axis.plot(points[:,0], points[:,1], ":", color=color, linewidth=1.5, zorder=6)
        viewpoints = getattr(robot, "local_viewpoints", [])
        if viewpoints:
            points = np.array([self.sim.world.xy(c) for c in viewpoints])
            axis.scatter(points[:,0],points[:,1],s=45,facecolors="none",edgecolors=color,zorder=7)

    def render(self):
        from matplotlib.colors import ListedColormap
        from matplotlib.patches import Circle
        from matplotlib.lines import Line2D
        sim, world, cfg = self.sim, self.sim.world, self.sim.config
        combined = sim.union_map()
        left = np.zeros_like(world.truth)
        left[combined == 0] = 1
        left[world.truth == 1] = 2
        if self.view_robot is None:
            right_known = combined
        else:
            right_known = sim.robots[self.view_robot].known
        right = np.zeros_like(right_known)
        right[right_known == 0] = 1
        right[right_known == 1] = 2
        maps = [left, right]
        palettes = [["#fbfcfd", "#bfe5f6", "#8055a0"], ["#e0e5eb", "#bfe5f6", "#8055a0"]]
        for axis, data, palette in zip(self.axes, maps, palettes):
            axis.clear()
            axis.imshow(data, origin="lower", extent=[0, cfg.width, 0, cfg.height],
                        cmap=ListedColormap(palette), vmin=0, vmax=2, interpolation="nearest")
            axis.set_xlim(0, cfg.width)
            axis.set_ylim(0, cfg.height)
            axis.set_aspect("equal")
            axis.set_xlabel("x (map units / metres)")
            axis.set_ylabel("y")
            axis.set_xticks(np.arange(0, cfg.width+cfg.cell_size/2, cfg.cell_size), minor=True)
            axis.set_yticks(np.arange(0, cfg.height+cfg.cell_size/2, cfg.cell_size), minor=True)
            axis.grid(which="minor", color="#738293", alpha=0.16, linewidth=0.35)
            axis.tick_params(which="minor", bottom=False, left=False)
            if getattr(cfg, "architecture", "flat") == "hierarchical":
                for x in np.arange(0,cfg.width,cfg.region_size):
                    axis.axvline(x,color="#405875",alpha=0.35,linewidth=0.65)
                for y in np.arange(0,cfg.height,cfg.region_size):
                    axis.axhline(y,color="#405875",alpha=0.35,linewidth=0.65)
        self.axes[0].set_title("Simulator truth + team-observed cells", fontsize=11)
        self.axes[1].set_title("Team observation union (evaluation view)" if self.view_robot is None else
                              f"Robot {self.view_robot}'s own knowledge and proposed team plan", fontsize=11)
        for circle in world.circles:
            self.axes[0].add_patch(Circle((circle.x, circle.y), circle.radius, color="#9b3ca8", zorder=3))
        for robot in sim.robots:
            color = self.COLORS[robot.robot_id % len(self.COLORS)]
            trace = np.array(robot.trajectory)
            self.axes[0].plot(trace[:, 0], trace[:, 1], color=color, linewidth=2, zorder=5)
            self.axes[0].scatter(*robot.position, c=color, s=85, edgecolors="white", zorder=7)
            self.axes[0].text(robot.position[0]+1, robot.position[1]+1, str(robot.robot_id), color=color, weight="bold", zorder=8)
            if robot.proposal:
                self._draw_route(self.axes[0], robot.proposal.routes[robot.robot_id], color)
            self._draw_local(self.axes[0], robot, color)
            if robot.selected_tasks:
                self.axes[0].scatter(*world.xy(robot.selected_tasks[0]), marker="*", c=color, s=110, zorder=7)
            if self.view_robot is None or self.view_robot == robot.robot_id:
                self.axes[1].plot(trace[:, 0], trace[:, 1], color=color, linewidth=2, zorder=5)
                self._draw_local(self.axes[1], robot, color)
                shown_position = robot.position
                label = str(robot.robot_id)
            else:
                peer = sim.robots[self.view_robot].peers[robot.robot_id]
                shown_position = peer.position
                label = f"{robot.robot_id} cached"
            self.axes[1].scatter(*shown_position, c=color, s=65, edgecolors="white", zorder=7)
            self.axes[1].text(shown_position[0]+1, shown_position[1]+1, label, color=color, fontsize=8, zorder=8)
        if self.view_robot is None:
            for robot in sim.robots:
                if robot.proposal:
                    self._draw_route(self.axes[1], robot.proposal.routes[robot.robot_id], self.COLORS[robot.robot_id])
        else:
            robot = sim.robots[self.view_robot]
            if robot.proposal:
                for vehicle, route in enumerate(robot.proposal.routes):
                    self._draw_route(self.axes[1], route, self.COLORS[vehicle])
        legend_handles = [Line2D([0], [0], color=self.COLORS[r.robot_id], label=f"Robot {r.robot_id}")
                          for r in sim.robots]
        if self.view_robot is not None and cfg.comms_range is not None:
            selected = sim.robots[self.view_robot]
            color = self.COLORS[selected.robot_id % len(self.COLORS)]
            label = f"Robot {selected.robot_id}: {cfg.comms_range:g} m comms radius"
            for axis in self.axes:
                circle = Circle(selected.position, cfg.comms_range, fill=False,
                                edgecolor=color, linestyle="--", linewidth=1.5,
                                alpha=0.85, zorder=6)
                circle.set_gid("selected_robot_comms_range")
                axis.add_patch(circle)
            legend_handles.append(Line2D([0], [0], color=color, linestyle="--", label=label))
        self.axes[0].legend(handles=legend_handles, loc="upper left", fontsize=8)
        stats = sim.stats()
        if self.failed:
            mode = "ERROR"
        elif self.finished or sim.done:
            from .completion import COMPLETION_REASONS
            mode = "COMPLETE" if sim.reason in COMPLETION_REASONS else "STOPPED"
        else:
            mode = "PAUSED" if self.paused else "RUNNING"
        comms = "full comms" if cfg.comms_range is None else f"{cfg.comms_range:g} m comms"
        self.figure.suptitle(f"M-TARE 2D | {mode} | {comms} | cycle {sim.tick} | "
                             f"observed {100*stats['explored_ratio']:.2f}%" +
                             (" | no remaining exploration work" if sim.reason ==
                              "no_remaining_exploration_work" else ""), fontsize=13)
        self.caption.set_text(f"Global VRP: {stats['vrp_calls']} | Local plans: {stats.get('local_planning_calls',0)}    Accepted messages: {stats['messages_received']}    "
                              f"Direct peer-route switches: {stats['direct_peer_route_switches']}    "
                              f"Executed distance: {stats['team_distance']:.1f} m")
        if self.view_robot is not None:
            counts = stats.get("region_states_per_robot", [])
            values = counts[self.view_robot] if self.view_robot < len(counts) else {}
            self.state_caption.set_text("Region states — " + ", ".join(f"{key}: {value}" for key,value in values.items())
                                        if values else "")
        else:
            self.state_caption.set_text("Region states remain robot-specific; Team displays observation union only.")
        self.figure.canvas.draw_idle()

    def advance(self):
        if self.finished or self.sim.done:
            return
        stats = self.sim.step()
        self.render()
        if self.save_frames and self.sim.tick % self.sim.config.save_every == 0:
            self.figure.savefig(self.journal.directory / "frames" / f"cycle_{self.sim.tick:06d}.png", dpi=100)
        if self.sim.tick % 20 == 0 or self.sim.done:
            logging.info("Cycle %d: observed %.2f%%, distance %.1f m, VRP calls %d, switches %d",
                         self.sim.tick, 100*stats["explored_ratio"], stats["team_distance"],
                         stats["vrp_calls"], stats["direct_peer_route_switches"])
        if self.sim.done:
            from .completion import COMPLETION_REASONS
            self.finish("finished" if self.sim.reason in COMPLETION_REASONS else "stopped")

    def timer_tick(self):
        if self.finished:
            return False
        if not self.paused:
            try:
                self.advance()
            except Exception as exc:
                logging.exception("Simulation failed")
                self.sim.reason = str(exc)
                self.finish("error")
                return False
        return True

    def key(self, event):
        if event.key in ("q", "escape"):
            if not self.finished:
                self.sim.reason = "user_stop"
                self.finish("stopped")
            self.plt.close(self.figure)
        elif not self.finished:
            if event.key in (" ", "space"):
                self.paused = not self.paused
            elif event.key in ("n", "right"):
                self.paused = True
                self.advance()
        if event.key == "a":
            self.view_robot = None
        elif event.key and event.key.isdigit() and int(event.key) < len(self.sim.robots):
            self.view_robot = int(event.key)
        if not self.closed:
            self.render()

    def close(self, event):
        self.closed = True
        if not self.finished:
            self.sim.reason = "window_closed"
            self.finish("stopped")

    def finish(self, status):
        if self.finished:
            return
        self.finished = True
        self.failed = status == "error"
        self.timer.stop()
        if not self.closed:
            self.render()
        self.figure.savefig(self.journal.directory / "final.png", dpi=120, bbox_inches="tight")
        self.journal.finish(self.sim, status)
        metrics = self.sim.stats()
        logging.info("Final observed coverage: %.2f%% (%d / %d reachable free cells)",
                     100*metrics["explored_ratio"], metrics["explored_reachable_free_cells"],
                     metrics["total_reachable_free_cells"])
        logging.info("Run %s: %s.", status, self.sim.reason)

    def show(self):
        if not self.finished:
            self.timer.start()
        self.plt.show()
