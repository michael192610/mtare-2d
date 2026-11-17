"""Browse saved maps, cached positions and proposals without running a solver."""
import argparse
import json
import os
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from .viewer import Viewer
from .regions import Regions


def read_json(path):
    return json.loads(path.read_text())


class SavedWorld:
    def __init__(self, scene, truth):
        self.truth = truth
        self.cols = scene["cols"]
        self.cell_size = scene["cell_size"]
        self.size = scene["rows"]*scene["cols"]
        self.regions = Regions(scene["rows"],scene["cols"],self.cell_size,scene.get("region_size",6))
        self.viewpoint_geometry=scene.get("viewpoint_geometry")
        self.circles = [SimpleNamespace(**circle) for circle in scene["circles"]]

    def xy(self, cell):
        v=self.viewpoint_geometry
        if v and int(cell)>=v['id_offset']:
            row,col=divmod(int(cell)-v['id_offset'],v['cols'])
            return ((col+.5)*v['spacing'],(row+.5)*v['spacing'])
        if int(cell) >= self.size:
            return self.regions.centre(self.regions.region(cell))
        row, col = divmod(int(cell), self.cols)
        return ((col + 0.5) * self.cell_size, (row + 0.5) * self.cell_size)


class SavedRun:
    """Only the fields needed by Viewer; intentionally has no step() method."""
    def __init__(self, directory):
        self.directory = Path(directory).expanduser().resolve()
        summary = read_json(self.directory / "summary.json")
        scene = read_json(self.directory / "scene.json")
        self.config = SimpleNamespace(**read_json(self.directory / "config.json"))
        self.final_stats = summary["final"]
        self.tick = self.final_stats["cycle"]
        self.reason = summary["reason"]
        self.saved_status = summary["status"]
        self.done = True
        traces = read_json(self.directory / "trajectories.json")
        with np.load(self.directory / "maps.npz", allow_pickle=False) as arrays:
            self.world = SavedWorld(scene, arrays["truth"].copy())
            self.robots = [SimpleNamespace(
                robot_id=i, trajectory=traces[str(i)], position=tuple(traces[str(i)][-1]),
                known=arrays[f"robot_{i}"].copy(), selected_tasks=[], proposal=None,
                local_viewpoints=[], path=[], local_plan_id=None, local_navigation_end=None,
                region_states=[], local_exploring_regions=[],
                peers={j: SimpleNamespace(position=tuple(scene["starts"][j]), position_tick=-1)
                       for j in range(self.config.robots) if j != i})
                for i in range(self.config.robots)]
        latest_calls = {}
        with (self.directory / "events.jsonl").open() as stream:
            for line in stream:
                event = json.loads(line)
                if event["kind"] == "receive":
                    peer = self.robots[event['receiver']].peers[event['sender']]
                    stamp = event.get('position_tick',event['sent_tick'])
                    if stamp >= peer.position_tick:
                        peer.position = tuple(event.get('cached_position',event['position']))
                        peer.position_tick = stamp
                elif event['kind'] == 'position_forwarded':
                    peer = self.robots[event['receiver']].peers[event['teammate']]
                    if event['position_tick'] > peer.position_tick:
                        peer.position = tuple(event['position'])
                        peer.position_tick = event['position_tick']
                elif event["kind"] == "plan_selected":
                    robot = self.robots[event["robot_id"]]
                    robot.selected_tasks = list(event["selected_own_tasks"])
                    latest_calls[robot.robot_id] = event["call_id"]
                elif event["kind"] in ("target_reached", "target_no_longer_frontier", "navigation_unreachable"):
                    tasks = self.robots[event["robot_id"]].selected_tasks
                    cell = event["target"] if event["kind"] == "navigation_unreachable" else event["cell_id"]
                    if tasks and tasks[0] == cell:
                        tasks.pop(0)
        for robot_id, call_id in latest_calls.items():
            call = read_json(self.directory / "calls" / f"{call_id}.json")
            self.robots[robot_id].proposal = SimpleNamespace(routes=call["team_routes_cell_ids"])

        state_path = self.directory / "final_planning_state.json"
        if state_path.exists():
            for robot_id, state in read_json(state_path).items():
                robot = self.robots[int(robot_id)]
                robot.selected_tasks = state["selected_tasks"]
                robot.local_viewpoints = state["local_viewpoints"]
                robot.path = state["local_path"]
                robot.local_plan_id = state["local_plan_id"]
                robot.local_navigation_end = state["local_navigation_end"]
                robot.local_exploring_regions = state.get("local_exploring_regions", [])
                for teammate, position in state.get('cached_positions', {}).items():
                    robot.peers[int(teammate)].position = tuple(position)
        region_path = self.directory / "region_states.json"
        if region_path.exists():
            for robot_id, states in read_json(region_path).items():
                self.robots[int(robot_id)].region_states = states

    def union_map(self):
        combined = np.full_like(self.world.truth, -1)
        for robot in self.robots:
            known = robot.known != -1
            combined[known] = robot.known[known]
        return combined

    def stats(self):
        return self.final_stats.copy()


def create_result_viewer(directory):
    from matplotlib.widgets import Button
    saved = SavedRun(directory)
    viewer = Viewer(saved, None, paused=True, save_frames=False, read_only=True)
    viewer.figure.canvas.manager.set_window_title("M-TARE 2D — saved results — select a robot")
    viewer.result_buttons = []  # Keep widgets alive for their event callbacks.
    count = len(saved.robots) + 1
    width = min(0.12, 0.85 / count)
    start = 0.5 - count * width / 2
    for slot, source in enumerate([*range(len(saved.robots)), None]):
        axis = viewer.figure.add_axes([start + slot * width, 0.115, width * 0.91, 0.04])
        button = Button(axis, "Team" if source is None else f"Robot {source}")
        def select(event, source=source):
            viewer.view_robot = source
            viewer.render()
        button.on_clicked(select)
        viewer.result_buttons.append(button)
    return viewer


def show_results(directory):
    import matplotlib
    matplotlib.use("TkAgg")
    viewer = create_result_viewer(directory)
    viewer.show()


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", nargs="?", type=Path, help="Saved run folder; defaults to latest_run.txt.")
    args = parser.parse_args()
    directory = args.run or Path((root / "latest_run.txt").read_text().strip())
    os.environ.setdefault("MPLCONFIGDIR", str(root / ".mplconfig"))
    show_results(directory)


if __name__ == "__main__":
    main()
