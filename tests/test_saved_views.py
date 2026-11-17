"""Regression: finished headless results retain switchable, robot-specific views."""
import json
from pathlib import Path
import tempfile
import unittest

import matplotlib
matplotlib.use("Agg")
from matplotlib.backend_bases import KeyEvent, MouseEvent
import numpy as np

from legacy_config import simple_config as Config
from mtare_2d.recording import Journal
from mtare_2d.results_viewer import SavedRun, create_result_viewer
from mtare_2d.simulation import Simulation


class SavedViewTests(unittest.TestCase):
    def test_saved_robot_maps_buttons_keys_and_cached_positions_without_advancing(self):
        cfg = Config(architecture="flat", width=40, height=40, obstacles=0, robots=3, reveal_radius=4.1,
                     comms_range=1, solver_ms=20, max_cycles=12)
        with tempfile.TemporaryDirectory() as folder:
            journal = Journal(Path(folder), cfg)
            simulation = Simulation(cfg, journal)
            journal.initialize(simulation)
            while not simulation.done:
                simulation.step()
            journal.finish(simulation, "stopped")
            saved = SavedRun(journal.directory)
            for original, loaded in zip(simulation.robots, saved.robots):
                np.testing.assert_array_equal(loaded.known, original.known)
                np.testing.assert_allclose(loaded.position, original.position)
                self.assertEqual(loaded.selected_tasks, original.selected_tasks)
                self.assertEqual(loaded.proposal.routes, [list(row) for row in original.proposal.routes])
                for other in original.peers:
                    self.assertEqual(loaded.peers[other].position, original.peers[other].position)
            self.assertFalse(np.array_equal(saved.robots[0].known, saved.robots[1].known))
            self.assertNotEqual(saved.robots[0].peers[1].position, saved.robots[1].position)
            viewer = create_result_viewer(journal.directory)
            original_tick = saved.tick
            for key, target in [("1", 1), ("2", 2), ("a", None), ("0", 0)]:
                viewer.figure.canvas.callbacks.process("key_press_event", KeyEvent(
                    "key_press_event", viewer.figure.canvas, key=key))
                self.assertEqual(viewer.view_robot, target)
                known = viewer.sim.union_map() if target is None else saved.robots[target].known
                expected = np.zeros_like(known)
                expected[known == 0] = 1
                expected[known == 1] = 2
                np.testing.assert_array_equal(viewer.axes[1].images[0].get_array(), expected)
            for index in [1, 2, 3, 0]:
                button = viewer.result_buttons[index]
                viewer.figure.canvas.draw()
                box = button.ax.get_window_extent()
                x, y = box.x0 + box.width/2, box.y0 + box.height/2
                for kind in ["button_press_event", "button_release_event"]:
                    viewer.figure.canvas.callbacks.process(kind, MouseEvent(
                        kind, viewer.figure.canvas, x, y, button=1))
                self.assertEqual(viewer.view_robot, None if index == 3 else index)
            for key in ["n", "space", "right"]:
                viewer.key(type("Event", (), {"key": key})())
            self.assertEqual(viewer.sim.tick, original_tick)
            viewer.key(type("Event", (), {"key": "q"})())
            self.assertEqual(json.loads((journal.directory / "summary.json").read_text())["status"], "stopped")


if __name__ == "__main__":
    unittest.main()
