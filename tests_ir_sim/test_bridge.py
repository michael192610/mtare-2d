import copy
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import matplotlib
matplotlib.use("Agg")
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ir_sim.mtare_bridge import Config, IRSimBridge, Journal, Simulation, scene_config
import mtare_2d.simulation as simulation_module


class BridgeTests(unittest.TestCase):
    def test_exported_scene_preserves_obstacles_and_starts(self):
        sim = Simulation(Config())
        try:
            scene = scene_config(sim)
            self.assertEqual(scene["world"]["step_mode"], "external")
            self.assertEqual(len(scene["obstacle"]), len(sim.world.circles))
            for item, circle in zip(scene["obstacle"], sim.world.circles):
                self.assertEqual(item["state"][:2], [circle.x, circle.y])
                self.assertEqual(item["shape"]["radius"], circle.radius)
            for item, robot in zip(scene["robot"], sim.robots):
                self.assertEqual(item["state"][:2], list(robot.position))
                self.assertNotIn("sensors", item)
        finally:
            sim.close()

    def test_external_backend_preserves_planner_inputs_and_state_evolution(self):
        cfg = Config(robots=3, width=24, height=24, obstacles=0, max_cycles=12,
                     planning_mode="sequential", solver_ms=10, comms_range=12)
        baseline = Simulation(cfg)
        recorded = []
        original_search = simulation_module.run_search

        def record(*args):
            result = original_search(*args)
            recorded.append((copy.deepcopy(args), copy.deepcopy(result)))
            return result

        with patch.object(simulation_module, "run_search", record):
            while not baseline.done:
                baseline.step()
        # Wall-time-limited global searches need not repeat bit-for-bit. Replay
        # identical search results while independently checking every input.
        cursor = 0
        def replay(*args):
            nonlocal cursor
            expected, result = recorded[cursor]
            cursor += 1
            np.testing.assert_array_equal(args[0], expected[0])
            self.assertEqual(args[1:], expected[1:])
            return copy.deepcopy(result)

        with tempfile.TemporaryDirectory() as folder:
            journal = Journal(Path(folder), cfg)
            sim = Simulation(cfg, journal)
            journal.initialize(sim)
            bridge = IRSimBridge(sim, journal, headless=True)
            try:
                with patch.object(simulation_module, "run_search", replay):
                    while not sim.done:
                        bridge.advance(render=False)
                self.assertEqual(cursor, len(recorded))
                self.assertEqual(bridge.steps, sim.tick)
                self.assertEqual(bridge.max_pose_error, 0)
                self.assertEqual(sim.tick, baseline.tick)
                self.assertEqual(sim.reason, baseline.reason)
                for actual, expected in zip(sim.robots, baseline.robots):
                    self.assertEqual(actual.position, expected.position)
                    self.assertEqual(actual.trajectory, expected.trajectory)
                    np.testing.assert_array_equal(actual.known, expected.known)
                    self.assertEqual(actual.selected_tasks, expected.selected_tasks)
                    self.assertEqual(actual.region_knowledge.snapshot(), expected.region_knowledge.snapshot())
                tick, steps = sim.tick, bridge.steps
                for _ in range(5):
                    bridge.advance(render=False)
                self.assertEqual((sim.tick, bridge.steps), (tick, steps))
            finally:
                bridge.plt.close(bridge.figure)
                bridge.env.end(ending_time=0)
                sim.close()
                baseline.close()


if __name__ == "__main__":
    unittest.main()
