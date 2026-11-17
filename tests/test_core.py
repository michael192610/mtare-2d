import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

from legacy_config import simple_config as Config
from mtare_2d.recording import Journal
from mtare_2d.simulation import Message, Simulation
from mtare_2d.vrp import INF, Proposal, adapt_proposal, distance_matrices, select_seed, solve
from mtare_2d.world import BLOCKED, FREE, UNKNOWN, GridGraph, World


class GridTests(unittest.TestCase):
    def test_scene_matches_original_em_seed(self):
        world = World(Config(architecture="flat"))
        self.assertEqual(len(world.circles), 30)
        self.assertEqual((world.rows, world.cols), (50, 50))
        np.testing.assert_allclose(world.starts, [[15.325477443379015, 47.13353410054418],
                                                 [15.13864351612643, 50.34978700821671],
                                                 [22.297599508446595, 51.74285893238813]])
        self.assertAlmostEqual(world.circles[0].x, 31.205784063290988)
        self.assertAlmostEqual(world.circles[-1].y, 1.9206941109639168)

    def test_reveal_is_partial_and_deterministic(self):
        world = World(Config(architecture="flat", obstacles=0))
        known = np.full_like(world.truth, UNKNOWN)
        revealed = world.reveal(known, world.starts[0])
        self.assertGreater(len(revealed), 0)
        self.assertLess(len(revealed), world.size//5)
        self.assertEqual(known[-1, -1], UNKNOWN)
        self.assertFalse(world.reveal(known, world.starts[0]))

    def test_grid_paths_detour_around_blocked_and_unknown_cells(self):
        world = World(Config(architecture="flat", robots=1, width=12, height=12, obstacles=0))
        known = np.full_like(world.truth, FREE)
        known[0, 1] = BLOCKED
        graph = GridGraph(world, known)
        route = graph.path(0, 2)
        self.assertEqual(len(route), 4)
        self.assertNotIn(1, route)
        known[:, 1] = UNKNOWN
        self.assertIsNone(GridGraph(world, known).path(0, 2))

    def test_distance_builder_cannot_read_hidden_truth(self):
        world = World(Config(architecture="flat", robots=1, width=12, height=12, obstacles=0, task_arc_offset=0))
        known = np.full_like(world.truth, FREE)
        before = distance_matrices(world, known, [0], [2], 0, set(), {})
        world.truth[:] = BLOCKED
        after = distance_matrices(world, known, [0], [2], 0, set(), {})
        np.testing.assert_array_equal(before, after)
        self.assertEqual(int(before[0, 0, 1]), 4)


class VRPTests(unittest.TestCase):
    def setUp(self):
        base = np.array([[0, 10, 1, 9], [10, 0, 9, 1],
                         [1, 9, 0, 508], [9, 1, 508, 0]], dtype=np.int64)
        self.matrices = np.repeat(base[None], 2, axis=0)
        self.config = Config(architecture="flat", robots=2, solver_ms=40)

    def test_peer_seed_wins_and_real_ortools_retains_unique_optimum(self):
        own = Proposal("own", 0, 0, ((10, 30, 40, 10), (20, 20)))
        peer = Proposal("peer", 1, 0, ((10, 30, 10), (20, 40, 20)))
        best, candidates = select_seed([own, peer], [30, 40], self.matrices, 2)
        self.assertEqual(best["source"], 1)
        self.assertLess(best["longest_route_cost"], candidates[0]["longest_route_cost"])
        result = solve(self.matrices, self.config, best["initial_routes"])
        self.assertTrue(result["initial_routes_accepted"])
        self.assertEqual(result["routes"], [[2], [3]])
        self.assertEqual(result["objective"], 204)
        self.assertEqual(result["verified_objective"], 204)

    def test_adaptation_removes_obsolete_duplicate_tasks_and_inserts_new(self):
        proposal = Proposal("old", 0, 0, ((10, 999, 30, 30, 10), (20, 20)))
        adapted = adapt_proposal(proposal, [30, 40], self.matrices, 2)
        self.assertEqual(sorted(node for row in adapted for node in row), [2, 3])

    def test_no_hint_uses_actual_solver(self):
        result = solve(self.matrices, self.config)
        self.assertTrue(result["solver_called"])
        self.assertFalse(result["initial_routes_accepted"])
        self.assertEqual(result["objective"], 204)

    def test_unreachable_task_is_dropped_and_penalty_verified(self):
        matrix = np.array([[[0, INF], [INF, 0]]], dtype=np.int64)
        result = solve(matrix, Config(architecture="flat", robots=1, solver_ms=30))
        self.assertEqual(result["routes"], [[]])
        self.assertEqual(result["dropped_node_indices"], [1])
        self.assertEqual(result["objective"], 9_999_999)

    def test_empty_targets_do_not_call_solver(self):
        result = solve(np.zeros((2, 2, 2), dtype=np.int64), self.config)
        self.assertFalse(result["solver_called"])


class MessageTests(unittest.TestCase):
    def test_message_is_copied_delayed_and_applied_without_shared_map_alias(self):
        sim = Simulation(Config(architecture="flat", robots=2, obstacles=0))
        self.assertFalse(np.shares_memory(sim.robots[0].known, sim.robots[1].known))
        sim.publish()
        unknown = next(i for i in range(sim.world.size)
                       if all(r.known.ravel()[i] == UNKNOWN for r in sim.robots))
        sim.robots[0].known.ravel()[unknown] = FREE
        sim.deliver()
        self.assertEqual(sim.event_counts["receive"], 0)
        sim.tick = 1
        sim.deliver()
        self.assertEqual(sim.robots[1].known.ravel()[unknown], UNKNOWN)
        sim.publish()
        sim.tick = 2
        sim.deliver()
        self.assertEqual(sim.robots[1].known.ravel()[unknown], FREE)

    def test_out_of_range_and_stale_messages_cannot_update_cache(self):
        sim = Simulation(Config(architecture="flat", robots=2, obstacles=0, comms_range=30))
        sim.robots[0].position, sim.robots[1].position = (1, 1), (95, 95)
        sim.publish()
        self.assertEqual(len(sim.queue), 0)
        sim.robots[1].position = (3, 1)
        packet = Message(1, 2, 0, (3, 1), (), None)
        sim.receive(0, packet)
        sim.receive(0, Message(1, 1, 0, (7, 7), (), None))
        self.assertEqual(sim.robots[0].peers[1].position, (3, 1))
        self.assertEqual(sim.event_counts["message_stale"], 1)

    def test_actual_received_proposal_can_cause_logged_route_switch(self):
        sim = Simulation(Config(architecture="flat", robots=2, width=20, height=20, obstacles=0, solver_ms=40))
        left, right, task_left, task_right = 0, 99, 1, 98
        for robot, cell in zip(sim.robots, [left, right]):
            robot.position = sim.world.xy(cell)
            robot.known[:] = FREE
        robot = sim.robots[0]
        robot.peers[1].position = sim.world.xy(right)
        robot.proposal = Proposal("own_previous", 0, -1, ((left, task_left, task_right, left), (right, right)))
        robot.selected_tasks = [task_left, task_right]
        peer = Proposal("peer_proposal", 1, -1, ((left, task_left, left), (right, task_right, right)))
        sim.receive(0, Message(1, 0, 0, sim.world.xy(right),
                               tuple((i, FREE) for i in range(sim.world.size)), peer))
        sim.world.targets = lambda known, depots: ([task_left, task_right], [task_left, task_right])
        record = sim.plan(robot)
        self.assertTrue(record["direct_peer_route_switch"])
        self.assertEqual(record["selected_seed_proposal_id"], "peer_proposal")
        self.assertEqual(robot.selected_tasks, [task_left])
        self.assertEqual(len(sim.switches), 1)
        self.assertEqual(sim.switches[0]["sender"], 1)


class IntegrationTests(unittest.TestCase):
    def test_three_robot_exploration_and_saved_vrp_calls(self):
        cfg = Config(architecture="flat", width=20, height=20, obstacles=0, robots=3, reveal_radius=5.5,
                     solver_ms=20, max_cycles=100)
        with tempfile.TemporaryDirectory() as folder:
            journal = Journal(Path(folder), cfg)
            sim = Simulation(cfg, journal)
            journal.initialize(sim)
            while not sim.done:
                sim.step()
            journal.finish(sim)
            self.assertEqual(sim.reason, "all_reachable_free_cells_observed")
            self.assertEqual(sim.stats()["explored_ratio"], 1.0)
            self.assertTrue(all(a["explored_ratio"] <= b["explored_ratio"]
                                for a, b in zip(sim.history, sim.history[1:])))
            for path in (journal.directory / "calls").glob("*.json"):
                record = json.loads(path.read_text())
                self.assertEqual(record["result"]["objective"], record["result"]["verified_objective"])
                self.assertEqual(len(record["node_cell_ids"]), len(record["distance_matrices"][0]))
            self.assertTrue((journal.directory / "RESULTS.txt").is_file())
            self.assertTrue((journal.directory / "SWITCHES.txt").is_file())
            self.assertTrue(math.isfinite(sim.stats()["team_distance"]))

    def test_no_estimation_or_em_dependencies_loaded(self):
        self.assertNotIn("gtsam", sys.modules)
        self.assertNotIn("torch", sys.modules)


if __name__ == "__main__":
    unittest.main()
