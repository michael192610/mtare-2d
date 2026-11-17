"""Coverage-gated simulation completion, independent of VRP optimization."""
from dataclasses import replace
import math
import unittest
from unittest.mock import patch

from mtare_2d.config import Config
from mtare_2d.simulation import Simulation
from mtare_2d.vrp import Proposal
from mtare_2d.world import FREE, UNKNOWN


class CoverageIdleCompletionTests(unittest.TestCase):
    def idle(self, coverage=0.91):
        sim = Simulation(Config(robots=3, width=20, height=20, obstacles=0,
                                completion_startup_cycles=0, planning_mode='sequential'))
        sim.tick = 4
        count = round(coverage * sim.world.size)
        for robot in sim.robots:
            robot.known[:] = UNKNOWN
            robot.known.ravel()[:count] = FREE
            robot.calls = robot.local_calls = 1
            robot.last_local_plan_tick = sim.tick
            robot.local_coverage_complete = True
            depot = sim.world.cell_at(robot.position)
            # Each private VRP solution gives a task to a teammate, leaving
            # its author idle. These nonempty global lists must not deadlock.
            routes = [(depot, depot) for _ in sim.robots]
            peer = (robot.robot_id + 1) % len(sim.robots)
            routes[peer] = (depot, sim.world.size - 1, depot)
            robot.proposal = Proposal('fixture', robot.robot_id, sim.tick, tuple(routes))
            robot.global_exploring_regions = [2]
        return sim

    def test_exactly_91_percent_and_everyone_waiting_for_peers_stops(self):
        sim = self.idle()
        self.assertEqual(sim.stats()['explored_ratio'], 0.91)
        sim.check_planner_completion()
        self.assertTrue(sim.done)
        self.assertTrue(all(r.exploration_finished for r in sim.robots))
        before = (sim.tick, [r.position for r in sim.robots], sim.stats()['team_distance'])
        for _ in range(10):
            sim.step()
        self.assertEqual(before, (sim.tick, [r.position for r in sim.robots], sim.stats()['team_distance']))

    def test_below_threshold_does_not_finish_even_if_all_global_lists_empty(self):
        sim = self.idle(0.90)
        for robot in sim.robots:
            robot.global_exploring_regions = []
        sim.check_planner_completion()
        self.assertFalse(sim.done)

    def test_full_coverage_with_work_continues(self):
        for work in ('assigned', 'proposal', 'local_gain', 'viewpoint', 'motion'):
            with self.subTest(work=work):
                sim = self.idle(1)
                robot = sim.robots[1]
                if work == 'assigned':
                    robot.selected_tasks = [9]
                elif work == 'proposal':
                    routes = list(robot.proposal.routes)
                    routes[1] = (0, 9, 0)
                    robot.proposal = replace(robot.proposal, routes=tuple(routes))
                elif work == 'local_gain':
                    robot.local_coverage_complete = False
                elif work == 'viewpoint':
                    robot.local_viewpoints = [9]
                else:
                    robot.path = [next(i for i in range(sim.world.size)
                                       if math.dist(robot.position, sim.world.xy(i)) > 1)]
                sim.check_planner_completion()
                self.assertFalse(sim.done)

    def test_stale_or_missing_plans_cannot_count_as_empty_successful_solutions(self):
        for missing in ('global', 'local', 'unplanned'):
            with self.subTest(missing=missing):
                sim = self.idle(1)
                robot = sim.robots[0]
                if missing == 'global':
                    robot.proposal = replace(robot.proposal, tick=0)
                elif missing == 'local':
                    robot.last_local_plan_tick = 0
                else:
                    robot.proposal = None
                sim.check_planner_completion()
                self.assertFalse(sim.done)

    def test_solver_exception_is_an_error_not_successful_completion(self):
        sim = self.idle(1)
        with patch.object(sim, 'update_region_knowledge'), \
             patch.object(sim, 'plan_all', side_effect=RuntimeError('no assignment')):
            with self.assertRaisesRegex(RuntimeError, 'no assignment'):
                sim.step()
        self.assertFalse(sim.done)

    def test_startup_guard_still_applies(self):
        sim = self.idle(1)
        sim.config = replace(sim.config, completion_startup_cycles=4)
        sim.check_planner_completion()
        self.assertFalse(sim.done)

    def test_threshold_validation(self):
        for value in (0, -0.01, 1.01, float('nan')):
            with self.assertRaises(ValueError):
                Config(completion_min_coverage=value)


if __name__ == '__main__':
    unittest.main()
