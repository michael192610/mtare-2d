"""New default: retained M-TARE decisions over explicit planar evidence."""
import unittest
from unittest.mock import patch

from mtare_2d.config import Config
from mtare_2d.dense_coverage import DenseCoveragePlanner
from mtare_2d.region_state import CellStatus as S, Evidence, RegionKnowledge, RegionUpdate
from mtare_2d.simulation import Simulation
from mtare_2d.world import FREE


class MTAREDefaultTests(unittest.TestCase):
    def sim(self):
        return Simulation(Config(robots=1, width=30, height=30, obstacles=0, solver_ms=10))

    def test_default_selects_coverage_ledger_and_original_state_protocol(self):
        sim = self.sim()
        robot = sim.robots[0]
        self.assertIs(type(sim.local_planner), DenseCoveragePlanner)
        self.assertIs(type(robot.region_knowledge), RegionKnowledge)
        self.assertIsNotNone(robot.coverage)
        self.assertEqual(sim.config.viewpoint_spacing, .6)
        self.assertEqual(sim.config.region_covered_to_exploring_count, 10)
        self.assertTrue(sim.config.strict_frontier)

    def test_ordinary_work_runs_repeated_trials_and_shortest_route_wins(self):
        sim = self.sim()
        robot = sim.robots[0]
        robot.known[:] = FREE
        record = sim.local_planner.plan(robot, 0)
        self.assertTrue(record['ordinary_sample_cells'])
        self.assertTrue(record['ordinary_queue'])
        self.assertEqual(len(record['trials']), 10)
        self.assertEqual(record['route']['length'], min(t['route_length'] for t in record['trials']))
        self.assertTrue(record['selected_viewpoints'])
        self.assertFalse(record['local_coverage_complete'])

    def test_frontier_queue_admission_and_initial_gate_have_distinct_boundaries(self):
        sim = self.sim()
        robot, planner = sim.robots[0], sim.local_planner
        ctx = planner.context(robot)
        candidate = next(c for c in ctx['candidates'] if c not in ctx['visited'])
        for count, gate in ((1, False), (2, True)):
            with self.subTest(count=count):
                context = dict(ctx, samples=set(), frontier_samples=set(range(count)))
                context['coverage'] = {c:set() for c in ctx['candidates']}
                context['frontier_coverage'] = {
                    c:(set(range(count)) if c == candidate else set()) for c in ctx['candidates']}
                with patch.object(planner, 'context', return_value=context), \
                     patch.object(planner, 'navigation', return_value=(ctx['start'], None, 'none')):
                    record = planner.plan(robot, 0)
                self.assertIn(candidate, record['frontier_queue'])  # >=1 admission
                self.assertEqual(record['frontier_gate'], gate)  # >1 initial gate
                self.assertEqual(candidate in record['selected_viewpoints'], gate)

    def test_frontier_presence_alone_does_not_activate_an_unselected_region(self):
        knowledge = RegionKnowledge(0, 1, Config())
        knowledge.update_local({0}, {0:Evidence(candidates=3, above_frontier=3)})
        self.assertEqual(knowledge.regions[0].status, S.UNSEEN)
        self.assertTrue(knowledge.regions[0].covered_to_others)
        knowledge.update_local({0}, {0:Evidence(candidates=3, selected=1)})
        self.assertEqual(knowledge.regions[0].status, S.EXPLORING)

    def test_reopening_and_message_suppression_use_original_count_rules(self):
        knowledge = RegionKnowledge(0, 1, Config(robots=2))
        cell = knowledge.regions[0]
        cell.status = S.COVERED
        knowledge.update_local({0}, {0:Evidence(candidates=11, above_big=10)})
        self.assertEqual(cell.status, S.COVERED)
        knowledge.update_local({0}, {0:Evidence(candidates=11, above_big=11)})
        self.assertEqual(cell.status, S.EXPLORING)
        self.assertEqual(cell.update_id, 1)  # Counter, not global cycle timestamp.
        knowledge.sync_peer(RegionUpdate(0, 1, 3, 0, int(S.EXPLORING)), 1)
        for _ in range(5):
            self.assertEqual(len(knowledge.updates({0})), 1)
        self.assertEqual(knowledge.updates({0}), ())
        self.assertEqual(len(knowledge.updates({0}, {0})), 1)  # Global-work exception.

    def test_idle_completion_settles_only_pending_regions_with_no_coverage_evidence(self):
        sim = self.sim()
        robot = sim.robots[0]
        knowledge = robot.region_knowledge
        knowledge.window = {0,1,2,3}
        for region in knowledge.window:
            knowledge.set_status(region, S.EXPLORING_BY_OTHERS, 0, 'fixture')
            knowledge.regions[region].almost_covered = True
        knowledge.evidence = {
            0:Evidence(candidates=3),
            1:Evidence(candidates=3, above_small=1),
            2:Evidence(candidates=3, above_frontier=1),
            3:Evidence(candidates=3, selected=1)}
        robot.calls = robot.local_calls = 1
        robot.local_coverage_complete = True
        robot.global_exploring_regions = [10]
        sim.check_planner_completion()
        self.assertEqual(knowledge.regions[0].status, S.EXPLORING_BY_OTHERS)
        robot.global_exploring_regions = []
        sim.check_planner_completion()
        self.assertEqual(knowledge.regions[0].status, S.COVERED)
        self.assertEqual(knowledge.regions[0].update_id, 1)
        for region in (1,2,3):
            self.assertEqual(knowledge.regions[region].status, S.EXPLORING_BY_OTHERS)


if __name__ == '__main__':
    unittest.main()
