"""Coarse peer completion removes redundant work, without map omniscience."""
import unittest
from unittest.mock import patch

import numpy as np

from mtare_2d.config import Config
from mtare_2d.region_state import CellStatus as S, RegionUpdate
from mtare_2d.simulation import Message, Simulation
from mtare_2d.world import FREE, UNKNOWN


class PeerCoverageTests(unittest.TestCase):
    def sim(self):
        return Simulation(Config(robots=2, width=40, height=40, obstacles=0,
                                 solver_ms=10, planning_mode='sequential'))

    def test_accepted_completion_suppresses_target_samples_without_sharing_occupancy(self):
        sim = self.sim()
        robot = sim.robots[0]
        before = sim.local_planner.context(robot)
        target = next(iter(before['frontier_samples']))
        region = sim.world.region_of(target)
        known = robot.known.copy()
        packet = Message(1, 0, 0, sim.robots[1].position, (), None,
                         (RegionUpdate(region, 100, 2, 0, int(S.COVERED)),))
        sim.receive(0, packet)
        sim.update_region_knowledge()
        self.assertEqual(robot.region_knowledge.regions[region].status, S.COVERED_BY_OTHERS)
        after = sim.local_planner.context(robot)
        self.assertIn(target, after['peer_coverage']['suppressed_frontier_sample_cells'])
        self.assertFalse(any(sim.world.region_of(c) == region for c in after['frontier_samples']))
        np.testing.assert_array_equal(robot.known, known)
        self.assertEqual(robot.known.ravel()[target], UNKNOWN)
        self.assertFalse(robot.coverage.observed[target] or robot.coverage.covered[target])
        # Paths and candidates retain their original private-map validity.
        self.assertEqual(before['candidates'], after['candidates'])

    def test_unknown_sample_across_completed_source_boundary_remains_work(self):
        sim = self.sim()
        robot, world = sim.robots[0], sim.world
        planner = sim.local_planner
        ctx = planner.context(robot)
        # Find a real private frontier crossing a coarse region boundary.
        pair = next((c,n) for c in world.frontier_ids(robot.known)
                    for n in world.neighbors[c] if robot.known.ravel()[n] == UNKNOWN
                    and world.region_of(c) != world.region_of(n)
                    and n in ctx['frontier_samples'])
        source, target = pair
        robot.region_knowledge.regions[world.region_of(source)].status = S.COVERED_BY_OTHERS
        updated = planner.context(robot)
        self.assertNotIn(target, updated['peer_coverage']['suppressed_frontier_sample_cells'])
        # Source ownership cannot erase adjacent unknown-region work. Require
        # some other eligible viewpoint support to distinguish visited masking.
        raw_target = any(target in planner.visible(c) for c in updated['candidates']
                         if c not in updated['visited'])
        self.assertEqual(target in updated['frontier_samples'], raw_target)

    def test_newer_exploring_report_reenables_previously_suppressed_work(self):
        sim = self.sim()
        robot = sim.robots[0]
        target = next(iter(sim.local_planner.context(robot)['frontier_samples']))
        region = sim.world.region_of(target)
        knowledge = robot.region_knowledge
        self.assertTrue(knowledge.apply_peer(RegionUpdate(region, 10, 2, 0, int(S.COVERED)), 1))
        self.assertNotIn(target, sim.local_planner.context(robot)['frontier_samples'])
        self.assertFalse(knowledge.apply_peer(RegionUpdate(region, 9, 2, 0, int(S.EXPLORING)), 1))
        self.assertTrue(knowledge.apply_peer(RegionUpdate(region, 11, 2, 0, int(S.EXPLORING)), 1))
        reopened = sim.local_planner.context(robot)
        self.assertEqual(knowledge.regions[region].status, S.EXPLORING_BY_OTHERS)
        self.assertNotIn(region, reopened['peer_coverage']['region_ids'])
        self.assertIn(target, reopened['frontier_samples'])

    def test_all_peer_completed_targets_finish_locally_with_private_unknown_cells(self):
        sim = self.sim()
        robot = sim.robots[0]
        before = sim.local_planner.plan(robot, 0)
        self.assertTrue(before['selected_viewpoints'])
        robot.local_viewpoints = before['selected_viewpoints'].copy()
        # Leave candidate regions UNSEEN. Peer-covered *target* samples must
        # cancel reuse from neighbouring candidate regions as well.
        targets = before['frontier_sample_cells']
        for region in {sim.world.region_of(c) for c in targets}:
            robot.region_knowledge.apply_peer(RegionUpdate(region, 10, 2, 0, int(S.COVERED)), 1)
        after = sim.local_planner.plan(robot, 1)
        self.assertTrue(after['local_coverage_complete'])
        self.assertFalse(after['reused_viewpoints'] or after['selected_viewpoints'])
        self.assertTrue((robot.known == UNKNOWN).any())
        self.assertTrue(after['local_candidate_cells'])  # Known-free transit retained.

    def test_own_covered_region_can_reopen_and_unreported_work_is_preserved(self):
        sim = self.sim()
        robot = sim.robots[0]
        before = sim.local_planner.context(robot)
        target = next(iter(before['frontier_samples']))
        robot.region_knowledge.regions[sim.world.region_of(target)].status = S.COVERED
        after = sim.local_planner.context(robot)
        self.assertIn(target, after['frontier_samples'])
        self.assertFalse(after['peer_coverage']['region_ids'])
        # Neither hidden truth nor another robot's uncommunicated map is consulted.
        known = robot.known.copy()
        sim.world.truth[:] = 1
        sim.robots[1].known[:] = FREE
        again = sim.local_planner.context(robot)
        self.assertEqual(after['frontier_samples'], again['frontier_samples'])
        np.testing.assert_array_equal(robot.known, known)

    def test_observed_ordinary_samples_in_peer_completed_regions_do_not_add_gain(self):
        sim = self.sim()
        robot = sim.robots[0]
        robot.known[:] = FREE
        ctx = sim.local_planner.context(robot)
        sample = next(iter(ctx['samples']))
        region = sim.world.region_of(sample)
        robot.region_knowledge.apply_peer(RegionUpdate(region, 10, 2, 0, int(S.COVERED)), 1)
        # Keep the ledger unmodified to isolate sample ownership from the
        # original visited-viewpoint coverage credit.
        with patch.object(robot.coverage, 'credit_visited_viewpoints'):
            after = sim.local_planner.context(robot)
        self.assertIn(sample, after['peer_coverage']['suppressed_ordinary_sample_cells'])
        self.assertFalse(any(sim.world.region_of(c) == region for c in after['samples']))
