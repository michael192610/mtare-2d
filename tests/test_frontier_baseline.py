"""Frontier-only baseline: task meaning, hierarchy, sharing and completion."""
import unittest

import numpy as np

from legacy_config import simple_config as Config
from mtare_2d.region_state import CellStatus as S, RegionUpdate
from mtare_2d.simple_frontier import FrontierCoveragePlanner, FrontierEvidence, FrontierRegionKnowledge
from mtare_2d.simulation import Simulation
from mtare_2d.world import FREE, UNKNOWN


class FrontierBaselineTests(unittest.TestCase):
    def sim(self,**kwargs):
        return Simulation(Config(robots=1,width=40,height=40,obstacles=0,solver_ms=10,**kwargs))

    def test_legacy_simple_is_frontier_only_with_no_ordinary_ledger(self):
        sim=self.sim();robot=sim.robots[0]
        self.assertEqual(sim.config.coverage_model,'simple')
        self.assertIsInstance(sim.local_planner,FrontierCoveragePlanner)
        self.assertIsInstance(robot.region_knowledge,FrontierRegionKnowledge)
        self.assertIsNone(robot.coverage)
        record=sim.local_planner.plan(robot,0)
        self.assertEqual(record['coverage_state'],{})
        self.assertFalse(record['ordinary_sample_cells'] or record['ordinary_queue'])
        self.assertEqual(len(record['trials']),1)
        self.assertTrue(record['frontier_sample_cells'])
        self.assertTrue(record['selected_viewpoints'])
        self.assertFalse(set(record['selected_viewpoints'])&set(record['visited_viewpoint_cells']))
        self.assertEqual(sim.config.viewpoint_spacing,2)
        self.assertEqual(sim.config.region_size,6)

    def test_frontier_is_known_free_next_to_unknown_not_hidden_truth(self):
        sim=self.sim();robot=sim.robots[0];known=robot.known.copy()
        first=sim.local_planner.plan(robot,0)
        for c in first['reachable_frontier_cells']:
            self.assertEqual(robot.known.ravel()[c],FREE)
            self.assertTrue(any(robot.known.ravel()[n]==UNKNOWN for n in sim.world.neighbors[c]))
        sim.world.truth[:]=1
        second=sim.local_planner.plan(robot,0)
        self.assertEqual(first['reachable_frontier_cells'],second['reachable_frontier_cells'])
        self.assertEqual(first['route'],second['route'])
        np.testing.assert_array_equal(known,robot.known)

    def test_fully_known_map_has_no_frontier_work_or_invented_coverage(self):
        sim=self.sim();robot=sim.robots[0];robot.known[:]=FREE
        sim.update_region_knowledge()
        local=sim.local_planner.plan(robot,0)
        self.assertFalse(local['frontier_sample_cells'] or local['selected_viewpoints'])
        self.assertTrue(local['local_coverage_complete'])
        self.assertIsNone(robot.coverage)
        self.assertTrue(all(robot.region_knowledge.regions[i].status==S.COVERED
                            for i in sim.world.regions.nearby(sim.world.cell_at(robot.position),2)))

    def test_region_is_active_from_frontier_even_without_selected_viewpoint(self):
        cfg=Config();knowledge=FrontierRegionKnowledge(0,1,cfg)
        ev=FrontierEvidence(reachable_cells=3,frontier_cells=(4,))
        knowledge.update_local({0},{0:ev})
        self.assertEqual(knowledge.regions[0].status,S.EXPLORING)
        knowledge.update_local({0},{0:FrontierEvidence(reachable_cells=3)})
        self.assertEqual(knowledge.regions[0].status,S.COVERED)

    def test_reopening_needs_frontier_work_not_old_point_count_guard(self):
        knowledge=FrontierRegionKnowledge(0,1,Config())
        for n in range(5):
            knowledge.update_local({0},{0:FrontierEvidence(reachable_cells=3,frontier_cells=(n,))})
            self.assertEqual(knowledge.regions[0].status,S.EXPLORING)
            knowledge.update_local({0},{0:FrontierEvidence(reachable_cells=3)})
            self.assertEqual(knowledge.regions[0].status,S.COVERED)
        self.assertGreater(knowledge.regions[0].exploring_count,3)
        self.assertFalse(knowledge.regions[0].almost_covered)

    def test_unknown_or_unreachable_region_is_not_declared_covered(self):
        knowledge=FrontierRegionKnowledge(0,2,Config())
        knowledge.update_local({0,1},{0:FrontierEvidence(),1:FrontierEvidence()})
        self.assertEqual([c.status for c in knowledge.regions],[S.UNSEEN,S.UNSEEN])

    def test_peer_completion_is_honored_until_newer_peer_frontier_report(self):
        knowledge=FrontierRegionKnowledge(0,1,Config())
        ev=FrontierEvidence(reachable_cells=3,frontier_cells=(4,))
        knowledge.update_local({0},{0:ev})
        knowledge.window=set()
        self.assertTrue(knowledge.apply_peer(RegionUpdate(0,10,2,0,int(S.COVERED)),1))
        knowledge.update_local({0},{0:ev})
        self.assertEqual(knowledge.regions[0].status,S.COVERED_BY_OTHERS)
        knowledge.update_local({0},{0:FrontierEvidence(reachable_cells=4,frontier_cells=(7,))})
        self.assertEqual(knowledge.regions[0].status,S.COVERED_BY_OTHERS)
        knowledge.window=set()
        self.assertTrue(knowledge.apply_peer(RegionUpdate(0,20,2,0,int(S.EXPLORING)),1))
        knowledge.update_local({0},{0:ev})
        self.assertEqual(knowledge.regions[0].status,S.EXPLORING)

    def test_newer_peer_frontier_can_reopen_completed_global_region(self):
        knowledge=FrontierRegionKnowledge(0,1,Config());knowledge.regions[0].status=S.COVERED
        self.assertTrue(knowledge.apply_peer(RegionUpdate(0,2,2,0,int(S.EXPLORING)),1))
        self.assertEqual(knowledge.regions[0].status,S.EXPLORING_BY_OTHERS)
        self.assertFalse(knowledge.apply_peer(RegionUpdate(0,1,2,0,int(S.COVERED)),1))

    def test_visited_neighborhood_and_prior_knowledge_are_kept_for_transit(self):
        sim=self.sim();robot=sim.robots[0];ctx=sim.local_planner.context(robot)
        self.assertGreater(len(ctx['visited_candidates']),1)
        self.assertTrue(set(ctx['visited_candidates'])<=set(ctx['candidates']))
        self.assertFalse(set(ctx['eligible_candidates'])&set(ctx['visited_candidates']))

    def test_logical_revisions_order_later_observations_above_older_updates(self):
        old=FrontierRegionKnowledge(0,1,Config(robots=3))
        newer=FrontierRegionKnowledge(1,1,Config(robots=3))
        old.cycle=10;old.set_status(0,S.EXPLORING,0,'test',True)
        newer.cycle=11;newer.set_status(0,S.COVERED,1,'test',True)
        self.assertGreater(newer.regions[0].update_id,old.regions[0].update_id)
        self.assertTrue(old.apply_peer(newer.updates({0})[0],1))
        self.assertEqual(old.regions[0].status,S.COVERED_BY_OTHERS)

    def test_simple_summaries_are_repeated_without_ack_suppression(self):
        k=FrontierRegionKnowledge(0,1,Config(robots=2))
        k.set_status(0,S.COVERED,0,'test',True)
        k.regions[0].known_by_robot_ids=3;k.regions[0].sync_count=999
        for _ in range(8):self.assertEqual(len(k.updates({0})),1)

    def test_actual_vrp_and_compact_exchange_still_run_for_three_robots(self):
        sim=Simulation(Config(robots=3,width=40,height=40,obstacles=0,solver_ms=10))
        try:
            for _ in range(12):sim.step()
            self.assertTrue(all(r.calls and r.local_calls and r.proposal for r in sim.robots))
            self.assertGreater(sim.event_counts['receive'],0)
            self.assertTrue(all(len(r.proposal.routes)==3 for r in sim.robots))
            self.assertGreater(sim.stats()['team_distance'],0)
            self.assertEqual(sim.event_counts['objective_verification_failure'],0)
            sim.publish()
            self.assertTrue(all(not packet.cells for _,_,packet in sim.queue))
        finally:sim.close()
