"""Behavioral checks for the 2D equivalent of covered-point/frontier bookkeeping."""
import unittest

import numpy as np

from legacy_config import sample_config as BaseConfig

def Config(**kwargs):
    """Explicit previous-model fixtures; new defaults are tested separately."""
    kwargs.setdefault("coverage_model","mtare")
    return BaseConfig(**kwargs)
from mtare_2d.coverage_bookkeeping import CoverageBookkeeping
from mtare_2d.coverage_planner import SampleCoveragePlanner
from mtare_2d.local_coverage import LocalCoveragePlanner
from mtare_2d.region_state import CellStatus
from mtare_2d.simulation import Simulation
from mtare_2d.world import FREE, UNKNOWN, World


class CoverageSampleTests(unittest.TestCase):
    def test_discovered_samples_and_actual_covered_samples_are_distinct_and_persistent(self):
        world=World(Config(robots=1,width=30,height=30,obstacles=0))
        known=np.full_like(world.truth,UNKNOWN)
        known[1,1]=known[1,4]=FREE  # (3,3) and (9,3), six metres apart.
        ledger=CoverageBookkeeping(world,0)
        ledger.observe(known,[(3,3)],(3,3))
        self.assertEqual(set(np.flatnonzero(ledger.observed)),{16,19})
        self.assertTrue(ledger.covered[16]);self.assertFalse(ledger.covered[19])
        ledger.observe(known,[(3,3),(9,3)],(9,3))
        self.assertTrue(ledger.covered[19])
        ledger.observe(known,[(3,3),(9,3),(25,25)],(25,25))
        self.assertTrue(ledger.covered[16] and ledger.covered[19])

    def test_history_can_cover_later_registered_samples_but_not_unobserved_truth(self):
        world=World(Config(robots=1,width=30,height=30,obstacles=0))
        ledger=CoverageBookkeeping(world,0)
        known=np.full_like(world.truth,UNKNOWN)
        ledger.observe(known,[(3,3)],(3,3))
        self.assertFalse(ledger.covered.any())
        known[1,1]=FREE
        ledger.observe(known,[(3,3),(25,25)],(25,25))
        self.assertTrue(ledger.covered[16])
        world.truth[:]=1
        before=ledger.snapshot()
        ledger.observe(known,[(3,3),(25,25)],(25,25))
        self.assertEqual(before,ledger.snapshot())

    def test_planning_masks_do_not_commit_coverage_or_consult_hidden_map(self):
        sim=Simulation(Config(robots=1,width=40,height=40,obstacles=0))
        robot=sim.robots[0]
        first=sim.local_planner.plan(robot,0)
        covered=robot.coverage.covered.copy()
        # Visited-neighbor coverage may cover the entire initial discovery disk.
        self.assertTrue(first['frontier_sample_cells'])
        self.assertTrue(first['selected_viewpoints'])
        sim.world.truth[:]=1
        second=sim.local_planner.plan(robot,0)
        np.testing.assert_array_equal(covered,robot.coverage.covered)
        self.assertEqual(first['route'],second['route'])
        self.assertFalse(set(first['ordinary_sample_cells']) & set(first['frontier_sample_cells']))
        self.assertTrue(all(robot.known.ravel()[c]==FREE for c in first['ordinary_sample_cells']))
        self.assertTrue(all(robot.known.ravel()[c]==UNKNOWN for c in first['frontier_sample_cells']))

    def test_viewpoint_scores_and_region_evidence_use_separate_pools(self):
        sim=Simulation(Config(robots=1,width=40,height=40,obstacles=0))
        robot=sim.robots[0];planner=sim.local_planner;cfg=sim.config
        context=planner.context(robot)
        window,evidence=planner.evidence(robot)
        for region in window:
            points=[c for c in context['candidates'] if sim.world.region_of(c)==region and c not in context.get('visited',robot.visited)]
            self.assertEqual(evidence[region].above_small,sum(len(context['coverage'][c])*context.get('score_weight',1)>=context.get('normal_small',cfg.region_small_gain) for c in points))
            self.assertEqual(evidence[region].above_frontier,sum(len(context['frontier_coverage'][c])>=context['frontier_evidence_min']
                if 'frontier_evidence_min' in context else
                len(context['frontier_coverage'][c])>cfg.region_frontier_gain for c in points))
        self.assertTrue(any(len(context['coverage'][c])!=len(context['frontier_coverage'][c]) for c in context['candidates']))

    def test_greedy_uses_marginal_gain_and_live_queue_bounds(self):
        world=World(Config(robots=1,width=30,height=30,obstacles=0,local_greedy_sample_range=1))
        planner=SampleCoveragePlanner(world)
        selected,mask,steps=planner.greedy([1,2,3],{1:{10,11},2:{10,11},3:{12}},set(),1,np.random.default_rng(0))
        self.assertEqual(selected,[1,3])
        self.assertEqual([s['gain'] for s in steps],[2,1])
        self.assertEqual(mask,{10,11,12})

    def test_reuse_navigation_phase_order_and_completion(self):
        sim=Simulation(Config(robots=1,width=40,height=40,obstacles=0))
        robot=sim.robots[0]
        first=sim.local_planner.plan(robot,0)
        robot.local_viewpoints=first['selected_viewpoints'].copy()
        reused=sim.local_planner.plan(robot,1)
        self.assertTrue(reused['reused_viewpoints'])
        pre=set(reused['reused_viewpoints'])|set(reused['navigation_viewpoints'])
        self.assertFalse(pre & set(reused['ordinary_queue']))
        self.assertFalse(set(reused['ordinary_queue']) & set(reused['frontier_queue']))
        for trial in reused['trials']:
            self.assertFalse(set(trial['ordinary_selected']) & set(trial['frontier_selected']))
            self.assertTrue(set(trial['selected_viewpoints']) <= set(reused['eligible_viewpoint_cells']))
        robot.known[:]=FREE
        # With actual coverage everywhere, neither queue invents missing work.
        robot.coverage.covered[:]=True
        robot.local_viewpoints=[]
        done=sim.local_planner.plan(robot,2)
        self.assertTrue(done['local_coverage_complete'])
        self.assertFalse(done['selected_viewpoints'])

    def test_covered_peer_regions_remain_transit_but_not_coverage_candidates(self):
        sim=Simulation(Config(robots=1,width=40,height=40,obstacles=0))
        robot=sim.robots[0]
        for region in robot.region_knowledge.regions:region.status=CellStatus.COVERED_BY_OTHERS
        record=sim.local_planner.plan(robot,0)
        self.assertTrue(record['local_candidate_cells'])
        self.assertFalse(record['eligible_viewpoint_cells'])
        self.assertTrue(record['local_coverage_complete'])

    def test_frontier_reference_model_is_preserved_and_radii_validate(self):
        sim=Simulation(Config(robots=1,coverage_model='frontier'))
        self.assertIs(type(sim.local_planner),LocalCoveragePlanner)
        self.assertIsNone(sim.robots[0].coverage)
        self.assertEqual(Config().effective_coverage_radius,5)
        self.assertEqual(Config(reveal_radius=6).effective_coverage_radius,4)
        for radius in (0,-1,8):
            with self.assertRaises(ValueError):Config(coverage_radius=radius)

    def test_original_strict_frontier_gate_and_sample_scores_change_region_state(self):
        sim=Simulation(Config(robots=1,width=40,height=40,obstacles=0))
        robot=sim.robots[0]
        robot.known[:]=FREE  # Broad observed evidence beyond prior visit footprints.
        window,evidence=sim.local_planner.evidence(robot)
        region=next(i for i,ev in evidence.items() if ev.above_big>sim.config.region_covered_to_exploring_count)
        cell=robot.region_knowledge.regions[region]
        cell.status=CellStatus.COVERED
        robot.region_knowledge.update_local(window,evidence)
        self.assertEqual(cell.status,CellStatus.EXPLORING)
        record=sim.local_planner.plan(robot,0)
        front=record['viewpoint_frontier_coverage']
        pre=set(record['reused_viewpoints'])|set(record['navigation_viewpoints'])
        mask=set().union(*(set(front.get(str(c),[])) for c in pre))
        queue=record['frontier_queue']
        threshold=record['score_thresholds']['frontier_min']
        expected=bool(queue and (max(len(set(front[str(c)])-mask) for c in queue)>=record['score_thresholds']['frontier_gate_min']
            if 'frontier_gate_min' in record['score_thresholds'] else
            max(len(set(front[str(c)])-mask) for c in queue)>threshold))
        self.assertEqual(record['frontier_gate'],expected)

    def test_new_bookkeeping_keeps_planner_work_completion_below_full_observation(self):
        sim=Simulation(Config(robots=3,width=40,height=40,obstacles=0,completion_startup_cycles=0))
        for robot in sim.robots:
            for i in range(len(robot.region_knowledge.regions)):
                robot.region_knowledge.set_status(i,CellStatus.COVERED_BY_OTHERS,
                                                  robot.robot_id,'completion_fixture',True)
        before=sim.stats()['explored_ratio']
        self.assertLess(before,1)
        try:
            while not sim.done and sim.tick<20:sim.step()
            self.assertTrue(sim.done)
            self.assertEqual(sim.reason,'no_remaining_exploration_work')
            self.assertEqual(sim.stats()['explored_ratio'],before)
            self.assertEqual(sim.stats()['team_distance'],0)
        finally:
            sim.close()


if __name__=='__main__':unittest.main()
