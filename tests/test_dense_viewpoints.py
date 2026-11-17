"""Source-fidelity checks for separate .6 m viewpoints and visited handling."""
import json
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

from legacy_config import sample_config as Config
from mtare_2d.dense_coverage import DenseCoveragePlanner
from mtare_2d.recording import Journal
from mtare_2d.region_state import CellStatus, Evidence
from mtare_2d.results_viewer import SavedRun, create_result_viewer
from mtare_2d.simulation import Simulation
from mtare_2d.world import FREE, UNKNOWN


class DenseViewpointTests(unittest.TestCase):
    def sim(self,**kwargs):
        # Keep checks of the previously supported .6 m geometry as option tests.
        return Simulation(Config(robots=1,obstacles=0,solver_ms=10,
                                 viewpoint_spacing=.6,region_covered_to_exploring_count=10,coverage_model='mtare',**kwargs))

    def test_optional_fine_spacing_and_region_counts_match_original_indoor(self):
        sim=self.sim();world=sim.world;vp=world.viewpoints
        self.assertIsInstance(sim.local_planner,DenseCoveragePlanner)
        self.assertEqual(world.truth.shape,(50,50))
        self.assertEqual(sim.config.viewpoint_spacing,.6)
        self.assertEqual(sim.config.visited_viewpoint_radius,3)
        self.assertEqual(sim.config.region_covered_to_exploring_count,10)
        self.assertEqual(sim.config.region_small_gain,1)
        self.assertEqual(sim.config.region_big_gain,2)
        self.assertEqual(sim.config.local_frontier_min_gain,1)
        self.assertAlmostEqual(vp.centres[1,0]-vp.centres[0,0],.6)
        self.assertTrue(np.all(vp.centres[:,0]<100) and np.all(vp.centres[:,1]<100))
        robot=sim.robots[0];robot.position=(50.1,50.1);robot.trajectory=[robot.position];robot.known[:]=FREE
        ctx=sim.local_planner.context(robot)
        self.assertEqual(len(ctx['candidates']),2500)  # 50 x 50 local viewpoints.
        self.assertTrue(all(world.viewpoints.contains(c) for c in ctx['candidates']))
        self.assertTrue(all(world.xy(c)[0]>=36 and world.xy(c)[0]<66 for c in ctx['candidates']))
        self.assertTrue(all(c>=world.size+world.regions.size for c in ctx['candidates']))

    def test_snapped_history_marks_euclidean_3m_neighbors(self):
        sim=self.sim();vp=sim.world.viewpoints
        pose=(50.1,50.1);index=vp.nearest_index(pose);region=int(vp.region_ids[index])
        mask=vp.mask_from_poses([pose],{region})
        self.assertIn(vp.offset+index+5,mask)  # exactly 3 m.
        self.assertNotIn(vp.offset+index+6,mask)
        self.assertIn(vp.offset+index+3+3*vp.cols,mask)
        self.assertNotIn(vp.offset+index+4+4*vp.cols,mask)
        self.assertEqual(vp.mask_from_poses([pose],{0}),set())

    def test_nearby_visited_viewpoints_remain_transit_but_are_not_selected(self):
        sim=self.sim();robot=sim.robots[0];planner=sim.local_planner
        ctx=planner.context(robot);record=planner.plan(robot,0)
        visited=set(record['visited_viewpoint_cells'])
        self.assertGreater(len(visited),1)
        self.assertTrue(all(not record['viewpoint_coverage'][str(c)] for c in visited))
        self.assertTrue(all(not record['viewpoint_frontier_coverage'][str(c)] for c in visited))
        self.assertFalse(visited&set(record['selected_viewpoints']))
        self.assertTrue(visited<=set(ctx['candidates']))
        self.assertIn(ctx['start'],visited)

    def test_region_covered_by_other_marks_visited_and_active_peer_resets_it(self):
        sim=self.sim();robot=sim.robots[0];world=sim.world
        robot.known[:]=FREE
        ctx=sim.local_planner.context(robot)
        far=next(c for c in ctx['candidates'] if c not in robot.visited_viewpoints
                 and math.dist(world.xy(c),robot.position)>8)
        region=world.region_of(far)
        robot.region_knowledge.regions[region].status=CellStatus.COVERED_BY_OTHERS
        sim.local_planner.context(robot)
        self.assertIn(far,robot.visited_viewpoints)
        robot.region_knowledge.regions[region].status=CellStatus.EXPLORING_BY_OTHERS
        sim.local_planner.context(robot)
        self.assertNotIn(far,robot.visited_viewpoints)

    def test_visited_candidate_coverage_is_persistent_without_revealing_truth(self):
        sim=self.sim();robot=sim.robots[0]
        ctx=sim.local_planner.context(robot)
        before=robot.coverage.covered.copy();known=robot.known.copy()
        sim.local_planner.plan(robot,0)
        np.testing.assert_array_equal(before,robot.coverage.covered)
        np.testing.assert_array_equal(known,robot.known)
        self.assertTrue(np.all(~robot.coverage.covered | robot.coverage.observed))
        sim.world.truth[:]=1
        second=sim.local_planner.context(robot)
        self.assertEqual(ctx['candidates'],second['candidates'])
        self.assertEqual(ctx['coverage'],second['coverage'])

    def test_simple_counts_select_single_sample_and_log_integer_gains(self):
        sim=self.sim()
        chosen,_,steps=sim.local_planner.greedy([1],{1:{9}},set(),1,np.random.default_rng(0))
        self.assertEqual(chosen,[1]);self.assertEqual(steps[0]['gain'],1)
        self.assertIs(type(steps[0]['gain']),int)
        record=sim.local_planner.plan(sim.robots[0],0)
        self.assertEqual(record['score_thresholds']['ordinary_min'],1)
        self.assertEqual(record['score_thresholds']['ordinary_big'],2)
        self.assertEqual(record['score_thresholds']['frontier_evidence_min'],1)
        self.assertEqual(record['score_thresholds']['frontier_gate_min'],1)
        self.assertNotIn('sample_weight',record['score_thresholds'])
        self.assertFalse(any(k.startswith('source_') for k in sim.config.to_dict()))

    def test_reopening_requires_more_than_10_useful_viewpoints(self):
        sim=self.sim();knowledge=sim.robots[0].region_knowledge
        cell=knowledge.regions[0];cell.status=CellStatus.COVERED
        knowledge.update_local({0},{0:Evidence(candidates=11,above_big=10)})
        self.assertEqual(cell.status,CellStatus.COVERED)
        knowledge.update_local({0},{0:Evidence(candidates=11,above_big=11)})
        self.assertEqual(cell.status,CellStatus.EXPLORING)

    def test_trajectory_uses_dense_points_without_changing_cycle_travel_allowance(self):
        sim=self.sim(width=40,height=40)
        try:
            for _ in range(6):sim.step()
            robot=sim.robots[0]
            self.assertGreater(robot.distance,10)
            self.assertLessEqual(robot.distance,12+1e-8)
            self.assertTrue(all(abs((x/.6-.5)-round(x/.6-.5))<1e-7 and
                                abs((y/.6-.5)-round(y/.6-.5))<1e-7
                                for x,y in robot.trajectory[1:]))
            self.assertGreater(len(robot.trajectory),sim.tick)
        finally:sim.close()

    def test_saved_result_resolves_viewpoint_ids_and_switches_robot_views(self):
        import matplotlib
        matplotlib.use('Agg')
        sim=self.sim(width=40,height=40)
        with tempfile.TemporaryDirectory() as folder:
            journal=Journal(Path(folder),sim.config);sim.journal=journal;journal.initialize(sim)
            try:
                sim.step();sim.reason='test_stop';journal.finish(sim,'stopped')
                saved=SavedRun(journal.directory)
                for c in sim.robots[0].path:
                    np.testing.assert_allclose(sim.world.xy(c),saved.world.xy(c))
                viewer=create_result_viewer(journal.directory)
                viewer.key(type('Key',(),{'key':'a'})());viewer.key(type('Key',(),{'key':'0'})())
                self.assertEqual(viewer.view_robot,0)
                viewer.plt.close(viewer.figure)
            finally:
                sim.close()
                if not journal.finished:journal.finish(sim,'stopped')

    def test_invalid_geometry_and_thresholds_are_rejected(self):
        for values in ({'viewpoint_spacing':0},{'visited_viewpoint_radius':-1},
                       {'local_frontier_min_gain':0}):
            with self.assertRaises(ValueError):Config(**values)
