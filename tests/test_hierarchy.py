"""Step 1: region allocation and local viewpoint coverage are distinct planners."""
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from legacy_config import simple_config as Config
from mtare_2d.local_coverage import order_viewpoints
from mtare_2d.recording import Journal
from mtare_2d.regions import select_global_regions
from mtare_2d.results_viewer import SavedRun
from mtare_2d.simulation import Simulation
from mtare_2d.world import BLOCKED, FREE, UNKNOWN, GridGraph, World


class HierarchyTests(unittest.TestCase):
    def test_global_regions_exclude_local_window_and_have_stable_distinct_keys(self):
        world = World(Config(robots=1,width=40,height=40,obstacles=0))
        known = np.full_like(world.truth,UNKNOWN)
        known[:,:8] = FREE
        current = 21
        result = select_global_regions(world,known,[current],current)
        self.assertTrue(result['targets'])
        self.assertFalse(set(result['global_region_ids']) & set(result['local_window_region_ids']))
        self.assertEqual(len(result['targets']),len(set(result['targets'])))
        for task,region,cell in zip(result['targets'],result['global_region_ids'],result['navigation_cells']):
            self.assertGreaterEqual(task,world.size)
            self.assertEqual(world.regions.region(task),region)
            self.assertEqual(world.regions.cell_region[cell],region)
            self.assertEqual(known.ravel()[cell],FREE)
        # Changing unseen truth does not change region eligibility or gateways.
        world.truth[:] = BLOCKED
        self.assertEqual(result,select_global_regions(world,known,[current],current))

    def test_local_tsp_orders_viewpoints_with_real_obstacle_avoiding_paths(self):
        world = World(Config(robots=1,width=12,height=12,obstacles=0))
        known = np.full_like(world.truth,FREE)
        known[0,1] = BLOCKED
        graph = GridGraph(world,known)
        route = order_viewpoints(world,graph,0,0,[2,8])
        self.assertEqual(route['ordered_cells'][0],0)
        self.assertEqual(route['ordered_cells'][-1],0)
        self.assertTrue({2,8} <= set(route['ordered_cells']))
        self.assertNotIn(1,route['execution_path'])
        navigation_only = order_viewpoints(world,graph,0,2,[])
        self.assertEqual(navigation_only['objective'],80)
        self.assertEqual(navigation_only['length'],8)
        self.assertEqual(route['objective'],round(len(route['execution_path'])*world.config.cell_size*10))
        left = 0
        for right in route['execution_path']:
            self.assertIn(right,world.neighbors[left])
            self.assertEqual(known.ravel()[right],FREE)
            left = right

    def test_local_coverage_reuses_useful_viewpoints_and_cannot_read_truth(self):
        sim = Simulation(Config(robots=1,width=40,height=40,obstacles=0))
        robot = sim.robots[0]
        first = sim.local_planner.plan(robot,0)
        self.assertTrue(first['selected_viewpoints'])
        self.assertTrue(first['route']['execution_path'])
        self.assertEqual(len(first['trials']),10 if first['ordinary_queue'] else 1)
        self.assertTrue(all(robot.known.ravel()[sim.world.cell_at(sim.world.xy(c))] == FREE for c in first['local_candidate_cells']))
        sim.world.truth[:] = BLOCKED
        second = sim.local_planner.plan(robot,0)
        self.assertEqual(first['route'],second['route'])
        robot.local_viewpoints = first['selected_viewpoints'].copy()
        reused = sim.local_planner.plan(robot,1)
        self.assertTrue(reused['reused_viewpoints'])
        self.assertTrue(set(reused['reused_viewpoints']) <= set(first['selected_viewpoints']))

    def test_hierarchical_exploration_records_both_levels_and_reopens_saved_state(self):
        cfg = Config(region_state_model="frontier",width=40,height=40,obstacles=0,robots=3,solver_ms=10,max_cycles=250,
                     local_optimization_iterations=3)
        with tempfile.TemporaryDirectory() as folder:
            journal = Journal(Path(folder),cfg)
            sim = Simulation(cfg,journal)
            journal.initialize(sim)
            try:
                while not sim.done:
                    sim.step()
                journal.finish(sim,'finished' if sim.stats()['explored_ratio']==1 else 'stopped')
                self.assertEqual(sim.stats()['explored_ratio'],1.0)
                self.assertGreater(sim.stats()['local_planning_calls'],0)
                calls=[json.loads(p.read_text()) for p in (journal.directory/'calls').glob('*.json')]
                self.assertTrue(any(c['target_cell_ids'] for c in calls))
                for call in calls:
                    self.assertTrue(all(key >= sim.world.size for key in call['target_cell_ids']))
                    self.assertTrue(all(kind=='region' for kind in call['node_kinds'][3:]))
                    self.assertFalse(set(call['hierarchy']['global_region_ids']) & set(call['hierarchy']['local_window_region_ids']))
                local=list((journal.directory/'local_calls').glob('*.json'))
                self.assertEqual(len(local),sim.stats()['local_planning_calls'])
                restored=SavedRun(journal.directory)
                for original,loaded in zip(sim.robots,restored.robots):
                    self.assertEqual(original.local_viewpoints,loaded.local_viewpoints)
                    self.assertEqual(original.path,loaded.path)
                    self.assertEqual(original.selected_tasks,loaded.selected_tasks)
            finally:
                sim.close()
                if not journal.finished:
                    journal.finish(sim,'stopped')
