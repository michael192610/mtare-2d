"""Count-only scores preserve the former default weighted selection decisions."""
import unittest
from unittest.mock import patch

import numpy as np

from legacy_config import simple_config as Config
from mtare_2d.simulation import Simulation


class SimpleCountTests(unittest.TestCase):
    def test_simple_defaults_keep_visited_neighborhood_with_two_metre_viewpoints(self):
        sim=Simulation(Config(robots=1,width=40,height=40,obstacles=0))
        self.assertEqual(sim.config.viewpoint_spacing,2)
        self.assertEqual(sim.config.visited_viewpoint_radius,3)
        self.assertEqual(sim.config.region_covered_to_exploring_count,1)
        ctx=sim.local_planner.context(sim.robots[0])
        visited=set(ctx['candidates'])&ctx['visited']
        self.assertGreater(len(visited),1)  # Not a return to exact-cell-only suppression.
        per_region={}
        for c in ctx['candidates']:
            reg=sim.world.region_of(c);per_region[reg]=per_region.get(reg,0)+1
        self.assertLessEqual(max(per_region.values()),9)


    def test_default_count_boundaries_equal_former_weighted_comparisons(self):
        old_weight=4/.09  # Historical reference only; runtime no longer uses it.
        for n in range(101):
            self.assertEqual(n*old_weight>=40,n>=1)
            self.assertEqual(n*old_weight>=60,n>=2)
            self.assertEqual(n*old_weight>=20,n>=1)
            self.assertEqual(n*old_weight>20,n>=1)

    def test_single_frontier_sample_opens_gate_and_is_selected(self):
        sim=Simulation(Config(robots=1,width=40,height=40,obstacles=0))
        robot=sim.robots[0];planner=sim.local_planner;ctx=planner.context(robot)
        candidate=next(c for c in ctx['candidates'] if c not in ctx['visited'] and ctx['frontier_coverage'][c])
        sample=next(iter(ctx['frontier_coverage'][candidate]))
        ctx['coverage']={c:set() for c in ctx['candidates']}
        ctx['frontier_coverage']={c:({sample} if c==candidate else set()) for c in ctx['candidates']}
        ctx['samples']=set();ctx['frontier_samples']={sample}
        with patch.object(planner,'context',return_value=ctx):record=planner.plan(robot,0)
        self.assertTrue(record['frontier_gate'])
        self.assertIn(candidate,record['selected_viewpoints'])
        self.assertEqual(record['trials'][0]['frontier_selection_steps'],[{'viewpoint':candidate,'gain':1}])

    def test_count_greedy_matches_historical_weighted_greedy(self):
        sim=Simulation(Config(robots=1,width=40,height=40,obstacles=0))
        lists={1:{1,2},2:{2,3},3:{4},4:{1,2},5:set()}
        def weighted_reference(seed):
            rng=np.random.default_rng(seed);remaining=set(lists);pred=set();selected=[]
            while remaining:
                ranked=sorted(((len(lists[c]-pred)*(4/.09),c) for c in remaining),key=lambda p:(-p[0],p[1]))
                qualifying=[p for p in ranked if p[0]>=40]
                if not qualifying:break
                c=qualifying[int(rng.integers(min(3,len(qualifying))))][1]
                selected.append(c);pred|=lists[c];remaining.remove(c)
            return selected,pred
        for seed in range(20):
            selected,mask,_=sim.local_planner.greedy(list(lists),lists,set(),1,np.random.default_rng(seed))
            self.assertEqual((selected,mask),weighted_reference(seed))
