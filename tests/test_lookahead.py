"""Source-guided route-anchor, direction and local-priority regressions."""
import math
import os
from pathlib import Path
import random
import shutil
import subprocess
import tempfile
import unittest

from mtare_2d.config import Config
from mtare_2d.lookahead import choose_forward, order_anchored, select_lookahead, visible_segment
from mtare_2d.simulation import Simulation
from mtare_2d.world import FREE, UNKNOWN


class LookaheadTests(unittest.TestCase):
    def sim(self,**kwargs):
        return Simulation(Config(robots=1,width=30,height=30,obstacles=0,solver_ms=10,**kwargs))

    def test_both_original_dummy_constraints_are_satisfied(self):
        sim=self.sim();robot=sim.robots[0];robot.known[:]=FREE
        ctx=sim.local_planner.context(robot);graph=ctx['graph'];start=ctx['start']
        others=[c for c in ctx['candidates'] if c!=start]
        previous=others[len(others)//2]
        a,b=others[0],others[-1]
        route=order_anchored(sim.world,graph,[a,start,previous,b],others[::50])
        ordered=route['ordered_cells']
        self.assertEqual((ordered[0],ordered[-1]),(a,b))
        self.assertEqual(abs(ordered.index(start)-ordered.index(previous)),1)
        matrix=route['solver_distance_matrix'];tour=route['solver_tour']
        cost=sum(matrix[x][y] for x,y in zip(tour,[*tour[1:],tour[0]]))
        self.assertEqual(cost,route['objective'])
        self.assertAlmostEqual(route['length'],(len(route['full_path'])-1)*.6)

    def test_closed_loop_preserves_robot_lookahead_connection(self):
        sim=self.sim();robot=sim.robots[0];robot.known[:]=FREE
        ctx=sim.local_planner.context(robot);start=ctx['start']
        others=[c for c in ctx['candidates'] if c!=start]
        previous=others[5]
        route=order_anchored(sim.world,ctx['graph'],[start,start,previous,start],others[::100])
        self.assertEqual(route['ordered_cells'][0],start)
        self.assertEqual(route['ordered_cells'][-1],start)
        self.assertIn(previous,(route['ordered_cells'][1],route['ordered_cells'][-2]))

    def test_previous_point_survives_global_replanning_as_a_navigation_anchor(self):
        sim=self.sim()
        try:
            sim.step();robot=sim.robots[0]
            self.assertTrue(robot.lookahead_valid)
            previous=robot.lookahead_point
            old_direction=robot.lookahead_direction
            sim.plan(robot)  # Clears the execution path, not the lookahead state.
            self.assertEqual(robot.lookahead_point,previous)
            record=sim.plan_local(robot)
            anchor=record['route']['anchors'][2]
            self.assertIn(anchor,record['navigation_viewpoints'])
            self.assertEqual(record['lookahead']['previous_point'],previous)
            self.assertEqual(record['lookahead']['previous_direction'],list(old_direction))
        finally:
            sim.close()

    def test_priority_changes_the_lookahead_goal_without_blocking_transit(self):
        sim=self.sim(lookahead_distance=.6);robot=sim.robots[0];world=sim.world;robot.known[:]=FREE
        lattice=world.viewpoints
        def node(x,y):return lattice.offset+lattice.nearest_index((x,y))
        r=node(5.7,10.5);e=node(6.3,10.5);ne=node(6.3,11.1)
        n=node(5.7,11.1);nw=node(5.1,11.1);w=node(5.1,10.5)
        robot.position=world.xy(r);robot.lookahead_direction=(1.,0.)
        route=dict(full_path=[r,e,ne,n,nw,w,r],anchors=[r,r,r,r])
        without=select_lookahead(world,robot,route,{e,w},{})
        with_priority=select_lookahead(world,robot,route,{e,w},{world.region_of(e):False})
        self.assertEqual(without['target'],e)
        self.assertNotEqual(world.region_of(with_priority['target']),world.region_of(e))
        self.assertIn(e,with_priority['forward']['skipped_nodes'])
        self.assertTrue(visible_segment(world,robot.known,robot.position,world.xy(e)))

    def test_lower_id_priority_rule_matches_original(self):
        sim=Simulation(Config(robots=3,width=30,height=30,obstacles=0,planning_mode='sequential'))
        knowledge=sim.robots[1].region_knowledge
        knowledge.window={0,1}
        knowledge.update_priority({0:[0],2:[1]})
        self.assertFalse(knowledge.regions[0].has_exploring_priority)
        self.assertTrue(knowledge.regions[1].has_exploring_priority)

    def test_visibility_uses_private_map_and_checks_intermediate_cells(self):
        sim=self.sim();robot=sim.robots[0];robot.known[:]=FREE
        robot.known[2,3]=UNKNOWN
        self.assertFalse(visible_segment(sim.world,robot.known,(3,5),(11,5)))
        self.assertTrue(visible_segment(sim.world,robot.known,(3,3),(11,3)))
        sim.world.truth[:]=1
        self.assertTrue(visible_segment(sim.world,robot.known,(3,3),(11,3)))

    def test_no_priority_goal_holds_position_with_finite_direction(self):
        sim=self.sim();robot=sim.robots[0];robot.known[:]=FREE;world=sim.world
        ctx=sim.local_planner.context(robot);start=ctx['start']
        far=max(ctx['candidates'],key=lambda c:math.dist(world.xy(c),robot.position))
        path=[start,*ctx['graph'].path(start,far),*ctx['graph'].path(far,start)]
        route=dict(full_path=path,anchors=[start,start,start,start])
        priority={r:False for r in ctx['window']}
        robot.position=world.xy(start)
        look=select_lookahead(world,robot,route,{far},priority)
        self.assertEqual(look['target'],start)
        self.assertFalse(look['valid'])
        self.assertTrue(all(math.isfinite(v) for v in look['direction']))

    def test_invalid_lookahead_distance_is_rejected(self):
        with self.assertRaises(ValueError):
            Config(lookahead_distance=0)

    def test_passing_a_selected_coordinate_as_a_via_node_does_not_double_count_it(self):
        sim=self.sim(lookahead_distance=.6);robot=sim.robots[0];world=sim.world
        robot.known[:]=FREE;lattice=world.viewpoints
        node=lambda x:lattice.offset+lattice.nearest_index((x,10.5))
        r,e,q,v=map(node,(5.7,6.3,6.9,7.5))
        robot.position=world.xy(r)
        route=dict(full_path=[r,e,q,v,q,e,r],
                   full_path_is_anchor=[True,False,False,True,True,False,True],
                   anchors=[r,r,r,r])
        look=select_lookahead(world,robot,route,{q},{})
        self.assertEqual(look['viewpoint_counts'],[1,1])

    def test_robot_anchor_is_used_instead_of_an_earlier_via_coordinate(self):
        sim=self.sim(lookahead_distance=.6);robot=sim.robots[0];world=sim.world
        robot.known[:]=FREE;lattice=world.viewpoints
        node=lambda x:lattice.offset+lattice.nearest_index((x,10.5))
        a,r,p,b=map(node,(5.1,5.7,6.3,6.9))
        robot.position=world.xy(r)
        path=[a,r,p,r,p,b]
        route=dict(full_path=path,full_path_is_anchor=[True,False,True,True,True,True],
                   anchors=[a,r,p,b])
        look=select_lookahead(world,robot,route,{p},{})
        expected=path[4:] if look['follow_forward'] else list(reversed(path[:3]))
        self.assertEqual(look['follow_path'],expected)
        self.assertAlmostEqual(look['distances_from_boundaries'][0],1.8)


class OriginalDirectionReferenceTests(unittest.TestCase):
    def test_direction_policy_matches_compiled_original_code(self):
        root=Path(os.environ.get('MTARE_ORIGINAL_ROOT',
            str(Path(__file__).resolve().parents[1]/'.references/mtare_planner')))
        path=root/'src/tare_planner/src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp'
        if not path.exists() or not shutil.which('g++'):
            self.skipTest('Original M-TARE source and g++ required')
        source=path.read_text()
        start=source.index('  if (relocation_)',source.index('  double forward_angle_score'))
        end=source.index('  if ((lookahead_point == forward_lookahead_point',start)
        block=source[start:end]
        cpp=r'''
#include <iostream>
#include <vector>
namespace exploration_path_ns { enum class NodeType { OTHER,ROBOT }; }
struct Node { exploration_path_ns::NodeType type_; };
int main() {
 double forward_angle_score,backward_angle_score,dist_from_start,dist_from_end;
 int forward_viewpoint_count,backward_viewpoint_count,start_robot,end_robot;
 while(std::cin>>forward_angle_score>>backward_angle_score>>forward_viewpoint_count
       >>backward_viewpoint_count>>dist_from_start>>dist_from_end>>start_robot>>end_robot) {
  struct {bool kUseMomentum=false;} pp_;
  bool relocation_=forward_viewpoint_count==0 && backward_viewpoint_count==0;
  bool use_momentum_=false,follow_local_path_from_start=false;
  int forward_lookahead_point=1,backward_lookahead_point=-1,lookahead_point=0;
  struct {std::vector<Node> nodes_;} local_path;
  local_path.nodes_={{start_robot?exploration_path_ns::NodeType::ROBOT:exploration_path_ns::NodeType::OTHER},
                    {end_robot?exploration_path_ns::NodeType::ROBOT:exploration_path_ns::NodeType::OTHER}};
'''+block+r'''
  std::cout<<follow_local_path_from_start<<"\n";
 }
}
'''
        rng=random.Random(755)
        cases=[(rng.choice([-2,-1,0,1]),rng.choice([-2,-1,0,1]),
                rng.randrange(3),rng.randrange(3),rng.randrange(4),rng.randrange(4),
                bool(rng.randrange(2)),bool(rng.randrange(2))) for _ in range(1200)]
        with tempfile.TemporaryDirectory(prefix='mtare-lookahead-reference-') as tmp:
            src=Path(tmp)/'reference.cpp';binary=Path(tmp)/'reference';src.write_text(cpp)
            subprocess.run(['g++','-std=c++17','-O2',str(src),'-o',str(binary)],
                           check=True,capture_output=True,text=True)
            result=subprocess.run([str(binary)],input='\n'.join(
                ' '.join(str(int(x)) for x in c) for c in cases)+'\n',
                check=True,capture_output=True,text=True)
        for case,value in zip(cases,result.stdout.splitlines()):
            self.assertEqual(choose_forward(*case),bool(int(value)),case)
