import math
import unittest

import numpy as np

from legacy_config import simple_config as Config
from mtare_2d.region_state import RegionKnowledge, RegionUpdate, CellStatus as S
from mtare_2d.roadmap import INF, Roadmap, RoadmapEdge, roadmap_matrices
from mtare_2d.simulation import Message, Simulation
from mtare_2d.world import FREE, UNKNOWN, World


class RoadmapProtocolTests(unittest.TestCase):
    def test_saved_cache_retains_newer_forwarded_position_after_older_direct_packet(self):
        from pathlib import Path
        import tempfile
        from mtare_2d.recording import Journal
        from mtare_2d.results_viewer import SavedRun
        with tempfile.TemporaryDirectory() as folder:
            config=Config(robots=3)
            journal=Journal(Path(folder),config)
            sim=Simulation(config,journal);journal.initialize(sim);sim.tick=20
            sim.receive(1,Message(0,0,20,sim.robots[0].position,(),None,
                robot_positions=(sim.robots[0].position,sim.robots[1].position,(40,40)),
                robot_update_ticks=(20,18,19)))
            sim.receive(1,Message(2,0,18,(38,40),(),None))
            self.assertEqual(sim.robots[1].peers[2].position,(40,40))
            journal.finish(sim,'stopped')
            restored=SavedRun(journal.directory)
            self.assertEqual(restored.robots[1].peers[2].position,(40,40))

    def test_cost_branches_use_centres_truncation_and_do_not_read_truth(self):
        world = World(Config(robots=1, width=30, height=30, obstacles=0))
        graph = Roadmap(world)
        self.assertEqual(graph.cost(0,6), (8,'straight_line_fallback'))  # sqrt(72), truncated
        graph.add_edge(0,1)
        graph.add_edge(1,6)
        self.assertEqual(graph.cost(0,6), (12,'roadmap'))
        graph.add_edge(20,21)
        self.assertEqual(graph.cost(0,20),(INF,'roadmap_no_path'))
        world.truth[:]=1
        self.assertEqual(graph.cost(0,6),(12,'roadmap'))

    def test_per_vehicle_masks_task_offsets_diagonal_and_symmetry(self):
        world=World(Config(robots=3,width=30,height=30,obstacles=0))
        graph=Roadmap(world)
        graph.add_edge(0,1); graph.add_edge(1,2)
        knowledge=RegionKnowledge(0,world.regions.size,world.config)
        knowledge.regions[1].known_by_robot_ids|=1<<2
        depots=[0,1,2]  # All three fine cells lie in region 0.
        matrices, provenance=roadmap_matrices(world,graph,depots,[world.regions.task(1),world.regions.task(2)],0,{1},knowledge)
        self.assertEqual(int(matrices[0,0,3]),6)
        self.assertEqual(int(matrices[0,3,4]),506)
        self.assertEqual(int(matrices[2,0,3]),6)
        self.assertEqual(int(matrices[2,0,4]),INF)
        self.assertEqual(int(matrices[2,3,4]),INF)
        self.assertEqual(int(matrices[1,0,4]),12)  # Connected peer bypasses knowledge restrictions.
        self.assertTrue(np.array_equal(matrices,matrices.transpose(0,2,1)))
        self.assertTrue(np.all(np.diagonal(matrices,axis1=1,axis2=2)==0))
        self.assertTrue(provenance['knowledge_restrictions'])

    def test_local_edges_require_observed_paths_and_wire_weight_is_not_distance(self):
        world=World(Config(robots=1,width=30,height=30,obstacles=0))
        known=np.full_like(world.truth,UNKNOWN)
        graph=Roadmap(world)
        self.assertFalse(graph.discover(known,0,{0,1}))
        known[:3,:6]=FREE
        self.assertTrue(graph.discover(known,0,{0,1}))
        edge=graph.messages()[0]
        self.assertEqual(edge.weight,0.0)
        self.assertEqual(graph.adjacency[edge.from_cell_id][edge.to_cell_id],6)
        self.assertEqual(graph.provenance[0,1]['kind'],'local')

    def test_default_messages_merge_graph_and_states_without_occupancy(self):
        sim=Simulation(Config(robots=2))
        source,receiver=sim.robots
        source.known[:]=FREE
        source.roadmap.add_edge(0,1)
        source.region_knowledge.set_status(0,S.COVERED,0,'test',True)
        before=receiver.known.copy()
        sim.publish()
        packet=next(m for _,r,m in sim.queue if r==1)
        self.assertEqual(packet.cells,())
        self.assertEqual(packet.roadmap_edges,(RoadmapEdge(0,1),))
        sim.tick=1;sim.deliver()
        np.testing.assert_array_equal(before,receiver.known)
        sim.update_region_knowledge()
        self.assertTrue(receiver.roadmap.connected(0))
        self.assertEqual(receiver.region_knowledge.regions[0].status,S.COVERED_BY_OTHERS)
        np.testing.assert_array_equal(before,receiver.known)
        self.assertEqual(receiver.roadmap.provenance[0,1]['kind'],'peer')
        with self.assertRaises(ValueError):
            sim.receive(1,Message(0,999,1,source.position,((1,FREE),),None))

    def test_equal_revision_sync_suppression_global_exception_and_resend(self):
        k=RegionKnowledge(0,9,Config(robots=2))
        k.set_status(4,S.COVERED,0,'test',True)
        update=RegionUpdate(4,1,3,0,int(S.COVERED))
        self.assertFalse(k.apply_peer(update,1))
        k.sync_peer(update,1)
        self.assertEqual(k.regions[4].known_by_robot_ids,3)
        for _ in range(5): self.assertTrue(k.updates({4}))
        self.assertFalse(k.updates({4}))
        self.assertTrue(k.updates({4},{4}))  # Current global tasks bypass suppression.
        for _ in range(5): k.sync_peer(update,1)
        self.assertLess(k.regions[4].sync_count,6)
        k.set_status(4,S.EXPLORING,0,'new_evidence',True)
        self.assertEqual((k.regions[4].sync_count,k.regions[4].known_by_robot_ids),(0,1))
        k.sync_peer(update,1)  # Stale revision cannot acknowledge the new one.
        self.assertEqual(k.regions[4].known_by_robot_ids,1)
        self.assertTrue(k.updates({4}))

    def test_forwarded_positions_are_fresh_but_do_not_create_direct_contact(self):
        sim=Simulation(Config(robots=3,comms_range=30))
        sim.robots[0].position=(20,10)
        sim.robots[1].position=(1,10)
        sim.robots[2].position=(45,10)
        sim.publish();sim.tick=1;sim.deliver()
        self.assertEqual(sim.robots[2].peers[1].position_tick,-1)
        sim.publish();sim.tick=2;sim.deliver()
        cached=sim.robots[2].peers[1]
        self.assertEqual(cached.position,(1,10))
        self.assertEqual(cached.position_tick,0)
        self.assertNotIn(1,sim.connected(2))

    def test_remote_target_can_be_selected_without_a_local_fine_gateway(self):
        from mtare_2d.regions import select_global_regions
        sim=Simulation(Config(robots=1,width=60,height=60,obstacles=0))
        robot=sim.robots[0]
        robot.position=(3,3);robot.known[:]=UNKNOWN;robot.known[:4,:4]=FREE
        # Global graph from a peer can describe a region absent from local occupancy.
        for a in range(9): robot.roadmap.add_edge(a,a+1)
        remote=9
        robot.region_knowledge.set_status(remote,S.EXPLORING_BY_OTHERS,0,'test',True)
        result=select_global_regions(sim.world,robot.known,[0],0,robot.region_knowledge,(),robot.roadmap)
        self.assertEqual(result['targets'],[sim.world.regions.task(remote)])
        self.assertEqual(result['navigation_cells'],[None])
        robot.selected_tasks=result['targets']
        record=sim.local_planner.plan(robot,0)
        self.assertEqual(record['global_navigation_mode'],'roadmap_guidance_to_local_free_anchor')
        self.assertTrue(record['route']['execution_path'])
        self.assertTrue(all(robot.known.ravel()[sim.world.cell_at(sim.world.xy(c))]==FREE for c in record['route']['execution_path']))


if __name__=='__main__': unittest.main()
