"""Original transition/update rules with explicit, deterministic 2D evidence."""
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from legacy_config import simple_config as Config
from mtare_2d.recording import Journal
from mtare_2d.region_state import CellStatus as S, Evidence, RegionKnowledge, RegionUpdate
from mtare_2d.regions import select_global_regions
from mtare_2d.results_viewer import SavedRun
from mtare_2d.simulation import Message, Simulation
from mtare_2d.world import FREE


class RegionStateTests(unittest.TestCase):
    def knowledge(self):
        return RegionKnowledge(0,9,Config())

    def test_original_state_codes_and_selected_covered_reopening_limit(self):
        self.assertEqual([int(s) for s in S],[0,1,2,3,4,5])
        k=self.knowledge()
        selected=Evidence(candidates=3,selected=1,above_small=3,above_frontier=3)
        low=Evidence(candidates=3)
        high=Evidence(candidates=11,above_big=11,above_frontier=11)
        k.update_local({4},{4:selected})
        cell=k.regions[4]
        self.assertEqual((cell.status,cell.exploring_count,cell.update_id),(S.EXPLORING,1,1))
        for expected in [2,3]:
            k.update_local({4},{4:low})
            self.assertEqual(cell.status,S.COVERED)
            k.update_local({4},{4:high})
            self.assertEqual((cell.status,cell.exploring_count),(S.EXPLORING,expected))
        k.update_local({4},{4:low})
        version=cell.update_id
        k.update_local({4},{4:high})
        self.assertEqual((cell.status,cell.exploring_count,cell.update_id),(S.COVERED,3,version))

    def test_visit_counter_window_entry_and_almost_covered_departure(self):
        k=self.knowledge()
        k.update_local({4},{4:Evidence(candidates=2,selected=1)})
        for _ in range(3):
            k.update_local({4},{4:Evidence(candidates=2,above_small=1)})
        self.assertEqual(k.regions[4].visit_count,1)
        self.assertTrue(k.regions[4].almost_covered)
        k.update_local(set(),{})
        self.assertEqual(k.regions[4].status,S.COVERED)
        k.update_local({4},{4:Evidence()})
        self.assertEqual(k.regions[4].visit_count,2)

    def test_unseen_candidate_can_be_covered_to_peers_without_local_covered_state(self):
        k=self.knowledge()
        k.update_local({4},{4:Evidence(candidates=1)})
        self.assertEqual(k.regions[4].status,S.UNSEEN)
        self.assertTrue(k.regions[4].covered_to_others)
        update=k.updates({4})[0]
        self.assertEqual((update.id,update.status,update.update_id),(4,int(S.COVERED),1))

    def test_peer_protection_versions_and_receiver_relative_state_translation(self):
        k=self.knowledge()
        k.update_local({4},{4:Evidence(candidates=1,selected=1)})
        covered=RegionUpdate(4,9,2,0,int(S.COVERED))
        self.assertFalse(k.apply_peer(covered,1))
        self.assertEqual(k.regions[4].status,S.EXPLORING)
        self.assertEqual(k.regions[4].update_id,1)
        k.update_local(set(),{})
        self.assertTrue(k.apply_peer(covered,1))
        self.assertEqual(k.regions[4].status,S.COVERED_BY_OTHERS)
        self.assertFalse(k.apply_peer(covered,1))
        self.assertTrue(k.apply_peer(RegionUpdate(4,10,2,0,int(S.EXPLORING)),1))
        self.assertEqual(k.regions[4].status,S.EXPLORING_BY_OTHERS)
        k.set_status(5,S.COVERED,0,'test',True)
        k.apply_peer(RegionUpdate(5,20,2,0,int(S.EXPLORING)),1)
        self.assertEqual(k.regions[5].status,S.COVERED)  # remote exploring does not reopen local covered
        k.set_status(6,S.NOGO,0,'test')
        k.update_local({6},{6:Evidence(candidates=3,selected=1,above_frontier=3)})
        self.assertEqual(k.regions[6].status,S.NOGO)

    def test_global_surrounded_and_disconnected_rules(self):
        k=self.knowledge()
        for i in range(9):
            k.set_status(i,S.COVERED,0,'test')
        k.set_status(4,S.EXPLORING,0,'test')
        k.update_global({4},lambda i:[j for j in range(9) if j!=i])
        self.assertEqual(k.regions[4].status,S.COVERED)
        k.set_status(0,S.EXPLORING,0,'test')
        k.update_global(set(),lambda i:[])
        self.assertEqual(k.regions[0].status,S.COVERED)

    def test_global_tasks_use_states_and_exclude_received_local_work(self):
        sim=Simulation(Config(robots=1,width=60,height=60,obstacles=0))
        robot=sim.robots[0]
        robot.known[:]=FREE
        region=sim.world.regions.size-1
        robot.region_knowledge.set_status(region,S.EXPLORING,0,'test',True)
        current=0
        result=select_global_regions(sim.world,robot.known,[current],current,robot.region_knowledge)
        self.assertEqual(result['global_region_ids'],[region])
        self.assertEqual(result['frontier_cell_ids'],[])  # state, not a current frontier, makes it a task
        excluded=select_global_regions(sim.world,robot.known,[current],current,robot.region_knowledge,{region})
        self.assertEqual(excluded['targets'],[])

    def test_local_coverage_filters_states_but_retains_free_cells_for_transit(self):
        sim=Simulation(Config(robots=1,width=40,height=40,obstacles=0))
        robot=sim.robots[0]
        initial=sim.local_planner.plan(robot,0)
        self.assertTrue(initial['selected_viewpoints'])
        for cell in robot.region_knowledge.regions:
            cell.status=S.COVERED_BY_OTHERS
        filtered=sim.local_planner.plan(robot,0)
        self.assertEqual(filtered['selected_viewpoints'],[])
        self.assertTrue(filtered['local_candidate_cells'])
        self.assertEqual(filtered['eligible_viewpoint_cells'],[])

    def test_messages_copy_summaries_cache_them_and_apply_before_planning(self):
        sim=Simulation(Config(robots=2,width=60,height=60,obstacles=0))
        source=sim.robots[0]
        source.region_knowledge.set_status(0,S.COVERED,0,'test',True)
        source.local_exploring_regions=[1,2]
        sim.publish()
        packet=next(p for due,receiver,p in sim.queue if receiver==1)
        source.region_knowledge.regions[0].update_id=9
        source.local_exploring_regions.clear()
        self.assertEqual(packet.region_updates[0].update_id,1)
        self.assertEqual(packet.local_exploring_region_ids,(1,2))
        sim.tick=1
        sim.deliver()
        peer=sim.robots[1].peers[0]
        self.assertTrue(peer.region_updates_pending)
        self.assertEqual(sim.robots[1].region_knowledge.regions[0].status,S.UNSEEN)
        sim.update_region_knowledge()
        self.assertEqual(sim.robots[1].region_knowledge.regions[0].status,S.COVERED_BY_OTHERS)
        self.assertFalse(peer.region_updates_pending)
        self.assertEqual(peer.local_exploring_region_ids,(1,2))
        sim.receive(1,Message(0,0,1,(99,99),(),None,(RegionUpdate(0,99,1,0,1),),(8,)))
        self.assertEqual(peer.local_exploring_region_ids,(1,2))  # old packet cannot change claimed work

    def test_complete_stateful_run_and_saved_region_snapshots(self):
        cfg=Config(width=40,height=40,obstacles=0,robots=3,solver_ms=10,max_cycles=350,
                   local_optimization_iterations=3)
        with tempfile.TemporaryDirectory() as folder:
            journal=Journal(Path(folder),cfg)
            sim=Simulation(cfg,journal)
            journal.initialize(sim)
            try:
                while not sim.done:
                    sim.step()
                journal.finish(sim,'finished' if sim.stats()['explored_ratio']==1 else 'stopped')
                self.assertEqual(sim.stats()['explored_ratio'],1)
                self.assertGreater(sim.event_counts['region_state_changed'],0)
                self.assertGreater(sim.event_counts['region_update_applied'],0)
                states=json.loads((journal.directory/'region_states.json').read_text())
                restored=SavedRun(journal.directory)
                self.assertEqual(states['0'],restored.robots[0].region_states)
                for path in (journal.directory/'calls').glob('*.json'):
                    call=json.loads(path.read_text())
                    info=call['hierarchy']
                    self.assertEqual(info['state_model'],'mtare')
                    self.assertFalse(set(info['global_region_ids']) & set(info['excluded_peer_local_region_ids']))
                    self.assertTrue(all(info['active_region_states'][str(i)] in (1,5) for i in info['global_region_ids']))
            finally:
                sim.close()
                if not journal.finished:
                    journal.finish(sim,'stopped')
