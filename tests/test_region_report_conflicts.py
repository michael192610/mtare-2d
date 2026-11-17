"""Conflicting reports must converge without altering VRP or completion rules."""
import unittest

from mtare_2d.config import Config
from mtare_2d.region_state import CellStatus as S, Evidence, RegionKnowledge, RegionUpdate
from mtare_2d.simulation import Simulation


class RegionReportConflictTests(unittest.TestCase):
    def conflict(self, repair=True):
        config = Config(robots=3, repair_region_reports=repair)
        peers = [RegionKnowledge(i, 1, config) for i in range(3)]
        # Region 255 at the end of the reported cycle-625 deadlock: identical
        # counters/ACKs, conflicting meanings, locally authored completion at R1.
        for peer, status in zip(peers, (S.EXPLORING_BY_OTHERS, S.COVERED, S.EXPLORING)):
            cell = peer.regions[0]
            cell.status = status
            cell.covered_to_others = status == S.COVERED
            cell.explored_robot_id = 1 if status == S.COVERED else -1
            cell.update_id = 2
            cell.known_by_robot_ids = 7
            cell.sync_count = 5
        return peers

    def exchange(self, peers):
        packets = [p.updates({0}, {0} if p.regions[0].status in (S.EXPLORING, S.EXPLORING_BY_OTHERS) else ())
                   for p in peers]
        for receiver, peer in enumerate(peers):
            for sender, updates in enumerate(packets):
                if sender == receiver:
                    continue
                for update in updates:
                    peer.apply_peer(update, sender)
                    peer.sync_peer(update, sender)

    def test_original_equal_version_conflict_remains_stuck(self):
        peers = self.conflict(False)
        for _ in range(15):
            self.exchange(peers)
        self.assertEqual([p.regions[0].status for p in peers],
                         [S.EXPLORING_BY_OTHERS, S.COVERED, S.EXPLORING])

    def test_saved_deadlock_pattern_converges_using_fresh_completion_report(self):
        peers = self.conflict()
        for _ in range(5):
            self.exchange(peers)
        self.assertEqual([p.regions[0].status for p in peers],
                         [S.COVERED_BY_OTHERS, S.COVERED, S.COVERED_BY_OTHERS])
        self.assertEqual([p.regions[0].update_id for p in peers], [3, 3, 3])
        self.assertTrue(all(p.regions[0].known_by_robot_ids == 7 for p in peers))
        version = peers[1].regions[0].update_id
        stale = RegionUpdate(0, 2, 7, 0, int(S.EXPLORING))
        for _ in range(20):
            peers[1].apply_peer(stale, 2)
            peers[1].sync_peer(stale, 2)
        self.assertEqual(peers[1].regions[0].update_id, version)

    def test_conflicting_state_is_not_an_acknowledgement(self):
        peer = self.conflict()[0]
        update = RegionUpdate(0, 2, 7, 0, int(S.COVERED))
        peer.sync_peer(update, 1)
        cell = peer.regions[0]
        self.assertEqual((cell.known_by_robot_ids, cell.sync_count), (1, 0))
        self.assertEqual(cell.status, S.EXPLORING_BY_OTHERS)
        self.assertTrue(peer.updates({0}))

    def test_current_local_exploration_remains_protected_from_fresh_completion(self):
        peers = self.conflict()
        active = peers[2]
        active.window = {0}
        active.evidence = {0: Evidence(candidates=12, selected=1, above_small=12)}
        for _ in range(5):
            self.exchange(peers)
        self.assertEqual(active.regions[0].status, S.EXPLORING)
        self.assertEqual(active.regions[0].update_id, 2)
        self.assertEqual(active.regions[0].known_by_robot_ids, 1 << 2)
        # Original local coverage decision, not an idle timer, ends protection.
        active.update_local({0}, {0: Evidence(candidates=12)})
        for _ in range(5):
            self.exchange(peers)
        self.assertTrue(all(p.report_status(p.regions[0]) == S.COVERED for p in peers))

    def test_peer_only_completion_claims_are_not_refreshed(self):
        for status, actor in [(S.COVERED_BY_OTHERS, 1), (S.UNSEEN, 1), (S.COVERED, 1)]:
            with self.subTest(status=status, actor=actor):
                peer = self.conflict()[0]
                cell = peer.regions[0]
                cell.status, cell.covered_to_others, cell.explored_robot_id = status, True, actor
                peer.apply_peer(RegionUpdate(0, 2, 7, 0, int(S.EXPLORING)), 2)
                self.assertEqual(cell.update_id, 2)
                self.assertEqual(cell.status, status)

    def test_original_covered_to_others_claim_can_be_republished_without_changing_local_state(self):
        peer = RegionKnowledge(0, 1, Config(robots=2))
        peer.update_local({0}, {0: Evidence(candidates=1)})
        peer.apply_peer(RegionUpdate(0, 1, 3, 0, int(S.EXPLORING)), 1)
        cell = peer.regions[0]
        self.assertEqual((cell.status, cell.update_id), (S.UNSEEN, 2))
        self.assertTrue(cell.covered_to_others)
        self.assertEqual(peer.updates({0})[0].status, int(S.COVERED))

    def test_matching_receiver_relative_states_still_sync_and_suppress(self):
        peer = RegionKnowledge(0, 1, Config(robots=2))
        peer.set_status(0, S.EXPLORING_BY_OTHERS, 1, 'fixture', True)
        peer.sync_peer(RegionUpdate(0, 1, 3, 0, int(S.EXPLORING)), 1)
        for _ in range(5):
            self.assertTrue(peer.updates({0}))
        self.assertEqual(peer.updates({0}), ())
        # A new revision starts a new ACK round; old echo counters cannot leak.
        peer.set_status(0, S.COVERED, 0, 'fixture', True)
        self.assertEqual(peer.regions[0].known_by_robot_ids, 1)
        self.assertEqual(peer.regions[0].exploring_synced_count, {})

    def test_newer_but_rejected_packet_cannot_acknowledge_current_revision(self):
        peer = self.conflict()[2]
        peer.window = {0}
        update = RegionUpdate(0, 9, 7, 0, int(S.COVERED))
        self.assertFalse(peer.apply_peer(update, 1))
        peer.sync_peer(update, 1)
        self.assertEqual((peer.regions[0].status, peer.regions[0].update_id), (S.EXPLORING, 2))
        self.assertEqual(peer.regions[0].known_by_robot_ids, 4)

    def test_conflict_repair_unblocks_existing_work_based_completion_predicate(self):
        sim = Simulation(Config(robots=3, width=30, height=30, obstacles=0,
                                completion_min_coverage=None))  # Historical source-style predicate.
        try:
            peers = self.conflict()
            for robot, peer in zip(sim.robots, peers):
                robot.region_knowledge = peer
                robot.calls = robot.local_calls = 1
                robot.local_coverage_complete = True
                robot.global_exploring_regions = ([0] if peer.regions[0].status in
                                                   (S.EXPLORING, S.EXPLORING_BY_OTHERS) else [])
            sim.tick = 400
            sim.check_planner_completion()
            self.assertFalse(sim.done)
            for _ in range(5):
                self.exchange(peers)
            for robot, peer in zip(sim.robots, peers):
                robot.global_exploring_regions = ([0] if peer.regions[0].status in
                                                   (S.EXPLORING, S.EXPLORING_BY_OTHERS) else [])
            sim.world.truth[:] = 1  # Completion must not consult evaluator truth.
            sim.check_planner_completion()
            self.assertTrue(sim.done)
            self.assertEqual(sim.reason, 'no_remaining_exploration_work')
        finally:
            sim.close()
