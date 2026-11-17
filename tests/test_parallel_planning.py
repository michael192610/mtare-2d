"""Parallel searches must overlap without changing cycle or message ownership."""
import os
from pathlib import Path
import tempfile
import unittest

from legacy_config import simple_config as Config
from mtare_2d.recording import Journal
from mtare_2d.simulation import Simulation
from mtare_2d.world import FREE


class ParallelPlanningTests(unittest.TestCase):
    def setup_unique_problem(self, mode):
        sim = Simulation(Config(architecture="flat", robots=3, width=20, height=20, obstacles=0,
                                solver_ms=80, planning_mode=mode))
        depots, targets = [0, 99, 90], [1, 98, 80]
        for robot, cell in zip(sim.robots, depots):
            robot.position = sim.world.xy(cell)
            robot.known[:] = FREE
            for other, peer in robot.peers.items():
                peer.position = sim.world.xy(depots[other])
                peer.received_tick = 0
                peer.known_cells = set(range(sim.world.size))
        sim.world.targets = lambda known, depots: (targets, targets)
        return sim

    def test_real_processes_overlap_and_barrier_precedes_application(self):
        parallel = self.setup_unique_problem("parallel")
        sequential = self.setup_unique_problem("sequential")
        try:
            parallel.plan_all()  # Warm persistent workers; avoids startup timing assumptions.
            records = parallel.plan_all()
            reference = sequential.plan_all()
            execution = [r["execution"] for r in records]
            self.assertEqual(len({e["worker_pid"] for e in execution}), 3)
            self.assertNotIn(os.getpid(), {e["worker_pid"] for e in execution})
            self.assertLess(max(e["started_seconds"] for e in execution),
                            min(e["finished_seconds"] for e in execution))
            self.assertGreaterEqual(min(e["applied_seconds"] for e in execution),
                                    max(e["finished_seconds"] for e in execution))
            self.assertEqual([r["result"]["objective"] for r in records], [412]*3)
            self.assertEqual([r["team_routes_cell_ids"] for r in records],
                             [r["team_routes_cell_ids"] for r in reference])
            self.assertTrue(all(r["execution"]["worker_pid"] == os.getpid() for r in reference))
            self.assertEqual(parallel.tick, 0)
            self.assertEqual(parallel.queue, [])
            self.assertEqual(parallel.event_counts["move"], 0)
        finally:
            parallel.close()
            sequential.close()
        self.assertIsNone(parallel._pool)

    def test_cycle_waits_for_all_plans_and_messages_keep_one_cycle_delay(self):
        cfg = Config(architecture="flat", robots=3, width=40, height=40, obstacles=0, reveal_radius=4.1,
                     solver_ms=20, max_cycles=8)
        with tempfile.TemporaryDirectory() as folder:
            journal = Journal(Path(folder), cfg)
            sim = Simulation(cfg, journal)
            journal.initialize(sim)
            try:
                sim.step()
                self.assertEqual(sim.tick, 1)
                self.assertEqual(sim.event_counts["receive"], 0)
                self.assertEqual(len(sim.queue), 6)
                self.assertTrue(all(due == 1 for due, recipient, packet in sim.queue))
                while not sim.done:
                    sim.step()
                self.assertIsNone(sim._pool)
                journal.finish(sim, "stopped")
                import json
                events = [json.loads(line) for line in (journal.directory / "events.jsonl").read_text().splitlines()]
                for tick in [0, 4]:
                    kinds = [e["kind"] for e in events if e["tick"] == tick]
                    self.assertEqual(kinds.count("plan_selected"), 3)
                    self.assertLess(kinds.index("planning_batch_finished"), kinds.index("publish"))
                    self.assertLess(kinds.index("planning_batch_finished"), kinds.index("move"))
                self.assertTrue(all(e["sent_tick"] < e["tick"] for e in events if e["kind"] == "receive"))
            finally:
                sim.close()
