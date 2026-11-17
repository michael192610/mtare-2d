import json
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import unittest

import matplotlib
matplotlib.use("Agg")

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ir_sim.mtare_bridge import Config, IRSimBridge, Journal, Simulation
from ir_sim.rerun_recorder import RerunRecorder


class RerunRecordingTests(unittest.TestCase):
    def test_cycle_timeline_is_complete_and_recording_does_not_extend_completion(self):
        cfg = Config(robots=1, width=12, height=12, obstacles=0, reveal_radius=30,
                     completion_startup_cycles=0, max_cycles=20, planning_mode="sequential")
        with tempfile.TemporaryDirectory() as folder:
            journal = Journal(Path(folder), cfg)
            sim = Simulation(cfg, journal)
            journal.initialize(sim)
            bridge = IRSimBridge(sim, journal, headless=True)
            try:
                self.assertIsNone(bridge.recorder)
                self.assertFalse((journal.directory / "simulation.rrd").exists())
                bridge.recorder = RerunRecorder(journal.directory, cfg)
                bridge.recorder.capture(bridge)
                while not sim.done:
                    bridge.advance(render=False)
                self.assertEqual(sim.reason, "no_remaining_exploration_work")
                self.assertEqual(sim.tick, 5)
                self.assertEqual(bridge.recorder.cycles, set(range(6)))
                for _ in range(5):
                    bridge.advance(render=False)
                self.assertEqual(sim.tick, 5)
                metadata = json.loads((journal.directory / "rerun_recording.json").read_text())
                self.assertEqual((metadata["frame_count"], metadata["first_cycle"], metadata["last_cycle"]),
                                 (6, 0, 5))
                executable = Path(sys.executable).with_name("rerun")
                verified = subprocess.run([str(executable), "rrd", "verify", str(bridge.recorder.path)],
                                          capture_output=True, text=True)
                self.assertEqual(verified.returncode, 0, verified.stderr)
                printed = subprocess.run([str(executable), "rrd", "print", "--entity",
                                          "metrics/coverage_percent", str(bridge.recorder.path)],
                                         capture_output=True, text=True, check=True)
                count = sum(map(int, re.findall(r"with (\d+) rows", printed.stdout)))
                self.assertEqual(count, 6)  # Independently read actual RRD rows.
                with self.assertRaises(FileExistsError):
                    RerunRecorder(journal.directory, cfg)
            finally:
                bridge.plt.close(bridge.figure)
                bridge.env.end(ending_time=0)
                sim.close()
