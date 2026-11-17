"""Completion follows eligible global work + local coverage, never evaluator truth."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from legacy_config import simple_config as Config
from mtare_2d.recording import Journal
from mtare_2d.region_state import CellStatus
from mtare_2d.results_viewer import SavedRun, create_result_viewer
from mtare_2d.simulation import Simulation


class CompletionTests(unittest.TestCase):
    def idle_sim(self, **kwargs):
        sim = Simulation(Config(robots=3, width=40, height=40, obstacles=0, **kwargs))
        for robot in sim.robots:
            robot.calls = robot.local_calls = 1
            robot.local_coverage_complete = True
        sim.tick = sim.config.completion_startup_cycles + 1
        return sim

    def test_partial_coverage_finishes_saves_honest_percentage_and_view(self):
        import matplotlib
        matplotlib.use('Agg')
        sim = self.idle_sim()
        observed = sim.stats()['explored_ratio']
        self.assertLess(observed, 1)
        sim.check_planner_completion()
        self.assertTrue(sim.done)
        self.assertEqual(sim.reason, 'no_remaining_exploration_work')
        self.assertEqual(sim.stats()['explored_ratio'], observed)
        previous_tick = sim.tick
        sim.step()
        self.assertEqual(sim.tick, previous_tick)
        with tempfile.TemporaryDirectory() as folder:
            journal = Journal(Path(folder), sim.config)
            journal.initialize(sim)
            journal.finish(sim, 'finished')
            summary = json.loads((journal.directory/'summary.json').read_text())
            self.assertEqual(summary['status'], 'finished')
            self.assertEqual(summary['final']['explored_ratio'], observed)
            self.assertIn('not proof of 100%', (journal.directory/'RESULTS.txt').read_text())
            self.assertEqual(SavedRun(journal.directory).reason, sim.reason)
            viewer = create_result_viewer(journal.directory)
            self.assertIn('COMPLETE', viewer.figure._suptitle.get_text())
            self.assertIn('no remaining exploration work', viewer.figure._suptitle.get_text())
            viewer.plt.close(viewer.figure)

    def test_startup_guard_is_strict_simulation_cycles_not_wall_time(self):
        sim = self.idle_sim()
        for tick in (0, 4, 299, 300):
            sim.tick = tick
            sim.check_planner_completion()
            self.assertFalse(sim.done)
        sim.tick = 301
        sim.check_planner_completion()
        self.assertTrue(sim.done)

    def test_no_finished_state_before_first_planning(self):
        sim = Simulation(Config(robots=3, width=40, height=40, obstacles=0))
        sim.tick = 400
        sim.check_planner_completion()
        self.assertFalse(sim.done)

    def test_global_task_list_not_empty_assigned_route_controls_completion(self):
        sim = self.idle_sim()
        robot = sim.robots[2]
        robot.global_exploring_regions = [2]  # Could be allocated to a teammate or dropped.
        robot.selected_tasks = []
        sim.check_planner_completion()
        self.assertFalse(sim.done)
        self.assertEqual([r.exploration_finished for r in sim.robots], [True, True, False])

    def test_each_robot_must_have_local_coverage_complete(self):
        sim = self.idle_sim()
        sim.robots[1].local_coverage_complete = False
        sim.check_planner_completion()
        self.assertFalse(sim.done)

    def test_active_ineligible_regions_and_transit_are_not_extra_stop_guards(self):
        sim = self.idle_sim()
        robot = sim.robots[0]
        robot.region_knowledge.regions[-1].status = CellStatus.EXPLORING_BY_OTHERS
        robot.path = [1]  # Navigation points can coexist with source local completion.
        robot.selected_tasks = [2501]
        robot.last_dropped = 1  # Diagnostic is not an extra source predicate.
        sim.check_planner_completion()
        self.assertTrue(sim.done)
        self.assertEqual(robot.region_knowledge.regions[-1].status, CellStatus.EXPLORING_BY_OTHERS)
        self.assertEqual(robot.path, [1])

    def test_finished_flags_are_recomputed_if_new_work_appears(self):
        sim = self.idle_sim()
        sim.robots[2].local_coverage_complete = False
        sim.check_planner_completion()
        self.assertTrue(sim.robots[0].exploration_finished)
        sim.robots[0].global_exploring_regions = [3]
        sim.check_planner_completion()
        self.assertFalse(sim.robots[0].exploration_finished)
        self.assertFalse(sim.done)

    def test_full_observation_does_not_stop_default_planners_with_work(self):
        sim = self.idle_sim(completion_startup_cycles=0)
        sim.tick = 4
        for robot in sim.robots:
            robot.known[:] = sim.world.truth
            robot.global_exploring_regions = [1]
        self.assertEqual(sim.stats()['explored_ratio'], 1)
        with patch.object(sim, 'update_region_knowledge'), patch.object(sim, 'plan_all'), \
             patch.object(sim, 'plan_local'), patch.object(sim, 'move'):
            sim.step()
        self.assertFalse(sim.done)
        self.assertEqual(sim.tick, 5)

    def test_guarded_completion_does_not_depend_on_truth_or_delay(self):
        sim = self.idle_sim(message_delay=8)
        sim.world.truth[:] = 1
        sim.check_planner_completion()
        self.assertTrue(sim.done)  # No extra delay-based stable-idle wait.

    def test_cycle_limit_remains_incomplete_when_work_remains(self):
        sim = self.idle_sim(max_cycles=302)
        sim.tick = 301
        for robot in sim.robots:
            robot.global_exploring_regions = [1]
        with patch.object(sim, 'move'):
            sim.step()
        self.assertTrue(sim.done)
        self.assertEqual(sim.reason, 'cycle_limit')
        self.assertFalse(all(r.exploration_finished for r in sim.robots))

    def test_legacy_prototype_keeps_its_original_percentage_exit(self):
        sim = Simulation(Config(robots=3, width=40, height=40, obstacles=0, architecture='flat'))
        for robot in sim.robots:
            robot.known[:] = sim.world.truth
        sim.step()
        self.assertTrue(sim.done)
        self.assertEqual(sim.reason, 'all_reachable_free_cells_observed')
