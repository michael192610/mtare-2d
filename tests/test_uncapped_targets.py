"""All eligible global tasks reach VRP preparation, including tasks beyond 32."""
import unittest

from legacy_config import simple_config as Config
from mtare_2d.region_state import CellStatus
from mtare_2d.simulation import Simulation
from mtare_2d.world import FREE


class UncappedTargetTests(unittest.TestCase):
    def test_more_than_32_tasks_reach_cost_matrices_without_losing_eligibility_guards(self):
        sim=Simulation(Config(robots=3,obstacles=0))
        robot=sim.robots[0]
        robot.known[:]=FREE
        for region in range(61):
            robot.region_knowledge.set_status(region,CellStatus.EXPLORING,0,'fixture',True)
            if region:robot.roadmap.add_edge(region-1,region)
        robot.region_knowledge.set_status(60,CellStatus.COVERED,0,'fixture',True)
        # A connected teammate's local work must still be excluded.
        robot.peers[1].received_tick=0
        robot.peers[1].local_exploring_region_ids=(5,)
        request=sim.prepare_plan(robot)
        expected=set(range(60))-{5}
        self.assertEqual(set(request['hierarchy']['global_region_ids']),expected)
        self.assertEqual(set(request['targets']),{sim.world.regions.task(i) for i in expected})
        self.assertEqual(request['matrices'].shape,(3,62,62))
        self.assertIn(sim.world.regions.task(59),request['targets'])
        self.assertNotIn(sim.world.regions.task(60),request['targets'])


if __name__=='__main__':unittest.main()
