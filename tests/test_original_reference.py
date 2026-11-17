"""Differential checks against decision code extracted from original M-TARE.

Requires g++ and the original checkout (MTARE_ORIGINAL_ROOT can override its
location). Geometry/ROS accessors are stubs; the original decision block and
LessCost function are compiled unchanged. This verifies decision semantics,
not the physical evidence model or full-system performance equivalence.
"""
import os
from pathlib import Path
import random
import shutil
import subprocess
import tempfile
import unittest

from mtare_2d.config import Config
from mtare_2d.region_state import CellStatus, Evidence, RegionKnowledge
from mtare_2d.vrp import Proposal, select_seed


class OriginalReferenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        root = Path(os.environ.get('MTARE_ORIGINAL_ROOT',
                    str(Path(__file__).resolve().parents[1] / '.references' / 'mtare_planner')))
        source = root / 'src/tare_planner/src/grid_world/grid_world.cpp'
        if not source.exists() or not shutil.which('g++'):
            raise unittest.SkipTest('Original M-TARE source and g++ required for differential checks')
        grid = source.read_text()
        start = grid.index('    bool has_selected_viewpoint =', grid.index('void GridWorld::UpdateLocalCellStatus'))
        end = grid.index('    if (cell_status == CellStatus::EXPLORING && has_candidate)', start)
        decision = grid[start:end]
        robot = (root / 'src/tare_planner/src/coordination/robot.cpp').read_text()
        start = robot.index('bool LessCost(')
        end = robot.index('\nvoid DecodeGlobalVRPPlan', start)
        less_cost = robot[start:end]
        cls.temp = tempfile.TemporaryDirectory(prefix='mtare-cpp-reference-')
        cls.addClassCleanup(cls.temp.cleanup)
        cpp = Path(cls.temp.name) / 'reference.cpp'
        cpp.write_text(r'''
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>
enum class CellStatus { UNSEEN, EXPLORING, COVERED, COVERED_BY_OTHERS, NOGO, EXPLORING_BY_OTHERS };
namespace geometry_msgs { struct Point { double x=0,y=0,z=0; }; }
namespace misc_utils_ns { template<class T,class U> double PointXYDist(T a,U b) {
    return std::hypot(a.x-b.x,a.y-b.y); } }
struct Cell {
    int visits=1; bool graph=false;
    void SetExploredRobotID(int) {}
    int GetVisitCount() { return visits; }
    std::vector<int> GetGraphNodeIndices() { return graph ? std::vector<int>{0}:std::vector<int>{}; }
    geometry_msgs::Point GetPosition() { return {}; }
};
struct Spaces { Cell cell; Cell& GetCell(int) { return cell; } };
struct Reference {
    CellStatus status; bool covered=false; int version=0, exploring=0, known=7;
    bool near=false;
    std::vector<int> almost_covered_cell_indices_;
    std::unique_ptr<Spaces> subspaces_=std::make_unique<Spaces>();
    geometry_msgs::Point robot_position_;
    double kCellSize=6, kCellHeight=3;
    int kRobotID=0,kCellExploringToCoveredThr=1,kCellCoveredToExploringThr=10,kMaxExploringCount=2;
    void SetCellStatus(int,CellStatus next) {
        if (next==CellStatus::EXPLORING || next==CellStatus::EXPLORING_BY_OTHERS) {
            if (status!=CellStatus::EXPLORING && status!=CellStatus::EXPLORING_BY_OTHERS) ++exploring;
            covered=false;
        } else if (next==CellStatus::COVERED || next==CellStatus::COVERED_BY_OTHERS) covered=true;
        status=next;
    }
    void RemoveFromAlmostCovered(int) { almost_covered_cell_indices_.clear(); }
    void AddToAlmostCovered(int) { almost_covered_cell_indices_={0}; }
    void ResetCellSync(int) { known=1; }
    void IncreaseCellUpdateID(int) { ++version; }
    void SetCellCoveredToOthers(int,bool value) { covered=value; }
    bool IsCellCoveredToOthers(int) { return covered; }
    int GetCellExploringCount(int) { return exploring; }
    bool IsCellTraversable(int) { return true; }
    geometry_msgs::Point GetCellPosition(int) { return {}; }
    void run(int candidate_count,int selected_viewpoint_count,int above_small_threshold_count,
             int above_big_threshold_count,int above_frontier_threshold_count) {
        int cell_ind=0;
        CellStatus cell_status=status;
        robot_position_.x=near?0:100;
'''+decision+r'''
    }
};
struct VRPCost { int dropped_nodes_number_,longest_route_length_; };
'''+less_cost+r'''
int main() {
    char mode;
    while (std::cin>>mode) {
        if (mode=='C') {
            VRPCost a,b;
            std::cin>>a.dropped_nodes_number_>>a.longest_route_length_>>b.dropped_nodes_number_>>b.longest_route_length_;
            std::cout<<LessCost(a,b)<<"\n"; continue;
        }
        Reference r; int status,candidates,selected,small,big,front,graph,near,covered,almost;
        std::cin>>status>>candidates>>selected>>small>>big>>front>>r.subspaces_->cell.visits
                >>graph>>near>>r.exploring>>covered>>almost;
        r.status=static_cast<CellStatus>(status);r.covered=covered;r.near=near;
        r.subspaces_->cell.graph=graph;
        if(almost)r.almost_covered_cell_indices_={0};
        // Original skips peer-covered/non-traversable regions before this block.
        if(r.status!=CellStatus::COVERED_BY_OTHERS) r.run(candidates,selected,small,big,front);
        std::cout<<static_cast<int>(r.status)<<' '<<r.version<<' '<<r.exploring<<' '
                 <<r.covered<<' '<<!r.almost_covered_cell_indices_.empty()<<' '<<r.known<<"\n";
    }
}
''')
        cls.binary = Path(cls.temp.name) / 'reference'
        subprocess.run(['g++','-std=c++17','-O2',str(cpp),'-o',str(cls.binary)],
                       check=True, capture_output=True, text=True)

    def reference(self, lines):
        result = subprocess.run([str(self.binary)], input='\n'.join(lines)+'\n',
                                check=True, capture_output=True, text=True)
        return [tuple(map(int, row.split())) for row in result.stdout.splitlines()]

    def test_local_region_decisions_match_original_cpp(self):
        rng = random.Random(755)
        cases = []
        for status in range(6):
            for n in range(400):
                cases.append((status,rng.choice([0,1,11]),rng.choice([0,1]),
                    rng.choice([0,1,11]),rng.choice([0,10,11]),rng.choice([0,1,10,11]),
                    rng.choice([1,2]),rng.randrange(2),rng.randrange(2),rng.choice([0,2,3]),
                    rng.randrange(2),rng.randrange(2)))
        expected = self.reference(['R '+' '.join(map(str,c)) for c in cases])
        for case, result in zip(cases, expected):
            status,candidates,selected,small,big,front,visits,graph,near,exploring,covered,almost = case
            k = RegionKnowledge(0, 1, Config())
            k.window={0}
            cell=k.regions[0]
            cell.status=CellStatus(status);cell.visit_count=visits;cell.exploring_count=exploring
            cell.covered_to_others=bool(covered);cell.almost_covered=bool(almost);cell.known_by_robot_ids=7
            k.update_local({0},{0:Evidence(candidates,selected,small,big,front,bool(graph),bool(near))})
            actual=(int(cell.status),cell.update_id,cell.exploring_count,int(cell.covered_to_others),
                    int(cell.almost_covered),cell.known_by_robot_ids)
            self.assertEqual(actual,result,case)

    def test_proposal_comparison_matches_original_cpp(self):
        import numpy as np
        rng = random.Random(12)
        cases=[]
        for _ in range(200):
            tasks=list(range(10,14));m=np.zeros((2,6,6),dtype=np.int64)
            m[0,0,2:]=rng.randrange(2000);m[1,1,2:]=rng.randrange(2000)
            a=Proposal('a',0,0,((0,*tasks[:rng.randrange(5)],0),(1,1)))
            b=Proposal('b',1,0,((0,0),(1,*tasks[:rng.randrange(5)],1)))
            best,(left,right)=select_seed([a,b],tasks,m,2)
            cases.append((best,left,right))
        expected=self.reference([
            f"C {left['dropped_count']} {left['longest_route_cost']} "
            f"{right['dropped_count']} {right['longest_route_cost']}"
            for _,left,right in cases])
        for (best,left,right),(prefer_left,) in zip(cases,expected):
            tie=(left['dropped_count'],left['longest_route_cost'])==(
                right['dropped_count'],right['longest_route_cost'])
            self.assertEqual(best['source'],0 if prefer_left or tie else 1)
