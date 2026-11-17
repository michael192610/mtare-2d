"""Simple frontier work generation, retaining hierarchical VRP coordination."""
from dataclasses import dataclass

from .dense_coverage import DenseCoveragePlanner
from .region_state import ACTIVE, COVERED, CellStatus, Evidence, RegionKnowledge, RegionUpdate
from .world import GridGraph


@dataclass(frozen=True)
class FrontierEvidence(Evidence):
    reachable_cells: int = 0
    frontier_cells: tuple = ()


class FrontierRegionKnowledge(RegionKnowledge):
    """Regions use reachable frontier work, with accepted peer completion retained.

    Wire states keep receiver-relative peer labels. No point thresholds,
    almost-covered rule or exploration-counter reopening gate is used here.
    """
    cycle = -1

    def set_status(self,region,status,actor,reason,bump=False):
        if bump and self.cycle>=0:
            cell=self.regions[region]
            cell.update_id=max(cell.update_id,(self.cycle+1)*self.config.robots+self.robot_id)
        super().set_status(region,status,actor,reason,bump)

    def updates(self,connected_regions,global_regions=()):
        # Simple mode repeats all current summaries; no ACK/echo suppression.
        result=[]
        for region,cell in enumerate(self.regions):
            if cell.status in COVERED:status=CellStatus.COVERED
            elif cell.status in ACTIVE and region in connected_regions:status=CellStatus.EXPLORING
            else:continue
            result.append(RegionUpdate(region,cell.update_id,cell.known_by_robot_ids,
                                       cell.mobility_type,int(status)))
        return tuple(result)

    def sync_peer(self,update,sender):
        cell=self.regions[update.id]
        if update.update_id==cell.update_id:
            cell.known_by_robot_ids|=update.known_by_robot_ids|(1<<sender)
            self.log('region_sync',region_id=update.id,sender=sender,
                incoming_update_id=update.update_id,update_id=cell.update_id,
                known_by_robot_ids=cell.known_by_robot_ids,sync_count=0)

    def update_local(self,window,evidence):
        window=set(window)
        for region in sorted(window-self.window):
            self.regions[region].visit_count+=1
            self.log('region_window_entered',region_id=region,visit_count=self.regions[region].visit_count)
        self.window,self.evidence=window,evidence
        for region in sorted(window):
            ev=evidence.get(region,FrontierEvidence());cell=self.regions[region]
            cell.almost_covered=False
            if cell.status==CellStatus.COVERED_BY_OTHERS:continue
            if not ev.reachable_cells:continue  # Unknown/unreachable is not proved covered.
            if ev.frontier_cells:
                if cell.status!=CellStatus.EXPLORING:
                    self.set_status(region,CellStatus.EXPLORING,self.robot_id,'simple_frontier_present',True)
            elif cell.status not in (CellStatus.COVERED,CellStatus.COVERED_BY_OTHERS):
                self.set_status(region,CellStatus.COVERED,self.robot_id,'simple_no_frontier',True)

    def update_global(self,connected_regions,neighbors):
        # Outside the local window, retain last observed/received frontier state.
        # Connectivity controls task eligibility, not a fabricated covered claim.
        return

    def apply_peer(self,update,sender):
        if not 0<=update.id<len(self.regions):raise ValueError('Out-of-grid region ID')
        cell=self.regions[update.id]
        if update.id in self.window and cell.status==CellStatus.EXPLORING:
            self.log('region_update_ignored',region_id=update.id,sender=sender,
                     reason='protected_local_exploration',incoming_update_id=update.update_id)
            return False
        if update.update_id<=cell.update_id:
            self.log('region_update_ignored',region_id=update.id,sender=sender,
                     reason='not_newer',incoming_update_id=update.update_id)
            return False
        previous=cell.status;cell.update_id=update.update_id;cell.sync_count=0
        cell.mobility_type=update.mobility_type
        status=CellStatus.EXPLORING_BY_OTHERS if update.status==CellStatus.EXPLORING else CellStatus.COVERED_BY_OTHERS
        self.set_status(update.id,status,sender,'simple_received_region')
        cell.known_by_robot_ids=1<<self.robot_id
        self.log('region_update_applied',region_id=update.id,sender=sender,update_id=cell.update_id,
                 incoming_status=int(update.status),previous_status=int(previous),status=int(cell.status))
        return True


class FrontierCoveragePlanner(DenseCoveragePlanner):
    def raw_frontiers(self,robot):
        reachable=GridGraph(self.world,robot.known).distances(self.world.cell_at(robot.position))>=0
        return reachable,[c for c in self.world.frontier_ids(robot.known) if reachable[c]]

    def context(self,robot):
        ctx=self.viewpoint_context(robot);world=self.world
        _,frontiers=self.raw_frontiers(robot)
        local=[c for c in frontiers if world.region_of(c) in ctx['window']
               and robot.region_knowledge.regions[world.region_of(c)].status!=CellStatus.COVERED_BY_OTHERS]
        raw={n for c in local for n in world.neighbors[c] if robot.known.ravel()[n]==-1}
        front={c:(self.visible(c)&raw if c not in ctx['visited'] else set()) for c in ctx['candidates']}
        ctx.update(samples=set(),coverage={c:set() for c in ctx['candidates']},
            frontier_samples=set().union(*front.values()) if front else set(),frontier_coverage=front,
            coverage_state={},normal_small=1,normal_big=2,frontier_min=1,
            frontier_evidence_min=1,frontier_gate_min=1,
            eligible_candidates=[c for c in ctx['candidates'] if c not in ctx['visited']],
            frontier_source_cells=local)
        return ctx

    def evidence(self,robot):
        ctx=self.context(robot);world=self.world
        reachable,frontiers=self.raw_frontiers(robot)
        counts={reg:dict(candidates=0,selected=0,above_frontier=0) for reg in ctx['window']}
        selected=set(robot.local_viewpoints)
        for c in ctx['candidates']:
            d=counts[world.region_of(c)];d['candidates']+=1;d['selected']+=c in selected
            d['above_frontier']+=bool(ctx['frontier_coverage'][c])
        by_region={reg:[] for reg in ctx['window']}
        for c in frontiers:
            reg=world.region_of(c)
            if reg in by_region:by_region[reg].append(c)
        ev={}
        for reg,d in counts.items():
            ev[reg]=FrontierEvidence(**d,reachable_cells=int(reachable[world.regions.cells[reg]].sum()),
                frontier_cells=tuple(by_region[reg]))
        return ctx['window'],ev

    def plan(self,robot,tick):
        record=super().plan(robot,tick)
        record['coverage_model']='simple_frontier'
        _,frontiers=self.raw_frontiers(robot)
        record['reachable_frontier_cells']=[c for c in frontiers if self.world.region_of(c) in record['local_window_region_ids']]
        record['region_work_model']='reachable_frontiers'
        return record
