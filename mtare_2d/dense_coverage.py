"""Count-based coverage with visited-neighborhood handling on a viewpoint lattice."""
import numpy as np

from .coverage_planner import SampleCoveragePlanner
from .region_state import CellStatus
from .world import GridGraph, UNKNOWN


class DenseCoveragePlanner(SampleCoveragePlanner):
    def viewpoint_context(self,robot):
        world=self.world;lattice=world.viewpoints
        start_fine=world.cell_at(robot.position)
        window=world.regions.nearby(start_fine,world.config.local_region_radius)
        graph=lattice.graph(robot.known,window,robot.position)
        candidates=graph.candidates
        # Original visited position history adds a pose only if >=1 m from all
        # recorded poses, then uses that history inside the current horizon.
        for pose in robot.trajectory[robot.visit_history_processed:]:
            if not any(np.linalg.norm(np.asarray(pose)-old)<1 for old in robot.visit_history):
                robot.visit_history.append(np.asarray(pose))
        robot.visit_history_processed=len(robot.trajectory)
        visited=robot.visited_viewpoints
        for c in candidates:
            status=robot.region_knowledge.regions[world.region_of(c)].status
            if status==CellStatus.COVERED_BY_OTHERS:visited.add(c)
            elif status==CellStatus.EXPLORING_BY_OTHERS:visited.discard(c)
        visited.update(lattice.mask_from_poses(robot.visit_history,window))
        candidate_visited=set(candidates)&visited
        if self.world.config.coverage_model=='simple':
            robot.region_knowledge.log('viewpoint_visit_update',start_viewpoint=graph.start,
                local_window_region_ids=sorted(window),candidate_viewpoints=candidates,
                visited_viewpoints=sorted(candidate_visited))
        return dict(start=graph.start,window=window,
            inside=np.isin(world.regions.cell_region,list(window)),graph=graph,candidates=candidates,
            visited=visited,viewpoint_geometry=lattice.metadata(),
            visited_pose_history=[p.tolist() for p in robot.visit_history],
            visited_candidates=sorted(candidate_visited))

    def context(self,robot):
        ctx=self.viewpoint_context(robot);world=self.world
        window=ctx['window'];graph=ctx['graph'];candidates=ctx['candidates'];visited=ctx['visited']
        lattice=world.viewpoints;candidate_visited=set(ctx['visited_candidates'])
        ledger=robot.coverage
        ledger.observe(robot.known,robot.trajectory,robot.position)
        # Original UpdateCoveredArea credits visibility of visited candidate
        # viewpoints (including near-own-path and COVERED_BY_OTHERS candidates).
        supported_cover=set().union(*(self.ordinary_visible(c) for c in candidate_visited)) if candidate_visited else set()
        ledger.credit_visited_viewpoints(supported_cover,sorted(candidate_visited))
        inside=np.isin(world.regions.cell_region,list(window))
        ordinary=set(np.flatnonzero(ledger.observed & ~ledger.covered & inside).tolist())
        frontier={neighbor for c in world.frontier_ids(robot.known) if inside[c]
                  for neighbor in world.neighbors[c] if robot.known.ravel()[neighbor]==UNKNOWN}
        peer_coverage = None
        if world.config.respect_peer_coverage:
            # A private-map boundary is not team exploration work when its
            # TARGET sample is in an accepted peer-completed subspace. Filtering
            # the viewpoint/source cell instead would lose real work across a
            # region boundary. This changes work evidence, never fine occupancy.
            completed = {i for i, cell in enumerate(robot.region_knowledge.regions)
                         if cell.status == CellStatus.COVERED_BY_OTHERS}
            suppressed_ordinary = {c for c in ordinary if world.region_of(c) in completed}
            suppressed_frontier = {c for c in frontier if world.region_of(c) in completed}
            ordinary -= suppressed_ordinary
            frontier -= suppressed_frontier
            peer_coverage = dict(model='peer_region_completion_v1',
                region_ids=sorted(completed),
                suppressed_ordinary_sample_cells=sorted(suppressed_ordinary),
                suppressed_frontier_sample_cells=sorted(suppressed_frontier))
        normal={c:self.ordinary_visible(c)&ordinary if c not in visited else set() for c in candidates}
        front={c:self.visible(c)&frontier if c not in visited else set() for c in candidates}
        return dict(start=graph.start,window=window,inside=inside,graph=graph,candidates=candidates,
            samples=set().union(*normal.values()) if normal else set(),coverage=normal,
            frontier_samples=set().union(*front.values()) if front else set(),frontier_coverage=front,
            coverage_state=ledger.snapshot(),visited=visited,viewpoint_geometry=lattice.metadata(),
            peer_coverage=peer_coverage,
            visited_pose_history=[p.tolist() for p in robot.visit_history],
            visited_candidates=sorted(candidate_visited),
            normal_small=world.config.region_small_gain,
            normal_big=world.config.region_big_gain,
            frontier_min=world.config.local_frontier_min_gain,
            # Original region evidence and initial frontier gate use >, while
            # queue admission uses >=. Samples have integer count scores here.
            frontier_evidence_min=world.config.region_frontier_gain+int(world.config.strict_frontier),
            frontier_gate_min=world.config.local_frontier_min_gain+int(world.config.strict_frontier))

    def navigation(self,robot,ctx):
        return self.navigation_tasks(robot,ctx,robot.selected_tasks)

    def navigation_tasks(self,robot,ctx,tasks):
        world=self.world;start=ctx['start'];end=start;target=None;mode='none'
        fine_start=world.cell_at(robot.position)
        for task in tasks:
            region=world.regions.region(task)
            navigation=world.regions.representative(world,robot.known,region)
            path=GridGraph(world,robot.known).path(fine_start,navigation) if navigation is not None else None
            anchor=None
            if path:
                anchor=fine_start
                for cell in path:
                    if world.region_of(cell) not in ctx['window']:break
                    anchor=cell
                point=world.xy(anchor);mode='known_fine_path_to_target'
            else:
                _,coarse=robot.roadmap.shortest_path(world.region_of(fine_start),region)
                point=world.regions.centre(coarse[1] if len(coarse)>1 else region)
                mode='roadmap_guidance_to_local_free_anchor'
            choices=[c for c in ctx['candidates'] if c!=start]
            if choices:
                end=min(choices,key=lambda c:((world.xy(c)[0]-point[0])**2+(world.xy(c)[1]-point[1])**2,c))
                target=task;break
        return end,target,mode

    def navigation_constraints(self,robot,ctx,forward_exit):
        reverse_exit,_,_=self.navigation_tasks(robot,ctx,reversed(robot.selected_tasks))
        current=ctx['start']
        previous=current
        point=getattr(robot,'lookahead_point',None)
        if getattr(robot,'lookahead_valid',False) and point is not None:
            region=self.world.region_of(self.world.cell_at(point))
            if region in ctx['window']:
                previous=min(ctx['candidates'],key=lambda c:(
                    float(np.sum((np.asarray(self.world.xy(c))-point)**2)),c))
        return [forward_exit,current,previous,reverse_exit]

    def plan(self,robot,tick):
        record=super().plan(robot,tick)
        if record.get('peer_coverage'):
            # Log before Simulation.plan_local updates region states. This lets
            # replay verify which accepted summaries actually informed scoring.
            robot.region_knowledge.log('peer_coverage_plan_evidence',
                                       peer_coverage=record['peer_coverage'])
        record['coverage_model']='mtare_dense_viewpoints'
        record['viewpoint_geometry']=self.world.viewpoints.metadata()
        record['visited_pose_history']=[p.tolist() for p in robot.visit_history]
        record['visited_viewpoint_radius']=self.world.config.visited_viewpoint_radius
        return record
