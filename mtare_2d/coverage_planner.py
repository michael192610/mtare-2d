"""Separate covered-point/frontier bookkeeping and M-TARE local selection.

Source methods: PlanningEnv::GetUncoveredArea, GridWorld::UpdateLocalCellStatus,
LocalCoveragePlanner::EnqueueViewpointCandidates, SelectViewPoint,
SelectViewPointFromFrontierQueue and SolveLocalCoverageProblem.
The geometric evidence is a documented 2D abstraction, not a sensor emulator.
"""
import time

import numpy as np

from .coverage_bookkeeping import CoverageBookkeeping
from .local_coverage import LocalCoveragePlanner, order_viewpoints
from .region_state import Evidence, LOCAL_ELIGIBLE


class SampleCoveragePlanner(LocalCoveragePlanner):
    def __init__(self, world):
        super().__init__(world)
        self.ordinary_footprints = {}

    def ordinary_visible(self, cell):
        if cell not in self.ordinary_footprints:
            delta = self.world.centres-np.asarray(self.world.xy(cell))
            self.ordinary_footprints[cell] = set(np.flatnonzero(np.sum(delta*delta, axis=1) <=
                self.world.config.effective_coverage_radius**2).tolist())
        return self.ordinary_footprints[cell]

    def context(self, robot):
        context = super().context(robot)
        if getattr(robot, 'coverage', None) is None:
            robot.coverage = CoverageBookkeeping(self.world, robot.robot_id)
        ledger = robot.coverage
        ledger.observe(robot.known, robot.trajectory, robot.position)
        ordinary = set(np.flatnonzero(ledger.observed & ~ledger.covered & context['inside']).tolist())
        frontier = context['samples']
        coverage = {cell: self.ordinary_visible(cell) & ordinary if cell not in robot.visited else set()
                    for cell in context['candidates']}
        frontier_coverage = {cell: self.visible(cell) & frontier if cell not in robot.visited else set()
                             for cell in context['candidates']}
        # Original GetUncoveredArea indexes only points visible from unvisited candidates.
        supported = set().union(*coverage.values()) if coverage else set()
        frontier_supported = set().union(*frontier_coverage.values()) if frontier_coverage else set()
        context.update(samples=supported, coverage=coverage, frontier_samples=frontier_supported,
                       frontier_coverage=frontier_coverage, coverage_state=ledger.snapshot())
        return context

    def evidence(self, robot):
        ctx = self.context(robot)
        world, cfg = self.world, self.world.config
        counts = {region:dict(candidates=0,selected=0,above_small=0,above_big=0,above_frontier=0)
                  for region in ctx['window']}
        selected = set(robot.local_viewpoints)
        visited=ctx.get('visited',robot.visited)
        small=ctx.get('normal_small',cfg.region_small_gain)
        big=ctx.get('normal_big',cfg.region_big_gain)
        front_min=ctx.get('frontier_min',cfg.region_frontier_gain)
        for cell in ctx['candidates']:
            values = counts[world.region_of(cell)]
            values['candidates'] += 1
            values['selected'] += cell in selected
            if cell not in visited:
                score, frontier = len(ctx['coverage'][cell]), len(ctx['frontier_coverage'][cell])
                values['above_small'] += score >= small
                values['above_big'] += score >= big
                values['above_frontier'] += (frontier >= ctx['frontier_evidence_min']
                    if 'frontier_evidence_min' in ctx else frontier > front_min)
        visited_regions = {world.region_of(c) for c in robot.visited}
        evidence = {region:Evidence(**value,has_graph_node=region in visited_regions,
            near_robot=bool(np.linalg.norm(np.asarray(world.regions.centre(region))-robot.position)<cfg.region_size))
            for region,value in counts.items()}
        return ctx['window'], evidence

    def greedy(self, queue, lists, covered, threshold, rng, continuation_min=None):
        """Recompute marginal counts after every selection; safely bound live queue."""
        selected, steps, remaining = [], [], set(queue)
        predicted = set(covered)
        while remaining:
            ranked = sorted(((len(lists[c]-predicted),c) for c in remaining), key=lambda x:(-x[0],x[1]))
            minimum = max(threshold,continuation_min or threshold) if selected else threshold
            qualifying = [item for item in ranked if item[0] >= minimum]
            if not qualifying:
                break
            pick = qualifying[int(rng.integers(min(len(qualifying),self.world.config.local_greedy_sample_range)))][1]
            gain = len(lists[pick]-predicted)
            selected.append(pick); steps.append(dict(viewpoint=pick,gain=gain))
            predicted.update(lists[pick]); remaining.remove(pick)
        return selected, predicted, steps

    def plan(self, robot, tick):
        started = time.perf_counter()
        cfg, world = self.world.config, self.world
        ctx = self.context(robot)
        start, graph = ctx['start'], ctx['graph']
        normal, front = ctx['coverage'], ctx['frontier_coverage']
        knowledge = getattr(robot,'region_knowledge',None)
        visited=ctx.get('visited',robot.visited)
        eligible = ctx.get('eligible_candidates', [c for c in ctx['candidates'] if knowledge is None or
                    knowledge.regions[world.region_of(c)].status in LOCAL_ELIGIBLE])
        end, global_target, navigation_mode = self.navigation(robot,ctx)
        anchors = None
        if cfg.use_lookahead and cfg.coverage_model == 'mtare' and hasattr(self,'navigation_constraints'):
            anchors = self.navigation_constraints(robot,ctx,end)
        normal_min, front_min = ctx.get('normal_small',cfg.region_small_gain), ctx.get('frontier_min',cfg.local_frontier_min_gain)
        eligible_set = set(eligible)
        # Reuse checks are made against empty predicted masks, as in the original.
        reused = [c for c in robot.local_viewpoints if c in eligible_set and c not in visited
                  and c not in (start,end) and (len(normal[c]) >= normal_min or len(front[c]) >= front_min)]
        navigation = list(dict.fromkeys(anchors if anchors else (start,end)))
        preselected = set(reused) | set(navigation)
        pre_normal = set().union(*(normal.get(c,set()) for c in preselected))
        pre_front = set().union(*(front.get(c,set()) for c in preselected))
        normal_queue, front_queue = [], []
        for cell in eligible:
            if cell in visited or cell in preselected:
                continue
            if len(normal[cell]-pre_normal) >= normal_min:
                normal_queue.append(cell)
            elif len(front[cell]-pre_front) >= front_min:
                front_queue.append(cell)
        rng = np.random.default_rng(cfg.seed+100003*robot.robot_id+tick)
        trials, cache, best = [], {}, None
        iterations = cfg.local_optimization_iterations if normal_queue else 1
        # Keep initial strict frontier gate separate from >= queue admission.
        # Legacy records can retain their relaxed integer thresholds explicitly.
        frontier_gate = bool(front_queue and (
            max(len(front[c]-pre_front) for c in front_queue) >= ctx['frontier_gate_min']
            if 'frontier_gate_min' in ctx else
            max(len(front[c]-pre_front) for c in front_queue) > front_min))
        for iteration in range(iterations):
            ordinary_selected, normal_mask, normal_steps = self.greedy(
                normal_queue,normal,pre_normal,normal_min,rng)
            frontier_mask = pre_front | set().union(*(front[c] for c in ordinary_selected))
            frontier_selected, frontier_steps = [], []
            if frontier_gate:
                # Original continuation also checks the ordinary minimum.
                frontier_selected, frontier_mask, frontier_steps = self.greedy(
                    front_queue,front,frontier_mask,front_min,rng,normal_min)
            selected = list(dict.fromkeys([*ordinary_selected,*frontier_selected,*reused]))
            key = tuple(sorted(selected))
            if key not in cache:
                if anchors:
                    from .lookahead import order_anchored
                    cache[key] = order_anchored(world,graph,anchors,selected)
                else:
                    cache[key] = order_viewpoints(world,graph,start,end,selected)
            route = cache[key]
            trials.append(dict(iteration=iteration, ordinary_selected=ordinary_selected,
                frontier_selected=frontier_selected, selected_viewpoints=selected.copy(),
                ordinary_selection_steps=normal_steps, frontier_selection_steps=frontier_steps,
                predicted_ordinary_samples=len(normal_mask),predicted_frontier_samples=len(frontier_mask),
                route_length=route['length']))
            if best is None or route['length'] < best['route']['length']:
                best = dict(route=route,selected=selected.copy(),iteration=iteration)
        selected = set(best['selected'])
        ordered = [c for c in best['route']['ordered_cells'][1:] if c in selected]
        lookahead = None
        if anchors:
            from .lookahead import select_lookahead
            # Original navigation anchor can itself be a coverage viewpoint.
            coverage_views = set(selected)-{anchors[0],anchors[1],anchors[3]}
            previous = anchors[2]
            if previous != start and (len(normal.get(previous,set())) >= normal_min
                                      or len(front.get(previous,set())) >= front_min):
                coverage_views.add(previous)
            priority = {r:knowledge.regions[r].has_exploring_priority for r in ctx['window']}
            lookahead = select_lookahead(world,robot,best['route'],coverage_views,priority)
            path = graph.path(start,lookahead['target'])
            if path is None:
                raise AssertionError('Lookahead is outside the private connected graph')
            best['route']['execution_path'] = path
            best['route']['execution_length'] = len(path)*graph.step_size
            lookahead['coverage_viewpoints'] = sorted(coverage_views)
        return dict(tick=tick,robot_id=robot.robot_id,start_cell=start,end_cell=end,
            lookahead=lookahead,
            coverage_model='mtare_samples', coverage_state=ctx['coverage_state'],
            peer_coverage=ctx.get('peer_coverage'),
            ordinary_sample_cells=sorted(ctx['samples']),frontier_sample_cells=sorted(ctx['frontier_samples']),
            viewpoint_coverage={str(c):sorted(normal[c]) for c in ctx['candidates']},
            viewpoint_frontier_coverage={str(c):sorted(front[c]) for c in ctx['candidates']},
            visited_viewpoint_cells=sorted(set(ctx['candidates']) & visited),
            global_target_task=global_target,global_navigation_mode=navigation_mode,
            local_window_region_ids=sorted(ctx['window']),local_candidate_cells=ctx['candidates'],
            eligible_viewpoint_cells=eligible,navigation_viewpoints=navigation,
            previous_viewpoints=robot.local_viewpoints.copy(),
            reused_viewpoints=reused,selected_viewpoints=ordered,ordinary_queue=normal_queue,
            frontier_queue=front_queue,frontier_gate=frontier_gate,
            score_thresholds=dict(ordinary_min=normal_min,frontier_min=front_min,
                ordinary_big=ctx.get('normal_big',cfg.region_big_gain),
                score_units='sample_count',
                **({key:ctx[key] for key in ('frontier_evidence_min','frontier_gate_min')}
                   if 'frontier_gate_min' in ctx else {})),
            trials=trials,selected_trial=best['iteration'],route=best['route'],
            local_coverage_complete=not selected,
            actual_tsp_calls=sum(item['solver_called'] for item in cache.values()),
            elapsed_seconds=time.perf_counter()-started)
