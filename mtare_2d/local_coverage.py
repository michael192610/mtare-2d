"""2D adaptation of M-TARE's reuse -> randomized greedy coverage -> local TSP.

Sources: local_coverage_planner.cpp SolveLocalCoverageProblem/SelectViewPoint/
SolveTSP and tsp_solver.cpp Solve. Abstract frontier-adjacent unknown samples
replace 3D uncovered surface/frontier points; no sensor or truth query is used.
"""
import time

import numpy as np
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

from .world import FREE, UNKNOWN, GridGraph
from .region_state import Evidence, LOCAL_ELIGIBLE


def order_viewpoints(world, graph, start, end, viewpoints):
    cells = list(dict.fromkeys([start, *viewpoints, end]))
    if len(cells) == 1:
        return dict(cells=cells, distance_matrix=[[0]], ordered_cells=[start],
                    execution_path=[], length=0, solver_called=False)
    step_size = getattr(graph,"step_size",world.config.cell_size)
    distances = [graph.distances(cell) for cell in cells]
    matrix = [[int(distances[i][cell])*step_size for cell in cells]
              for i in range(len(cells))]
    if any(cost < 0 for row in matrix for cost in row):
        raise AssertionError("Local viewpoint route includes an unreachable endpoint")
    # Integer scale follows the original local TSP's 10 * path-length costs.
    costs = [[round(cost*10) for cost in row] for row in matrix]
    manager = pywrapcp.RoutingIndexManager(len(cells), 1, [cells.index(start)], [cells.index(end)])
    routing = pywrapcp.RoutingModel(manager)
    # An open navigation-only route still has physical length even without
    # interior coverage viewpoints; OR-Tools otherwise treats it as an idle vehicle.
    routing.SetVehicleUsedWhenEmpty(True, 0)
    callback = routing.RegisterTransitCallback(lambda a,b: costs[manager.IndexToNode(a)][manager.IndexToNode(b)])
    routing.SetArcCostEvaluatorOfAllVehicles(callback)
    parameters = pywrapcp.DefaultRoutingSearchParameters()
    parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GREEDY_DESCENT
    solution = routing.SolveWithParameters(parameters)
    if solution is None:
        raise RuntimeError("Local TSP returned no route")
    index, ordered = routing.Start(0), []
    while not routing.IsEnd(index):
        ordered.append(cells[manager.IndexToNode(index)])
        index = solution.Value(routing.NextVar(index))
    ordered.append(cells[manager.IndexToNode(index)])
    path = []
    for left, right in zip(ordered, ordered[1:]):
        segment = graph.path(left, right)
        if segment is None:
            raise AssertionError("Local TSP execution path is disconnected")
        path.extend(segment)
    length = round(len(path)*step_size,10)
    if round(length*10) != solution.ObjectiveValue():
        raise AssertionError("Local TSP objective does not match its grid execution path")
    return dict(cells=cells, distance_matrix=costs, ordered_cells=ordered,
                execution_path=path, length=length, objective=int(solution.ObjectiveValue()),
                solver_called=True)


class LocalCoveragePlanner:
    def __init__(self, world):
        self.world = world
        self.footprints = {}

    def visible(self, cell):
        if cell not in self.footprints:
            delta = self.world.centres-np.asarray(self.world.xy(cell))
            self.footprints[cell] = set(np.flatnonzero(np.sum(delta*delta, axis=1) <=
                                                     self.world.config.reveal_radius**2).tolist())
        return self.footprints[cell]

    def context(self, robot):
        world, cfg = self.world, self.world.config
        start = world.cell_at(robot.position)
        window = world.regions.nearby(start, cfg.local_region_radius)
        inside = np.isin(world.regions.cell_region, list(window))
        local_known = robot.known.copy()
        local_known.ravel()[~inside] = UNKNOWN
        graph = GridGraph(world, local_known)
        reachable = graph.distances(start) >= 0
        candidates = np.flatnonzero(reachable & (local_known.ravel() == FREE)).tolist()
        frontiers = world.frontier_ids(robot.known)
        samples = {neighbor for cell in frontiers if inside[cell]
                   for neighbor in world.neighbors[cell] if robot.known.ravel()[neighbor] == UNKNOWN}
        coverage = {cell: self.visible(cell) & samples for cell in candidates}
        return dict(start=start, window=window, inside=inside, graph=graph,
                    candidates=candidates, samples=samples, coverage=coverage)

    def evidence(self, robot):
        context = self.context(robot)
        world, cfg = self.world, self.world.config
        counts = {region: dict(candidates=0, selected=0, above_small=0, above_big=0, above_frontier=0)
                  for region in context["window"]}
        selected = set(robot.local_viewpoints)
        for cell in context["candidates"]:
            values = counts[int(world.regions.cell_region[cell])]
            values["candidates"] += 1
            values["selected"] += cell in selected
            if cell not in robot.visited:
                gain = len(context["coverage"][cell])
                values["above_small"] += gain >= cfg.region_small_gain
                values["above_big"] += gain >= cfg.region_big_gain
                values["above_frontier"] += gain > cfg.region_frontier_gain
        visited_regions = {int(world.regions.cell_region[cell]) for cell in robot.visited}
        evidence = {region: Evidence(**values, has_graph_node=region in visited_regions,
                       near_robot=np.linalg.norm(np.asarray(world.regions.centre(region))-robot.position)<cfg.region_size)
                    for region, values in counts.items()}
        return context["window"], evidence

    def navigation(self, robot, context):
        world = self.world
        start, inside, graph, candidates = [context[k] for k in ("start","inside","graph","candidates")]
        # A local path exit is a mandatory navigation anchor toward the global row.
        end, global_target = start, None
        navigation_mode = 'none'
        for task in robot.selected_tasks:
            if task < world.size:
                continue
            region = world.regions.region(task)
            navigation = world.regions.representative(world, robot.known, region)
            global_path = (GridGraph(world, robot.known).path(start, navigation)
                           if navigation is not None else None)
            if global_path is None and getattr(robot, 'roadmap', None) is not None:
                current_region = int(world.regions.cell_region[start])
                _, region_path = robot.roadmap.shortest_path(current_region, region)
                anchor = region_path[1] if len(region_path) > 1 else region
                centre = world.regions.centre(anchor)
                # Only reachable local free cells can become navigation anchors.
                # Remote roadmap edges never fill the detailed occupancy map.
                choices = [cell for cell in candidates if cell != start]
                if choices:
                    gateway = min(choices, key=lambda cell: (
                        (world.xy(cell)[0]-centre[0])**2+(world.xy(cell)[1]-centre[1])**2,
                        cell))
                    global_path = graph.path(start, gateway)
                    navigation_mode = 'roadmap_guidance_to_local_free_anchor'
            if global_path:
                for cell in global_path:
                    if not inside[cell]:
                        break
                    end = cell
                global_target = task
                if navigation_mode == 'none':
                    navigation_mode = 'known_fine_path_to_target'
                break
        return end, global_target, navigation_mode

    def plan(self, robot, tick):
        started = time.perf_counter()
        world, cfg = self.world, self.world.config
        context = self.context(robot)
        start, window, inside, graph = [context[k] for k in ("start", "window", "inside", "graph")]
        candidates, samples, coverage = [context[k] for k in ("candidates", "samples", "coverage")]
        knowledge = getattr(robot, "region_knowledge", None)
        eligible = [cell for cell in candidates if knowledge is None or
                    knowledge.regions[int(world.regions.cell_region[cell])].status in LOCAL_ELIGIBLE]
        eligible_set = set(eligible)

        end, global_target, navigation_mode = self.navigation(robot, context)
        # Initial poses can be off-centre. Do not credit observations from the
        # current cell centre unless the execution route will actually reach it.
        covered = set(coverage.get(end, set())) if end != start else set()
        reused = []
        for cell in robot.local_viewpoints:
            if cell in eligible_set and cell not in (start, end) and cell not in robot.visited:
                gain = coverage[cell]-covered
                if len(gain) >= cfg.local_min_gain:
                    reused.append(cell)
                    covered.update(gain)
        rng = np.random.default_rng(cfg.seed + 100003*robot.robot_id + tick)
        best = None
        routes_cache = {}
        trials = []
        for iteration in range(cfg.local_optimization_iterations):
            selected, predicted = reused.copy(), covered.copy()
            while True:
                queue = sorted(((len(coverage[cell]-predicted), cell) for cell in eligible
                                if cell not in selected and cell not in (start,end)), key=lambda p:(-p[0],p[1]))
                queue = [entry for entry in queue if entry[0] >= cfg.local_min_gain]
                if not queue:
                    break
                cell = queue[int(rng.integers(min(len(queue),cfg.local_greedy_sample_range)))][1]
                selected.append(cell)
                predicted.update(coverage[cell])
            key = tuple(sorted(selected))
            if key not in routes_cache:
                routes_cache[key] = order_viewpoints(world, graph, start, end, selected)
            route = routes_cache[key]
            trials.append(dict(iteration=iteration, selected_viewpoints=selected,
                               predicted_frontier_samples=len(predicted), route_length=route["length"]))
            if best is None or route["length"] < best["route"]["length"]:
                best = dict(route=route, selected=selected.copy(), predicted=predicted.copy())
        route = best["route"]
        ordered_viewpoints = [cell for cell in route["ordered_cells"][1:] if cell in best["selected"]]
        return dict(tick=tick, robot_id=robot.robot_id, start_cell=start, end_cell=end,
                    global_target_task=global_target, global_navigation_mode=navigation_mode,
                    local_window_region_ids=sorted(window),
                    local_candidate_cells=candidates, eligible_viewpoint_cells=eligible,
                    frontier_sample_cells=sorted(samples),
                    viewpoint_coverage={str(cell): sorted(coverage[cell]) for cell in candidates},
                    reused_viewpoints=reused, selected_viewpoints=ordered_viewpoints,
                    trials=trials, route=route,
                    local_coverage_complete=not best["selected"],
                    actual_tsp_calls=sum(item["solver_called"] for item in routes_cache.values()),
                    elapsed_seconds=time.perf_counter()-started)
