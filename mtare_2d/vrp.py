"""M-TARE's candidate adaptation and OR-Tools VRP, with 2D graph costs.

Reference: grid_world.cpp GetInitialRoutes/GetInitialRoute/AddNodeToVRPPlan;
tsp_solver.cpp SolveVRP; coordination/robot.cpp LessCost.
"""
from dataclasses import dataclass

import numpy as np
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

from .world import GridGraph

INF = 99_999_999


@dataclass(frozen=True)
class Proposal:
    proposal_id: str
    source: int
    tick: int
    routes: tuple[tuple[int, ...], ...]  # Complete cell-ID routes, including depots.

    def to_dict(self):
        return dict(proposal_id=self.proposal_id, source=self.source, tick=self.tick,
                    routes=[list(row) for row in self.routes])


def distance_matrices(world, known, depot_cells, targets, receiver, connected, peer_known, navigation_cells=None):
    task_cells = list(targets) if navigation_cells is None else list(navigation_cells)
    cells = list(depot_cells) + task_cells
    count = len(depot_cells)
    graph = GridGraph(world, known)
    base = np.full((len(cells), len(cells)), INF, dtype=np.int64)
    cache = {}
    for row, cell in enumerate(cells):
        if cell not in cache:
            cache[cell] = graph.distances(cell)
        steps = cache[cell][cells]
        reachable = steps >= 0
        base[row, reachable] = np.rint(steps[reachable]*world.config.cell_size).astype(np.int64)
    for row in range(count, len(cells)):
        for col in range(count, len(cells)):
            if row != col and base[row, col] < INF:
                base[row, col] += world.config.task_arc_offset
    np.fill_diagonal(base, 0)
    matrices = np.repeat(base[None, :, :], count, axis=0)
    # Like GetDistanceMatricesNoComms: do not assign disconnected peers tasks
    # the receiver has no evidence that they know. This does not command them.
    for vehicle in range(count):
        if vehicle != receiver and vehicle not in connected:
            knowledge = peer_known.get(vehicle, set())
            for index, cell in enumerate(task_cells, count):
                if cell not in knowledge:
                    matrices[vehicle, index, :] = INF
                    matrices[vehicle, :, index] = INF
                    matrices[vehicle, index, index] = 0
    return matrices


def route_costs(routes, matrices):
    costs = []
    for vehicle, route in enumerate(routes):
        closed = [vehicle, *route, vehicle]
        costs.append(sum(int(matrices[vehicle, a, b]) for a, b in zip(closed, closed[1:])))
    count = len(routes)
    assigned = {node for row in routes for node in row}
    dropped = [node for node in range(count, matrices.shape[1]) if node not in assigned]
    return costs, dropped


def adapt_proposal(proposal, targets, matrices, vehicles, filter_unreachable=True):
    """Remove old tasks/duplicates, refresh depots, insert missing tasks cheaply."""
    cell_to_node = {cell: vehicles+i for i, cell in enumerate(targets)}
    routes = [[] for _ in range(vehicles)]
    assigned = set()
    for vehicle, old in enumerate(proposal.routes[:vehicles]):
        for cell in old[1:-1]:
            node = cell_to_node.get(cell)
            if node is not None and node not in assigned and (not filter_unreachable or matrices[vehicle, vehicle, node] < INF):
                routes[vehicle].append(node)
                assigned.add(node)
    for node in range(vehicles, vehicles+len(targets)):
        if node in assigned:
            continue
        best = None
        for vehicle, route in enumerate(routes):
            closed = [vehicle, *route, vehicle]
            for position, (left, right) in enumerate(zip(closed, closed[1:])):
                incoming, outgoing = int(matrices[vehicle, left, node]), int(matrices[vehicle, node, right])
                if filter_unreachable and (incoming >= INF or outgoing >= INF):
                    continue
                key = (incoming+outgoing-int(matrices[vehicle, left, right]), vehicle, position)
                if key[0] < INF and (best is None or key < best):
                    best = key
        if best is not None:
            routes[best[1]].insert(best[2], node)
            assigned.add(node)
    return routes


def select_seed(proposals, targets, matrices, vehicles, filter_unreachable=True):
    candidates = []
    best = None
    for proposal in sorted(proposals, key=lambda p: p.source):
        routes = adapt_proposal(proposal, targets, matrices, vehicles, filter_unreachable)
        costs, dropped = route_costs(routes, matrices)
        candidate = dict(source=proposal.source, proposal_id=proposal.proposal_id,
                         original_routes=[list(r) for r in proposal.routes], initial_routes=routes,
                         dropped_node_indices=dropped, dropped_count=len(dropped),
                         longest_route_cost=max(costs, default=0), route_costs=costs)
        candidates.append(candidate)
        key = (len(dropped), candidate["longest_route_cost"])
        if best is None or key < (best["dropped_count"], best["longest_route_cost"]):
            best = candidate
    return best, candidates


def solve(matrices, config, initial_routes=None):
    vehicles, nodes = matrices.shape[0], matrices.shape[1]
    if nodes == vehicles:
        return dict(routes=[[] for _ in range(vehicles)], route_costs=[0]*vehicles,
                    dropped_node_indices=[], objective=0, verified_objective=0,
                    initial_routes_accepted=False, solver_called=False, status="no_tasks")
    manager = pywrapcp.RoutingIndexManager(nodes, vehicles, list(range(vehicles)), list(range(vehicles)))
    routing = pywrapcp.RoutingModel(manager)
    callbacks = []
    callback_functions = []
    for vehicle in range(vehicles):
        def transit(left, right, vehicle=vehicle):
            return int(matrices[vehicle, manager.IndexToNode(left), manager.IndexToNode(right)])
        callback_functions.append(transit)
        callback = routing.RegisterTransitCallback(transit)
        callbacks.append(callback)
        routing.SetArcCostEvaluatorOfVehicle(callback, vehicle)
    routing.AddDimensionWithVehicleTransits(callbacks, 0, 2_147_483_647, True, "Distance")
    routing.GetDimensionOrDie("Distance").SetGlobalSpanCostCoefficient(config.span_coefficient)
    for node in range(vehicles, nodes):
        index = manager.NodeToIndex(node)
        routing.AddDisjunction([index], config.drop_penalty)
        # Hard reachability is explicit in this 2D adaptation, rather than
        # accepting an INF-distance path as physically executable.
        allowed = ([v for v in range(vehicles)] if config.architecture == 'hierarchical'
                   and config.region_state_model == 'mtare' else
                   [v for v in range(vehicles) if matrices[v, v, node] < INF])
        for vehicle in range(vehicles):
            if vehicle not in allowed:
                routing.VehicleVar(index).RemoveValue(vehicle)
        if not allowed:
            routing.ActiveVar(index).SetValue(0)
    parameters = pywrapcp.DefaultRoutingSearchParameters()
    parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION
    parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    parameters.time_limit.FromMilliseconds(config.solver_ms)
    routing.CloseModelWithParameters(parameters)
    assignment = None
    seed_accepted = False
    if initial_routes is not None:
        seed = routing.ReadAssignmentFromRoutes(initial_routes, True)
        if seed is not None:
            seed_accepted = True
            assignment = routing.SolveFromAssignmentWithParameters(seed, parameters)
    if assignment is None:
        assignment = routing.SolveWithParameters(parameters)
    if assignment is None:
        raise RuntimeError(f"OR-Tools returned no assignment (status {routing.status()}); no fallback plan fabricated.")
    routes = []
    for vehicle in range(vehicles):
        route = []
        index = routing.Start(vehicle)
        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            if node >= vehicles:
                route.append(node)
            index = assignment.Value(routing.NextVar(index))
        routes.append(route)
    costs, dropped = route_costs(routes, matrices)
    verified = sum(costs)+config.span_coefficient*max(costs, default=0)+config.drop_penalty*len(dropped)
    if verified != assignment.ObjectiveValue():
        raise AssertionError(f"Objective mismatch: {verified} != {assignment.ObjectiveValue()}")
    if any(cost >= INF for cost in costs):
        raise AssertionError("Solver selected an unreachable arc.")
    return dict(routes=routes, route_costs=costs, dropped_node_indices=dropped,
                objective=int(assignment.ObjectiveValue()), verified_objective=verified,
                initial_routes_accepted=seed_accepted, solver_called=True, status=int(routing.status()))
