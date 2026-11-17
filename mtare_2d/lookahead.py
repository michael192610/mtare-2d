"""Planar counterparts of original local TSP anchors and GetLookAheadPoint.

The coverage objective, state machine and graph geometry are not changed.
Momentum remains disabled as in the original indoor configuration. Private
occupancy supplies the line-of-sight/navigation interface instead of a sensor.
"""
import math

from ortools.constraint_solver import pywrapcp, routing_enums_pb2

INF = 99_999_999


def visible_segment(world, known, start, end):
    """Check all grid cells intersected by a segment, using private occupancy."""
    size = world.config.cell_size
    dx, dy = end[0]-start[0], end[1]-start[1]
    cuts = {0.0, 1.0}
    for axis, delta in ((0, dx), (1, dy)):
        if abs(delta) < 1e-12:
            continue
        low, high = sorted((start[axis], end[axis]))
        for k in range(math.floor(low/size)+1, math.ceil(high/size)):
            t = (k*size-start[axis])/delta
            if 0 < t < 1:
                cuts.add(t)
    cuts = sorted(cuts)
    for left, right in zip(cuts, cuts[1:]):
        t = (left+right)/2
        point = (start[0]+t*dx, start[1]+t*dy)
        if known.ravel()[world.cell_at(point)] != 0:
            return False
    return all(known.ravel()[world.cell_at(p)] == 0 for p in (start, end))


def choose_forward(forward_score, backward_score, forward_count, backward_count,
                   dist_from_start, dist_from_end, start_is_robot, end_is_robot):
    """Original GetLookAheadPoint direction branch, with momentum disabled."""
    if forward_count == 0 and backward_count == 0:
        if dist_from_start < dist_from_end and not start_is_robot:
            return False
        if dist_from_end < dist_from_start and not end_is_robot:
            return True
        return forward_score > backward_score
    if forward_score > backward_score:
        return forward_count > 0
    return not backward_count > 0


def order_anchored(world, graph, anchors, viewpoints):
    """Original dummy-node local TSP construction over the existing 2D graph."""
    boundary_start, robot, previous, boundary_end = anchors
    cells = sorted(set([*anchors, *viewpoints]))
    if len(cells) == 1:
        return dict(cells=cells, distance_matrix=[[0]], ordered_cells=cells.copy(),
                    full_path=cells.copy(), execution_path=[], length=0.0,
                    full_path_is_anchor=[True],
                    solver_called=False, solver_tour=[], solver_distance_matrix=[],
                    anchor_model='mtare_dummy_nodes', anchors=list(anchors))
    distances = [graph.distances(c) for c in cells]
    step = getattr(graph, 'step_size', world.config.cell_size)
    # Original TSPSolver costs truncate 10*geometric path length.
    lengths = [[float(distances[i][c])*step for c in cells] for i in range(len(cells))]
    if any(v < 0 for row in lengths for v in row):
        raise AssertionError('Unreachable local anchor')
    costs = [[int(10*d) for d in row] for row in lengths]
    matrix = [row.copy() for row in costs]
    start_end_dummy = None

    def dummy(left, right):
        index = len(matrix)
        for i, row in enumerate(matrix):
            row.append(0 if i in (left, right) else INF)
        matrix.append([0 if i in (left, right, index) else INF for i in range(index+1)])
        return index

    # Add the two dummy nodes in the original ordering.
    if robot != previous:
        dummy(cells.index(robot), cells.index(previous))
    if boundary_start != boundary_end:
        start_end_dummy = dummy(cells.index(boundary_start), cells.index(boundary_end))
    manager = pywrapcp.RoutingIndexManager(len(matrix), 1, cells.index(boundary_start))
    routing = pywrapcp.RoutingModel(manager)
    callback = routing.RegisterTransitCallback(
        lambda a,b: matrix[manager.IndexToNode(a)][manager.IndexToNode(b)])
    routing.SetArcCostEvaluatorOfAllVehicles(callback)
    parameters = pywrapcp.DefaultRoutingSearchParameters()
    parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    solution = routing.SolveWithParameters(parameters)
    if solution is None:
        raise RuntimeError('Anchored local TSP returned no assignment')
    index, tour = routing.Start(0), []
    while not routing.IsEnd(index):
        tour.append(manager.IndexToNode(index))
        index = solution.Value(routing.NextVar(index))
    objective = sum(matrix[a][b] for a,b in zip(tour, [*tour[1:], tour[0]]))
    if objective != solution.ObjectiveValue() or any(
            matrix[a][b] >= INF for a,b in zip(tour, [*tour[1:],tour[0]])):
        raise AssertionError('Invalid constrained local tour')
    if start_end_dummy is not None:
        cut = tour.index(start_end_dummy)
        chain = [i for i in [*tour[cut+1:],*tour[:cut]] if i < len(cells)]
        if cells[chain[0]] != boundary_start:
            chain.reverse()
        ordered = [cells[i] for i in chain]
        assert ordered[0] == boundary_start and ordered[-1] == boundary_end
    else:
        ordered = [cells[i] for i in tour if i < len(cells)]
        ordered.append(ordered[0])
    path = [ordered[0]]
    is_anchor = [True]
    length = 0.0
    for left,right in zip(ordered,ordered[1:]):
        segment = graph.path(left,right)
        if segment is None:
            raise AssertionError('Disconnected anchored tour')
        path.extend(segment)
        is_anchor.extend([False]*(len(segment)-1)+[True] if segment else [])
        length += math.dist(world.xy(left),world.xy(right)) if not segment else len(segment)*step
    return dict(cells=cells, distance_matrix=costs, ordered_cells=ordered,
                full_path=path, execution_path=[], length=round(length,10),
                full_path_is_anchor=is_anchor,
                objective=int(objective), solver_called=True,
                solver_tour=tour, solver_distance_matrix=matrix,
                anchor_model='mtare_dummy_nodes', anchors=list(anchors))


def select_lookahead(world, robot, route, coverage_views, priority):
    """Original bidirectional scanning/priority/heading rules on a planar path."""
    path = route['full_path']
    boundary_start, current, previous, boundary_end = route['anchors']
    direction = tuple(getattr(robot, 'lookahead_direction', (1.0,0.0)))
    loop = path[0] == path[-1] == current
    flags = route.get('full_path_is_anchor',[True]*len(path))
    index = next(i for i,n in enumerate(path) if n==current and flags[i])
    forward_indices = list(range(index+1,len(path)))
    backward_indices = list(range(index-1,-1,-1))
    if loop:
        forward_indices = list(range(1,len(path)))
        backward_indices = list(range(len(path)-2,-1,-1))
    forward = [path[i] for i in forward_indices]
    backward = [path[i] for i in backward_indices]
    coverage_views = set(coverage_views)
    counts = [sum(flags[i] and path[i] in coverage_views for i in indices)
              for indices in (forward_indices,backward_indices)]
    distances = [sum(math.dist(world.xy(a),world.xy(b)) for a,b in zip(path,path[1:index+1])),
                 sum(math.dist(world.xy(a),world.xy(b)) for a,b in zip(path[index:],path[index+1:]))]
    # The original global fallback handles paths shorter than lookahead/5.
    too_short = all(math.dist(robot.position,world.xy(c)) <= world.config.lookahead_distance/5 for c in path)
    branches = []
    for branch,indices in ((forward,forward_indices),(backward,backward_indices)):
        last, travelled, target, skipped = current, 0.0, None, []
        for i,node in enumerate(branch):
            previous_node = last
            last = node
            if any(counts) and not priority.get(world.region_of(node), True):
                skipped.append(node)
                continue
            travelled += math.dist(world.xy(previous_node),world.xy(node))
            next_visible = (i+1 == len(branch) or visible_segment(
                world,robot.known,robot.position,world.xy(branch[i+1])))
            if (travelled > world.config.lookahead_distance or not next_visible
                    or (flags[indices[i]] and (node in coverage_views or node in (boundary_start,boundary_end)))
                    or i == len(branch)-1):
                target = node
                break
        score = -2.0
        if target is not None:
            delta = [world.xy(target)[i]-robot.position[i] for i in (0,1)]
            norm = math.hypot(*delta)
            if norm > 1e-12:
                score = sum(direction[i]*delta[i]/norm for i in (0,1))
        branches.append(dict(target=target,score=score,skipped_nodes=skipped))
    follow = choose_forward(branches[0]['score'],branches[1]['score'],*counts,
                            *distances,boundary_start==current,boundary_end==current)
    chosen = branches[0 if follow else 1]['target']
    valid = chosen is not None and chosen != current and not too_short
    if too_short:
        # No physical home/global controller here: keep a reachable local exit
        # as the abstract global fallback. It is not retained as a new anchor.
        chosen = boundary_start if distances[0] < distances[1] else boundary_end
    if chosen is None:
        chosen = current  # Guard the original zero-direction normalization.
    delta = [world.xy(chosen)[i]-robot.position[i] for i in (0,1)]
    norm = math.hypot(*delta)
    next_direction = tuple(d/norm for d in delta) if norm > 1e-12 and valid else direction
    return dict(robot_position=list(robot.position),
                previous_point=getattr(robot,'lookahead_point',None),
                previous_valid=bool(getattr(robot,'lookahead_valid',False)),
                previous_direction=list(direction),previous_anchor=previous,
                forward=branches[0],backward=branches[1],viewpoint_counts=counts,
                distances_from_boundaries=distances,priority_region_ids=sorted(r for r,v in priority.items() if not v),
                follow_forward=follow,target=chosen,point=list(world.xy(chosen)),
                follow_path=list(forward if follow else backward),
                direction=list(next_direction),valid=valid,local_path_too_short=too_short)
