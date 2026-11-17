from collections import Counter, deque
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, dataclass, field
import math
import multiprocessing
import time

import numpy as np

from .config import Config
from .local_coverage import LocalCoveragePlanner
from .dense_coverage import DenseCoveragePlanner
from .simple_frontier import FrontierCoveragePlanner, FrontierRegionKnowledge
from .coverage_bookkeeping import CoverageBookkeeping
from .regions import select_global_regions
from .region_state import ACTIVE, RegionKnowledge, RegionUpdate
from .roadmap import Roadmap, RoadmapEdge, roadmap_matrices
from .planning import run_search
from .vrp import Proposal, distance_matrices, select_seed
from .world import BLOCKED, FREE, UNKNOWN, GridGraph, World


@dataclass(frozen=True)
class Message:
    sender: int
    sequence: int
    sent_tick: int
    position: tuple[float, float]
    cells: tuple[tuple[int, int], ...]
    proposal: Proposal | None
    region_updates: tuple[RegionUpdate, ...] = ()
    local_exploring_region_ids: tuple[int, ...] = ()
    roadmap_edges: tuple[RoadmapEdge, ...] = ()
    robot_positions: tuple[tuple[float, float], ...] = ()
    robot_update_ticks: tuple[int, ...] = ()

    def to_dict(self):
        return dict(sender=self.sender, sequence=self.sequence, sent_tick=self.sent_tick,
                    position=self.position, cells=self.cells,
                    proposal=self.proposal.to_dict() if self.proposal else None,
                    region_updates=[update.to_dict() for update in self.region_updates],
                    local_exploring_region_ids=list(self.local_exploring_region_ids),
                    roadmap_edges=[edge.to_dict() for edge in self.roadmap_edges],
                    robot_positions=self.robot_positions, robot_update_ticks=self.robot_update_ticks)


@dataclass
class Peer:
    position: tuple[float, float]
    sequence: int = -1
    received_tick: int = -1
    known_cells: set[int] = field(default_factory=set)
    proposal: Proposal | None = None
    region_updates: tuple[RegionUpdate, ...] = ()
    region_updates_pending: bool = False
    local_exploring_region_ids: tuple[int, ...] = ()
    roadmap_edges: tuple[RoadmapEdge, ...] = ()
    position_tick: int = -1


@dataclass
class Robot:
    robot_id: int
    position: tuple[float, float]
    known: np.ndarray
    peers: dict[int, Peer]
    trajectory: list = field(default_factory=list)
    selected_tasks: list[int] = field(default_factory=list)
    path: list[int] = field(default_factory=list)
    path_goal: int | None = None
    proposal: Proposal | None = None
    calls: int = 0
    sequence: int = 0
    distance: float = 0.0
    visited: set[int] = field(default_factory=set)
    last_seed_source: int | None = None
    last_objective: int = 0
    last_dropped: int = 0
    local_viewpoints: list[int] = field(default_factory=list)
    local_calls: int = 0
    local_plan_id: str | None = None
    last_local_plan_tick: int | None = None
    local_navigation_end: int | None = None
    local_coverage_complete: bool = False
    region_knowledge: RegionKnowledge | None = None
    local_exploring_regions: list[int] = field(default_factory=list)
    connected_regions: set[int] = field(default_factory=set)
    roadmap: Roadmap | None = None
    global_exploring_regions: list[int] = field(default_factory=list)
    coverage: CoverageBookkeeping | None = None
    exploration_finished: bool = False
    visited_viewpoints: set = field(default_factory=set)
    visit_history: list = field(default_factory=list)
    visit_history_processed: int = 0
    movement_credit: float = 0.0
    lookahead_point: tuple[float,float] | None = None
    lookahead_direction: tuple[float,float] = (1.0,0.0)
    lookahead_valid: bool = False


class Simulation:
    def __init__(self, config=Config(), journal=None):
        self.config = config
        self.stateful = config.architecture == "hierarchical" and config.region_state_model == "mtare"
        self.world = World(config)
        self.journal = journal
        self.tick = 0
        self.done = False
        self.reason = "running"
        self.queue = []
        self.event_counts = Counter()
        self.recent_events = deque(maxlen=30)
        self.history = []
        self.plan_summaries = []
        self.switches = []
        self._pool = None
        planner=(FrontierCoveragePlanner if config.coverage_model=='simple' else DenseCoveragePlanner)
        self.local_planner = (planner(self.world)
            if self.stateful and config.coverage_model in ('simple','mtare') else LocalCoveragePlanner(self.world))
        self.robots = []
        for robot_id, position in enumerate(self.world.starts):
            known = np.full((self.world.rows, self.world.cols), UNKNOWN, dtype=np.int8)
            self.world.reveal(known, position)
            robot = Robot(robot_id, position, known,
                          {i: Peer(tuple(p)) for i, p in enumerate(self.world.starts) if i != robot_id})
            if self.stateful:
                knowledge_type=FrontierRegionKnowledge if config.coverage_model=='simple' else RegionKnowledge
                robot.region_knowledge = knowledge_type(robot_id, self.world.regions.size, config, self.event)
                robot.roadmap = Roadmap(self.world)
            robot.trajectory.append(position)
            robot.visited.add(self.world.cell_at(position))
            if self.stateful and config.coverage_model == 'mtare':
                robot.coverage = CoverageBookkeeping(self.world,robot_id,self.event)
                robot.coverage.observe(robot.known,robot.trajectory,position)
            self.robots.append(robot)

    def event(self, kind, **data):
        record = {"tick": self.tick, "kind": kind, **data}
        self.event_counts[kind] += 1
        self.recent_events.append(record)
        if self.journal:
            self.journal.event(record)

    def in_range(self, left, right):
        return (self.config.comms_range is None or
                math.dist(self.robots[left].position, self.robots[right].position) <= self.config.comms_range)

    def connected(self, receiver):
        robot = self.robots[receiver]
        return {other for other, peer in robot.peers.items()
                if self.in_range(receiver, other) and peer.received_tick >= 0
                and self.tick-peer.received_tick <= self.config.message_ttl}

    def publish(self):
        for robot in self.robots:
            indices = ([] if self.stateful else np.flatnonzero(robot.known.ravel() != UNKNOWN).tolist())
            positions = tuple(tuple(robot.position if i == robot.robot_id else robot.peers[i].position)
                              for i in range(self.config.robots)) if self.stateful else ()
            stamps = tuple(self.tick if i == robot.robot_id else robot.peers[i].position_tick
                           for i in range(self.config.robots)) if self.stateful else ()
            packet = Message(robot.robot_id, robot.sequence, self.tick, tuple(robot.position),
                             tuple((i, int(robot.known.ravel()[i])) for i in indices), robot.proposal,
                             robot.region_knowledge.updates(robot.connected_regions, robot.global_exploring_regions) if self.stateful else (),
                             tuple(robot.local_exploring_regions),
                             robot.roadmap.messages() if self.stateful else (), positions, stamps)
            robot.sequence += 1
            recipients = [i for i in range(self.config.robots)
                          if i != robot.robot_id and self.in_range(robot.robot_id, i)]
            if recipients:
                self.event("publish", recipients=recipients, message=packet.to_dict())
            for recipient in recipients:
                self.queue.append((self.tick+self.config.message_delay, recipient, packet))

    def receive(self, recipient, message):
        robot = self.robots[recipient]
        peer = robot.peers[message.sender]
        if not self.in_range(recipient, message.sender):
            self.event("message_out_of_range", receiver=recipient, sender=message.sender,
                       sequence=message.sequence)
            return
        if message.sequence <= peer.sequence:
            self.event("message_stale", receiver=recipient, sender=message.sender,
                       sequence=message.sequence)
            return
        if self.tick-message.sent_tick > self.config.message_ttl:
            self.event("message_expired", receiver=recipient, sender=message.sender,
                       sequence=message.sequence)
            return
        changed = []
        if self.stateful and message.cells:
            raise ValueError('Detailed occupancy is not part of M-TARE-style knowledge sharing')
        for cell, value in message.cells:
            previous = int(robot.known.ravel()[cell])
            if previous == UNKNOWN:
                robot.known.ravel()[cell] = value
                changed.append((cell, value))
            elif previous != value:
                raise AssertionError("Conflicting deterministic map observations.")
        if message.sent_tick >= peer.position_tick:
            peer.position = tuple(message.position)
            peer.position_tick = message.sent_tick
        peer.sequence = message.sequence
        peer.received_tick = self.tick
        peer.known_cells = {cell for cell, _ in message.cells}
        peer.proposal = message.proposal
        peer.region_updates = message.region_updates
        peer.region_updates_pending = self.stateful or bool(message.region_updates)
        peer.local_exploring_region_ids = message.local_exploring_region_ids
        peer.roadmap_edges = message.roadmap_edges
        if self.stateful and message.robot_positions:
            if len(message.robot_positions) != self.config.robots or len(message.robot_update_ticks) != self.config.robots:
                raise ValueError('Incomplete forwarded position table')
            for teammate, (position, stamp) in enumerate(zip(message.robot_positions, message.robot_update_ticks)):
                if teammate in (recipient, message.sender):
                    continue
                cached = robot.peers[teammate]
                if stamp > cached.position_tick:
                    cached.position, cached.position_tick = tuple(position), stamp
                    self.event('position_forwarded', receiver=recipient, sender=message.sender,
                               teammate=teammate, position=position, position_tick=stamp,
                               sequence=message.sequence)
        self.event("receive", receiver=recipient, sender=message.sender,
                   sequence=message.sequence, sent_tick=message.sent_tick,
                   proposal_id=message.proposal.proposal_id if message.proposal else None,
                   position=message.position, newly_learned_cells=changed,
                   cached_position=peer.position, position_tick=peer.position_tick,
                   region_updates=[update.to_dict() for update in message.region_updates],
                   local_exploring_region_ids=list(message.local_exploring_region_ids),
                   roadmap_edges=[edge.to_dict() for edge in message.roadmap_edges])

    def deliver(self):
        pending = []
        for due, recipient, message in self.queue:
            if due <= self.tick:
                self.receive(recipient, message)
            else:
                pending.append((due, recipient, message))
        self.queue = pending

    def update_local_region_state(self, robot):
        if not self.stateful:
            return
        if isinstance(robot.region_knowledge,FrontierRegionKnowledge):
            robot.region_knowledge.cycle=self.tick
        window, evidence = self.local_planner.evidence(robot)
        robot.region_knowledge.update_local(window, evidence)
        reachable = GridGraph(self.world, robot.known).distances(self.world.cell_at(robot.position)) >= 0
        robot.connected_regions = set(map(int, np.unique(self.world.regions.cell_region[reachable])))
        for left, right, path in robot.roadmap.discover(robot.known, self.world.cell_at(robot.position), window):
            self.event('roadmap_edge_discovered', robot_id=robot.robot_id,
                       from_cell_id=left, to_cell_id=right, known_free_path=path)
        robot.connected_regions |= robot.roadmap.component(
            int(self.world.regions.cell_region[self.world.cell_at(robot.position)]))

    def update_region_knowledge(self):
        for robot in self.robots:
            self.update_local_region_state(robot)
            knowledge = robot.region_knowledge
            for sender, peer in sorted(robot.peers.items()):
                if peer.region_updates_pending:
                    for update in peer.region_updates:
                        knowledge.apply_peer(update, sender)
                    for region in peer.local_exploring_region_ids:
                        if knowledge.regions[region].status not in ACTIVE:
                            knowledge.regions[region].sync_count = 0
                    for edge in peer.roadmap_edges:
                        if edge.weight != 0.0:
                            raise ValueError('This 2D scope supports homogeneous NONE mobility only')
                        if robot.roadmap.add_edge(edge.from_cell_id, edge.to_cell_id,
                            {'kind':'peer', 'sender':sender, 'sequence':peer.sequence}):
                            self.event('roadmap_edge_received', robot_id=robot.robot_id,
                                       sender=sender, sequence=peer.sequence, **edge.to_dict())
                    for update in peer.region_updates:
                        knowledge.sync_peer(update, sender)
                    peer.region_updates_pending = False
            robot.connected_regions |= robot.roadmap.component(
                int(self.world.regions.cell_region[self.world.cell_at(robot.position)]))
            peer_work = {sender: robot.peers[sender].local_exploring_region_ids
                         for sender in self.connected(robot.robot_id)}
            knowledge.update_priority(peer_work)
            knowledge.update_global(robot.connected_regions, self.world.regions.neighbors)
            if self.config.use_lookahead and self.config.coverage_model == 'mtare':
                self.event('local_priority_updated',robot_id=robot.robot_id,
                           window=sorted(knowledge.window),
                           peer_work={str(i):list(ids) for i,ids in peer_work.items()},
                           deprioritized=[i for i,c in enumerate(knowledge.regions) if not c.has_exploring_priority])

    def prepare_plan(self, robot):
        robot.calls += 1
        robot_id = robot.robot_id
        depot_positions = [robot.position if i == robot_id else robot.peers[i].position
                           for i in range(self.config.robots)]
        depots = [self.world.cell_at(p) for p in depot_positions]
        hierarchy = None
        if self.config.architecture == "hierarchical":
            peer_local = {region for peer in self.connected(robot_id)
                          for region in robot.peers[peer].local_exploring_region_ids}
            hierarchy = select_global_regions(self.world, robot.known, depots, depots[robot_id],
                                               robot.region_knowledge, peer_local, robot.roadmap)
            frontier_ids, targets = hierarchy["frontier_cell_ids"], hierarchy["targets"]
        else:
            frontier_ids, targets = self.world.targets(robot.known, set(depots))
        connected = self.connected(robot_id)
        cost_provenance = None
        if self.stateful:
            matrices, cost_provenance = roadmap_matrices(self.world, robot.roadmap, depots,
                                                         targets, robot_id, connected, robot.region_knowledge)
        else:
            matrices = distance_matrices(self.world, robot.known, depots, targets, robot_id, connected,
                                     {i: peer.known_cells for i, peer in robot.peers.items()},
                                     hierarchy["navigation_cells"] if hierarchy else None)
        proposals = ([robot.proposal] if robot.proposal else []) + [
            robot.peers[i].proposal for i in sorted(connected) if robot.peers[i].proposal is not None]
        best, candidates = select_seed(proposals, targets, matrices, self.config.robots,
                                       filter_unreachable=not self.stateful)
        for candidate in candidates:
            source = candidate["source"]
            if source != robot_id:
                candidate["receipt_sequence"] = robot.peers[source].sequence
                candidate["received_tick"] = robot.peers[source].received_tick
        return dict(robot_id=robot_id, depot_positions=depot_positions, depots=depots,
                    frontier_ids=frontier_ids, targets=targets, connected=connected,
                    matrices=matrices, best=best, candidates=candidates,
                    before=robot.selected_tasks.copy(), hierarchy=hierarchy, cost_provenance=cost_provenance)

    def finish_plan(self, robot, request, outcome, mode):
        result, execution = outcome
        robot_id = robot.robot_id
        depot_positions, depots = request["depot_positions"], request["depots"]
        frontier_ids, targets = request["frontier_ids"], request["targets"]
        connected, matrices = request["connected"], request["matrices"]
        best, candidates, before = request["best"], request["candidates"], request["before"]
        node_cells = depots + targets
        full_routes = tuple(tuple([depots[i], *[node_cells[n] for n in row], depots[i]])
                            for i, row in enumerate(result["routes"]))
        selected = list(full_routes[robot_id][1:-1])
        call_id = f"robot{robot_id}_cycle{self.tick:06d}_call{robot.calls:06d}"
        own_candidate = next((c for c in candidates if c["source"] == robot_id), None)
        peer_seed = bool(best is not None and best["source"] != robot_id and result["initial_routes_accepted"])
        seed_cells = ([node_cells[n] for n in best["initial_routes"][robot_id]] if best else [])
        original_peer_tasks = best["original_routes"][robot_id][1:-1] if best else []
        hint_changed_own = bool(peer_seed and own_candidate is not None and
                                own_candidate["initial_routes"][robot_id] != best["initial_routes"][robot_id])
        retained = bool(peer_seed and selected == seed_cells)
        direct_switch = bool(hint_changed_own and retained and selected and
                             selected == original_peer_tasks and before != selected)
        robot.proposal = Proposal(call_id, robot_id, self.tick, full_routes)
        robot.selected_tasks = selected
        robot.global_exploring_regions = request['hierarchy']['global_region_ids'] if self.stateful else []
        robot.path = []
        robot.path_goal = None
        robot.last_seed_source = best["source"] if best else None
        robot.last_objective = result["objective"]
        robot.last_dropped = len(result["dropped_node_indices"])
        record = dict(call_id=call_id, tick=self.tick, robot_id=robot_id,
                      num_vehicles=self.config.robots, depots=list(range(self.config.robots)),
                      depot_positions=depot_positions, depot_cell_ids=depots,
                      all_frontier_cell_ids=frontier_ids, target_cell_ids=targets,
                      node_cell_ids=node_cells, node_positions=[self.world.xy(c) for c in node_cells],
                      distance_matrices=matrices.tolist(),
                      cost_provenance=request['cost_provenance'],
                      initial_routes=best["initial_routes"] if best else [],
                      selected_seed_source=best["source"] if best else None,
                      selected_seed_proposal_id=best["proposal_id"] if best else None,
                      candidate_comparisons=candidates, connected_peers=sorted(connected),
                      settings=dict(search="PARALLEL_CHEAPEST_INSERTION + GUIDED_LOCAL_SEARCH",
                                    solver_ms=self.config.solver_ms, span_coefficient=self.config.span_coefficient,
                                    drop_penalty=self.config.drop_penalty, task_arc_offset=self.config.task_arc_offset,
                                    max_route_length=2_147_483_647, time_windows=False),
                      execution=dict(mode=mode, applied_seconds=time.perf_counter(), **execution),
                      hierarchy=request["hierarchy"],
                      node_navigation_cell_ids=depots + (request["hierarchy"]["navigation_cells"]
                                                        if request["hierarchy"] else targets),
                      node_kinds=["depot"]*self.config.robots +
                                 ["region" if request["hierarchy"] else "frontier_cell"]*len(targets),
                      result=result, team_routes_cell_ids=full_routes,
                      previous_selected_tasks=before, selected_own_tasks=selected,
                      peer_seed_used=peer_seed, peer_hint_changed_own=hint_changed_own,
                      solver_retained_peer_own_row=retained, direct_peer_route_switch=direct_switch)
        summary = {key: record[key] for key in ["call_id", "tick", "robot_id", "selected_seed_source",
                                               "peer_seed_used", "peer_hint_changed_own",
                                               "solver_retained_peer_own_row", "direct_peer_route_switch"]}
        summary.update(target_count=len(targets), frontier_count=len(frontier_ids), objective=result["objective"],
                       dropped=robot.last_dropped, selected_own_tasks=selected)
        self.plan_summaries.append(summary)
        if self.journal:
            self.journal.call(record)
        self.event("plan_selected", **summary)
        if peer_seed:
            self.event("peer_seed_used", receiver=robot_id, sender=best["source"],
                       source_proposal_id=best["proposal_id"], own_route_retained=retained,
                       receiver_route_changed=before != selected)
        if direct_switch:
            event = dict(event_number=len(self.switches)+1, tick=self.tick, receiver=robot_id,
                         sender=best["source"], source_proposal_id=best["proposal_id"],
                         receiver_call_id=call_id, previous_tasks=before, new_tasks=selected,
                         own_candidate_tasks=[node_cells[n] for n in own_candidate["initial_routes"][robot_id]],
                         peer_candidate_tasks=seed_cells,
                         own_cost=[own_candidate["dropped_count"], own_candidate["longest_route_cost"]],
                         peer_cost=[best["dropped_count"], best["longest_route_cost"]])
            self.switches.append(event)
            self.event("direct_peer_route_switch", **{k: v for k, v in event.items() if k != "tick"})
        return record

    def search_arguments(self, request):
        best = request["best"]
        return request["matrices"], self.config, best["initial_routes"] if best else None

    def plan(self, robot):
        """Single-robot synchronous path, also retained for direct inspections."""
        request = self.prepare_plan(robot)
        try:
            outcome = run_search(*self.search_arguments(request))
        except AssertionError as exc:
            self.event("objective_verification_failure", robot_id=robot.robot_id, error=str(exc))
            raise
        return self.finish_plan(robot, request, outcome, "sequential")

    def plan_all(self):
        """Freeze all inputs, overlap searches, then apply all results before moving."""
        if self.config.planning_mode == "sequential" or len(self.robots) == 1:
            return [self.plan(robot) for robot in self.robots]
        requests = [self.prepare_plan(robot) for robot in self.robots]
        started = time.perf_counter()
        active = [request for request in requests if request["targets"]]
        if active and self._pool is None:
            # Separate processes avoid Python callback/GIL contention and Tk state.
            # Persistent workers amortize their startup cost across planning cycles.
            self._pool = ProcessPoolExecutor(max_workers=self.config.robots,
                                             mp_context=multiprocessing.get_context("spawn"))
        self.event("planning_batch_started", mode="parallel",
                   robot_ids=[r.robot_id for r in self.robots], snapshot_tick=self.tick)
        futures = {request["robot_id"]: self._pool.submit(run_search, *self.search_arguments(request))
                   for request in active}
        outcomes = []
        try:
            for request in requests:
                robot_id = request["robot_id"]
                try:
                    outcome = (futures[robot_id].result() if robot_id in futures else
                               run_search(*self.search_arguments(request)))
                except AssertionError as exc:
                    self.event("objective_verification_failure", robot_id=robot_id, error=str(exc))
                    raise
                outcomes.append(outcome)
        except BaseException:
            for future in futures.values():
                future.cancel()
            self.close()
            raise
        # Barrier: no plan is applied, published or executed until all jobs return.
        records = [self.finish_plan(robot, request, outcome, "parallel")
                   for robot, request, outcome in zip(self.robots, requests, outcomes)]
        self.event("planning_batch_finished", mode="parallel", elapsed_seconds=time.perf_counter()-started,
                   searches=[dict(robot_id=r.robot_id, **outcome[1])
                             for r, outcome in zip(self.robots, outcomes)])
        return records

    def close(self):
        """Release workers on completion, window closure, interruption or failure."""
        if self._pool is not None:
            pool, self._pool = self._pool, None
            pool.shutdown(wait=True, cancel_futures=True)

    def plan_local(self, robot):
        record = self.local_planner.plan(robot, self.tick)
        if self.stateful:
            record["region_states_before"] = {str(i): int(robot.region_knowledge.regions[i].status)
                                              for i in record["local_window_region_ids"]}
        robot.local_calls += 1
        robot.local_plan_id = f"robot{robot.robot_id}_cycle{self.tick:06d}_local{robot.local_calls:06d}"
        robot.last_local_plan_tick = self.tick
        record["local_plan_id"] = robot.local_plan_id
        robot.local_viewpoints = record["selected_viewpoints"].copy()
        robot.local_navigation_end = record["end_cell"]
        robot.local_coverage_complete = record["local_coverage_complete"]
        robot.path = record["route"]["execution_path"].copy()
        if record.get('lookahead'):
            lookahead=record['lookahead']
            robot.lookahead_point=tuple(lookahead['point'])
            robot.lookahead_direction=tuple(lookahead['direction'])
            robot.lookahead_valid=lookahead['valid']
        robot.local_exploring_regions = []
        if robot.local_viewpoints:
            following=(record['lookahead']['follow_path'] if record.get('lookahead') else robot.path)
            for cell in [record["start_cell"], *following]:
                region = self.world.region_of(cell)
                if not robot.local_exploring_regions or region != robot.local_exploring_regions[-1]:
                    robot.local_exploring_regions.append(region)
                if len(robot.local_exploring_regions) >= 3:
                    break
        self.update_local_region_state(robot)
        if record.get('coverage_model') in ('mtare_samples','mtare_dense_viewpoints','simple_frontier'):
            record['region_coverage_evidence'] = {str(i):asdict(evidence)
                for i,evidence in robot.region_knowledge.evidence.items()}
        record["local_exploring_region_ids"] = robot.local_exploring_regions.copy()
        if self.journal:
            self.journal.local_call(record)
        self.event("local_plan_selected", robot_id=robot.robot_id, local_plan_id=robot.local_plan_id,
                   selected_viewpoints=robot.local_viewpoints.copy(), execution_path=robot.path.copy(),
                   navigation_end=robot.local_navigation_end, global_target_task=record["global_target_task"],
                   local_exploring_region_ids=robot.local_exploring_regions.copy(),
                   coverage_complete=record["local_coverage_complete"],
                   tsp_calls=record["actual_tsp_calls"],
                   expected_route_length=record["route"].get("execution_length",record["route"]["length"]),
                   planned_local_route_length=record["route"]["length"])
        return record

    def move_local(self, robot):
        current = self.world.cell_at(robot.position)
        active = ({i for i,cell in enumerate(robot.region_knowledge.regions) if cell.status in ACTIVE}
                  if self.stateful else
                  {int(self.world.regions.cell_region[c]) for c in self.world.frontier_ids(robot.known)})
        while robot.selected_tasks and self.world.regions.region(robot.selected_tasks[0]) not in active:
            task = robot.selected_tasks.pop(0)
            self.event("global_region_state_ineligible" if self.stateful else "global_region_no_longer_frontier", robot_id=robot.robot_id, task_id=task,
                       region_id=self.world.regions.region(task))
        if isinstance(self.local_planner,DenseCoveragePlanner):
            return self.move_dense_local(robot)
        if not robot.path:
            self.plan_local(robot)
        if not robot.path:
            return
        next_cell = robot.path.pop(0)
        if next_cell not in self.world.neighbors[current] or robot.known.ravel()[next_cell] != FREE:
            raise AssertionError("Local coverage path crosses an unknown, blocked or non-adjacent cell")
        if self.world.truth.ravel()[next_cell] != FREE:
            raise AssertionError("Local coverage moved into a truth obstacle")
        before = robot.position
        robot.position = self.world.xy(next_cell)
        distance = math.dist(before, robot.position)
        robot.distance += distance
        revisit = next_cell in robot.visited
        robot.visited.add(next_cell)
        robot.trajectory.append(robot.position)
        if next_cell in robot.local_viewpoints:
            robot.local_viewpoints.remove(next_cell)
        self.event("move", robot_id=robot.robot_id, from_cell=current, to_cell=next_cell,
                   position=robot.position, distance=distance, revisited=revisit,
                   active_target=robot.local_viewpoints[0] if robot.local_viewpoints else robot.local_navigation_end,
                   planning_level="local", local_plan_id=robot.local_plan_id,
                   proposal_id=robot.proposal.proposal_id if robot.proposal else None)

    def move_dense_local(self,robot):
        if not robot.path:
            self.plan_local(robot)
        if not robot.path:
            robot.movement_credit=0.0
            return
        # Keep the existing 2 m/cycle travel allowance when viewpoint
        # spacing changes. Remaining substep allowance carries over.
        budget=self.config.cell_size+robot.movement_credit
        moved=False
        while robot.path:
            node=robot.path[0];point=self.world.xy(node)
            distance=math.dist(robot.position,point)
            if distance>budget+1e-9:break
            before=robot.position
            # Viewpoint edges are axis-adjacent lattice steps; initial off-lattice pose is
            # allowed only if the short segment stays in private known-free map.
            steps=max(2,math.ceil(distance/(self.config.cell_size/10)))
            for f in np.linspace(0,1,steps+1):
                p=(before[0]+f*(point[0]-before[0]),before[1]+f*(point[1]-before[1]))
                if robot.known.ravel()[self.world.cell_at(p)]!=FREE:
                    raise AssertionError('Dense local path crosses unknown/blocked occupancy')
            old_cell=self.world.cell_at(before);next_cell=self.world.cell_at(point)
            if self.world.truth.ravel()[next_cell]!=FREE:raise AssertionError('Movement hit truth obstacle')
            robot.path.pop(0);robot.position=point;robot.distance+=distance;budget-=distance;moved=True
            revisit=next_cell in robot.visited;robot.visited.add(next_cell);robot.trajectory.append(point)
            if node in robot.local_viewpoints:robot.local_viewpoints.remove(node)
            self.event('move',robot_id=robot.robot_id,from_cell=old_cell,to_cell=next_cell,
                position=point,distance=distance,revisited=revisit,viewpoint_id=node,
                planning_level='local',local_plan_id=robot.local_plan_id,
                proposal_id=robot.proposal.proposal_id if robot.proposal else None)
        robot.movement_credit=max(0.0,budget) if robot.path else 0.0

    def move(self, robot):
        if self.config.architecture == "hierarchical":
            return self.move_local(robot)
        frontier = set(self.world.frontier_ids(robot.known))
        current = self.world.cell_at(robot.position)
        while robot.selected_tasks and (robot.selected_tasks[0] == current or
                                         robot.selected_tasks[0] not in frontier):
            task = robot.selected_tasks.pop(0)
            self.event("target_reached" if task == current else "target_no_longer_frontier",
                       robot_id=robot.robot_id, cell_id=task)
            robot.path = []
            robot.path_goal = None
        if not robot.selected_tasks:
            robot.path = []
            return
        target = robot.selected_tasks[0]
        if robot.path_goal != target or not robot.path:
            robot.path = GridGraph(self.world, robot.known).path(current, target)
            robot.path_goal = target
        if robot.path is None:
            self.event("navigation_unreachable", robot_id=robot.robot_id, target=target)
            robot.path = []
            robot.selected_tasks.pop(0)
            return
        next_cell = robot.path.pop(0)
        if next_cell not in self.world.neighbors[current]:
            raise AssertionError("Non-adjacent grid movement.")
        if robot.known.ravel()[next_cell] != FREE or self.world.truth.ravel()[next_cell] != FREE:
            raise AssertionError("Movement into unknown/blocked cell.")
        before = robot.position
        robot.position = self.world.xy(next_cell)
        distance = math.dist(before, robot.position)
        robot.distance += distance
        revisit = next_cell in robot.visited
        robot.visited.add(next_cell)
        robot.trajectory.append(robot.position)
        self.event("move", robot_id=robot.robot_id, from_cell=current, to_cell=next_cell,
                   position=robot.position, distance=distance, revisited=revisit,
                   active_target=target, proposal_id=robot.proposal.proposal_id if robot.proposal else None)

    def union_map(self):
        combined = np.full_like(self.world.truth, UNKNOWN)
        for robot in self.robots:
            mask = robot.known != UNKNOWN
            combined[mask] = robot.known[mask]
        return combined

    def stats(self):
        union = self.union_map().ravel()
        denominator = int(self.world.reachable_truth.sum())
        observed_free = int(((union == FREE) & self.world.reachable_truth).sum())
        claims = Counter(cell for robot in self.robots for cell in robot.selected_tasks)
        return dict(cycle=self.tick, explored_ratio=observed_free/denominator if denominator else 1.0,
                    explored_reachable_free_cells=observed_free, total_reachable_free_cells=denominator,
                    known_cells=int((union != UNKNOWN).sum()), total_grid_cells=self.world.size,
                    team_distance=sum(r.distance for r in self.robots),
                    per_robot_distance=[r.distance for r in self.robots],
                    per_robot_known_cells=[int((r.known != UNKNOWN).sum()) for r in self.robots],
                    coverage_samples_per_robot=[dict(observed=int(r.coverage.observed.sum()),
                        covered=int(r.coverage.covered.sum()),
                        uncovered=int((r.coverage.observed & ~r.coverage.covered).sum()))
                        if r.coverage else {} for r in self.robots],
                    vrp_calls=sum(r.calls for r in self.robots),
                    local_planning_calls=sum(r.local_calls for r in self.robots),
                    region_states_per_robot=[dict(Counter(cell.status.name for cell in r.region_knowledge.regions))
                                             if r.region_knowledge else {} for r in self.robots],
                    solver_calls=sum(1 for p in self.plan_summaries if p["target_count"]),
                    peer_seed_uses=self.event_counts["peer_seed_used"],
                    last_dropped_tasks_per_robot=[r.last_dropped for r in self.robots],
                    robot_exploration_finished=[r.exploration_finished for r in self.robots],
                    direct_peer_route_switches=len(self.switches),
                    overlapping_own_task_claims=sum(n > 1 for n in claims.values()),
                    messages_sent=self.event_counts["publish"], messages_received=self.event_counts["receive"],
                    pending_messages=len(self.queue))

    def _check_coverage_idle_completion(self):
        """Simulation-level stop; evaluator truth is never fed into planning.

        Every robot must have freshly planned successfully, with an empty own
        VRP row and no local coverage/navigation left to execute. Other rows in
        its private proposal may still allocate work to peers. This handles the
        everyone-waits-for-a-teammate case without changing those allocations.
        A solver exception propagates before this check, never counting as idle.
        """
        coverage = self.stats()["explored_ratio"]
        coverage_passed = coverage >= self.config.completion_min_coverage
        startup_passed = self.tick > self.config.completion_startup_cycles
        checks = []
        for robot in self.robots:
            # Preserve the pre-existing idle-region bookkeeping independently
            # of the observer's new coverage/termination gate.
            if (robot.calls and robot.local_calls and not robot.global_exploring_regions
                    and robot.local_coverage_complete and self.config.coverage_model == "mtare"
                    and self.config.settle_idle_almost_covered):
                settled = robot.region_knowledge.settle_idle()
                if settled:
                    self.event("idle_regions_settled", robot_id=robot.robot_id, region_ids=settled)
            fresh = bool(robot.calls and robot.local_calls and robot.proposal is not None
                         and robot.proposal.tick == self.tick
                         and robot.last_local_plan_tick == self.tick)
            own_route = (robot.proposal.routes[robot.robot_id][1:-1]
                         if robot.proposal is not None else ())
            no_vrp_tasks = not robot.selected_tasks and not own_route
            no_local_motion = not robot.local_viewpoints and not any(
                math.dist(robot.position, self.world.xy(node)) > 1e-9 for node in robot.path)
            complete = bool(fresh and startup_passed and coverage_passed and no_vrp_tasks
                            and robot.local_coverage_complete and no_local_motion)
            if complete != robot.exploration_finished:
                self.event("robot_exploration_finished", robot_id=robot.robot_id,
                           finished=complete)
            robot.exploration_finished = complete
            checks.append(dict(robot_id=robot.robot_id, fresh_plans=fresh,
                no_own_vrp_tasks=no_vrp_tasks, no_local_motion=no_local_motion,
                selected_tasks=robot.selected_tasks.copy(),
                global_region_ids=robot.global_exploring_regions.copy(),
                local_coverage_complete=bool(robot.local_coverage_complete),
                exploration_finished=complete))
        details = dict(rule="coverage_threshold_and_no_executable_work",
                       minimum_coverage=self.config.completion_min_coverage,
                       observed_coverage=coverage, coverage_passed=coverage_passed,
                       startup_cycles=self.config.completion_startup_cycles,
                       startup_passed=startup_passed, robots=checks)
        self.event("planner_completion_check", **details)
        if all(robot.exploration_finished for robot in self.robots):
            self.done, self.reason = True, "no_remaining_exploration_work"
            self.event("planner_completion_confirmed", **details)

    def check_planner_completion(self):
        """Recompute the original per-robot completion predicate after fresh plans.

        NoExploringCells means the ELIGIBLE global task list is empty, not that
        every region is COVERED or the robot's assigned route is empty. Original
        local completion can coexist with navigation points. The original >300 s
        startup guard maps to >300 logical cycles here; no wall-clock wait or
        physical timing model is introduced. Return-home remains out of scope.
        """
        if not self.stateful:
            return
        if self.config.completion_min_coverage is not None:
            self._check_coverage_idle_completion()
            return
        startup_passed = self.tick > self.config.completion_startup_cycles
        checks = []
        for robot in self.robots:
            planned = bool(robot.calls and robot.local_calls)
            no_global_tasks = not robot.global_exploring_regions
            if (planned and no_global_tasks and robot.local_coverage_complete
                    and self.config.coverage_model == "mtare"
                    and self.config.settle_idle_almost_covered):
                settled = robot.region_knowledge.settle_idle()
                if settled:
                    self.event("idle_regions_settled", robot_id=robot.robot_id, region_ids=settled)
            complete = bool(planned and startup_passed and no_global_tasks
                            and robot.local_coverage_complete)
            if complete != robot.exploration_finished:
                self.event("robot_exploration_finished", robot_id=robot.robot_id,
                           finished=complete)
            robot.exploration_finished = complete  # Recomputed, never latched.
            checks.append(dict(robot_id=robot.robot_id, planned=planned,
                global_region_ids=robot.global_exploring_regions.copy(),
                no_global_tasks=no_global_tasks,
                local_coverage_complete=bool(robot.local_coverage_complete),
                exploration_finished=complete))
        self.event("planner_completion_check", rule="mtare_no_global_tasks_and_local_complete",
                   startup_cycles=self.config.completion_startup_cycles,
                   startup_passed=startup_passed, robots=checks)
        if all(robot.exploration_finished for robot in self.robots):
            self.done, self.reason = True, "no_remaining_exploration_work"
            self.event("planner_completion_confirmed",
                       rule="mtare_no_global_tasks_and_local_complete",
                       startup_cycles=self.config.completion_startup_cycles, robots=checks)

    def step(self):
        if self.done:
            return self.stats()
        self.deliver()
        for robot in self.robots:
            changed = self.world.reveal(robot.known, robot.position)
            if changed:
                self.event("reveal", robot_id=robot.robot_id, cells=changed)
            if robot.coverage:
                robot.coverage.observe(robot.known,robot.trajectory,robot.position)
        if not self.stateful and self.stats()["explored_ratio"] >= 1.0:
            self.done, self.reason = True, "all_reachable_free_cells_observed"
        else:
            if self.tick % self.config.plan_every == 0:
                if self.stateful:
                    self.update_region_knowledge()
                self.plan_all()
                if self.config.architecture == "hierarchical":
                    for robot in self.robots:
                        self.plan_local(robot)
                self.check_planner_completion()
            if not self.done and self.tick % self.config.publish_every == 0:
                self.publish()
            if not self.done:
                for robot in self.robots:
                    self.move(robot)
        self.tick += 1
        if not self.done and self.tick >= self.config.max_cycles:
            self.done, self.reason = True, "cycle_limit"
        if self.done:
            self.close()
        metrics = self.stats()
        self.history.append(metrics)
        if self.journal:
            self.journal.metrics(metrics)
        return metrics
