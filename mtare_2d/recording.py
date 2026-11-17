import csv
from datetime import datetime
import importlib.metadata
import json
from pathlib import Path
import time
from zoneinfo import ZoneInfo

import numpy as np


def dump(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+"\n")


class Journal:
    def __init__(self, root, config):
        self.root = Path(root)
        label = ("FULL_COMMS" if config.comms_range is None else f"{config.comms_range:g}m_COMMS")
        label += f"_{config.robots}_ROBOTS"
        now = datetime.now(ZoneInfo("Asia/Nicosia"))
        self.directory = self.root / "runs" / (now.strftime("%Y%m%d_%H%M%S_%f")+"_"+label)
        (self.directory / "calls").mkdir(parents=True)
        (self.directory / "frames").mkdir()
        (self.directory / "local_calls").mkdir()
        self.root.mkdir(parents=True, exist_ok=True)
        (self.root / "latest_run.txt").write_text(str(self.directory.resolve())+"\n")
        dump(self.directory / "config.json", dict(config.to_dict(), started_at=now.isoformat(),
             implementation_revision="mtare_2d_region_report_repair_v5",
             packages={n: importlib.metadata.version(n) for n in ["numpy", "matplotlib", "ortools"]}))
        self.events = (self.directory / "events.jsonl").open("w", buffering=1)
        self.csv = (self.directory / "metrics.csv").open("w", newline="", buffering=1)
        self.writer = None
        self.start = time.monotonic()
        self.finished = False

    def initialize(self, simulation):
        dump(self.directory / "scene.json", simulation.world.scene_dict())

    def event(self, event):
        self.events.write(json.dumps(event, allow_nan=False)+"\n")

    def call(self, record):
        dump(self.directory / "calls" / (record["call_id"]+".json"), record)

    def local_call(self, record):
        dump(self.directory / "local_calls" / (record["local_plan_id"]+".json"), record)

    def metrics(self, metrics):
        flat = {k: json.dumps(v) if isinstance(v, list) else v for k, v in metrics.items()}
        if self.writer is None:
            self.writer = csv.DictWriter(self.csv, fieldnames=list(flat))
            self.writer.writeheader()
        self.writer.writerow(flat)

    def finish(self, simulation, status="finished", reason=None):
        if self.finished:
            return
        self.finished = True
        metrics = simulation.stats()
        summary = dict(status=status, reason=reason or simulation.reason,
                       elapsed_wall_seconds=time.monotonic()-self.start, final=metrics,
                       objective_verification_failures=simulation.event_counts["objective_verification_failure"],
                       metric_note="Light blue means observed free cells, not necessarily physically visited. "
                                   "Coverage denominator is truth-reachable free cells; used by evaluation and "
                                   "the optional simulation-level coverage stop gate, never planner inputs. " +
                                   ("Frontier-only task generation; no ordinary covered-sample ledger. " if simulation.config.coverage_model=="simple" else
                                    "Ordinary covered samples include executed-pose and visited-viewpoint visibility. ") +
                                   "Wall time includes GUI pauses/rendering.")
        dump(self.directory / "summary.json", summary)
        dump(self.directory / "switches.json", simulation.switches)
        dump(self.directory / "plan_summaries.json", simulation.plan_summaries)
        np.savez_compressed(self.directory / "maps.npz", truth=simulation.world.truth,
                            **{f"robot_{r.robot_id}": r.known for r in simulation.robots})
        dump(self.directory / "final_planning_state.json", {str(r.robot_id):
             dict(selected_tasks=r.selected_tasks, local_viewpoints=r.local_viewpoints,
                  local_path=r.path, local_navigation_end=r.local_navigation_end,
                  local_plan_id=r.local_plan_id, local_exploring_regions=r.local_exploring_regions,
                  global_exploring_regions=r.global_exploring_regions,
                  local_coverage_complete=r.local_coverage_complete,
                  exploration_finished=r.exploration_finished,
                  lookahead_point=r.lookahead_point,lookahead_direction=r.lookahead_direction,
                  lookahead_valid=r.lookahead_valid,
                  cached_positions={str(i):peer.position for i,peer in r.peers.items()})
             for r in simulation.robots})
        dump(self.directory / "region_states.json", {str(r.robot_id):
             r.region_knowledge.snapshot() if r.region_knowledge else [] for r in simulation.robots})
        dump(self.directory / 'roadmaps.json', {str(r.robot_id):
             r.roadmap.snapshot() if r.roadmap else [] for r in simulation.robots})
        dump(self.directory / 'coverage_bookkeeping.json', {str(r.robot_id):
             r.coverage.snapshot() if r.coverage else {} for r in simulation.robots})
        dump(self.directory / "trajectories.json", {str(r.robot_id): r.trajectory for r in simulation.robots})
        labels = ["MTARE 2D — RESULTS", "", f"Status: {status}; reason: {summary['reason']}",
                  f"Planning architecture: {simulation.config.architecture}",
                  f"Region state model: {simulation.config.region_state_model}",
                  f"Coverage model: {simulation.config.coverage_model}",
                  f"Per-robot exploration finished: {metrics['robot_exploration_finished']}",
                  f"Completion startup guard: cycle > {simulation.config.completion_startup_cycles}",
                  f"Completion minimum observed fraction: {simulation.config.completion_min_coverage}",
                  f"Region-state counts per robot: {metrics['region_states_per_robot']}",
                  f"Local coverage calls: {metrics['local_planning_calls']}",
                  ("Frontier-only coverage: no ordinary sample ledger." if simulation.config.coverage_model=="simple" else
                   f"Ordinary coverage samples per robot: {metrics['coverage_samples_per_robot']}"),
                  f"Cycles executed: {simulation.tick}", f"Robots: {simulation.config.robots}",
                  f"Observed reachable free cells: {metrics['explored_reachable_free_cells']} / "
                  f"{metrics['total_reachable_free_cells']} ({100*metrics['explored_ratio']:.2f}%)",
                  f"Combined executed travel distance: {metrics['team_distance']:.2f} m",
                  f"Per-robot distance (m): {[round(d, 2) for d in metrics['per_robot_distance']]}",
                  f"Planning calls: {metrics['vrp_calls']}; actual OR-Tools calls: {metrics['solver_calls']}",
                  f"Accepted peer initial-route seeds: {metrics['peer_seed_uses']}",
                  f"Direct peer-route selection switches: {len(simulation.switches)}",
                  f"Dropped targets in each robot's most recent plan: {metrics['last_dropped_tasks_per_robot']}",
                  f"Messages published: {metrics['messages_sent']}; accepted deliveries: {metrics['messages_received']}",
                  "", "WHAT THESE RESULTS MEAN",
                  "Light blue cells have been revealed as free, locally or through accepted messages.",
                  "In default stateful mode detailed occupancy is private; discovery comes from own poses.",
                  ("Simplified model explores reachable frontiers; no ordinary coverage ledger is used." if simulation.config.coverage_model=="simple" else
                   "Ordinary 2D samples use persistent executed-pose and visited-viewpoint coverage."),
                  "Planned viewpoint masks do not commit actual coverage. Frontier samples are separate.",
                  "This is distinct from a robot physically visiting every cell.",
                  ("Completion: minimum team coverage reached, every fresh own VRP route empty, and no local work/motion."
                   if simulation.config.completion_min_coverage is not None else
                   "Historical completion: every robot has no eligible global tasks and local coverage is complete,"),
                  f"after cycle > {simulation.config.completion_startup_cycles}; return-home is excluded.",
                  "The coverage-gated observer stop is an adaptation rule, not an original M-TARE 91% condition.",
                  "Idle pending almost-covered regions may settle only with no global/local work and low local evidence.",
                  "100% observed coverage alone never stops the default stateful hierarchy.",
                  "Legacy flat/frontier modes retain the 100% observation exit. See the recorded reason.",
                  "Planner completion is not proof of 100% coverage; the measured percentage is kept.",
                  "It does not mean every robot has received all final updates or agreed on one plan.",
                  "A route switch means selection of a retained peer task order, not proof of full execution.",
                  "A move event records executed motion and identifies its active proposal and target.",
                  "Planning costs include the configured task-to-task offset; they are not pure metres.",
                  "This is an algorithm adaptation, not the original full M-TARE autonomy system.",
                  "", "FILES", "final.png: final visualization; frames/: intermediate snapshots.",
                  "calls/: global VRP inputs and outputs; node_kinds distinguishes depot cells from region tasks.",
                  "local_calls/: viewpoint coverage gains, reuse, sampled selections, local TSP and execution path.",
                  "region_states.json: each robot's final exploration states, actors, versions and local counters.",
                  "roadmaps.json: geometric region connections and local/received provenance per robot.",
                  "coverage_bookkeeping.json: persistent observed/covered ordinary samples per robot.",
                  "local_calls/: separate ordinary/frontier lists, queues, masks and selection traces.",
                  "Default stateful messages share compact global knowledge, never detailed fine occupancy.",
                  "Region task key = fine_grid_size + region_id; depot IDs are fine cells.",
                  "Dense local viewpoint IDs have a separate namespace; scene.json records their geometry.",
                  "events.jsonl: publication, acceptance, map updates, selection and movement events.",
                  "metrics.csv: per-cycle measurements; trajectories.json: actual robot traces.",
                  "scene.json/config.json: scenario and settings; maps.npz: final individual grids.",
                  "SWITCHES.txt/switches.json: strict direct peer-route selection switches.",
                  "", "PROGRESS", " Cycle   Observed_%   Team_distance_m   Planning_calls   Peer_switches"]
        for row in simulation.history:
            labels.append(f"{row['cycle']:6d}   {100*row['explored_ratio']:10.2f}   {row['team_distance']:15.2f}"
                          f"   {row['vrp_calls']:14d}   {row['direct_peer_route_switches']:13d}")
        (self.directory / "RESULTS.txt").write_text("\n".join(labels)+"\n")
        lines = ["DIRECT PEER-ROUTE SELECTION SWITCHES", "",
                 "Definition: a received proposal is chosen over a different adapted own candidate;",
                 "OR-Tools accepts that seed and retains the original peer-proposed own task order;",
                 "the receiver's selected task sequence changes. This is not a physical-completion claim.", ""]
        if not simulation.switches:
            lines.append("No switches satisfying this strict definition occurred in this run.")
        for event in simulation.switches:
            lines.extend([f"EVENT {event['event_number']} — cycle {event['tick']}: robot {event['receiver']} "
                          f"selected robot {event['sender']}'s proposal",
                          f"Source proposal: {event['source_proposal_id']}",
                          f"Previous selected tasks: {event['previous_tasks']}",
                          f"Own adapted candidate: {event['own_candidate_tasks']}; "
                          f"cost [dropped, longest] = {event['own_cost']}",
                          f"Peer adapted candidate: {event['peer_candidate_tasks']}; "
                          f"cost [dropped, longest] = {event['peer_cost']}",
                          f"New selected tasks: {event['new_tasks']}",
                          f"Full calculation: calls/{event['receiver_call_id']}.json", ""])
        (self.directory / "SWITCHES.txt").write_text("\n".join(lines)+"\n")
        self.events.close()
        self.csv.close()
