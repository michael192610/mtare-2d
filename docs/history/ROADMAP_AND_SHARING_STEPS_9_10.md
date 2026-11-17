# Items 9 and 10: region roadmap and compact knowledge sharing

Implemented 11 November 2026. The original M-TARE repository is unchanged.
The existing planner-work stopping procedure remains in place, alongside its
100% observation and maximum-cycle exits. This is a homogeneous, static 2D
adaptation; it does not add ROS, sensing, SLAM, pursuit, convoy or rendezvous.

## Source mapping

Original source root: `<reference-workspace>/mtare_planner/src/tare_planner`.

| Original method | Current 2D implementation |
| --- | --- |
| GridWorld::UpdateRoadmap, HasDirectKeyposeGraphConnection | roadmap.py, Roadmap.discover: adjacent regions require a known-free local representative path <=12 m. Explicit substitute for keypose connectivity. |
| IsConnectedOnRoadmap, CellToCellDistance | Roadmap.connected/cost: incident edge test, shortest path when both connected, otherwise centre-distance fallback; disconnected components retain INF; integer truncation. |
| GetDistanceMatricesWithTraversability | roadmap_matrices: region endpoints, zero diagonal, symmetric costs, task-only offset; homogeneous NONE mobility scope. |
| GetDistanceMatricesNoComms | Disconnected-vehicle task arcs require relevant regions' known-by bits; connected peers bypass the restriction. |
| GetInitialRoute/GetInitialRoutesFromVRPSolution/AddNodeToVRPPlan | Stateful candidate adaptation retains current tasks and uses original insertion increments; removes prototype fine-grid reachability filtering. |
| GetExplorationInfo | publish: region summaries, full known edges, complete proposals, local-work IDs, positions and stamps. Fine occupancy is absent. |
| UpdateCells | apply_peer: original protection/version/status cases, sync count resets and mobility field. |
| SyncCells/SyncCellWithIDs | sync_peer: equal/newer mask merges, state-specific counts, counter reduction and suppression. |
| UpdateRoadmap (coordination manager) | Per-planning receiver merges endpoint edges and reconstructs geometric lengths; no local occupancy write. |
| GetRobots position forwarding | Newer cached position stamps update teammate estimates without direct-contact status. |
| NoExploringCells + IsLocalCoverageComplete principle | Existing stable team no-work stopping remains; does not assert true 100% observation or add original 300 s/home behavior. |

## Important details

- Fine occupancy belongs to each robot; received global edges do not reveal free
  cells. Remote tasks can therefore lack a local detailed gateway.
- A remote roadmap path supplies a next-region direction. The local planner
  chooses a reachable known-free anchor toward it. Coverage and execution remain
  the existing reuse/greedy/local-TSP procedure on private occupancy.
- Roadmap edge message `weight` encodes mobility, not length. All robots/regions
  currently use NONE/NONE, encoded 0.0. Mixed mobility is explicitly rejected.
- Original SyncRoadmap is empty. No edge acknowledgement protocol was invented;
  all known edges are included, as in the original GetExplorationInfo.
- Equal revisions can acknowledge knowledge without overwriting local state.
  Fully known cells suppress after five publications, except current global
  exploring cells. State-specific peer echo counters can lower suppression counts.
- Outgoing positions use simulation-cycle freshness in place of elapsed seconds.
  Direct message delay, range/age/sequence gates, planning cadence and synchronized
  parallel solver batches are unchanged. This is not asynchronous ROS scheduling.
- Region IDs remain aligned. Task key = fine-grid size + region ID; depots remain
  fine-cell IDs in stored routes, but their global costs use containing regions.
- No new separation penalty, hard task ownership or forced proposal acceptance
  was added. Movement together remains possible.
- Flat/frontier reference modes preserve their old occupancy-sharing/fine-cost
  implementation. The changes apply to default hierarchical/mtare mode.

## Validation

42 unit/integration tests passed, including eight targeted roadmap/protocol tests.
Full-scene tests used three robots, seed 755, 50 ms searches and parallel mode.
Both full and 30 m communication runs passed independent cost recomputation,
state transitions, proposal provenance, local TSP/path/movement checks, compact
packet checks, observed local-edge path checks and final private-map replay.
The 30 m run also passed the no-work completion certificate audit.

Full comms: 707 cycles, 100.00%, 4,235.80 m, 21 strict route switches.
30 m: 761 cycles, 99.66%, 4,219.80 m, 2 strict route switches.
See ROADMAP_AND_SHARING_RESULTS.txt and saved per-run analysis/validation files.
These are functional demonstrations, not repeated performance experiments.

## Remaining differences

Local evidence is still 2D frontier samples with adapted thresholds; viewpoint
density is 2 m. Direct roadmap connectivity uses observed 2D grid paths rather
than a real keypose graph. Heterogeneous mobility, real stuck-edge cutting,
sensor/SLAM/navigation realism and independent asynchronous execution are outside
this scope. Return-home and communication-seeking strategies remain excluded.
