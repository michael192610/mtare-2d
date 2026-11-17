# Historical task-2 notes (followed by items 9/10)

Current status: roadmap costs and compact synchronization were subsequently
implemented. Statements below about pending tasks and occupancy sharing describe
the earlier task-2 snapshot. Use MTARE_2D_ALGORITHM_COMPARISON.txt and
ROADMAP_AND_SHARING_STEPS_9_10.md for the current implementation.

# Fidelity step 2 — exploration states and supporting region messages

Status: implemented and tested. Stop here until the user authorizes task 3.

This step builds on the global/local hierarchy. The default local window remains
30 x 30 m, using 6 m regions and 2 m fine occupancy/viewpoint cells. Full comms,
parallel global searches and the configured 10 ms launcher budget are unchanged.
Pursuit, convoy and rendezvous remain excluded.

## Original source references

Root: <reference-workspace>/mtare_planner/src/tare_planner/

- include/grid_world/grid_world.h: CellStatus codes and per-cell bookkeeping.
- src/grid_world/grid_world.cpp: SetCellStatus, UpdateNeighborCells,
  UpdateLocalCellStatus, UpdateGlobalCellStatus and GetExploringCellIDsAndPositions.
- UpdateNeighboringCellPriority records lower-ID connected teammates' local
  priority. The original viewpoint filter based on that flag is commented out;
  we likewise record the flag without inventing a hard exclusion of local views.
- GetPeerRobotExploringCellIDs excludes connected peers' reported local work
  from global task lists.
- UpdateLocalExploringCellIDs reports the first three consecutive region changes
  along a local path if that path contains coverage viewpoints.
- src/coordination/multi_robot_exploration_manager.cpp: UpdateCells guards and
  receiver-relative state translations.
- ../tare_msgs/msg/Cell.msg: id, update_id, known_by_robot_ids, mobility_type,
  status. These exact fields are used by the new immutable RegionUpdate.

New main implementation: mtare_2d/region_state.py. Evidence is supplied by
LocalCoveragePlanner.evidence; state-based tasks are built in regions.py.

## Separate occupancy and exploration knowledge

Fine occupancy remains unknown/free/blocked. Each robot now also owns a distinct
table of exploration-region records, with the original numeric statuses:

| Code | State |
|---|---|
| 0 | UNSEEN |
| 1 | EXPLORING |
| 2 | COVERED |
| 3 | COVERED_BY_OTHERS |
| 4 | NOGO |
| 5 | EXPLORING_BY_OTHERS |

Records include covered_to_others, exploring/explored reporter IDs, update_id,
known-by bitmask, visit_count, exploring_count, almost_covered and priority.
These are per-robot records; this is not a shared central state table.

UNSEEN here does not mean every fine occupancy cell is unknown. The original
algorithm allows a locally UNSEEN region with candidates but no selected local
viewpoint to be marked covered_to_others. Its outgoing summary can therefore
say COVERED while its local exploration status remains UNSEEN. The flags and
underlying fine map must be inspected together.

## Transition structure retained

- UNSEEN plus selected local viewpoint -> EXPLORING.
- UNSEEN plus candidates but no selected viewpoint -> covered_to_others flag,
  without forcing the local status to COVERED.
- EXPLORING plus no selected viewpoint and sufficiently low coverage scores ->
  COVERED. Useful remaining candidates can instead mark almost_covered.
- No-candidate branches use local window visit count, a 2D visited-position
  substitute for keypose nodes, and distance to the region centre.
- EXPLORING_BY_OTHERS plus own selected viewpoint -> EXPLORING. As in the
  original branch, this transition does not itself increment update_id.
- COVERED reopens only with sufficiently high gain counts and exploring_count
  <= 2. The guard checks the prior count, so the final permitted entry can raise
  it to 3; subsequent reopenings are blocked.
- COVERED_BY_OTHERS and NOGO have no local reopening branch.
- An almost-covered region is finalized when it leaves the local window.
- Remote active regions can be marked covered if surrounded by eight covered
  neighbors or disconnected. Until task 3, connectivity uses the existing known
  fine-grid component rather than the original roadmap/keypose graphs.

The exploring counter increments only when entering an active state from a
nonactive one. The visit counter increments when a region enters the nearby
planning window, including first initialization, not each cycle or physical
fine-cell visit. Neither counter forbids transit through a region.

## Explicit 2D evidence translation

Original indoor thresholds count uncovered 3D points per viewpoint, then useful
viewpoints per region. The original configuration uses small/big point scores
40/60, frontier score >20, exploring-to-covered count <1, and covered-to-exploring
count >10. There are nominally 100 original 0.6 m viewpoint positions per 6 m
region, versus nine positions with our 2 m spacing.

Our existing abstract frontier-adjacent unknown grid samples supply the coverage
gain proxy. Default 2D thresholds are small >=1 sample, big >=2 samples, frontier
>0 samples, exploring-to-covered count <1, and covered-to-exploring count >1.
These keep the branch structure and strict/non-strict comparisons, but they are
explicit model parameters, not a claim that a grid sample equals a surface point.
They require sensitivity evaluation before research performance claims.

NOGO is supported by the state machine but is not inferred for an entire region
merely because one fine cell contains an obstacle. Mobility/boundary modeling is
outside this step. All current wire mobility codes are NONE (0).

## Messages and processing

Packets continue to contain copied fine occupancy, sender position and a complete
global proposal. They now also contain:

- region_updates: immutable RegionUpdate objects with the five Cell.msg fields.
- local_exploring_region_ids: up to three consecutive region changes along the
  chosen local coverage path, not all regions in the local window.

Outgoing summaries normalize local EXPLORING/BY_OTHERS to EXPLORING and covered
states/covered_to_others to COVERED, as in the original message construction.
Visit and exploration counters stay local; they are not transmitted.

Accepted incoming packets update the peer cache. Region summaries are processed
at the next planning update, after own local state assessment and before global
VRP construction. Range, packet-age and sequence guards remain in place.

UpdateCells behavior retained:

1. A nearby locally EXPLORING region protects its local state from peer updates.
2. Otherwise, only strictly newer update_id values are considered.
3. UNSEEN or COVERED_BY_OTHERS receiving EXPLORING -> EXPLORING_BY_OTHERS.
4. UNSEEN/EXPLORING/EXPLORING_BY_OTHERS receiving COVERED -> COVERED_BY_OTHERS.
5. Other original cases can update metadata without changing status. In
   particular, a peer's EXPLORING summary does not reopen locally COVERED.

The recorded exploring/explored peer ID is the immediate reporter, potentially
forwarding knowledge; it is not proof of an exclusive assignment or original
observer identity. Receiving a summary or local-work report never commits the
robot to that peer's complete proposal. Candidate comparison remains unchanged.

## Planning effects and inspection

Only active exploration states supply global region tasks. Local/global
classification also uses candidate availability and nearby-window visit count,
following the original branches. Fresh connected peer local-work regions are
excluded from the global task list. Covered/NOGO regions cannot supply local
coverage viewpoints, while their free cells remain traversable for navigation.

The viewer shows per-robot region-state counts under its main statistics.
Light blue still means observed fine free space, not a COVERED exploration state.
region_states.json saves every robot's complete final state table.
Calls record the eligible state inputs; local calls record region_states_before
and eligible_viewpoint_cells separately from transit candidates. Events include
window entry, state transitions, covered_to_others, applied updates and explicit
rejection reasons. Repeated equal-version or protected updates are normal, not
evidence of communication loss.

Completion update (11 November 2026): the stateful hierarchy now also finishes
when all robots consistently have no active regions or global/local work. It
reports the actual observed percentage, even below 100%. The earlier 100%
observation and cycle-limit exits remain. No fake convergence or all-covered
rewrite is performed. See README.md and item 12 of
MTARE_2D_ALGORITHM_COMPARISON.txt; original return-home behavior remains excluded.

## Validation and scope

30 tests passed, including state codes, thresholds/reopening limit, visit-window
counts, almost-covered departure, peer overwrite protection, stale versions,
receiver-relative translation, message immutability, original local-work
exclusions, state-based viewpoints, a complete small-world run and all earlier
parallel/hierarchy/viewer regressions.

Full scene, three robots, 10 ms global VRP, 30 m local horizon:

- Full comms: runs/20261111_151119_778346_FULL_COMMS_3_ROBOTS.
  100% observed, 458 cycles, 345 global planning records and 345 local plans.
- 30 m comms: runs/20261111_151318_558626_30m_COMMS_3_ROBOTS.
  100% observed, 411 global planning records and 411 local plans.

Independent verification passed global/local objectives and paths, proposal
receipt provenance, wire payload provenance, state transitions, local counters,
version guards and final state snapshots in both runs. Actual Tk stepping,
state-count display, robot/team switching and reopening also passed. Older
saved hierarchy runs remain readable and independently verifiable.

This is functional validation on selected scenes, not a proof of general
equivalence or performance superiority.

Task 3 (original roadmap/cost construction) remains pending. The later full
communication task still needs acknowledgement-based knowledge synchronization,
transmission suppression, roadmap summaries, position forwarding and complete
message scheduling semantics. The known-by mask is carried/reset for this
minimal state exchange; it is not a complete implementation of that protocol.
Persistent source backup: backups/before_cell_state_step2/source.tar.gz.
Use --region-state-model frontier to select the prior step-1 eligibility rules.
