# Full-communication test of the simplified 2D baseline

This is a **M-TARE-inspired hierarchical frontier/VRP baseline**, rather than an
exact port of its local surface-coverage procedure. The user authorized simplifying
coverage and region-work rules while retaining real VRP allocation and hierarchy.

## What runs now

- The 100 × 100 m scene and 2 m occupancy map are retained.
- Global tasks are 6 × 6 m exploration regions.
- Local planning operates inside the 30 × 30 m window with 2 m viewpoints.
- A reachable frontier is a known-free cell beside unknown space. Viewpoints
  score how many adjacent unknown samples they could reveal.
- Region activity is based on reachable frontier work. There is no ordinary
  covered-point ledger, weighted cloud conversion, almost-covered heuristic or
  reopening count gate in this default model.
- A robot retains its own visited-position history and the 3 m viewpoint mask.
- Local selection reuses useful viewpoints, applies marginal greedy frontier
  coverage, then orders the selected viewpoints with a local TSP.
- Global allocation still uses OR-Tools, previous/received route adaptation,
  candidate comparison, seeded solving, longest-route balancing and task dropping.
- Detailed occupancy remains private. Messages contain region summaries,
  connections, team proposals and positions. Receiving a proposal does not force
  its adoption or force the team to execute a single common plan.

The simplified state updater keeps receiver-relative peer labels for wire
compatibility. Own regions activate when reachable frontiers exist and finish
when none remain. Peer-covered regions remain covered locally until a newer
accepted peer report changes them; fresh local active work retains protection.
Outside the local window, last observed/received summaries are retained.
Connectivity controls task eligibility rather than inventing coverage evidence.

Summary synchronization is deliberately simpler than original M-TARE: current
summaries are repeated, with logically ordered cycle-based revisions. The earlier
ACK/echo suppression and counter machinery is not used in the default mode.
Known-by bits still record matching-version knowledge for communication-restricted
cost rules. A trial retaining the old suppression logic stranded an obsolete
active task; the final simpler protocol addresses that case. This is a documented
protocol adaptation, not an exact port of the original synchronization procedure.

## Verified full-comms result

Run: `runs/20261112_180426_158153_FULL_COMMS_3_ROBOTS`.
Settings: 3 robots, seed 755, full comms, 80 ms global VRP budget, parallel searches.

| Measurement | Result |
|---|---:|
| Completion reason | `no_remaining_exploration_work` |
| Recorded final cycle | 457 |
| Observed reachable free cells | 2,385 / 2,385 (100%) |
| Executed team distance | 2,702.45 m |
| Headless wall time in this run | 21.67 s |
| Global planning records / actual solver calls | 345 / 335 |
| Local planning records | 369 |
| Accepted message deliveries | 2,736 |
| Accepted peer seeds / strict task-order switches | 128 / 10 |
| Distinct physically visited cells (team union) | 996 |
| Cells physically visited by at least two robots | 135 |

Observed progress was 25% at cycle 75, 50% at 182, 75% at 267, 95% at 366, and
100% first appeared at 423. Planning continued until all robots passed their
completion checks. Blue observed cells do not mean every cell was physically
visited, and 100% did not trigger a special evaluator stop.

Pairwise shared physical cells were 31 (0/1), 27 (0/2), and 93 (1/2); these overlap
because cells visited by all three appear in multiple pair counts. While both
were moving, pairs were within 4 m for 20, 3 and 16 cycles respectively. The
longest close-moving episode was nine cycles for robots 1/2. Some shared transit
can be necessary; these counts are not a direct wasted-distance calculation.

## Tests and audits

**86 automated checks passed**, including the retained previous-model regressions
and new frontier-only tests: no ordinary ledger, frontier definition, hidden-map
independence, region activation/completion, unknown/unreachable regions, reopening,
peer-summary handling, ordered revisions, repeated publication and actual
three-robot VRP/proposal exchange.

Independent replay passed all 345 global records, 369 local records, 408,288
roadmap costs, 1,351 movement events, compact messages, private maps, frontier
pools, greedy masks, TSP lengths, versions and final completion. It independently
reconstructed 1,083 intermediate visited-mask contexts rather than trusting only
saved masks. There were zero objective-verification failures. No ordinary-ledger
update events were present.

Actual Tk checks also passed: paused live stepping, robot/team selection, saving
and reopening the interactive final result. Normal and headless use the same
simulation. Both launchers now default to 80 ms; 10 ms remains an explicit option.
The previous model remains selectable with `--coverage-model mtare`.

## Is the way the map is covered similar to M-TARE?

**The broad exploration architecture is similar:** global region allocation
provides destinations, local coverage viewpoints drive nearby exploration,
observations update region work, and messages/proposals influence replanning.
The saved map shows the robots expanding observation coverage through local tours
and transitions between regions, with overlapping parts of their trajectories.

**The local coverage criterion is different:** this baseline seeks unknown space
at frontiers, while original M-TARE uses uncovered surface and frontier evidence
and more detailed region-state rules. The test demonstrates that the simplified
hierarchy works and covered this scene; it does not establish equal trajectories,
surface coverage quality, overlap frequency or performance to the 3D original.
No matched original-M-TARE experiment was run in this task.

Some staging variants took longer or stalled before the summary protocol was
simplified. The reported result is functional validation of the final version,
not an average across random seeds or a claim of superiority over earlier drafts.

## Main differences for supervisors

| Area | Simplified 2D baseline | Original M-TARE |
|---|---|---|
| Exploration evidence | Reachable frontiers and unknown-cell gain only | Ordinary uncovered surface points plus frontier points |
| Region completion/reopening | Frontier present/absent, with peer-completion protection | Coverage-score/count, selected-viewpoint, visit and almost-covered rules |
| Local coverage | Reuse, greedy frontier selection, local TSP | Ordinary-first/frontier-second selection; repeated trials when ordinary work exists |
| Region synchronization | Repeated summaries and cycle-ordered revisions; no ACK suppression | Original revision guards, known-by masks and synchronization/suppression machinery |
| Viewpoint density | 2 m | Indoor reference 0.6 m |
| Roadmap discovery | Known-free grid paths, simpler maintenance | Keypose connectivity and additional connection handling |
| Execution timing | Parallel searches with synchronized cycle barriers | Independent robot planner scheduling |
| Completion timing | No eligible global tasks + local complete, >300 cycles, team stop | Per-robot predicate after >300 seconds, with return-home handling |

Realistic sensing, SLAM, vehicle dynamics, pursuit, convoy, rendezvous and return
home remain excluded as requested. The global VRP methodology is retained; its
frontier-generated inputs can differ substantially from original M-TARE inputs.

For a future comparison, keep map/discovery/movement/communication assumptions
matched, vary only the proposed method, and test multiple seeds/scenes. A supported
claim would be improvement over **this documented 2D hierarchical frontier/VRP
baseline**. Outperforming original M-TARE requires additional validation.

Questions for supervisors:

1. Is a frontier-only hierarchical VRP baseline adequate for our research question,
   or must we preserve M-TARE's ordinary surface-coverage objective?
2. Is simplified region synchronization acceptable for studying the proposed
   belief-sharing method, or is the original merge/ACK protocol part of the target?
3. Must planners run asynchronously, or can both methods use synchronized cycles?
