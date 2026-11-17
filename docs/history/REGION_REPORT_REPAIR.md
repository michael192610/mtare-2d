# Region-report repair — 16 November 2026

The cycle-625 normal run reached 100% team observation but remained idle because
region 255 was COVERED at Robot 1 and active at Robots 0/2, all at revision 2.
Remote VRPs assigned it to Robot 1, which considered it finished. Strict-newer
acceptance rejected its equal-version completion; the ACK pass nevertheless
merged known-by masks, eventually suppressing the completion report.

## Scope and upstream relationship

The original C++ UpdateCells uses strictly-newer revision acceptance and protects
local EXPLORING states. Its SyncCells admits equal/newer revisions. It has no
explicit equivalent of the collision repair below. The repair reuses original
revision/resend mechanisms, but is a documented extension of that protocol.

Only report acceptance metadata/publication and ACK bookkeeping are repaired:

- A robot receiving an equal-version EXPLORING report against its own locally
  authored completion claim reissues the same claim at counter+1. This includes
  original UNSEEN/covered-to-others claims: their local UNSEEN state stays UNSEEN.
- A relayed peer-only covered label cannot create this fresh local claim.
- Ordinary strict-newer reception processes the refreshed completion. Original
  protection still rejects completion while the receiver is locally EXPLORING.
- ACKs require identical serialized meaning and revision. Receipt of the same
  number with a contradictory state is not agreement. Mismatch resets suppression;
  new revisions also clear old ACK/echo counters.
- Duplicate/stale packets do not repeatedly advance the counter.

No wire fields, map-sharing feature, coverage measurements, thresholds, greedy/TSP
procedure, VRP objective/search, roadmap geometry, motion rule, completion
predicate, startup guard, idle timeout or truth-percentage exit is added/changed.
Nine algorithm/execution files were byte-compared; hashes are in the artifacts.
Changed report propagation can change subsequent task lists and routes. Identical
trajectories across independent runs are not promised.

The repair does NOT certify that a completion claim is geometrically correct.
Existing 2D coverage/threshold/peer-suppression limitations remain. Nor does it
guarantee consensus during permanent communication loss. It addresses conflicting
version/ACK bookkeeping using existing local completion decisions.

## Validation

120 tests passed, including nine report-conflict regressions and the compiled
original local-state, proposal-comparison and lookahead-direction reference checks.
Historical fixtures explicitly disable repair; current-default regressions cover
convergence, duplicates, protected useful work, peer-only claims, matching ACKs,
revision freshness, original covered-to-others labels and unchanged completion.

A continuation reconstructed from saved final private maps, trajectories, region
snapshots, local pose/visited information, coarse roadmaps and cached proposals:

| Protocol | Start cycle | End cycle | Outcome | Additional distance |
|---|---:|---:|---|---:|
| Original rules | 625 | 665 | Still idle, region states disagree at revision 2 | 0 m |
| Repair enabled | 625 | 633 | Existing no-work predicate finishes; revision 3 agrees | 0 m |

This is a reconstruction, not a serialized resume of every original in-flight
message/solver state. Proposals restart from saved caches, packet sequences are
fresh and searches are sequential. Both continuations use the same reconstruction.
The normal Viewer.advance path also finalized at 633 and ignored five subsequent
advance calls (Agg canvas test); the direct headless stepping path also finished.

Fresh final-source runs: seed 755, three robots, default 100x100 m environment,
80 ms parallel VRP, no frames/window, all current exploration defaults unchanged.

| Communication | Finish cycle | Team observed | Distance | Completion reissues | Rejected mismatched ACKs |
|---|---:|---:|---:|---:|---:|
| Full communication | 421 | 99.6226% | 2227.05 m | 38 | 275 |
| 30 m communication | 489 | 99.9581% | 2640.45 m | 13 | 202 |

Both stopped through no_remaining_exploration_work, not the safety cap. Saved-run
replay audits verify region states, report-version refreshes, matching ACKs, private
coverage/visited evidence, local TSP/lookahead, movement, global cost matrices,
VRP objectives and completion. Old v4 region-state replay remains valid.
These are correctness checks on one environment seed, not a performance benchmark
or a guarantee of exact 100% observation.

Artifacts: comparisons/20261116_region_report_repair/full_run and range30_run,
with validation.json in each, continuation JSON, normal-viewer check, algorithm
hashes and test logs. The user's runs and latest_run.txt are not overwritten.

## Use

Normal and headless launchers share the repaired RegionKnowledge code by default.
The finish predicate remains no eligible global work AND local coverage complete
for every robot after the startup guard. To reproduce original report behavior
for a controlled comparison, add --original-region-protocol. Saved configs record
repair_region_reports and implementation_revision=mtare_2d_region_report_repair_v5.
