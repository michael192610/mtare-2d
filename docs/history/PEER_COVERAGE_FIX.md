> Historical peer-coverage-v3 validation. Lookahead/priority restoration is
> documented in [LOOKAHEAD_RESULTS.md](LOOKAHEAD_RESULTS.md).

# Peer-completed regions and local work

Validated on 12 November 2026; applied on 14 November 2026.

## Cause

The stopped run was `runs/20261112_201755_246600_FULL_COMMS_3_ROBOTS`.
Team observation reached 2385/2385 reachable free cells at cycle 489. At the
user stop, cycle 620, yellow (robot 2) was still selecting local viewpoints.
It had acquired 347 additional private-map cells and travelled another 262.2 m.

This was not a failure to deliver all coverage messages. At yellow's cycle-616
local decision, 9 of the 10 scored frontier samples were already in regions
whose accepted local state was COVERED_BY_OTHERS. Two selected viewpoints had
positive gain exclusively from those samples.

The adapter marked viewpoints in peer-covered regions visited. However, a
viewpoint in a neighbouring active region could still earn gain for looking
into privately unknown samples in those completed regions. Thus coarse
completion was applied to candidate positions but not consistently to the
locations of the work being scored.

## Change

The default `respect_peer_coverage=true` now removes ordinary/frontier target
samples whose own region has accepted state COVERED_BY_OTHERS. Coverage masks,
reuse, marginal selection and region evidence all consume the filtered pools.
The underlying occupancy and persistent observation ledger are not changed.

This preserves M-TARE's use of coarse subspace completion for coordination while
fine maps remain private. It is an explicit correction to the 2D task adapter,
not a claim that the original C++ point-cloud code literally masks grid samples.
Original reference points are ViewPointManager::UpdateViewPointVisited,
UpdateCandidateViewPointCellStatus, PlanningEnv::UpdateCoveredArea and the
coordination manager's versioned UpdateCells procedure.

Guards:

- Only an accepted peer completion state suppresses work. Unreceived knowledge
  and simulator truth are never consulted.
- The TARGET sample's region controls suppression. A completed known-side cell
  cannot erase real work across a boundary into an uncompleted region.
- Own COVERED regions retain the original reopening procedure.
- A newer accepted EXPLORING report re-enables work; stale reports do not.
- Privately unknown cells stay unknown and cannot become traversable through
  this change. Known-free transit through completed areas remains possible.
- Planner completion is unchanged. There is no stop-at-100% shortcut.

No message transport, proposal comparator, VRP objective, state transition or
protected-local-work rule was changed. Historical profiles explicitly disable
the new adapter rule in regression fixtures.

## Replay of the actual problematic decision

The exact private observations, visited mask, coverage ledger and region snapshot
at cycle 616 were reconstructed from saved records. With suppression disabled,
replay reproduced the old frontier masks and selected viewpoints exactly.

| Local decision | Before | Fixed |
|---|---:|---:|
| Scored frontier samples | 10 | 1 |
| Selected viewpoints | 19402, 19404, 15403 | 15403 |
| Local route length | 39.6 m | 32.4 m |

The remaining target has no accepted peer-completion report and is deliberately
preserved. This is one local-decision replay, not a claim of a 7.2 m saving on
every route or of equivalent full-run trajectories.

## Validation

100 automated tests passed, including six new peer-coverage regressions and the
existing compiled-original reference checks. New tests cover actual receipt and
application of a completion packet, private-map preservation, boundary work,
newer/stale reports, cancellation of redundant reuse, ordinary samples, own-region
reopening and independence from uncommunicated maps/hidden truth.

The saved-run verifier now reconstructs accepted region states at the moment of
planning, checks the suppression metadata, and recomputes supported frontier and
ordinary samples. It also retains the prior route, objective, private-map,
message-provenance and completion checks. Historical records without the new
metadata retain their old audit interpretation.

Final full-size checks: three robots, seed 755, 100 x 100 m, 30 obstacles,
0.6 m viewpoints, 2 m occupancy, 80 ms VRP budget.

| Measurement | Full communication | 30 m communication |
|---|---:|---:|
| Planner completion | Passed | Passed |
| Observed reachable free cells | 2384 / 2385 | 2376 / 2385 |
| Actual observed percentage | 99.96% | 99.62% |
| Final cycle | 673 | 557 |
| Executed team travel | 3940.65 m | 3313.65 m |
| Global planning records | 507 | 420 |
| Local planning records | 552 | 432 |
| Calls suppressing peer-completed frontier samples | 400 | 267 |
| Final dropped tasks | 0 | 0 |
| Objective failures | 0 | 0 |

These runs do not establish a general travel-distance or runtime improvement.
Changed task sets and time-limited decentralized searches change whole-run
trajectories. Earlier checks of the same correction produced 99.83%/100% observed
coverage. Exact 100% is not guaranteed by thresholded coarse-region completion.
The percentages above are not rounded up to 100% or used by the planners.

Validation artifacts, the exact decision replay and audit results are saved at
`comparisons/20261112_peer_coverage_fix/`. Existing runs and latest-run pointer
are preserved. Source changes are backed up under
`backups/before_peer_coverage_fix/`.

Open the validated maps:

```bash
./view_results.sh comparisons/20261112_peer_coverage_fix/full_run
./view_results.sh comparisons/20261112_peer_coverage_fix/range30_run
```

Some motion after team 100% can remain legitimate: a robot can have unreported
local work or pending coverage/state updates. The corrected invariant is that
accepted peer-completed target samples contribute zero local exploration gain,
not that every robot immediately stops when the evaluator displays 100%.
