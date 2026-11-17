> Historical alignment-v2 results. The current peer-coverage correction and
> validation are documented in [PEER_COVERAGE_FIX.md](PEER_COVERAGE_FIX.md).

# M-TARE 2D alignment v2 — 12 November 2026

The default now follows M-TARE's retained planning procedures over planar
coverage evidence. The EM scene and map-view layout remain; no EM method,
estimator, uncertainty objective or controller is used.

## Changes

| Area | Previous default | New default |
|---|---|---|
| Local evidence | Frontier-only | Persistent ordinary coverage plus separate frontier samples |
| Local selection | One frontier trial | Ordinary-first/frontier-second; ten trials when ordinary work exists |
| Region decisions | Reachable frontier present/absent | Selection, coverage thresholds, almost-covered, visits and reopening guards |
| Viewpoint spacing | 2 m | 0.6 m, independent of 2 m occupancy |
| Reopening | More than one useful viewpoint | More than ten useful viewpoints |
| Frontier comparisons | Relaxed >=1 throughout | >=1 admission; >1 gate and region evidence |
| Revision/sharing | Cycle revisions, repeated summaries | Counter revisions, typed echoes, synchronization and suppression |
| Global VRP | Original-style solver and proposals | Retained |

The earlier sample/state/protocol implementation was reused and checked rather
than replaced with another exploration method. Historical baseline tests now
select their old profile explicitly, so changing the default does not silently
change what those regression tests measure.

## Validation

94 tests passed, including new-default coverage/state/sync checks and 86 tests
of explicit historical profiles. The source-reference tests compile an unchanged
state-decision block extracted from original GridWorld::UpdateLocalCellStatus
and the original LessCost function. They check 2,400 local evidence combinations
and 200 evaluated proposal comparisons. ROS/geometry accessors are stubbed; the
full original ROS planner was not executed.

Original checkout revision: `57b0a183c4578cb1412c0c35326232372b60bd25`.

Full-size validation used three robots, seed 755, 100 x 100 m, 30 obstacles,
2 m occupancy/ordinary samples, .6 m viewpoints, 6 m regions, 30 m local horizon,
80 ms VRP budget and parallel robot searches. Runs were generated in a temporary
copy before applying the same tested algorithm to the existing project. Saved
artifacts are copied under `comparisons/20261112_mtare_alignment_v2/`.

| Measurement | Full communication | 30 m communication |
|---|---:|---:|
| Planner completion | Passed | Passed |
| Observed reachable free cells | 2385 / 2385 | 2385 / 2385 |
| Final cycle | 609 | 573 |
| Team travel (m) | 3473.85 | 3307.05 |
| Global planning records | 459 | 432 |
| Local planning records | 549 | 508 |
| Calls with ordinary work and ten trials | 28 | 18 |
| Accepted peer seeds | 147 | 73 |
| Compact publications | 1824 | 1005 |
| Accepted deliveries | 3648 | 1256 |
| Dropped tasks in final plans | 0 | 0 |
| Objective verification failures | 0 | 0 |

These are execution results for one scene, not a conclusion about which
communication setting or planner is better. Searches have wall-time limits and
can yield different routes on repeated runs. There was no matched original-3D
benchmark or general performance-equivalence experiment.

Independent saved-record replay passed both runs: 891 global calls, 1057 local
calls, 809,076 roadmap-cost entries, 11,300 executed movements, coverage ledgers,
visited masks, private occupancy, message provenance and final completion.
Each run contains its own `validation.json` and `final.png`.

Reopen the saved maps:

```bash
./view_results.sh comparisons/20261112_mtare_alignment_v2/full_run
./view_results.sh comparisons/20261112_mtare_alignment_v2/range30_run
```

## Return-home exclusion regression

An initial restored-procedure run observed 100% of reachable free cells, but
remained idle with one stale global task until the 1200-cycle cap. A peer's
almost-covered region was still inside its local horizon: original M-TARE would
finalize it upon departure, normally enabled by return-home/convoy motion.
Those motions are excluded here.

The added idle finalization requires no eligible global work and completed
local coverage. It touches only pending ACTIVE/almost-covered regions with
candidate viewpoints, no selected viewpoints, and ordinary/frontier evidence
below the completion threshold. It increments the revision, resets sync, and
logs the transition. Regions with remaining evidence or selected views are
protected. The rule never queries truth or uses the observed percentage.

The full-comms run settled five such regions in two events; the 30 m run settled
two in two events. This is a documented adaptation to excluded home motion,
not an original M-TARE decision branch. A focused regression verifies its guards.

## Remaining fidelity boundary

Ordinary samples are free-area cell centres rather than 3D surfaces. Disk
visibility has no occlusion; planar sample thresholds 1/2/1 are not equivalent
to original cloud counts. Grid paths and geometric coarse connections replace
keypose/viewpoint geometry; fixed route endpoints replace navigation anchors.
Cycles and direct range/delay replace ROS scheduling and network transport.
Return-home, pursuit, rendezvous, convoy, LiDAR and physical navigation remain
excluded. See MTARE_2D_ALGORITHM_COMPARISON.txt for the complete current mapping.
