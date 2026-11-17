# Original-style lookahead and local priority in 2D — 14 November 2026

The default now retains the previous lookahead in local route planning and uses
original-style forward/backward goal selection with local exploration priority.
The display stays as dots and straight segments. No physical controller,
curvature smoother, new ownership rule or coverage representation is introduced.

## Source mapping

Reference checkout: `57b0a183c4578cb1412c0c35326232372b60bd25`.

- LocalCoveragePlanner::GetNavigationViewPointIndices: forward boundary, robot,
  previous valid lookahead and reverse boundary are mandatory route anchors.
- LocalCoveragePlanner::SolveTSP: zero-cost dummy nodes connect the two boundary
  endpoints and force robot/previous-lookahead adjacency in the local tour.
  Costs use the original integer truncation of ten times geometric path length.
- SensorCoveragePlanner3D::GetLookAheadPoint: scan both sides of the robot,
  consider remaining coverage, line of sight, distance and prior direction;
  choose a forward or backward short-term goal. The momentum-disabled decision
  branch follows the original indoor configuration.
- GridWorld::UpdateNeighboringCellPriority: communicating lower-ID robots have
  priority in their reported local-working regions. This priority is applied
  during lookahead scanning, as in the original code. It is not a permanent
  reservation or a ban on traversing those regions.

The priority eligibility filter is commented out in the original viewpoint
manager, but priority IS used in GetLookAheadPoint. The adaptation now preserves
that distinction; no new candidate-exclusion heuristic was added.

## Execution and preserved scope

Coverage selection still reuses useful viewpoints and performs ordinary-first,
frontier-second randomized trials. The complete geometric local tour chooses the
best trial. The executor then follows a private known-free graph path to the
selected lookahead goal. That command path and its length are logged separately
from the full TSP route and its integer optimization objective.

The previous point/direction survive global replanning. Via nodes and actual
route anchors remain distinct, including when a route passes the robot or a
selected viewpoint before visiting it as an anchor. Missing/zero-length goals
hold position and retain a finite previous direction.

Unchanged: free-area sample representation, observation/coverage radii, coverage
thresholds, VRP objective, proposal comparison, region state/synchronization,
peer-completed sample suppression and the no-return-home idle-finalization rule.
No LiDAR, vehicle dynamics, pursuit, rendezvous, convoy or return-home movement.

**Grid geometry is still four-neighbour.** This task restores lookahead and local
priority; it does not add diagonal graph edges or draw artificial rounded curves.
It can improve route continuity but does not guarantee smooth paths or zero
robot overlap. Navigation boundary positions, local boundary-distance inputs,
private-map visibility and a reachable local-exit fallback for very short paths
remain explicit 2D substitutes for the physical/global navigation interfaces.
Waypoint extension and optional momentum are not enabled.

## Validation

111 tests passed. New checks cover both dummy constraints, closed tours,
continuity across global replanning, priority goal selection while retaining
transit, lower-ID priority, private-map visibility, safe holds, via/anchor typing
and invalid distances. A test compiles the original momentum-disabled direction
branch and checks 1,200 cases. The existing 2,400 original local-state cases and
200 proposal comparisons remain covered. These are procedure-level checks,
not a full ROS-system equivalence proof.

The saved-run auditor checks complete local tours and dummy matrices, actual
command paths, retained anchors, priority from received peer-work messages,
visibility, chosen direction, coverage, distances, objectives and completion.
Old runs without lookahead metadata keep their original audit interpretation.

Three robots, seed 755, 100 x 100 m, 30 obstacles, 80 ms global solver budget:

| Measurement | Full communication | 30 m communication |
|---|---:|---:|
| Planner completion | Passed | Passed |
| Observed free cells | 2384 / 2385 | 2376 / 2385 |
| Observed percentage | 99.96% | 99.62% |
| Final cycle | 461 | 457 |
| Team executed distance | 2435.25 m | 2435.85 m |
| Global planning records | 348 | 345 |
| Local planning records | 574 | 533 |
| Previous lookahead distinct from robot anchor | 233 | 221 |
| Decisions with priority-skipped nodes | 43 | 30 |
| Final dropped tasks | 0 | 0 |
| Objective verification failures | 0 | 0 |

These are individual execution checks, not proof of a general efficiency gain.
Time-limited searches and altered local route constraints can change trajectories
between runs. Completion remains planner-based; exact 100% is not required.

## Run and inspect

```bash
cd <project-root>
./run.sh --robots 3 --range full --solver-ms 80 --paused
```

Space starts/pauses; N advances one cycle. Use `--range 30` for limited direct
communication. `--no-lookahead` selects the historical fixed-start executor for
controlled comparisons, without changing the coverage representation.

Saved validation maps:

```bash
./view_results.sh comparisons/20261114_lookahead_v4/full_run
./view_results.sh comparisons/20261114_lookahead_v4/range30_run
```

Historical runs and the latest-run pointer are preserved. Modified source and
documents are backed up under `backups/before_lookahead_v4/`.
