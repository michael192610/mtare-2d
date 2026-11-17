CURRENT DEFAULT HAS CHANGED: the user requested simple sample counts and a
return to 2 m viewpoints, retaining 3 m visited-neighborhood suppression.
See SIMPLE_COVERAGE_COUNTS.md for current behavior. The document below records
an earlier implementation stage and its historical validation.

# Source-guided local coverage update — 12 November 2026

The default stateful 2D baseline now uses the original indoor M-TARE horizontal
viewpoint spacing and visited-viewpoint procedure. The original source was read
locally; no changes were made to original M-TARE or the retained EM project.

| Setting / procedure | Current default | Original indoor reference |
|---|---|---|
| Viewpoint spacing | 0.6 m in x/y, separate from occupancy | `viewpoint_manager/resolution_x/y: 0.6` |
| Interior local viewpoint layout | 50 × 50 across 30 × 30 m | `number_x/y: 50` |
| Own-history viewpoint exclusion | Snap own poses, mark Euclidean neighbours ≤3 m | `UpdateViewPointVisited(positions)`, `kNeighborRange: 3.0` |
| Stored visited poses | Add a pose when at least 1 m from all stored poses | `UpdateVisitedPositions()` |
| Region-dependent visited flags | COVERED_BY_OTHERS marks visited; EXPLORING_BY_OTHERS clears, then own-history mask is applied | `UpdateViewPointVisited(grid_world)` |
| Covered sample update | Executed-pose visibility and visibility from visited candidate viewpoints | `PlanningEnv::UpdateCoveredArea` |
| Coverage queues | Ordinary/frontier lists only for unvisited candidates; reuse, mask updates, ordinary first, frontier second, local TSP | Original local coverage procedure |
| Point score thresholds | 40 / 60 / 20 in weighted sample units | Original indoor raw point thresholds 40 / 60 / 20 |
| Region count thresholds | Below 1 useful viewpoint to cover; more than 10 to reopen, with existing counter guard | Original indoor 1 / 10 |
| Randomized selection | Top 3; ten trials when ordinary queue exists | Original indoor top 3 / ten trials |

## Explicit score translation

Our observations are still samples of 2 m occupancy cells, not a dense 3D surface
cloud. Copying raw thresholds 40/60/20 onto those few samples would suppress work
for a numerical reason unrelated to the original procedure.

The dense planner uses an explicit 2D area-equivalent score:

```
weight per 2 m sample = 2² / 0.3² = 44.444...
viewpoint score       = number of uncovered sample cells × sample weight
```

The 0.3 m reference comes from original indoor `kSurfaceCloudDwzLeafSize`.
This is an assumed 2D translation, not a prediction of actual point-cloud density
or proof of equivalent behaviour. Thresholds, score weights and marginal gains
are recorded in local call files. Strict comparisons are made on weighted scores
without first rounding thresholds to whole sample counts. For example, one sample
scores >20, while two are needed to reach ≥60. Source frontier gating and ordinary
continuation conditions remain separate.

An early staging experiment retained the old rounded comparisons and stalled at
99.96% until the cycle cap. That was diagnosed before installation. The final
implementation uses the weighted comparisons and passed the complete runs below.
The early result is not presented as a successful current-baseline run.

## What stays simple

The occupancy map remains 50 × 50 cells at 2 m. Global regions remain 6 m, with the
same IDs, roadmap costs, VRP objective, proposal evaluation, compact messages and
completion predicates. Viewpoints have a separate ID namespace after fine cells
and region tasks. `scene.json` records spacing, dimensions and offset. Historical
saved scenes without this metadata remain readable.

Discovery remains a 7.5 m abstract circle; ordinary sample visibility is the
existing abstract 5 m radius. Visited-candidate visibility can cover observed
samples beyond the own-pose circle, following the source decision procedure.
No LiDAR, SLAM, noise, dynamics, occlusion model, ROS, pursuit, convoy or rendezvous
was added. Sample geometry, score weighting and disk visibility still differ
from 3D surfaces/FOV/occlusion. With the disk abstraction initial ordinary work
may already be credited by nearby visited candidates, leaving frontier-driven
selection; that is not replaced with invented uncovered work.

Only known-free connected viewpoints are candidates. Visited viewpoints remain
available for transit. Planned coverage masks never reveal unknown occupancy or
commit actual covered flags. Coverage flags can also come from source-style
visited candidate evidence, including candidate viewpoints in peer-covered regions;
this does not copy detailed peer occupancy or point clouds.

Movement uses 0.6 m lattice substeps while retaining a 2 m/cycle distance allowance
with residual carry. This prevents the denser lattice from silently reducing speed
by a factor of 3.33. The initial pose may be off lattice, so its actual first
segment distance is recorded independently from the snapped-start local TSP cost.
Fine occupancy and viewpoint IDs are distinct in move records. Unchanged global
VRP and publication periods remain measured in logical cycles.

## Validation

- **70 unit/regression tests passed.** New tests cover exact spacing and 50 × 50
  window density, Euclidean visited-neighbour boundary, horizon filtering,
  peer-region visited resets, persistence, unknown-truth independence, weighted
  strict comparisons, >10 reopening, movement distance and saved-result viewing.
- Actual **Tk** checks passed for paused live stepping, three robot views, team
  view, dense local paths, saving and reopening the interactive final result.
- Full communication, three robots, **80 ms**:
  `runs/20261112_165248_840245_FULL_COMMS_3_ROBOTS`.
  Finished via planner completion at cycle **685**, observed **100%**, travelled
  **4013.85 m**, wall time **48.91 s** for this validation run.
- 30 m communication, three robots, **10 ms**:
  `runs/20261112_165420_380691_30m_COMMS_3_ROBOTS`.
  Finished via planner completion at cycle **609**, observed **100%**, travelled
  **3565.05 m**, wall time **34.79 s** for this validation run.
- Both passed independent replay of global objectives, route costs, state changes,
  private occupancy, messages, local sample lists/marginal selection, dense BFS
  routes, executed distances and final completion. The new independent mask audit
  reconstructs candidate connectivity, historical-pose masks and visited coverage
  sources from recorded events rather than trusting the saved visited lists.
  Full: 563 local masks, 473 visited coverage commits, 516 global calls, 6689 moves.
  Range: 503 local masks, 407 visited coverage commits, 459 global calls, 5941 moves.

These are functional runs with different budgets/ranges, not a controlled
comparison of range performance or proof that overlap is reduced. Multiple runs
with matched budgets are needed for an improvement claim. Global independent
plans and local shared work can still overlap; no forced consensus, exclusive
local ownership or separation penalty has been added.

## Run the current version

Normal, initially paused:

```bash
cd <project-root>
./run.sh --robots 3 --solver-ms 80 --paused
```

Headless, then interactive final results:

```bash
cd <project-root>
./run_fast.sh --robots 3 --solver-ms 80
```

Both execute the same simulation and .6 m viewpoint model. Replace 80 with 10
in either command to alter the global search budget. `--viewpoint-spacing 2`
selects the fine-cell-viewpoint reference implementation; historical numerical
settings require the corresponding old run configuration, not just this flag.

## Sources inspected

Original `config/indoor.yaml`, `ViewPointManager::UpdateViewPointVisited`,
`SensorCoveragePlanner3D::UpdateVisitedPositions`, `PlanningEnv::UpdateCoveredArea`
and `GetUncoveredArea`, `GridWorld::UpdateLocalCellStatus`, and local coverage
queue/reuse/frontier/TSP routines.

Current implementation: `viewpoints.py`, `dense_coverage.py`,
`coverage_planner.py`, `coverage_bookkeeping.py`, and `simulation.py`.
`verify_dense.py` contains independent saved-run geometry and visited-mask audits.
