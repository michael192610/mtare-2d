> Historical implementation stage. For the current default, see [README.md](README.md) and [M-TARE alignment results](MTARE_2D_ALIGNMENT_RESULTS.md).

# Fidelity step 1 — global/local hierarchy

Status: implemented and tested. Task 2 has since been authorized and completed;
see CELL_STATE_STEP_2.md. This document records the original step-1 scope.

## Structural source references

Original root: <reference-workspace>/mtare_planner/src/tare_planner/

- src/grid_world/grid_world.cpp:GetExploringCellIDsAndPositions distinguishes
  nearby exploration from global exploration subspaces.
- src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp:GlobalPlanning
  and LocalPlanning perform distinct global and local solves.
- src/local_coverage_planner/local_coverage_planner.cpp:SolveLocalCoverageProblem
  retains useful unvisited viewpoints, accounts for navigation viewpoints,
  selects additional coverage viewpoints and keeps the shortest candidate route.
- SelectViewPoint and SelectViewPointFromFrontierQueue recompute marginal
  coverage and sample among high-scoring candidates.
- SolveTSP and src/tsp_solver/tsp_solver.cpp:Solve order viewpoints using a
  path-distance matrix and PATH_CHEAPEST_ARC routing search.

New code: regions.py, local_coverage.py and hierarchical branches in simulation.py.
Abstract observations and grid geometry replace 3D point-cloud/viewpoint inputs.
One 2 m sample is not numerically equated to one original 3D surface point;
2D sample thresholds and coverage geometry are explicitly defined.

## Change

Global tasks identify 6 m regions outside a nearby 5 x 5 region window (30 x 30 m). Fine
known-free cell centres inside are local viewpoints. The global row provides a
navigation exit toward its next region. Local coverage reuses viewpoints, runs
ten seeded randomized-greedy trials, orders selections with an actual local TSP,
and executes its fine-grid path. Both planning levels are logged separately.

The earlier prototype remains selectable via --architecture flat. Earlier runs
remain readable. Original C++ M-TARE source was not changed. The pre-step-1
standalone source is archived in backups/before_hierarchy_step1/source.tar.gz.

## Scope boundary

Region eligibility is temporarily frontier-derived. M-TARE's full cell-state
machine, ownership, reopening, counters and version synchronization are later
steps. Roadmap costs, summary messages and completion/home rules also remain
pending. Pursuit, convoy and rendezvous stay excluded by user instruction.

## Local window dimensions

The default local_region_radius is now 2: the current 6 m region plus two rings,
5 x 5 regions = 30 x 30 m, clipped at map edges. This matches the original indoor
M-TARE horizontal planning extent (50 x 0.6 m). Our 2 m viewpoint spacing remains
unchanged, so there are at most 225 potential local positions, versus 2500 in
the original 0.6 m viewpoint lattice. Existing saved runs keep their recorded
window sizes. This configuration change does not advance the cell-state task.

## Evidence

The run below validated the initial 18 x 18 m window configuration.

21 tests passed. The full 3-robot 10 ms parallel run completed 100% observation
in approximately 9.6 seconds in runs/20261111_125150_288913_FULL_COMMS_3_ROBOTS.
Independent audit verified 333 global planning records, 152 peer-seed receipt
links, 357 local coverage/TSP records and 1320 actual movements. The 100% stopping
criterion is still the former evaluator rule. Timing/travel are not a controlled
comparison with the earlier flat prototype.

Desktop validation also passed: actual Tk one-cycle stepping, local viewpoint
display, robot/team switching and reopening saved hierarchical results. The
extended auditor also passed an older flat-mode run.
