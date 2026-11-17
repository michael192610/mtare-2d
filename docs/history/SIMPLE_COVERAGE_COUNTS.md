HISTORICAL STAGE: the default has since been simplified to frontier-only
coverage. It still uses 6 m regions, 2 m viewpoints and a 3 m visited rule, but
ordinary sample bookkeeping and count-based surface decisions are not used.
See FRONTIER_BASELINE_RESULTS.md and the current comparison file. The remainder
records the preceding simple ordinary/frontier sample stage and its validation.

# Simple 2D coverage baseline — 12 November 2026

The user selected the earlier hierarchical layout: **6 m regions and 2 m
viewpoints**, with the recently added **3 m visited-neighbourhood rule retained**.
This replaces the default .6 m viewpoint layout and weighted coverage scores.

## Current settings

| Item | Simple baseline |
|---|---|
| Occupancy map | 100 × 100 m, 2 m cells |
| Global region | 6 × 6 m |
| Local window | 5 × 5 regions, 30 × 30 m nominal, clipped at boundaries |
| Viewpoint spacing | 2 m: up to 9 positions per region and 225 per interior window |
| Ordinary useful coverage | At least 1 uncovered sample |
| Strong ordinary gain | At least 2 uncovered samples |
| Frontier useful coverage / gate | At least 1 frontier sample |
| Visited neighbourhood | Viewpoint and neighbours within 3 m of snapped own-history poses |
| Cover / reopen region counts | Below 1 useful viewpoint / more than 1 useful viewpoint, with existing guards |

No runtime area weights, source cloud-leaf spacing, or 40/60/20 score conversion
remains. Selection gains are integers in sample units. The optional denser
--viewpoint-spacing .6 setting still works, but does not restore old weighted
scores or the original >10 reopening threshold automatically.

## What is retained

Global region-level OR-Tools VRP and local viewpoint coverage remain distinct.
Previous/received proposals are adapted and compared using the receiver's own
inputs before a seeded solve. Private map knowledge, compact messages, roadmap
costs, independent proposal acceptance, parallel planning and completion are
retained. There is no return to a flat frontier-only allocation algorithm.

Coverage retains persistent ordinary flags, separate frontier samples, viewpoint
reuse, marginal greedy coverage, local TSP and region-state transitions. Visited
viewpoints are excluded from coverage selection but can be used for transit.
Peer-region visited/reset rules and visited-candidate coverage credit remain.
Predicted coverage masks do not reveal unknown cells or commit actual coverage.

The original indoor count threshold >10 cannot be met with at most 9 viewpoints
per region, so the earlier adapted >1 rule is restored. This is explicitly a 2D
adaptation, not a claim that original 3D numeric settings are unchanged.
Discovery is still an abstract 7.5 m disk, and ordinary visibility a 5 m disk.
The 3 m rule concerns previously visited viewpoints, not separation from peers.

## Validation

- **74 automated checks passed.** New checks cover simple defaults, exclusion
  of neighbouring visited viewpoints, the single-frontier-sample gate, absence
  of runtime weights, and selection-order equivalence. Existing global/local
  solver, messages, region-state, completion and viewing tests also passed.
- A separate migration audit converted and checked **1,066 historical weighted
  local calls** (563 full comms, 503 range comms). At the SAME geometry and
  configuration, the count comparisons preserve those recorded choices and
  marginal-mask decisions. Historical weights remain only in migration/audit
  tools, not in the simulator.
- Returning viewpoint spacing to 2 m and adapting the reopening count can change
  tasks and routes. The above equivalence check therefore does not claim the
  final mission is identical to an older .6 m run.
- The complete final default scene, **3 robots, full comms, 80 ms**, finished
  through `no_remaining_exploration_work` at **cycle 581**, with **100% observed
  coverage**, **3381.87 m** team travel and **25.86 s** headless wall time in this
  particular validation run.
  Saved run: `runs/20261112_171525_047038_FULL_COMMS_3_ROBOTS`.
- Independent replay passed **438 global planning calls**, **494 local calls**,
  **369,072 roadmap cost entries**, **1,740 compact publications**, **1,691 moves**,
  private-map/state/coverage replay and the final completion certificate. All
  494 visited masks and 407 visited-candidate coverage credits were verified.
- Actual Tk checks passed for live paused stepping, robot/team selection, saving
  and reopening the final interactive result. The normal and headless launchers
  execute the same simulation.

These are functional checks and one final scene run, not proof of better overlap
or superior performance. Original M-TARE remains a source reference; coverage
samples, thresholds, timing and connectivity are still documented 2D adaptations.

## Run

Normal, initially paused:

```bash
cd <project-root>
./run.sh --robots 3 --solver-ms 80 --paused
```

Headless, opening interactive results afterward:

```bash
cd <project-root>
./run_fast.sh --robots 3 --solver-ms 80
```

Change 80 to 10 in either command to change only the global VRP time budget.
The three-robot scene and current 2 m/simple-count defaults remain the same.
