# Selected-robot communication circle and 30 m run — 16 November 2026

The selected robot has a dashed communication-range circle on BOTH map panels.
Its color matches the robot, radius is the configured finite communication range,
and its center follows the selected robot's own current position. Selecting 0/1/2
or saved-result robot buttons changes the circle. Team view and full-range
communication omit it. Circle clipping at map boundaries is intentional.

Only Viewer.render and documentation change. Ten planning/protocol/configuration
files were byte-compared with the installed v5 repair before and after the change.
No movement, coverage, VRP, communication or stopping-rule edits were made here.

## Headless validation

Three robots, seed 755, 30 m communication, 80 ms parallel VRP, normal defaults,
no intermediate frames. The isolated test does not overwrite the user's latest run.

- Status: finished; reason: no_remaining_exploration_work.
- Finished at cycle 469; team observation 99.9581%.
- Last robot motion: cycle 464.
- Final stationary tail: 5 cycles. At cycle 465 some
  robots still had pending global regions 50/153. Completion reports were applied
  at the next global update (cycle 469), clearing those remaining task lists.
- Movement after any robot's individual finished flag: none in this run.
- Movement after team completion: 0 events; the
  last metric row and completion event agree, with no subsequent simulation cycles.
- Completion reissues: 18; mismatched ACKs rejected:
  152. The repaired protocol handled these conflicts
  rather than leaving an indefinitely idle active task.
- Full saved-run replay passed: VRP objectives/costs, local planning, coverage,
  movement, region reports and work-based completion. Objective failures: zero.

## Remaining end-phase travel

The last new team observation was at cycle 424.
Robots subsequently travelled 138.0 m
across 45 cycles while still having
private coverage or global navigation work. This is not movement after completion.
It is team-level redundancy, and the test does NOT establish that all such travel
is necessary. Eliminating it would require further review of coverage/coordination
semantics, outside this display-only change and the user's requested VRP scope.
Private maps remain separate; completion does not require exactly 100% observation.

## Visualization checks

Radius/center on both panels, movement-following, keyboard selection, team view,
and full-range omission passed. The existing saved-view key/button test passed.
The actual completed test run was loaded and all three robot circles checked;
five subsequent saved-view advance calls did not change the completed cycle.
The saved result was also visually inspected with Robot 1 selected.

Artifacts: comparisons/20261116_range_circle_check/run, finish_analysis.json,
algorithm_hashes.json, actual-final-circle.png and validation logs.
