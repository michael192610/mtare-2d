# Comparison scope

Label the baseline an M-TARE-based 2D adaptation with stated exclusions.
The upstream source revisions are recorded in `source_manifest.json`.

For solver-only tests, replay identical tasks, depots, costs and initial-hint
conditions. Compare dropped tasks, total and maximum route costs, common
objective and time to a usable result under matching compute budgets.

For complete exploration tests, use paired maps/starts/seeds and matching
sensor, local planning, movement, communication and termination settings.
Measure cycles and actual distance to the same coverage milestones, coverage
curves, final coverage and failure-to-reach rates. Do not average only successful
runs without also reporting failures. Report coverage-qualified idle separately
from exhaustion of all eligible global regions.

For belief-sharing extensions, measure bytes sent/forwarded, stale target
choices and conflicting next-target duration, and compare the new method with
its sharing disabled and enabled where meaningful. Coarse sharing already
exists in this baseline; this is not a no-information-sharing baseline.

Keep Rerun disabled for runtime benchmarks. Search is wall-time-limited and
recording adds rendering/compression overhead. Simulation cycles are abstract,
not calibrated real robot seconds. Planned objective costs include offsets
and balancing terms, so report them separately from executed metres.

The sample run includes long team-observation plateaus and temporary duplicate
targets. Those are diagnostic observations, not proof that every repeated metre
is avoidable or that every overlap is an adaptation error. Check the original
global procedure before adding a new ownership/consensus mechanism to a baseline.
