# System overview

## Defaults

`mtare_2d/config.py` defines the current configuration: seed 755, 100 × 100 m,
three robots, 30 circular obstacles, 2 m occupancy cells, 0.6 m viewpoint
spacing, 6 m regions, a 5 × 5-region local window, 7.5 m abstract observation
radius and 5 m ordinary coverage radius. Global planning runs every four
logical cycles. Per-search global budget is 80 ms. Communication is full by
default or a finite radius selected by `--range`.

## One simulation cycle

1. Deliver eligible delayed messages, respecting communication range and TTL.
2. Reveal observations from each robot's own pose and update actual coverage.
3. On planning cycles: update region knowledge/roadmaps, build and solve each
   robot's global problem, then select/order local coverage viewpoints.
4. Evaluate completion using fresh plans and the configured observer policy.
5. If continuing, publish compact information and execute abstract movement.
6. Save metrics/events; optional IR-SIM/Rerun consume the resulting state.

Global search workers receive costs and hints, not global truth maps. Results
are applied after all workers finish. Local/global searches have real wall-time
components, so repeated runs need not choose identical routes.

## Global planning: principal review targets

- `simulation.py:prepare_plan`: depot estimates, eligible tasks, cost matrices
  and candidate self/peer proposals.
- `regions.py:select_global_regions`: eligible region selection and navigation
  representatives from private/coarse knowledge.
- `roadmap.py:roadmap_matrices`: coarse connectivity and mobility assumptions.
- `vrp.py:adapt_proposal`, `select_seed`: refresh proposals to the current task
  set and compare candidate dropped counts/longest-route costs.
- `vrp.py:solve`: OR-Tools routing with optional initial routes; global search
  uses parallel cheapest insertion and guided local search.
- `simulation.py:finish_plan`: install the private team proposal, then execute
  only this robot's row. Other robots are not commanded by that proposal.

The solver objective is the sum of route costs plus 100 times the longest
route cost plus 9,999,999 per dropped task. Task-to-task arcs also retain the
500 offset. These cost values are not physical travel metres. Candidate-seed
comparison and the routing objective are distinct steps.

## Local planning

The current dense planner is `dense_coverage.py`, using coverage bookkeeping
from `coverage_bookkeeping.py`, a viewpoint lattice from `viewpoints.py` and
lookahead/direction logic from `lookahead.py`. Candidate viewpoints have
ordinary and frontier marginal gains. Randomized greedy trials select coverage
viewpoints; a local single-robot TSP orders them. `coverage_planner.py` and
`local_coverage.py` also retain earlier coverage implementations/profile support.
Grid pathfinding supplies connection costs/paths; it is not the whole local
coverage algorithm.

## Communication

`simulation.py` implements publishing, receipt, delays, range and cached peers;
`region_state.py` implements status/revision/acknowledgement handling. Compact
region reports, roadmaps, cached positions and proposals are exchanged.
Detailed private occupancy is not broadcast in stateful mode. A displayed
teammate pose may therefore be cached. The team-union map is an observer view.

## Rendering and recording

`viewer.py` draws the original two-panel view. `ir_sim/mtare_bridge.py` exports
the same generated scene and supplies poses/velocities to pinned IR-SIM
external-state mode. `ir_sim/rerun_recorder.py` logs every logical cycle to
`simulation.rrd`. The engine checks supplied pose agreement. Collision response
in IR-SIM is disabled; the source executor retains known-free checks and the
displayed robot radius is visual.

## Results and completion

`recording.py:Journal` stores config, events, global/local solver inputs and
outputs, maps, coverage state and metrics. `verify_run.py` independently audits
recorded global objectives, local decisions, protocol, movement and completion.
`simulation.py:_check_coverage_idle_completion` implements the current stop.
`completion_min_coverage=None` from Python selects the historical global-list
plus local-complete predicate; tests use it explicitly for historical profiles.
