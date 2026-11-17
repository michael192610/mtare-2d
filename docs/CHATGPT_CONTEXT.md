# Context to give ChatGPT or a reviewer

I am developing a Python M-TARE-based 2D multi-robot exploration baseline for
research on global task allocation/routing and a future algorithm with richer
belief sharing. This is an adaptation, not the original ROS system. Please
inspect actual source before assuming fidelity or recommending changes.

The world is initially unknown, occupancy maps are private, and the planner
has hierarchical coarse-region global VRP plus detailed local coverage/TSP.
Robots share coarse exploration states, roadmap links, cached poses and team
proposals, with full or distance-limited communication. Each robot optimizes a
private team plan and executes only its own row; temporary conflicting
assignments can occur. There is no central commanding planner.

The EM example contributes environment-generation context, not its planning,
SLAM or uncertainty methodology. Motion and sensing are abstract. Physical
LiDAR/navigation, ROS/SLAM, pursuit, rendezvous, convoy and return-home are
excluded. IR-SIM uses external supplied states. Rerun optionally records every
logical cycle.

Current stopping policy: after cycle 300, at least 91% team-observed reachable
free cells plus freshly empty own VRP routes, local coverage complete and no
remaining local movement/viewpoints for every robot. This is a custom observer
rule. It can stop residual global-list disagreements; it does not repair them
or guarantee 100% coverage. The cycle limit is an incomplete stop.

Read first:

1. `README.md` and `docs/SYSTEM_OVERVIEW.md`.
2. `mtare_2d/simulation.py` (prepare_plan, finish_plan, completion).
3. `mtare_2d/regions.py`, `roadmap.py`, `vrp.py` (global task/cost/solver pipeline).
4. `mtare_2d/region_state.py` (communication/state protocol).
5. `mtare_2d/dense_coverage.py`, `coverage_bookkeeping.py`, `lookahead.py` (local planning).
6. `tests/`, `verify_run.py`, and `examples/range30/` (validation/evidence).

Our intended metrics are cycles/distance to equal coverage, success rate,
assignment conflicts and their duration, outdated targets, computation time
and communication bytes. For comparisons, keep environment, sensing, local
planning and communication constraints fixed unless explicitly varied. More
accurate beliefs can change the global problem inputs, so comparing objective
numbers from different inputs alone does not establish better exploration.

Open research questions include fidelity of task selection/cost construction
and peer route adoption, assignment consistency, and which published global
allocation/routing methods make defensible baselines. Do not assume passing
internal tests proves full original-M-TARE equivalence.
