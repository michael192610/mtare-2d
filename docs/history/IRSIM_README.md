# M-TARE 2D in IR-SIM

Installed and tested on 16 November 2026. IR-SIM **2.12.1**, official source
revision `dbfb4f980c8f1c94171bffae3f67c58ad84dbdf1`.

## Run the existing project through IR-SIM

```bash
cd <project-root>/ir_sim
./run_mtare.sh --robots 3 --range 30 --solver-ms 80 --paused
```

Space starts/pauses the live window; N advances one logical cycle. Keys 0/1/2
select a robot's private map and communication circle; A shows the observation
union. Q saves and closes. The finished window remains open for inspection.
Use `--range full` for unrestricted communication.

Headless, automatically opening the finished result **in IR-SIM**:

```bash
./run_mtare.sh --robots 3 --range 30 --solver-ms 80 --headless
```

For unattended tests, append `--no-open`. Without recording enabled, headless
mode renders only the final figure offscreen and saves no intermediate frames.

Reopen this backend's latest saved result without advancing the simulation:

```bash
./view_results.sh
```

An explicit run folder can be passed to that viewer. Results live in this
directory's `runs/`; the original project's runs and latest-run pointer are
separate. Console output does not print file locations.

The original `<project-root>/run.sh` still launches its original
viewer. Use **run_mtare.sh in this directory** to use IR-SIM.

## Optional Rerun recording and frame-by-frame inspection

Rerun SDK/viewer **0.43.2** is installed in this directory's environment.
Add `--rerun` whenever you want to record. Without that flag, recording stays off.

Record headlessly, then automatically open Rerun:

```bash
./run_mtare.sh --robots 3 --range 30 --solver-ms 80 --headless --rerun
```

Record while watching the normal live simulation:

```bash
./run_mtare.sh --robots 3 --range 30 --solver-ms 80 --paused --rerun
```

Space starts the live simulation. After closing its window, open the recording:

```bash
./view_rerun.sh
```

That command opens the latest **recorded** run, even if a later run was not
recorded. You can also pass a run directory or an explicit `.rrd` file.
Append `--no-open` to a headless recording run to save without opening a viewer.

Rerun starts paused on the `cycle` timeline. Drag the playhead or use the
timeline's previous/next event controls to inspect one cycle at a time. Space
plays/pauses. The recording contains the initial state and every logical cycle,
including the final state; individual movement substeps are not separate frames.
Replay reads saved data and does not rerun the planner.

The layout includes the IR-SIM image, private-map tabs for every robot, proposed
global routes, local paths, trajectories, finite communication circles, team
coverage and planner status. Private-map coordinates use occupancy cells;
`*` marks cached teammate positions. Each run saves `simulation.rrd` and
`rerun_recording.json`. Closing the live window or pressing Q saves a stopped
recording, too. Recordings remain local.

Recording adds rendering, compression and disk overhead, including in headless
mode. Planner algorithms and inputs are unchanged, but global searches use a
wall-time budget, so extra load can affect their results. Use unrecorded runs
for runtime measurements and keep recording settings consistent in comparisons.

See [RERUN_RECORDING.md](RERUN_RECORDING.md) for validation details and the
interrupted long test preserved for diagnosing idle unfinished work.

## What this integration does

IR-SIM's documented **external-state mode** accepts poses and velocities from
an external planner/executor. This matches the project's abstract, algorithm-only
simulation scope:

1. The shared M-TARE Simulation advances one logical cycle. It calculates
   observations, exchanges reports within the configured range, runs local
   planning/global VRP and executes its existing abstract grid movement.
2. The bridge supplies the resulting robot states/velocities to IR-SIM.
3. `env.step()` refreshes IR-SIM's geometry/spatial index, object status and
   world clock. Every IR-SIM robot pose is checked against the supplied state.
4. IR-SIM draws the live world and obstacles; the right panel presents the
   original private occupancy, cached teammate positions and planning overlays.
5. M-TARE's work-based exploration condition finishes the mission. IR-SIM's
   default goal-arrival check is deliberately not the mission stop condition.

The generated map, obstacle circles and starting positions are exported from the
existing seeded scene. **No new IR-SIM random map is sampled.** The scene is
saved as `irsim_scene.yaml` alongside each run. It is a generated export: editing
it alone is not a supported way to change the planner's map.

The simulator/recorder adapters do not replace VRP/TSP, observation/coverage
models, communication radius/delays, report repair or travel allowance. On
17 November the shared simulation's completion policy was separately updated
at the user's request: after the existing cycle >300 startup guard, stop only
when team observation is at least **91%** and every robot has a fresh empty own
VRP route, complete local coverage and no remaining local movement/viewpoints.
Passing 91% alone does not stop exploration. Solver failure remains an error.
This observer-level rule also applies to the original live/headless launchers;
it does not resolve conflicting assignments or merge private maps. The known
truth-based coverage metric is used only by the observer's stop gate and never
as planner input. It is not an original M-TARE 91% condition. See
[completion policy validation](../completion_policy_91/README.md).
IR-SIM contains A*, RVO, dash controllers
and simulated LiDAR; this bridge does not substitute them for M-TARE.

This first integration is **not a migration to IR-SIM's internal robot dynamics
or LiDAR-driven mapping**. Abstract motion and circular observations remain owned
by the project. Collision response is disabled in IR-SIM to preserve the point-
robot baseline; the existing grid motion collision checks still run. The IR-SIM
robot circle is visual. Trajectories are drawn from the original substep history
so per-cycle pose snapshots do not introduce diagonal shortcuts in the picture.
One IR-SIM second is used per logical cycle, not a calibrated physical-time model.

The display adapter accesses EnvPlot's figure/axes in the pinned source revision;
upgrading upstream requires retesting that small interface.

## Installation already performed

The official source-install instructions recommend cloning the repository and
installing it into a fresh environment:

- https://ir-sim.readthedocs.io/en/latest/get_started/install.html
- https://ir-sim.readthedocs.io/en/latest/usage/make_environment.html
- https://github.com/hanruihua/ir-sim

Layout:

```text
upstream/         Official IR-SIM source; no modifications
.venv/            Separate Python environment; original .venv is untouched
mtare_bridge.py   Live adapter importing the existing mtare_2d project
run_mtare.sh      Launcher using the correct environment
requirements.lock  All installed runtime dependency versions except editable IR-SIM
INSTALLATION.json Installed version, upstream commit and planner source hashes
```

To reproduce in this folder if the environment needs rebuilding:

```bash
git clone https://github.com/hanruihua/ir-sim.git upstream  # only if upstream is absent
git -C upstream checkout dbfb4f980c8f1c94171bffae3f67c58ad84dbdf1
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.lock -e upstream
.venv/bin/python -m pip check
.venv/bin/python -c 'import irsim; print(irsim.__version__)'
```

The environment includes the exact NumPy, Matplotlib and OR-Tools versions from
the original project's requirements.lock. Additional dependencies are installed
only here. No sudo/global Python installation is needed.

## Validation

- `pip check`: no broken requirements.
- All **120 existing M-TARE tests** passed in the new environment.
- Two bridge tests passed: exact scene export, and independent baseline/backend
  runs with recorded/replayed global search results. Every global solver input,
  private map, trajectory and region state agrees. Replay is used because the
  global searches have wall-time budgets and need not independently repeat
  bit-for-bit.
- A fresh default three-robot, 30 m, 80 ms mission finished at **cycle 453**, with
  **99.9581%** observed free cells. IR-SIM advanced **453 external steps**, with
  **zero pose mismatch**.
- The original saved-run audit passed for global objectives, costs, local
  selection/TSP/lookahead, coverage, protocol, movements and completion, including
  **3,944** verified moves and zero objective failures.
- A real Tk window test passed for selection, pause/resume and work-based finish;
  subsequent advances did not increase the completed cycle. The Tk tests used a
  small fully observed fixture with a reduced startup guard, not the full mission.
- The completed mission reopens read-only in IR-SIM without modifying its files.
  The headless-to-IR-SIM-results handoff was also tested with a short fixture.

The full run is under `validation/range30/runs/`. Validation logs are in this
directory; each run records `irsim_integration.json` and its exported scene.
The cycle-453 result above predates the 17 November completion-policy change.
With the updated policy, a new 30 m run finished at cycle **509** with **99.9581%**
observation; its independent saved-run audit passed. All **127 planner tests**
and **3 bridge/recording tests** passed. Both original and IR-SIM live windows
were checked with fully observed fixtures and stopped without subsequent cycles.

One seed is a compatibility check, not a performance benchmark or a guarantee
of exact 100% observation. This simulator switch does not change the remaining
coverage-model limitations or remove all redundant exploration.
