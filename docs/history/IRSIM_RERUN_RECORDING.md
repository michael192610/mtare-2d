# Optional Rerun recorder — 17 November 2026

Rerun SDK and native viewer 0.43.2 are installed in the IR-SIM virtualenv.
`--rerun` enables recording; existing commands without it remain unrecorded.
The original `mtare_2d` project and IR-SIM upstream source were not modified.

Validation:

- All three bridge/recording tests pass. A fully observed small fixture finishes
  at cycle 5, with all six states (0–5) present in the actual RRD data. Repeated
  advance calls after completion add no cycles. Native `rerun rrd verify`
  validates the file and its footer. Existing files cannot be overwritten.
- A three-robot, 30 m CLI smoke run records cycles 0–3 and stops at its explicit
  cycle limit. This checks the launcher and saving, not mission completion.
- The real native desktop viewer loaded the file, private-map tabs, status,
  coverage and paused cycle timeline. Preview screenshots are
  `rerun-preview.png` and `rerun-preview-final.png`.
- Dependency validation passes. NumPy, Matplotlib and OR-Tools retain the
  original project's locked versions.
- All 120 original planner tests also pass after installing Rerun. All 21
  original planner-module hashes still match the pre-integration manifest.

A longer default 30 m run is preserved in
`runs/20261117_124842_876673_30m_COMMS_3_ROBOTS/`. It became stationary with
unfinished planner work: around cycle 808, robots reported global regions
`[279]`, `[134]`, `[134]`, but no assigned tasks. Distance remained about
2638.1 m while cycles continued. This is an unresolved planner/completion case,
not a successfully completed mission. No stopping or exploration rule was
changed to hide it.

The test process was interrupted before its final run summary was written.
Its streamed RRD was recovered using the native `rrd optimize` command into
`simulation_recovered.rrd`; the recovered file passes native footer verification
and contains 837 coverage samples. The original stream is preserved. The
latest-recording pointer initially opens this recovered test for inspection;
your next recorded run replaces that pointer.

Rerun support references:

- https://rerun.io/docs/reference/sdk/operating-modes
- https://rerun.io/docs/getting-started/configure-the-viewer

The recorded visual frame is compressed at JPEG quality 85. Private occupancy,
routes, cached poses and status are also recorded as separate inspectable data.
Logging samples every logical cycle, not every grid movement substep. Search
results can vary with added recording overhead because of wall-time budgets.
