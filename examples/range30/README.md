# Small example result

Seed 755, three robots, 30 m communication, 80 ms global search, with IR-SIM
and optional Rerun recording. The completed run stopped at cycle **593** with
**99.7065%** reachable-free observation and **3140.853 m** combined travel.

The PNG shows the finished scene. Config, summary and metrics are included;
the large event/solver/recording files are omitted. This compact folder is an
example for inspection, not a complete saved-run bundle: the interactive saved
viewer and `verify_run.py` require the full generated result files.

The record is one run, not an average. Runs with an 80 ms wall-time search
budget can differ even on the same map. Wall duration includes recording
overhead and is not a planner benchmark. The recorded stop uses the custom
91%-minimum plus idle-work policy, not an original fixed coverage threshold.
