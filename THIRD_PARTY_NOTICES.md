# Attribution and license provenance

This repository is a Python research adaptation. It does not vendor upstream
M-TARE, EM, IR-SIM, OR-Tools or Rerun distributions. Dependencies are installed
separately. No blanket license for this adaptation has been selected on the
researcher's behalf, and this notice does not relicense upstream material.

## M-TARE / TARE

- Repository: https://github.com/caochao39/mtare_planner
- Reference revision: `57b0a183c4578cb1412c0c35326232372b60bd25`.
- Referenced source attributes Chao Cao and contains copyright notices dated
  2021. The `tare_planner/package.xml` metadata declares `BSD`; the inspected
  checkout does not provide an unambiguous top-level BSD license text/variant.
- The adaptation follows selected hierarchical, coverage, routing and compact
  coordination procedures. It excludes original system components documented
  in the README. Original source is not copied into this distribution.

## Multi-Robot-EM-Exploration environment context

- Repository: https://github.com/RobustFieldAutonomyLab/Multi-Robot-EM-Exploration
- Reference revision: `e17bf2444b48b89a5f776abccb6e71e3837d599b`.
- Seeded scene generation in `mtare_2d/world.py` was adapted from the
  `MarineNavEnv2.reset()` environment. The upstream README credits the
  Distributional_RL_Navigation environment:
  https://github.com/RobustFieldAutonomyLab/Distributional_RL_Navigation
- No explicit top-level license file was found in the inspected EM checkout.
  Preserve this provenance; absence of a file is not a license grant. The EM
  planner, GTSAM/SLAM and model assets are not included.

## IR-SIM

- Repository: https://github.com/hanruihua/ir-sim
- Version 2.12.1, revision `dbfb4f980c8f1c94171bffae3f67c58ad84dbdf1`.
- MIT, copyright (c) 2022 Ruihua Han. The exact upstream license is retained in
  `LICENSES/IR-SIM-MIT.txt`. Installed from pinned official source, not vendored.

## Routing, recording and numerical dependencies

- Google OR-Tools: https://github.com/google/or-tools (Apache-2.0).
- Rerun: https://github.com/rerun-io/rerun (license terms supplied with the pinned
  SDK/viewer distribution; see that upstream package).
- NumPy, Matplotlib and other dependencies retain their own notices/licenses.
  Consult installed distributions and upstream projects for those terms.

Before granting downstream reuse rights or public redistribution, the research
owner should resolve the incomplete upstream license provenance above and
choose appropriate terms for their own contributions. Attribution alone does
not replace applicable permissions.
