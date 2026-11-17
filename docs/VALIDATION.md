# Repository packaging validation — 17 November 2026

This package was installed with `./setup.sh recording` in a fresh, independent
virtual environment. The exact runtime dependencies and pinned IR-SIM Git
revision installed successfully; `pip check` reported no broken requirements.

- All **127 planner tests** passed, with optional compiled checks pointed at the
  original M-TARE reference checkout using `MTARE_ORIGINAL_ROOT`.
- All **3 IR-SIM/Rerun integration tests** passed in the new environment.
- Original and IR-SIM headless CLI runs each executed eight cycles using
  three robots, 30 m communication, sequential planning and a 10 ms search.
  Both stopped at their explicit test cycle limit with 7.34% observation.
  These are portability smoke checks, not successful exploration missions.
- The IR-SIM CLI smoke test enabled Rerun, saving cycles 0–8 and its final
  state; the resulting file was checked with the installed native Rerun tool.
- SHA-256 checks show all **21 planner package modules** are byte-identical
  to the current development source. Packaging changes concern adapter paths,
  module imports, launchers, optional reference-test locations and documentation.
- No original planner/development files or original run archives were changed.

The full three-robot example predates packaging and is included only as a
compact sample. Historical validations in `docs/history/` describe their
specific development revisions and environments, not additional tests run on
this package. Original-source tests skip explicitly if their optional checkout
or g++ is absent. No equivalence to the complete original M-TARE system or
success across every seed is claimed.

Setup, test and CLI verification were performed locally before the first
commit. Generated environments, runs and logs are ignored rather than included
as source. The project-wide reuse license is not selected; see the provenance
notice for the known upstream license metadata.
