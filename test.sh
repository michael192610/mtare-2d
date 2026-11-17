#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$project_root"
export MPLBACKEND=Agg
export MPLCONFIGDIR="$project_root/.mplconfig"
.venv/bin/python -B -m unittest discover -s tests -v
if [[ "${1:-}" == "--all" ]]; then
  .venv/bin/python -B -m unittest discover -s tests_ir_sim -v
fi
