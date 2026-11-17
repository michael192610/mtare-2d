#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$project_root"
exec "$project_root/.venv/bin/python" -u -m ir_sim.rerun_recorder "$@"
