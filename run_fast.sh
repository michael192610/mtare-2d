#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$project_dir"
exec "$project_dir/.venv/bin/python" -u "$project_dir/run_fast.py" "$@"
