#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$project_root"
profile="${1:-recording}"
case "$profile" in
  core) requirements="requirements.lock" ;;
  irsim) requirements="requirements-irsim.lock" ;;
  recording) requirements="requirements-rerun.lock" ;;
  *) echo "Usage: ./setup.sh [core|irsim|recording]" >&2; exit 2 ;;
esac
python3 -m venv .venv
.venv/bin/python -m pip install -r "$requirements"
.venv/bin/python -m pip install --no-deps -e .
.venv/bin/python -m pip check
echo "Setup complete ($profile). See README.md for run commands."
