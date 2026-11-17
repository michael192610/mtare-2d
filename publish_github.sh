#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$project_root"
repository_name="${1:-mtare-2d}"
if ! [[ "$repository_name" =~ ^[A-Za-z0-9._-]+$ ]]; then
  echo "Use a repository name without an owner, spaces or slashes." >&2; exit 2
fi
if ! command -v gh >/dev/null; then
  echo "Install the GitHub CLI first: https://cli.github.com/" >&2; exit 1
fi
if ! gh auth status --hostname github.com >/dev/null 2>&1; then
  echo "Sign in locally first: gh auth login --hostname github.com --git-protocol https --web" >&2
  exit 1
fi
if git remote get-url origin >/dev/null 2>&1; then
  echo "An origin remote already exists. Use git push after reviewing that remote." >&2; exit 1
fi
if [[ -n "$(git status --porcelain)" ]]; then
  echo "Review and commit local changes before publishing: git status" >&2; exit 1
fi
# The user's chosen visibility is private. Never silently publish publicly.
gh auth setup-git --hostname github.com
gh repo create "$repository_name" --private --source . --remote origin --push \
  --description "M-TARE-based 2D exploration baseline with optional IR-SIM and Rerun recording"
