#!/usr/bin/env bash
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# Shared lock, recorder and supervisor; profile excludes legacy scenarios 13-21.
exec "$workspace_dir/run_scenario3_10.sh" "$@" --scenario all
