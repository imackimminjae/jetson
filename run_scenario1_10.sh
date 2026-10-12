#!/usr/bin/env bash
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# Use the same supervisor and lock as scenario 3; never run scenarios concurrently.
exec "$workspace_dir/run_scenario3_10.sh" "$@" --scenario scenario1
