#!/usr/bin/env bash
# Broad coverage with a shortened northern return to reduce repeated travel.
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$workspace_dir/run_scenario_bias.sh" 5 4 2 8 1 10 11 12 --route-prefix 11:7 --bias 0 --runs 1 "$@"
