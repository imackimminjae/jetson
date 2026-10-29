#!/usr/bin/env bash
# Short junction missions; delegates to the established guarded bias runner.
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$workspace_dir/src/virtual_control/scripts/junction_bias_sweep.py" "$@"
