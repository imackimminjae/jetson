#!/usr/bin/env bash
# Essential topics, full-rate BEV, lossless MCAP/Zstd compression.
set -eo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd -- "$script_dir/../../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$workspace_dir/install/setup.bash"
export ROS_DOMAIN_ID=1
record_dir="${1:-$workspace_dir/drive_debug_$(date +%Y%m%d_%H%M%S)_light}"
if [[ -e "$record_dir" ]]; then
  printf 'Output already exists: %s\n' "$record_dir" >&2
  exit 1
fi
mkdir -p -- "$record_dir"
cp -- "$workspace_dir/src/virtual_control/config/tracking_control_split.yaml" \
  "$record_dir/config_snapshot.yaml"
printf 'Recording compressed essential diagnostics to %s/bag (Ctrl+C to finish)\n' "$record_dir"
export PYTHONPATH="$script_dir${PYTHONPATH:+:$PYTHONPATH}"
exec python3 - "$record_dir" <<'PY'
import json, os, sys
from pathlib import Path
from drive_debug_profile import TOPICS, recorder_command
root = Path(sys.argv[1])
(root/'recording_profile.json').write_text(json.dumps({
    'format': 'MCAP Zstd lossless', 'topics': TOPICS,
    'note': 'No BEV decimation. No automatic decoded.pkl duplication.'}, indent=2)+'\n')
os.execvp('ros2', recorder_command(root/'bag'))
PY
