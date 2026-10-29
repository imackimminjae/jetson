#!/usr/bin/env bash
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$workspace_dir/install/setup.bash"
export ROS_DOMAIN_ID=1
hardware_output=false
if [[ "${1:-}" == '--drive' ]]; then
  hardware_output=true
  shift
fi
exec ros2 launch virtual_control model_real_control.launch.py \
  hardware_output:="$hardware_output" record_root:="$workspace_dir/model_real_runs" "$@"
