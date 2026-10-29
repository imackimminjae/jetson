#!/usr/bin/env bash
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$workspace_dir/install/setup.bash"
export ROS_DOMAIN_ID=1
exec python3 "$workspace_dir/src/virtual_control/scripts/duty_calibration.py" "$@"
