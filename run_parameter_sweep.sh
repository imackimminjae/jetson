#!/usr/bin/env bash
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$workspace_dir/install/setup.bash"
export ROS_DOMAIN_ID=1
# Use the same ownership lock as the existing single-scenario runner.
exec 9>"$workspace_dir/.scenario3_repeat.lock"
if ! flock -n 9; then
  echo '다른 반복 주행 또는 파라미터 비교가 실행 중입니다.' >&2
  exit 1
fi
exec python3 "$workspace_dir/src/virtual_control/scripts/parameter_sweep.py" "$@"
