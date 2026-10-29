#!/usr/bin/env bash
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$workspace_dir"
source /opt/ros/humble/setup.bash
source "$workspace_dir/install/setup.bash"
export ROS_DOMAIN_ID=1 OPENBLAS_NUM_THREADS=1
export ROS_LOG_DIR="$workspace_dir/log/c5_solver_comparison"
export DRIVE_RUNTIME_ROS_LOG_ROOT="${DRIVE_RUNTIME_ROS_LOG_ROOT:-/dev/shm/branch_ablation_ros}"
export DRIVE_DEBUG_QOS_OVERRIDES="${DRIVE_DEBUG_QOS_OVERRIDES:-$workspace_dir/src/virtual_control/config/scenario_recording_qos.yaml}"
exec 9>"$workspace_dir/.scenario3_repeat.lock"
if ! flock -n 9; then
  echo '다른 주행 실행기가 사용 중입니다.' >&2
  exit 1
fi
exec python3 "$workspace_dir/src/virtual_control/scripts/miqp_solver_comparison.py" "$@"
