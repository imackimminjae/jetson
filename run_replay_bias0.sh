#!/usr/bin/env bash
# Replay recorded poses to Isaac only; never launches a vehicle controller.
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if (( $# == 0 )); then
  echo "사용법: bash $workspace_dir/run_replay_bias0.sh 시나리오번호 [--attempt 회차] [--source ekf|truth] [--delay 준비초]" >&2
  exit 2
fi
cd "$workspace_dir"
source /opt/ros/humble/setup.bash
source "$workspace_dir/install/setup.bash"
export ROS_DOMAIN_ID=1
export OPENBLAS_NUM_THREADS=1
export ROS_LOG_DIR="$workspace_dir/log/recorded_pose_replay"
exec 9>"$workspace_dir/.scenario3_repeat.lock"
if ! flock -n 9; then
  echo '다른 주행 또는 리플레이 실행기가 사용 중입니다.' >&2
  exit 1
fi
exec python3 "$workspace_dir/src/virtual_control/scripts/replay_recorded_pose.py" \
  --latest-zero "$workspace_dir/parameter_sweeps" --scenario "$@"
