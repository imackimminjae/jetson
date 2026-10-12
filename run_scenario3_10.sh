#!/usr/bin/env bash
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$workspace_dir/install/setup.bash"
export ROS_DOMAIN_ID=1
keep_open=false
if [[ "${1:-}" == '--keep-open' ]]; then
  keep_open=true
  shift
fi
# A lock also catches a simultaneous double click before DDS discovery completes.
exec 9>"$workspace_dir/.scenario3_repeat.lock"
if ! flock -n 9; then
  echo '반복 주행 실행기가 이미 열려 있습니다.' >&2
  exit 1
fi
result=0
python3 "$workspace_dir/src/virtual_control/scripts/repeat_drive_session.py" "$@" || result=$?
if "$keep_open"; then
  read -r -p '창을 닫으려면 Enter: ' || true
fi
exit "$result"
