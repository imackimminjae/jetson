#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=1
case "${1:-status}" in
  arm|start|stop) exec ros2 service call "/duty_calibration/$1" std_srvs/srv/Trigger '{}' ;;
  status) exec ros2 topic echo /duty_calibration/status --once --field data --full-length ;;
  *) echo "Usage: $0 [arm|start|stop|status]" >&2; exit 2 ;;
esac
