#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=1
exec ros2 topic pub --once /controller/stop std_msgs/msg/Bool '{data: true}'
