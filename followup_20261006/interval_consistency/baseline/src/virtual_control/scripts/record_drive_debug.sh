#!/usr/bin/env bash
# Record controller diagnostics without subscribing to camera images.
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
printf 'Recording driving diagnostics to %s/bag (Ctrl+C to finish)\n' "$record_dir"
# Explicit whitelist keeps future image/point-cloud topics out as well.
# Keep both the actual controller reference and SIH/EKF heading diagnostics.
exec ros2 bag record -o "$record_dir/bag" \
  /motive/vehicle/odom_map \
  /px4/ekf_odom \
  /px4_ekf_bridge/status \
  /navigation/global_path \
  /planner/upper_path_sparse \
  /planner/lower_reference_path/with_arclength \
  /planner/goal_reached \
  /debug/lower_mpc_trace \
  /debug/upper_miqp_trace \
  /debug/upper_stage_timing \
  /debug/upper_solver_timing \
  /debug/upper_branch_event \
  /debug/upper_constraint_intervals \
  /debug/upper_interval_info \
  /debug/upper_interval_centering_state \
  /control/applied_cmd \
  /controller/reset \
  /bev/occupancy_grid \
  /rosout
