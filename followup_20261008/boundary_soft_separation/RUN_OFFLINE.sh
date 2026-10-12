#!/usr/bin/env bash
set -eo pipefail
cd /home/imac/ros2_ws
source /home/imac/ros2_ws/install/setup.bash
TRIAL_DIR=/home/imac/ros2_ws/followup_20261008/boundary_soft_separation
export ROS_DOMAIN_ID=177 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR="$TRIAL_DIR/ros_logs"
python3 "$TRIAL_DIR/restore.py"
python3 "$TRIAL_DIR/export.py"
python3 "$TRIAL_DIR/build.py"
python3 "$TRIAL_DIR/run.py" batch
python3 "$TRIAL_DIR/run.py" history
python3 "$TRIAL_DIR/analyze.py"
python3 "$TRIAL_DIR/refine.py"
python3 "$TRIAL_DIR/run_refine.py"
python3 "$TRIAL_DIR/run_current.py"
python3 "$TRIAL_DIR/crossover.py"
python3 "$TRIAL_DIR/feasibility.py"
python3 "$TRIAL_DIR/detail.py"
python3 "$TRIAL_DIR/build_candidate.py"
python3 "$TRIAL_DIR/verify_candidate.py"
python3 "$TRIAL_DIR/build_tests.py"
"$TRIAL_DIR/candidate_tests" > "$TRIAL_DIR/tests.log" 2>&1
python3 "$TRIAL_DIR/validate.py"
python3 "$TRIAL_DIR/regression.py"
python3 "$TRIAL_DIR/plot.py"
python3 "$TRIAL_DIR/restore.py"
