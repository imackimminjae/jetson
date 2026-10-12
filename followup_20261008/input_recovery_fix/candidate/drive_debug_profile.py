"""Shared essential recording profile. Preserve every received BEV frame."""
from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[1]
TOPICS = (
    '/bev/occupancy_grid',
    '/motive/vehicle/odom_map',
    '/navigation/global_path',
    '/planner/lower_reference_path/with_arclength',
    '/planner/goal_reached',
    '/debug/lower_mpc_trace',
    '/debug/upper_branch_event',
    '/debug/upper_miqp_trace',
    '/debug/upper_interval_info',
    '/debug/upper_interval_centering_state',
    '/debug/upper_stage_timing',
    '/debug/bev_source_timing',
    '/controller/reset',
    '/knu_batch/status',
    '/knu_batch/active_route',
    '/knu_batch/run_permission',
    '/knu_batch/recorder_ready',
    '/knu_batch/abort',
    '/px4/sih/odom',
    '/px4_ekf_bridge/status',
    '/parameter_events',
    '/rosout',
)
REQUIRED_TOPICS = (
    '/bev/occupancy_grid', '/motive/vehicle/odom_map',
    '/navigation/global_path', '/planner/lower_reference_path/with_arclength',
    '/debug/lower_mpc_trace', '/debug/upper_branch_event', '/knu_batch/status',
)


def recorder_command(destination):
    return ['ros2', 'bag', 'record', '-s', 'mcap', '-o', str(destination),
            '--storage-config-file', str(PACKAGE/'config/drive_debug_mcap.yaml'),
            '--max-cache-size', '8388608', '--max-bag-size', '16777216',
            '--max-bag-duration', '60', '--polling-interval', '100', *TOPICS]
