"""Upper/lower control using externally published Motive state, BEV and path."""
from datetime import datetime
import json
from pathlib import Path
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo,
                            OpaqueFunction, SetEnvironmentVariable, Shutdown)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def recording_actions(context, share, base, profile, upper, lower, enabled):
    record = LaunchConfiguration('record_debug').perform(context).lower()
    if record not in ('true', 'false'):
        raise ValueError('record_debug must be true or false')
    if record == 'false':
        return []
    root = Path(LaunchConfiguration('record_root').perform(context)).expanduser().resolve()
    folder = root / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True, exist_ok=False)
    shutil.copyfile(base, folder / 'base_config.yaml')
    shutil.copyfile(profile, folder / 'model_real_config.yaml')
    # Include the actual PoseStamped input and model outputs, unlike the HILS profile.
    topics = sorted(set([
        upper['grid_map_topic'], upper['global_path_topic'],
        upper.get('pose_stamped_topic', '/motive/vehicle/pose'),
        lower.get('pose_stamped_topic', '/motive/vehicle/pose'),
        upper.get('odom_topic', '/motive/vehicle/odom_map'),
        lower.get('odom_topic', '/motive/vehicle/odom_map'),
        upper.get('upper_sparse_path_topic', '/planner/upper_path_sparse'),
        upper.get('lower_reference_path_topic', '/planner/lower_reference_path'),
        upper.get('lower_reference_path_topic', '/planner/lower_reference_path') + '/with_arclength',
        lower.get('lower_reference_path_topic', '/planner/lower_reference_path'),
        lower.get('lower_reference_path_topic', '/planner/lower_reference_path') + '/with_arclength',
        upper.get('goal_reached_topic', '/planner/goal_reached'),
        lower.get('goal_reached_topic', '/planner/goal_reached'),
        lower.get('reset_topic', '/controller/reset'), '/debug/upper_preview_reference',
        '/debug/upper_constraint_intervals', '/debug/upper_branch_event',
        '/debug/upper_miqp_trace', '/debug/upper_interval_info',
        '/debug/upper_interval_centering_state', '/debug/upper_interval_b_samples',
        '/debug/upper_stage_timing', '/debug/upper_solver_timing', '/debug/lower_mpc_trace',
        '/debug/lower_solver_time_ms', '/debug/upper_solver_time_ms',
        '/debug/control_cmd', '/control/applied_cmd',
        lower.get('virtual_cmd_topic', '/model/cmd_control'),
        lower.get('duty_topic', '/model/control/duty_cmd'),
        '/controller/reset', '/controller/stop', '/parameter_events', '/rosout',
    ]))
    (folder / 'run.json').write_text(json.dumps({
        'started': datetime.now().astimezone().isoformat(),
        'hardware_output': enabled, 'topics': topics,
        'upper_parameters': upper, 'lower_parameters': lower,
        'format': 'MCAP with lossless Zstd chunks; original BEV cells preserved',
    }, indent=2) + '\n')
    return [
        SetEnvironmentVariable('ROS_LOG_DIR', str(folder / 'node_logs')),
        LogInfo(msg=f'Automatic drive recording: {folder}'),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-s', 'mcap', '-o', str(folder / 'bag'),
                 '--storage-config-file', str(share / 'config/drive_debug_mcap.yaml'),
                 '--max-cache-size', '8388608', '--max-bag-size', '16777216',
                 '--max-bag-duration', '60', '--polling-interval', '100', *topics],
            name='model_real_recorder', output='log',
            on_exit=Shutdown(reason='Required model recording process exited')),
    ]


def launch_setup(context):
    share = Path(get_package_share_directory('virtual_control'))
    base = str(share / 'config/tracking_control_split.yaml')
    profile = LaunchConfiguration('config_file').perform(context)
    settings = yaml.safe_load(Path(profile).read_text())
    upper = settings['upper_planner_node']['ros__parameters']
    lower = settings['lower_tracking_mpc_node']['ros__parameters']
    if (upper['wheelbase'] != lower['wheelbase']
            or upper['target_speed_mps'] != lower['target_speed_mps']):
        raise ValueError('upper/lower controller geometry and speed must agree')
    upper_overrides, lower_overrides = {}, {}
    odom = LaunchConfiguration('odom_topic').perform(context).strip()
    if odom:
        upper_overrides['odom_topic'] = lower_overrides['odom_topic'] = odom
    yaw_scalar = LaunchConfiguration('odom_yaw_is_orientation_z').perform(context).strip().lower()
    if yaw_scalar:
        if yaw_scalar not in ('true', 'false'):
            raise ValueError('odom_yaw_is_orientation_z must be true or false')
        upper_overrides['odom_yaw_is_orientation_z'] = yaw_scalar == 'true'
        lower_overrides['odom_yaw_is_orientation_z'] = yaw_scalar == 'true'
    for name in ('grid_map_topic', 'global_path_topic', 'path_frame_id'):
        value = LaunchConfiguration(name).perform(context).strip()
        if value:
            upper_overrides[name] = value
    output = LaunchConfiguration('hardware_output').perform(context).lower()
    if output not in ('true', 'false'):
        raise ValueError('hardware_output must be true or false')
    enabled = output == 'true'
    lower_overrides.update(
        mavlink_enable=enabled,
        pixhawk_output_backend='mavlink_udp' if enabled else 'disabled')
    effective = yaml.safe_load(Path(base).read_text())
    effective_upper = effective['upper_planner_node']['ros__parameters']
    effective_lower = effective['lower_tracking_mpc_node']['ros__parameters']
    effective_upper.update(upper)
    effective_upper.update(upper_overrides)
    effective_lower.update(lower)
    effective_lower.update(lower_overrides)
    actions = recording_actions(context, share, base, profile,
                                effective_upper, effective_lower, enabled)
    return actions + [
        Node(package='virtual_control', executable=executable, name=name,
             parameters=[base, profile, overrides], output='screen',
             on_exit=Shutdown(reason=f'Required model node exited: {name}'))
        for name, executable, overrides in (
            ('upper_planner_node', 'upper_planner_node', upper_overrides),
            ('lower_tracking_mpc_node', 'lower_tracking_mpc_node', lower_overrides),
        )
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('virtual_control'))
    return LaunchDescription([
        DeclareLaunchArgument('config_file',
                              default_value=str(share / 'config/model_real_control.yaml')),
        DeclareLaunchArgument('odom_topic', default_value='',
                              description='External Motive Odometry: position and velocity in metres.'),
        DeclareLaunchArgument('odom_yaw_is_orientation_z', default_value='',
                              description='true: yaw scalar in orientation.z; false: quaternion.'),
        DeclareLaunchArgument('grid_map_topic', default_value='',
                              description='External BEV already converted to physical metres.'),
        DeclareLaunchArgument('global_path_topic', default_value='',
                              description='External path in the same frame/metres as Motive state.'),
        DeclareLaunchArgument('path_frame_id', default_value='',
                              description='Frame for published planner reference paths.'),
        DeclareLaunchArgument('hardware_output', default_value='false',
                              description='Enable existing MAVLink UDP vehicle output.'),
        DeclareLaunchArgument('record_debug', default_value='true',
                              description='Automatically save BEV, state, paths and control outputs.'),
        DeclareLaunchArgument('record_root', default_value=str(Path.cwd() / 'model_real_runs'),
                              description='Parent directory for one recording folder per execution.'),
        OpaqueFunction(function=launch_setup),
    ])
