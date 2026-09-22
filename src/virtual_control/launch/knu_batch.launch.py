"""SIH batch launch; existing map provider or the accompanying MATLAB bridge."""

import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share = Path(get_package_share_directory('virtual_control'))
    config = LaunchConfiguration('config_file').perform(context)
    grid_source = LaunchConfiguration('grid_source').perform(context)
    if grid_source not in ('existing', 'matlab'):
        raise ValueError('grid_source must be existing or matlab')
    first_text = LaunchConfiguration('start_scenario').perform(context).strip()
    last_text = LaunchConfiguration('end_scenario').perform(context).strip()
    profile = LaunchConfiguration('route_profile').perform(context)
    if first_text or last_text:
        first, last = int(first_text or '1'), int(last_text or '21')
        if not 1 <= first <= last <= 21:
            raise ValueError('require 1 <= start_scenario <= end_scenario <= 21')
        routes = [f'scenario{i}' for i in range(first, last+1)]
    elif profile == 'all':
        routes = [f'scenario{i}' for i in range(1, 22)]
    elif profile == 'compact10':
        routes = json.loads((share/'data/knu_routes/knu_batch_profile.json').read_text())['routes']
        if len(routes) != 10 or len(set(routes)) != 10:
            raise ValueError('compact10 must contain ten distinct routes')
    else:
        raise ValueError('route_profile must be compact10 or all')
    # Keep current controller tuning while changing only batch coordination and map input.
    import yaml
    with open(config, encoding='utf-8') as stream:
        current = yaml.safe_load(stream)
    upper_current = current['upper_planner_node']['ros__parameters']
    grid_topic = ('/grid_map' if grid_source == 'matlab'
                  else upper_current.get('grid_map_topic', '/bev/occupancy_grid'))
    odom_topic = '/motive/vehicle/odom_map'
    upper = {'reset_waypoint_on_path_update': True, 'use_csv_global_path': False,
             'global_path_topic': '/navigation/global_path', 'grid_map_topic': grid_topic,
             'state_input_type': 'odom', 'odom_topic': odom_topic,
             'odom_yaw_is_orientation_z': True, 'enable_waypoint_bias': False,
             'goal_stop_distance_m': -1.0}  # Runner owns progress-aware completion.
    if grid_source == 'matlab':
        upper.update(grid_positive_is_drivable=True, grid_value_threshold=50,
                     expected_grid_frame_id='base_link')
    runner = {'routes': routes,
              'grid_topic': grid_topic, 'odom_topic': odom_topic,
              'require_matlab_ack': grid_source == 'matlab'}
    lower = {'batch_supervision': True, 'state_input_type': 'odom',
             'odom_topic': odom_topic, 'odom_yaw_is_orientation_z': True,
             'reset_on_path_change': True,
             'mavlink_enable': (LaunchConfiguration('mavlink_enable').perform(context).lower()
                                == 'true'),
             'pixhawk_output_backend': (
                 LaunchConfiguration('pixhawk_output_backend').perform(context))}
    result_dir = LaunchConfiguration('results_directory').perform(context)
    if result_dir:
        runner['results_directory'] = result_dir
    return [
        Node(package='virtual_control', executable='px4_ekf_bridge.py',
             name='px4_ekf_bridge', parameters=[config], output='screen',
             condition=IfCondition(LaunchConfiguration('enable_px4_bridge'))),
        Node(package='virtual_control', executable='px4_odom_map_bridge',
             name='px4_odom_map_bridge', parameters=[config, {
                 'route_name': routes[0], 'enable_batch_reanchor': True,
                 'output_topic': odom_topic, 'output_yaw_is_orientation_z': True}],
             output='screen'),
        Node(package='virtual_control', executable='upper_planner_node',
             name='upper_planner_node', parameters=[config, upper], output='screen'),
        Node(package='virtual_control', executable='lower_tracking_mpc_node',
             name='lower_tracking_mpc_node', parameters=[config, lower], output='screen'),
        Node(package='virtual_control', executable='knu_batch_runner.py',
             name='knu_batch_runner', parameters=[str(share/'config/knu_batch.yaml'), runner],
             output='screen'),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('virtual_control'))
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=str(
            share/'config/tracking_control_split.yaml')),
        DeclareLaunchArgument('grid_source', default_value='existing',
                              description='existing map provider, or matlab'),
        DeclareLaunchArgument('enable_px4_bridge', default_value='true'),
        DeclareLaunchArgument('mavlink_enable', default_value='true'),
        DeclareLaunchArgument('pixhawk_output_backend', default_value='mavlink_udp'),
        DeclareLaunchArgument('route_profile', default_value='compact10',
                              description='compact10 (default) or all; explicit start/end overrides this'),
        DeclareLaunchArgument('start_scenario', default_value=''),
        DeclareLaunchArgument('end_scenario', default_value=''),
        DeclareLaunchArgument('results_directory', default_value=''),
        OpaqueFunction(function=setup),
    ])
