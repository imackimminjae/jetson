"""Managed by run_scenario3_10.sh; runner also owns the global path publisher."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = LaunchConfiguration('config_file')
    return LaunchDescription([
        DeclareLaunchArgument('config_file', description='Prepared effective session YAML'),
        *[Node(package='virtual_control', executable=executable, name=name,
               parameters=[config], output='screen',
               on_exit=Shutdown(reason=f'Required node exited: {name}'))
          for name, executable in [
              ('px4_ekf_bridge', 'px4_ekf_bridge.py'),
              ('px4_odom_map_bridge', 'px4_odom_map_bridge'),
              ('upper_planner_node', 'upper_planner_node'),
              ('lower_tracking_mpc_node', 'lower_tracking_mpc_node'),
              ('knu_batch_runner', 'knu_batch_runner.py'),
          ]],
    ])
