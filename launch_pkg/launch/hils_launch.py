# launch_pkg/launch/hils_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # --- Toggle args
    use_localmap = LaunchConfiguration('use_localmap')
    use_visualization = LaunchConfiguration('use_visualization')
    use_vehicle  = LaunchConfiguration('use_vehicle')
    use_fsm      = LaunchConfiguration('use_fsm')

    # --- New params (for practical_local_map)
    hils_count = LaunchConfiguration('hils_count')
    hils_obstacles_enabled = LaunchConfiguration('hils_obstacles_enabled')
    csv_path = LaunchConfiguration('csv_path')
    


    decl_args = [
        DeclareLaunchArgument('use_localmap', default_value='true'),
        DeclareLaunchArgument('use_visualization',      default_value='true'),
        DeclareLaunchArgument('use_vehicle',  default_value='false'),
        DeclareLaunchArgument('use_fsm',      default_value='true'),
        

        # ★ 추가된 런치 인자들
        DeclareLaunchArgument('hils_count', default_value='3'),
        DeclareLaunchArgument('hils_obstacles_enabled', default_value='true'),
        DeclareLaunchArgument('csv_path', default_value='/home/imac/glocal_map.csv'),
	]

    # --- Params path (local_map)
    local_map_share = get_package_share_directory('local_map_tools')
    local_map_params = os.path.join(local_map_share, 'params.yaml')

    # --- Nodes
    hils_local_map = Node(
        package='imac_detection_system',
        executable='practical_local_map',
        name='practical_local_map',
        parameters=[
            local_map_params,
            {'hils_count': hils_count,
             'hils_obstacles_enabled': hils_obstacles_enabled,
             'csv_path': csv_path}
        ] if os.path.exists(local_map_params) else [
            {'hils_count': hils_count,
             'hils_obstacles_enabled': hils_obstacles_enabled,
             'csv_path': csv_path}
        ],
        output='screen',
        condition=IfCondition(use_localmap)
    )

    vehicle_viz_node = Node(
        package='imac_detection_system',
        executable='vehicle_viz_node',
        name='vehicle_viz_node',
        output='screen',
        condition=IfCondition(use_fsm)
    )


    hils_fsm_mode = Node(
        package='imac_detection_system',
        executable='fsm_node',
        name='fsm_node',
        output='screen',
        condition=IfCondition(use_fsm)
    )
    


    # --- 실행 순서
    return LaunchDescription(
        decl_args + [
            TimerAction(period=1.0, actions=[hils_local_map]),
            TimerAction(period=2.0, actions=[vehicle_viz_node]),
            TimerAction(period=3.0, actions=[hils_fsm_mode]),
            

        ]
    )

