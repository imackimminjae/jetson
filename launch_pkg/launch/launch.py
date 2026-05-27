# launch_pkg/launch/launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # --- Toggle args
    use_oak      = LaunchConfiguration('use_oak')
    use_yolo     = LaunchConfiguration('use_yolo')
    use_tl       = LaunchConfiguration('use_traffic_light')
    use_depths   = LaunchConfiguration('use_depths')
    use_fusion   = LaunchConfiguration('use_fusion')
    use_localmap = LaunchConfiguration('use_localmap')
    use_fsm      = LaunchConfiguration('use_fsm')
    use_visualization = LaunchConfiguration('use_visualization')


    decl_args = [
        DeclareLaunchArgument('use_oak',            default_value='true'),
        DeclareLaunchArgument('use_yolo',           default_value='true'),
        DeclareLaunchArgument('use_traffic_light',  default_value='true'),
        DeclareLaunchArgument('use_depths',         default_value='true'),
        DeclareLaunchArgument('use_fusion',         default_value='true'),
        DeclareLaunchArgument('use_localmap',       default_value='true'),
        DeclareLaunchArgument('use_fsm',            default_value='true'),
        #DeclareLaunchArgument('csv_path',           default_value='/home/imac/glocal_map.csv'),
        DeclareLaunchArgument('use_visualization',      default_value='true'),

    ]

    # --- 기존 노드들
    oak_total = Node(
        package='imac_detection_system',
        executable='oak_total_publisher',
        name='oak_total_publisher',
        output='screen',
        condition=IfCondition(use_oak)
    )
    yolo_node = Node(
        package='imac_detection_system',
        executable='yolo_detect_node.py',
        name='yolo_detect_node',
        output='screen',
        condition=IfCondition(use_yolo)
    )
    traffic_light_node = Node(
        package='imac_detection_system',
        executable='traffic_light_color_node',
        name='traffic_light_color_node',
        output='screen',
        condition=IfCondition(use_tl)
    )
    red_bbox_node = Node(
        package='imac_detection_system',
        executable='red_bbox_node',
        name='red_bbox_node',
        output='screen',
        condition=IfCondition(use_depths)
    )
    yolo_depth_fusion = Node(
        package='imac_detection_system',
        executable='bbox_depth_fusion_node',
        name='bbox_depth_fusion_node',
        output='screen',
        condition=IfCondition(use_fusion)
    )
    local_map = Node(
        package='imac_detection_system',
        executable='practical_local_map',
        name='practical_local_map',
        output='screen',
        condition=IfCondition(use_localmap)
    )
    fsm_node = Node(
        package='imac_detection_system',
        executable='fsm_node',
        name='fsm_node',
        output='screen',
        condition=IfCondition(use_fsm),
    )
    vehicle_viz_node = Node(
        package='imac_detection_system',
        executable='vehicle_viz_node',
        name='vehicle_viz_node',
        output='screen',
        condition=IfCondition(use_fsm)
    )

    # --- 기동 순서: OAK → YOLO/Depth → Fusion → LocalMap → FSM → (약간 뒤) rosbag
    return LaunchDescription(
        decl_args + [
            oak_total,
            TimerAction(period=1.0, actions=[yolo_node, traffic_light_node, red_bbox_node]),
            TimerAction(period=6.0, actions=[yolo_depth_fusion]),
            TimerAction(period=6.3, actions=[vehicle_viz_node]),
            TimerAction(period=6.5, actions=[local_map]),
            TimerAction(period=7.0, actions=[fsm_node]),
            
        ]
    )

