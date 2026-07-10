from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # 1) CSV -> /path + /global_path
    csv_path_publisher = Node(
        package='virtual_control',
        executable='csv_path_publisher_node',
        name='csv_path_publisher',
        output='screen',
        parameters=[{
            # 워크스페이스 기준 절대/상대 경로로 맞춰서 수정
            'csv_path': 'glocal_map.csv',
            'path_topic': '/path',            # nav_msgs/Path
            'global_path_topic': '/global_path',  # imac_interfaces/GlobalPath
            'frame_id': 'map',
            'publish_rate_hz': 1.0,
        }]
    )

    # 2) TrajectoryPlanNode : /global_path, /obstacles 등 받아서 /controller/local_curve 발행
    trajectory_plan = Node(
        package='virtual_control',                 # 지금 TrajectoryPlanNode를 virtual_control에 넣었다고 가정
        executable='trajectory_plan_node',
        name='trajectory_plan_node',
        output='screen',
        parameters=[{
            'loop_rate_hz': 20.0,
            's_step_init': 0.20,
            # 필요하면 dpsi_deg 등도 여기서 override 가능
        }]
    )

    # 3) LeatherbackStanley : /odom + /controller/local_curve -> /ackermann_cmd
    leatherback_stanley = Node(
        package='virtual_control',
        executable='leatherback_stanley_node',
        name='leatherback_stanley',
        output='screen',
        parameters=[{
            'odom_topic': '/odom',
            'path_topic': '/controller/local_curve',   # TrajectoryPlanNode 출력
            'cmd_topic': '/ackermann_cmd',
            'wheelbase': 0.256,
            'steer_max_deg': 30.0,
            'steer_rate_deg': 120.0,
            'v_ref': 3.0,
            'v_max': 6.0,
            'control_rate': 30.0,
            'align_path_to_start': True,
            'yaw_body_offset_deg': -1.57,
            'steer_is_degrees': False,  # Isaac이 deg로 받을 거면 True
        }]
    )

    # 4) Ackermann -> VirtualControlCommand : /ackermann_cmd -> /virtual_control_command
    ack_to_control = Node(
        package='virtual_control',
        executable='ackermann_to_control_node',
        name='ackermann_to_control',
        output='screen',
        parameters=[{
            'loop_rate_hz': 50.0,
            'steer_limit_rad': 0.5,
            'kp_throttle': 0.5,
            'kp_brake': 0.5,
            # Isaac 쪽에서 구독할 토픽 이름도 여기에서 맞춰주면 됨
            # 예: 'output_topic': '/isaac_vehicle_low_level_cmd'
        }]
    )

    return LaunchDescription([
        csv_path_publisher,
        trajectory_plan,
        leatherback_stanley,
        ack_to_control,
    ])

