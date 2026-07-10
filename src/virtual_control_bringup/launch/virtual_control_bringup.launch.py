from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # 1) CSV → /path + /global_path
    csv_path_publisher = Node(
        package='virtual_control',
        executable='csv_path_publisher_node',
        name='csv_path_publisher',
        output='screen',
        parameters=[{
            'csv_path': 'glocal_map.csv',
            'path_topic': '/path',
            'global_path_topic': '/global_path',
            'frame_id': 'odom',
            'publish_rate_hz': 1.0,
        }]
    )

    # 2) TrajectoryPlanNode → /controller/local_curve
    trajectory_plan = Node(
        package='virtual_control',
        executable='trajectory_plan_node',
        name='trajectory_plan_node',
        output='screen',
        parameters=[{
            'loop_rate_hz': 20.0,
            's_step_init': 0.20,
        }]
    )

    # 3) **TrackingControllerNode (NEW)**
    #   입력:
    #     /sim/odom_in
    #     /global_path
    #   출력:
    #     /control_command  (가상 차량 입력: steer/throttle 정규화)
    #     /vehicle_status_hils (내부 ODE 시뮬레이션 상태)
    tracking_controller = Node(
        package='virtual_control',
        executable='tracking_controller_node',   # 맞는 executable name
        name='tracking_controller',
        output='screen',
        parameters=[{
            'kp': 1.0,
            'ki': 0.1,
            'kd': 0.1,
            'loop_rate_hz_ctrl': 100.0,
            'loop_rate_hz_trj': 20.0,
            'stanley_k': 1.0,
            'max_steer_left': 0.52,
            'max_steer_right': -0.52,
            'v_goal': 0.8,
            'dpsi_deg': [0.0],
        }]
    )

    # 4) Leatherback Stanley (원래 있던 노드)
    leatherback_stanley = Node(
        package='virtual_control',
        executable='leatherback_stanley_node',
        name='leatherback_stanley',
        output='screen',
        parameters=[{
            'odom_topic': '/odom',
            'path_topic': '/controller/local_curve',
            'cmd_topic': '/ackermann_cmd',
            'wheelbase': 0.32,
            'steer_max_deg': 30.0,
            'steer_rate_deg': 120.0,
            'v_ref': 3.0,
            'v_max': 6.0,
            'control_rate': 30.0,
            'align_path_to_start': True,
            'yaw_body_offset_deg': -1.57,
            'steer_is_degrees': False,
        }]
    )

    # 5) Ackermann → VirtualControlCommand
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
        }]
    )

    return LaunchDescription([
        csv_path_publisher,
        tracking_controller,

    ])

