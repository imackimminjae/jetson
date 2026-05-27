from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    # sudo 환경에서 ROS를 로드하고 워크스페이스 활성화


    return LaunchDescription([


        # joy_node
        ExecuteProcess(
            cmd=[ 'bash', '-lc',  'ros2 run joy joy_node'],
            output='screen'
        ),

        # trajectory_plan_node
        ExecuteProcess(
            cmd=['bash', '-lc',
                 'ros2 run imac_control_system trajectory_plan_node'],
            output='screen'
        ),

        # vehicle_controller_node
        ExecuteProcess(
            cmd=[ 'bash', '-lc',
                  'ros2 run imac_control_system vehicle_controller_node'],
            output='screen'
        ),
    ])


