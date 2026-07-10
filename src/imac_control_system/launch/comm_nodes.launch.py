from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    imac_dir = get_package_share_directory('imac_control_system')

    sudo_prefix = (
        'cd /home/pi/ros2_ws && '
        'source /opt/ros/humble/setup.bash && '
        'source /home/pi/ros2_ws/install/setup.bash && '
    )

    return LaunchDescription([

        # ───── MAVROS px4.launch ─────
        ExecuteProcess(
            cmd=[
                'sudo', 'bash', '-c',
                sudo_prefix + 'ros2 launch mavros px4.launch '
                'fcu_url:=serial:///dev/ttyACM0:57600 '
                'gcs_url:=udp://@192.168.0.6:14550 '
                'tgt_system:=1 '
                'tgt_component:=1'
            ],
            output='screen'
        ),

        # ───── bridge_node ─────
        ExecuteProcess(
            cmd=[
                'sudo', 'bash', '-c',
                sudo_prefix + f'ros2 run imac_control_system bridge_node --ros-args --params-file {os.path.join(imac_dir, "config", "system_params.yaml")}'
            ],
            output='screen'
        ),

        # ───── udp_publisher_node (주석) ─────
        # ExecuteProcess(
        #     cmd=[
        #         'sudo', 'bash', '-c',
        #         sudo_prefix + f'ros2 run imac_control_system udp_publisher_node --ros-args --params-file {os.path.join(imac_dir, "config", "system_params.yaml")}'
        #     ],
        #     output='screen'
        # )
    ])
