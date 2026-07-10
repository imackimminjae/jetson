from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PACKAGE_NAME = "virtual_control"


def launch_setup(context, *args, **kwargs):
    del args, kwargs

    config_file = (
        Path(get_package_share_directory(PACKAGE_NAME)) /
        "config" /
        "straight_mocap_pwm.yaml"
    )
    path_csv_file = LaunchConfiguration("path_csv_file").perform(context)

    parameters = [str(config_file)]
    if path_csv_file:
        parameters.append({"path_csv_file": path_csv_file})

    return [
        Node(
            package=PACKAGE_NAME,
            executable="straight_mocap_pwm_node",
            name="straight_mocap_pwm_node",
            output="screen",
            parameters=parameters,
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "path_csv_file",
            default_value="",
            description="CSV path file. Empty keeps the YAML/default internal straight path.",
        ),
        OpaqueFunction(function=launch_setup),
    ])
