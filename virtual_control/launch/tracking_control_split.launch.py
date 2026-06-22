from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PACKAGE_NAME = "virtual_control"


def _optional_string_override(context, name):
    value = LaunchConfiguration(name).perform(context)
    return value if value else None


def _optional_bool_override(context, name):
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if not value:
        return None
    return value in ("1", "true", "yes", "on")


def launch_setup(context, *args, **kwargs):
    del args, kwargs

    default_config_file = (
        Path(get_package_share_directory(PACKAGE_NAME)) /
        "config" /
        "tracking_control_split.yaml"
    )
    config_file = LaunchConfiguration("config_file").perform(context)
    if not config_file:
        config_file = str(default_config_file)

    tracking_overrides = {}
    lower_overrides = {}

    state_input_type = _optional_string_override(context, "state_input_type")
    if state_input_type is not None:
        tracking_overrides["state_input_type"] = state_input_type

    pose_stamped_topic = _optional_string_override(context, "pose_stamped_topic")
    if pose_stamped_topic is not None:
        tracking_overrides["pose_stamped_topic"] = pose_stamped_topic

    grid_map_topic = _optional_string_override(context, "grid_map_topic")
    if grid_map_topic is not None:
        tracking_overrides["grid_map_topic"] = grid_map_topic

    solver_cmd_topic = _optional_string_override(context, "solver_cmd_topic")
    if solver_cmd_topic is not None:
        lower_overrides["solver_cmd_topic"] = solver_cmd_topic

    applied_cmd_topic = _optional_string_override(context, "applied_cmd_topic")
    if applied_cmd_topic is not None:
        lower_overrides["applied_cmd_topic"] = applied_cmd_topic

    duty_topic = _optional_string_override(context, "duty_topic")
    if duty_topic is not None:
        lower_overrides["duty_topic"] = duty_topic

    pwm_topic = _optional_string_override(context, "pwm_topic")
    if pwm_topic is not None:
        lower_overrides["pwm_topic"] = pwm_topic

    mavlink_enable = _optional_bool_override(context, "mavlink_enable")
    if mavlink_enable is not None:
        lower_overrides["mavlink_enable"] = mavlink_enable

    tracking_parameters = [config_file]
    if tracking_overrides:
        tracking_parameters.append(tracking_overrides)

    lower_parameters = [config_file]
    if lower_overrides:
        lower_parameters.append(lower_overrides)

    return [
        Node(
            package=PACKAGE_NAME,
            executable="tracking_control_node",
            name="tracking_control_node",
            output="screen",
            parameters=tracking_parameters,
        ),
        Node(
            package=PACKAGE_NAME,
            executable="lower_duty_node",
            name="lower_duty_node",
            output="screen",
            parameters=lower_parameters,
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value="",
            description="Parameter YAML file. Empty uses virtual_control/config/tracking_control_split.yaml.",
        ),
        DeclareLaunchArgument(
            "state_input_type",
            default_value="",
            description="Override tracking input type, for example odom or pose_stamped.",
        ),
        DeclareLaunchArgument(
            "pose_stamped_topic",
            default_value="",
            description="Override PoseStamped input topic when state_input_type is pose_stamped.",
        ),
        DeclareLaunchArgument(
            "grid_map_topic",
            default_value="",
            description="Override occupancy grid topic.",
        ),
        DeclareLaunchArgument(
            "solver_cmd_topic",
            default_value="",
            description="Override lower_duty_node raw solver command input topic.",
        ),
        DeclareLaunchArgument(
            "applied_cmd_topic",
            default_value="",
            description="Override lower_duty_node filtered applied command output topic.",
        ),
        DeclareLaunchArgument(
            "duty_topic",
            default_value="",
            description="Override normalized duty output topic.",
        ),
        DeclareLaunchArgument(
            "pwm_topic",
            default_value="",
            description="Override PWM debug/output topic.",
        ),
        DeclareLaunchArgument(
            "mavlink_enable",
            default_value="",
            description="Override lower_duty_node MAVLink output enable flag.",
        ),
        OpaqueFunction(function=launch_setup),
    ])
