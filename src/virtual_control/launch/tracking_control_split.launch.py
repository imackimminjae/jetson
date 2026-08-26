from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
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


def _optional_float_override(context, name):
    value = LaunchConfiguration(name).perform(context).strip()
    return float(value) if value else None


def _local_map_file(context):
    explicit_file = LaunchConfiguration("local_map_file").perform(context).strip()
    if explicit_file:
        return explicit_file

    map_id = LaunchConfiguration("local_map_id").perform(context).strip()
    if map_id:
        return f"local_map_{map_id}.csv"

    return None


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

    bridge_overrides = {}
    map_bridge_overrides = {}
    upper_overrides = {}
    lower_overrides = {}

    route_name = _optional_string_override(context, "route_name")
    if route_name is not None:
        map_bridge_overrides["route_name"] = route_name

    scale_mode = _optional_string_override(context, "scale_mode")
    if scale_mode is not None:
        upper_overrides["scale_mode"] = scale_mode
        lower_overrides["scale_mode"] = scale_mode

    for geometry_parameter in (
        "wheelbase",
        "vehicle_length_m",
        "bev_forward_m",
        "preview_interval_nominal_road_width_m",
        "preview_interval_boundary_margin_m",
    ):
        value = _optional_float_override(context, geometry_parameter)
        if value is not None:
            upper_overrides[geometry_parameter] = value
            lower_overrides[geometry_parameter] = value

    local_map_file = _local_map_file(context)
    if local_map_file is not None:
        upper_overrides["csv_global_path_file"] = local_map_file

    state_input_type = _optional_string_override(context, "state_input_type")
    if state_input_type is not None:
        upper_overrides["state_input_type"] = state_input_type
        lower_overrides["state_input_type"] = state_input_type

    odom_topic = _optional_string_override(context, "odom_topic")
    if odom_topic is not None:
        map_bridge_overrides["output_topic"] = odom_topic
        upper_overrides["odom_topic"] = odom_topic
        lower_overrides["odom_topic"] = odom_topic

    motive_pose_topic = _optional_string_override(context, "motive_pose_topic")
    if motive_pose_topic is not None:
        bridge_overrides["motive_pose_topic"] = motive_pose_topic

    pose_stamped_topic = _optional_string_override(context, "pose_stamped_topic")
    if pose_stamped_topic is not None:
        bridge_overrides["output_pose_topic"] = pose_stamped_topic
        map_bridge_overrides["input_topic"] = pose_stamped_topic
        upper_overrides["pose_stamped_topic"] = pose_stamped_topic
        lower_overrides["pose_stamped_topic"] = pose_stamped_topic

    grid_map_topic = _optional_string_override(context, "grid_map_topic")
    if grid_map_topic is not None:
        upper_overrides["grid_map_topic"] = grid_map_topic

    global_path_topic = _optional_string_override(context, "global_path_topic")
    if global_path_topic is not None:
        upper_overrides["global_path_topic"] = global_path_topic

    duty_topic = _optional_string_override(context, "duty_topic")
    if duty_topic is not None:
        lower_overrides["duty_topic"] = duty_topic

    pwm_topic = _optional_string_override(context, "pwm_topic")
    if pwm_topic is not None:
        lower_overrides["pwm_topic"] = pwm_topic

    mavlink_enable = _optional_bool_override(context, "mavlink_enable")
    if mavlink_enable is not None:
        lower_overrides["mavlink_enable"] = mavlink_enable

    pixhawk_output_backend = _optional_string_override(
        context, "pixhawk_output_backend"
    )
    if pixhawk_output_backend is not None:
        lower_overrides["pixhawk_output_backend"] = pixhawk_output_backend

    bridge_parameters = [config_file]
    if bridge_overrides:
        bridge_parameters.append(bridge_overrides)

    map_bridge_parameters = [config_file]
    if map_bridge_overrides:
        map_bridge_parameters.append(map_bridge_overrides)

    upper_parameters = [config_file]
    if upper_overrides:
        upper_parameters.append(upper_overrides)

    lower_parameters = [config_file]
    if lower_overrides:
        lower_parameters.append(lower_overrides)

    return [
        Node(
            package=PACKAGE_NAME,
            executable="px4_ekf_bridge.py",
            name="px4_ekf_bridge",
            output="screen",
            parameters=bridge_parameters,
            condition=IfCondition(LaunchConfiguration("enable_px4_bridge")),
        ),
        Node(
            package=PACKAGE_NAME,
            executable="px4_odom_map_bridge",
            name="px4_odom_map_bridge",
            output="screen",
            parameters=map_bridge_parameters,
            condition=IfCondition(
                LaunchConfiguration("enable_px4_odom_map_bridge")
            ),
        ),
        Node(
            package=PACKAGE_NAME,
            executable="upper_planner_node",
            name="upper_planner_node",
            output="screen",
            parameters=upper_parameters,
            condition=IfCondition(LaunchConfiguration("enable_upper_planner")),
        ),
        Node(
            package=PACKAGE_NAME,
            executable="lower_tracking_mpc_node",
            name="lower_tracking_mpc_node",
            output="screen",
            parameters=lower_parameters,
            condition=IfCondition(
                LaunchConfiguration("enable_lower_tracking_mpc")
            ),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value="",
            description=(
                "Parameter YAML file. Empty uses "
                "virtual_control/config/tracking_control_split.yaml."
            ),
        ),
        DeclareLaunchArgument(
            "scale_mode",
            default_value="",
            description="Override upper/lower geometry profile: model or fullscale.",
        ),
        DeclareLaunchArgument(
            "wheelbase",
            default_value="",
            description="Override upper/lower wheelbase in metres.",
        ),
        DeclareLaunchArgument(
            "vehicle_length_m",
            default_value="",
            description="Override upper/lower vehicle length in metres.",
        ),
        DeclareLaunchArgument(
            "bev_forward_m",
            default_value="",
            description="Override upper/lower BEV forward extent in metres.",
        ),
        DeclareLaunchArgument(
            "preview_interval_nominal_road_width_m",
            default_value="",
            description="Override upper/lower nominal corridor width in metres.",
        ),
        DeclareLaunchArgument(
            "preview_interval_boundary_margin_m",
            default_value="",
            description="Override upper/lower corridor boundary margin in metres.",
        ),
        DeclareLaunchArgument(
            "local_map_id",
            default_value="",
            description="Select local_map<ID>.csv for the MIQP upper planner.",
        ),
        DeclareLaunchArgument(
            "local_map_file",
            default_value="",
            description="Explicit upper-planner CSV path. Overrides local_map_id.",
        ),
        DeclareLaunchArgument(
            "state_input_type",
            default_value="",
            description="Override upper/lower state input type: odom or pose_stamped.",
        ),
        DeclareLaunchArgument(
            "route_name",
            default_value="scenario1",
            description="KNU map anchor route: scenario1, scenario2, scenario3, or scenario4.",
        ),
        DeclareLaunchArgument(
            "odom_topic",
            default_value="",
            description=(
                "Override map-aligned Odometry output and upper/lower input topic."
            ),
        ),
        DeclareLaunchArgument(
            "motive_pose_topic",
            default_value="",
            description="Override raw Motive PoseStamped input to the PX4 bridge.",
        ),
        DeclareLaunchArgument(
            "pose_stamped_topic",
            default_value="",
            description="Override fused yaw-scalar PoseStamped output/input topic.",
        ),
        DeclareLaunchArgument(
            "grid_map_topic",
            default_value="",
            description="Override occupancy grid topic.",
        ),
        DeclareLaunchArgument(
            "global_path_topic",
            default_value="",
            description="Override external map-frame global path topic.",
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
            description="Override lower MPC MAVLink output enable flag.",
        ),
        DeclareLaunchArgument(
            "pixhawk_output_backend",
            default_value="",
            description="Override Pixhawk backend: mavlink_udp or disabled.",
        ),
        DeclareLaunchArgument(
            "enable_px4_bridge",
            default_value="true",
            description="Start px4_ekf_bridge node.",
        ),
        DeclareLaunchArgument(
            "enable_px4_odom_map_bridge",
            default_value="true",
            description="Start the single PX4-local to KNU-map alignment bridge.",
        ),
        DeclareLaunchArgument(
            "enable_upper_planner",
            default_value="true",
            description="Start upper_planner_node.",
        ),
        DeclareLaunchArgument(
            "enable_lower_tracking_mpc",
            default_value="true",
            description="Start lower_tracking_mpc_node.",
        ),
        OpaqueFunction(function=launch_setup),
    ])
