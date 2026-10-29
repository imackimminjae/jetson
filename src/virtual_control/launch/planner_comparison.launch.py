"""Separate comparison launch; the existing MIQP node and launch are untouched."""
from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    del args, kwargs
    planner = LaunchConfiguration("planner").perform(context)
    if planner not in ("miqp", "nvp", "mpcc"):
        raise ValueError("planner must be miqp, nvp, or mpcc")
    if planner == "miqp":
        return []  # The included original launch exclusively owns MIQP.
    share = Path(get_package_share_directory("virtual_control"))
    config_file = LaunchConfiguration("config_file").perform(context)
    config_file = config_file or str(share / "config" / "tracking_control_split.yaml")
    with open(config_file, encoding="utf-8") as stream:
        base = yaml.safe_load(stream)
    common = dict(base["upper_planner_node"]["ros__parameters"])
    # Honour the existing launch's shared topic / geometry arguments.
    for name in ("scale_mode", "state_input_type", "odom_topic", "grid_map_topic", "global_path_topic"):
        value = LaunchConfiguration(name).perform(context).strip()
        if value:
            common[name] = value
    for name in ("wheelbase", "vehicle_length_m"):
        value = LaunchConfiguration(name).perform(context).strip()
        if value:
            common[name] = float(value)
    if common.get("state_input_type", "odom") != "odom":
        raise ValueError("comparison planners require map-frame Odometry")
    if common.get("scale_mode", "fullscale") != "fullscale":
        raise ValueError("comparison launch currently supports the fullscale geometry profile")
    if common.get("grid_origin_x_override_enabled", False):
        raise ValueError("comparison planners use the published BEV origin; disable grid origin override")
    if common.get("use_csv_global_path", False):
        raise ValueError("comparison planners require a topic reference; disable use_csv_global_path")
    comparison_config = LaunchConfiguration("comparison_config_file").perform(context)
    comparison_config = comparison_config or str(share / "config" / "comparison_planners.yaml")
    with open(comparison_config, encoding="utf-8") as stream:
        tuning = yaml.safe_load(stream)["/**"]["ros__parameters"]
    shared_keys = {
        "target_speed_mps", "upper_planner_rate_hz", "upper_prediction_dt_sec",
        "upper_preview_steps", "lower_prediction_dt_sec", "lower_min_path_spacing_m",
        "wheelbase", "vehicle_length_m", "max_steering_angle_rad",
        "max_steering_rate_radps", "reference_point_to_rear_axle_m",
        "grid_positive_is_drivable", "grid_value_threshold", "odom_topic",
        "grid_map_topic", "global_path_topic", "lower_reference_path_topic",
    }
    if shared_keys.intersection(tuning):
        raise ValueError("Set shared comparison conditions in config_file, not comparison_config_file")
    lower = base["lower_tracking_mpc_node"]["ros__parameters"]
    if abs(common["max_steering_angle_rad"] - lower["lower_max_steering_angle_rad"]) > 1e-9:
        raise ValueError("upper and lower physical steering limits must agree for comparison")
    common.update({
        "max_steering_rate_radps": lower["lower_max_steering_rate_radps"],
        "reference_point_to_rear_axle_m": lower["lower_reference_point_to_rear_axle_m"],
    })
    common.update(tuning)
    common.update({
        "planner": planner,
        "reference_source": LaunchConfiguration("reference_source").perform(context),
        "exact_reference_topic": LaunchConfiguration("exact_reference_topic").perform(context),
    })
    return [Node(package="virtual_control", executable=f"{planner}_planner_node",
                 name=f"{planner}_planner_node", output="screen", parameters=[common])]


def generate_launch_description():
    original = Path(get_package_share_directory("virtual_control")) / "launch" / "tracking_control_split.launch.py"
    return LaunchDescription([
        DeclareLaunchArgument("planner", default_value="miqp",
                              description="Exclusive upper planner: miqp, nvp, mpcc."),
        DeclareLaunchArgument("comparison_config_file", default_value="",
                              description="Alternative-planner tuning YAML."),
        DeclareLaunchArgument("reference_source", default_value="exact",
                              description="MPCC reference provenance: exact or sparse."),
        DeclareLaunchArgument("exact_reference_topic", default_value="/navigation/exact_reference_path",
                              description="Independent measured/designed curve; never MIQP output."),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(original)), launch_arguments={
            "enable_upper_planner": PythonExpression(["'", LaunchConfiguration("planner"), "' == 'miqp'"]),
        }.items()),
        OpaqueFunction(function=launch_setup),
    ])
