"""Exercise launch construction without starting vehicle output nodes."""
import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions
from launch_ros.actions import Node


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "comparison_launch", ROOT / "launch" / "planner_comparison.launch.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
ORIGINAL_SPEC = importlib.util.spec_from_file_location(
    "original_split_launch", ROOT / "launch" / "tracking_control_split.launch.py")
ORIGINAL = importlib.util.module_from_spec(ORIGINAL_SPEC)
ORIGINAL_SPEC.loader.exec_module(ORIGINAL)


def context(planner):
    result = LaunchContext()
    for action in (ORIGINAL.generate_launch_description().entities +
                   MODULE.generate_launch_description().entities):
        if isinstance(action, DeclareLaunchArgument):
            result.launch_configurations[action.name] = perform_substitutions(
                result, action.default_value)
    result.launch_configurations.update({
        "planner": planner,
        "config_file": str(ROOT / "config" / "tracking_control_split.yaml"),
        "comparison_config_file": str(ROOT / "config" / "comparison_planners.yaml"),
    })
    return result


@pytest.mark.parametrize("planner", ["miqp", "nvp", "mpcc"])
def test_exactly_one_upper_and_common_lower(planner):
    ctx = context(planner)
    for action in MODULE.generate_launch_description().entities:
        if isinstance(action, IncludeLaunchDescription):
            for key, value in action.launch_arguments:
                ctx.launch_configurations[key] = perform_substitutions(
                    ctx, normalize_to_list_of_substitutions(value))
    nodes = [action for action in ORIGINAL.launch_setup(ctx) + MODULE.launch_setup(ctx)
             if isinstance(action, Node) and
             (action.condition is None or action.condition.evaluate(ctx))]
    executables = [n.node_executable for n in nodes]
    assert executables.count("upper_planner_node") == (planner == "miqp")
    assert executables.count("nvp_planner_node") == (planner == "nvp")
    assert executables.count("mpcc_planner_node") == (planner == "mpcc")
    assert executables.count("lower_tracking_mpc_node") == 1


def test_invalid_planner_rejected():
    with pytest.raises(ValueError, match="planner must"):
        MODULE.launch_setup(context("invalid"))


def test_shared_conditions_cannot_be_silently_overridden(tmp_path):
    ctx = context("mpcc")
    filename = tmp_path / "unfair.yaml"
    filename.write_text("/**:\n  ros__parameters:\n    target_speed_mps: 3.0\n")
    ctx.launch_configurations["comparison_config_file"] = str(filename)
    with pytest.raises(ValueError, match="shared comparison conditions"):
        MODULE.launch_setup(ctx)
