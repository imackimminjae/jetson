"""Regression tests for invalid SIH feedback reaching the controller."""
import importlib.util
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

SPEC = importlib.util.spec_from_file_location(
    'px4_ekf_bridge', Path(__file__).resolve().parents[1] / 'scripts/px4_ekf_bridge.py')
BRIDGE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = BRIDGE
SPEC.loader.exec_module(BRIDGE)


def q(deg):
    return [math.cos(math.radians(deg) / 2), 0, 0, math.sin(math.radians(deg) / 2)]


def feed(guard, truth, estimate, start=1.0, duration=2.1):
    for i in range(round(duration * 30) + 1):
        t = start + i / 30
        guard.update_truth(q(truth), t)
        ready = guard.check(math.radians(estimate), t, t)
    return ready


@pytest.mark.parametrize('truth,estimate', [(-126.4, -30.1), (-56.8, 82.5)])
def test_recorded_bad_heading_never_reaches_output(truth, estimate):
    guard = BRIDGE.SihHeadingGuard()
    assert not feed(guard, truth, estimate)
    assert guard.reason == 'heading_mismatch'


def test_wraparound_and_continuous_recovery():
    guard = BRIDGE.SihHeadingGuard()
    assert feed(guard, 179, -179)
    assert guard.error_deg == pytest.approx(2)
    assert not feed(guard, 0, 90, start=4, duration=0)
    assert not feed(guard, 0, 1, start=5, duration=1)
    assert feed(guard, 0, 1, start=6, duration=1.1)


def test_stale_truth_and_attitude_block_output():
    guard = BRIDGE.SihHeadingGuard()
    assert not guard.check(0, 1, 1)
    assert feed(guard, 0, 0)
    assert not guard.check(0, 4, 4)
    assert guard.reason == 'missing_or_stale_sih'
    guard.update_truth(q(0), 4)
    assert not guard.check(0, 3, 4)
    assert guard.reason == 'missing_or_stale_attitude'


@pytest.mark.parametrize('quaternion', [[0, 0, 0, 0], [float('nan'), 0, 0, 1], [1, 0, 0]])
def test_invalid_quaternion_clears_previously_valid_truth(quaternion):
    guard = BRIDGE.SihHeadingGuard()
    assert feed(guard, 0, 0)
    guard.update_truth(quaternion, 4)
    assert not guard.check(0, 4, 4)


def test_guard_blocks_before_creating_or_publishing_ros_pose():
    # The node method must stop before touching publishers or map alignment.
    node = SimpleNamespace(
        sih_heading_guard_enabled=True, sih_heading_guard=BRIDGE.SihHeadingGuard(),
        latest_attitude_yaw_ned=0, latest_attitude_monotonic=0,
        heading_guard_last_reason=None,
        get_logger=lambda: SimpleNamespace(warn=lambda _: None))
    BRIDGE.Px4EkfBridge._publish_fused_odometry(node, object())
    assert node.heading_guard_last_reason == 'missing_or_stale_sih'


def test_other_mavlink_vehicle_cannot_supply_truth():
    node = SimpleNamespace(target_system=1, target_component=1)
    msg = SimpleNamespace(get_type=lambda: 'HIL_STATE_QUATERNION',
                          get_srcSystem=lambda: 2, get_srcComponent=lambda: 1)
    BRIDGE.Px4EkfBridge._handle_mavlink_message(node, msg)


def test_message_gap_requires_new_settling_period():
    guard = BRIDGE.SihHeadingGuard()
    assert feed(guard, 0, 0)
    assert not feed(guard, 0, 0, start=10, duration=0)
    assert guard.reason == 'settling'
