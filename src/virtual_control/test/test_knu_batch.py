"""Regression checks for unattended route decisions and imported MATLAB geometry."""

from pathlib import Path
import re
import sys

import pytest

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / 'scripts'))
from knu_batch_monitor import Limits, RouteMonitor  # noqa: E402
from knu_global_path_publisher import load_route, load_start_positions  # noqa: E402


def test_noise_does_not_hide_ten_second_stop():
    monitor = RouteMonitor([(0, 0), (100, 0)], 0)
    for i in range(100):
        assert monitor.update(i/10, (0.1*(i % 2), 0)) is None
    assert monitor.update(10, (0, 0)) == 'stopped_10s'


def test_goal_is_generous_without_requiring_exact_endpoint():
    monitor = RouteMonitor([(0, 0), (100, 0)], 0)
    for i in range(92):
        assert monitor.update(i, (i, 0)) is None
    assert monitor.update(92, (92, 0)) == 'completed'


def test_start_close_to_end_of_loop_does_not_pass():
    monitor = RouteMonitor([(0, 0), (100, 0), (100, 100), (0, 100), (0, 1)], 0)
    assert monitor.update(0, (0, 0)) is None
    assert monitor.progress == 0
    assert monitor.update(10, (0, 0)) == 'stopped_10s'


def test_off_route_requires_persistence_and_can_recover():
    monitor = RouteMonitor([(0, 0), (100, 0)], 0)
    for t, p in [(0, (0, 0)), (1, (1, 9)), (2, (2, 18)), (3, (3, 10)),
                 (4, (4, 18)), (5, (5, 19)), (6, (6, 20))]:
        assert monitor.update(t, p) is None
    assert monitor.update(7, (7, 21)) == 'off_route'


def test_reversing_skips_route():
    monitor = RouteMonitor([(0, 0), (100, 0)], 0)
    for i in range(31):
        assert monitor.update(i, (i, 0)) is None
    for i in range(1, 12):
        assert monitor.update(30+i, (30-i, 0)) is None
    assert monitor.update(42, (18, 0)) == 'wrong_direction'


def test_teleport_cannot_award_completion():
    monitor = RouteMonitor([(0, 0), (100, 0)], 0)
    monitor.update(0, (0, 0))
    assert monitor.update(0.1, (100, 0)) == 'pose_jump'


def test_moving_without_route_progress_is_bounded():
    monitor = RouteMonitor([(0, 0), (100, 0)], 0)
    for i in range(30):
        assert monitor.update(i, (0, (i % 4)*0.6)) is None
    assert monitor.update(30, (0, 0)) == 'no_progress'


def test_route_deadline_even_when_progressing():
    monitor = RouteMonitor([(0, 0), (1000, 0)], 0, Limits(route_timeout_sec=20))
    for i in range(20):
        assert monitor.update(i, (i, 0)) is None
    assert monitor.update(20, (20, 0)) == 'route_timeout'


@pytest.mark.parametrize('setting', [{'stop_sec': 0}, {'off_route_m': float('nan')},
                                     {'min_progress_ratio': 1.2}])
def test_reject_invalid_limits(setting):
    with pytest.raises(ValueError):
        Limits(**setting)


@pytest.mark.parametrize('number', range(1, 22))
def test_every_route_loads_and_can_finish_in_order(number):
    points = load_route(f'scenario{number}', PACKAGE / 'data/knu_routes')
    monitor = RouteMonitor(points, 0, Limits(route_timeout_sec=10000))
    now = 0.0
    result = None
    for a, b, distance in zip(points, points[1:], monitor.lengths):
        steps = max(1, int(distance)+1)
        for step in range(steps):
            p = tuple(a[j]+(b[j]-a[j])*step/steps for j in range(2))
            result = monitor.update(now, p)
            now += 0.5
            if result:
                break
        if result:
            break
    if not result:
        result = monitor.update(now, points[-1])
    assert result == 'completed'


def test_section_csv_matches_matlab_and_bridge_anchors():
    matlab = PACKAGE.parents[1] / 'matlab/knu_batch/knu_section_routes.m'
    if not matlab.exists():
        pytest.skip('source-only cross-language validation')
    source = matlab.read_text()
    cpp = (PACKAGE/'src/px4_odom_map_transform.cpp').read_text()
    matches = re.findall(r'case "(scenario\d+)"(.*?)(?=    case |    otherwise)', source, re.S)
    assert len(matches) == 17
    for name, body in matches:
        block = re.search(r'waypoints = \[ \.\.\.(.*?)\];', body, re.S)[1]
        points = [tuple(map(float, line.split())) for line in block.strip().splitlines()]
        assert points == load_route(name, PACKAGE/'data/knu_routes')
        block = re.search(r'if \(route_name == "'+name+r'"\) \{(.*?)\}', cpp, re.S)[1]
        first = tuple(map(float, re.search(r'RouteAnchor\{\{(.*)', block)[1].split(',')))
        assert first == load_start_positions(PACKAGE/'data/knu_routes')[name]
