"""Check mission file validation before a global path can be published."""

import importlib.util
from pathlib import Path

import pytest


PACKAGE_DIR = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'knu_global_path_publisher', PACKAGE_DIR / 'scripts' / 'knu_global_path_publisher.py'
)
PUBLISHER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PUBLISHER)


@pytest.mark.parametrize('route_name, count, first, last', [
    ('scenario1', 6, (150, -140, 0), (173, -2, 0)),
    ('scenario2', 7, (-250, 101, 0), (-4, 95, 0)),
    ('scenario3', 8, (187, 220, 0), (278, 172, 0)),
    ('scenario4', 7, (0, 7, 0), (150, -4, 0)),
])
def test_installed_mission_geometry(route_name, count, first, last):
    """Preserve MATLAB mission endpoints, including scenario2's off-road waypoint."""
    points = PUBLISHER.load_route(route_name, PACKAGE_DIR / 'data' / 'knu_routes')
    assert len(points) == count
    assert points[0] == first
    assert points[-1] == last


@pytest.mark.parametrize('contents', [
    'x,y,z\n',
    'x,y,z\n1,2,0\n',
    'x,y,z\n1,2,0\nnan,3,0\n',
    'x,y,z\n1,2,0\n4,inf,0\n',
    'x,y,z\n1,2,0\n4,5\n',
    'x,y,z\n1,2,0\n4,5,0,extra\n',
    'x,y,z\n1,2,0\n4,invalid,0\n',
    'x,y,z\n1,2,0\n1,2,1\n',
    'X,Y\n1,2\n3,4\n',
])
def test_invalid_route_is_rejected_instead_of_shortened(tmp_path, contents):
    """An invalid row must not silently change which waypoints form the mission."""
    (tmp_path / 'scenario1.csv').write_text(contents, encoding='utf-8')
    with pytest.raises(ValueError):
        PUBLISHER.load_route('scenario1', tmp_path)


@pytest.mark.parametrize('route_name', ['scenario22', '../scenario1', '', 'Scenario1'])
def test_unknown_route_is_rejected(tmp_path, route_name):
    """A mistyped route must not select a different mission or file."""
    with pytest.raises(ValueError, match='route_name'):
        PUBLISHER.load_route(route_name, tmp_path)


def test_missing_route_is_reported(tmp_path):
    """Missing data must stop the publisher instead of emitting an empty route."""
    with pytest.raises(FileNotFoundError):
        PUBLISHER.load_route('scenario1', tmp_path)
