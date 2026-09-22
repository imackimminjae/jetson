"""Spawn-only correction agrees across the compiled bridge and batch runner."""
import json
import math
from pathlib import Path
import re
import sys

import pytest

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / 'scripts'))
from knu_global_path_publisher import load_route, load_start_positions
from knu_batch_monitor import RouteMonitor


@pytest.mark.parametrize('number', range(1, 22))
def test_all_starts_match_bridge_and_keep_paths_and_yaw(number):
    directory = PACKAGE / 'data/knu_routes'
    name = f'scenario{number}'
    rows = json.loads((directory/'knu_start_poses.json').read_text())['starts']
    row = next(r for r in rows if r['scenario'] == name)
    start = load_start_positions(directory)[name]
    points = load_route(name, directory)
    assert list(points[0]) == row['path_first']
    cpp = (PACKAGE/'src/px4_odom_map_transform.cpp').read_text()
    pattern = (r'if \(route_name == "' + name + r'"\) \{\s*'
               r'return RouteAnchor\{\{([^}]+)\}, \{([^}]+)\}\};')
    match = re.search(pattern, cpp)
    assert match
    first, second = [tuple(map(float, group.split(','))) for group in match.groups()]
    assert first == start
    yaw = math.atan2(second[1]-first[1], second[0]-first[0])
    assert abs(yaw-row['ros_heading_rad']) < 1e-10
    if not row['corrected']:
        assert list(start) == row['original_position']
    assert RouteMonitor(points, 0).update(0, start[:2]) is None


@pytest.mark.parametrize('mutation', ['missing', 'duplicate', 'nan'])
def test_reject_invalid_spawn_table(tmp_path, mutation):
    data = json.loads((PACKAGE/'data/knu_routes/knu_start_poses.json').read_text())
    if mutation == 'missing':
        data['starts'].pop()
    elif mutation == 'duplicate':
        data['starts'].append(data['starts'][0])
    else:
        data['starts'][0]['position'][0] = float('nan')
    (tmp_path/'knu_start_poses.json').write_text(json.dumps(data))
    with pytest.raises(ValueError):
        load_start_positions(tmp_path)
