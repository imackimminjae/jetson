"""Additional long routes remain separate from the default batch."""
import json
import math
from pathlib import Path
import re
import sys

import pytest

PACKAGE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PACKAGE/'scripts'))
from knu_global_path_publisher import load_route,load_start_positions
from knu_batch_monitor import RouteMonitor
from repeat_drive_session import SCENARIOS,AVAILABLE_SCENARIOS,effective_config


def test_default_batch_stays_twelve_routes(tmp_path):
    assert SCENARIOS==tuple(f'scenario{i}' for i in range(1,13))
    assert set(AVAILABLE_SCENARIOS)-set(SCENARIOS)=={f'scenario{i}' for i in range(22,26)}
    assert effective_config(1,tmp_path,'all')['knu_batch_runner']['ros__parameters']['routes']==list(SCENARIOS)
    with pytest.raises(ValueError):effective_config(1,tmp_path,'scenario13')


@pytest.mark.parametrize('number',range(22,26))
def test_long_route_progress_spawn_and_sections_are_consistent(number,tmp_path):
    directory=PACKAGE/'data/knu_routes';name=f'scenario{number}'
    points=load_route(name,directory);start=load_start_positions(directory)[name]
    assert points[0]==start
    profile=json.loads((directory/'knu_long_profile.json').read_text())
    meta=next(m for m in profile['definitions'] if m['scenario']==name)
    lengths=[math.dist(a[:2],b[:2]) for a,b in zip(points,points[1:])]
    assert (350 if number==24 else 390)<sum(lengths)<700
    assert max(lengths)<=100.01 and min(lengths)>.01
    assert 8<=len(points)<=12
    assert sum(lengths)/(len(points)-1)>35.
    assert abs(sum(lengths)-meta['length_m'])<.002
    assert meta['reference_length_m']>=meta['length_m']
    assert meta['waypoint_sampling']['automatic_reference_deviation_tolerance_m']==2.5
    sections=meta['sections']
    assert sections[0]['start_wp']==1 and sections[-1]['end_wp']==len(points)
    assert all(a['end_wp']==b['start_wp'] for a,b in zip(sections,sections[1:]))
    assert meta['continuous'] and meta['dense_min_radius_m']>6.5
    assert meta['reference_mask_footprint_margin_m']>0.
    if number!=24:assert meta['reference_mask_footprint_margin_m']>.15
    assert not meta['actual_drive_validated']
    config=effective_config(1,tmp_path,name,guarded=True)
    assert config['knu_batch_runner']['ros__parameters']['routes']==[name]
    header=(PACKAGE/'include/virtual_control/knu_long_route_anchors.hpp').read_text()
    pattern=r'if \(route_name == "'+name+r'"\) \{\s*return RouteAnchor\{\{([^}]+)\}, \{([^}]+)\}\};'
    match=re.search(pattern,header);assert match
    first,second=[tuple(map(float,g.split(','))) for g in match.groups()]
    assert first==start
    assert abs(math.atan2(second[1]-first[1],second[0]-first[0])-meta['heading_rad'])<1e-10
    monitor=RouteMonitor(points,0.);now=0.;outcome=None
    for a,b,distance in zip(points,points[1:],lengths):
        steps=max(1,math.ceil(distance/.5))
        for i in range(steps):
            p=[a[k]+(b[k]-a[k])*i/steps for k in range(2)]
            outcome=monitor.update(now,p);now+=distance/steps/6.
            if outcome:break
        if outcome:break
    outcome=outcome or monitor.update(now,points[-1][:2])
    assert outcome=='completed'
