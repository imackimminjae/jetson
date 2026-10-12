"""Offline evidence checks: unknown maps never excuse route deviation."""
from pathlib import Path
from types import SimpleNamespace as NS
import math
import sys
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from knu_batch_monitor import RoadRouteCheck, RoadLimits


def grid(cut=10, value_after=100, positive=False):
    data = [(100 if positive else 0) if x*.5 < cut
            else value_after for y in range(40) for x in range(80)]
    return NS(header=NS(frame_id='Vehicle'),data=data,
              info=NS(width=80,height=40,resolution=.5,
                      origin=NS(position=NS(x=0.,y=-10.),
                                orientation=NS(x=0.,y=0.,z=0.,w=1.))))


def evidence(check=None,g=None):
    check = check or RoadRouteCheck([(0,0),(100,0)])
    return check.sample(g or grid(),(0,0),0,0)


def test_known_road_ending_is_separate_and_persistent():
    check=RoadRouteCheck([(0,0),(100,0)])
    e=evidence(check)
    assert e['state']=='route_unavailable'
    for i in range(16):
        check.observe(i*.2,100+i*.2,e)
        assert check.confirmed(i*.2)==(i==15)
    assert not check.confirmed(4)


@pytest.mark.parametrize('after',[-1,0])
def test_unknown_or_continuing_road_cannot_prove_end(after):
    assert evidence(g=grid(value_after=after))['state']!='route_unavailable'


def test_outside_bev_is_not_a_road_end():
    g=grid(cut=100);g.info.width=12;g.data=[0]*480
    assert evidence(g=g)['state']=='unavailable'


def test_small_hole_or_narrow_path_is_not_a_road_end():
    g=grid()
    for y in range(40):
        for x in range(26,80): g.data[y*80+x]=0
    assert evidence(g=g)['state']!='route_unavailable'
    g=grid()
    for x in range(80): g.data[24*80+x]=0  # road 2 m beside nominal path
    assert evidence(g=g)['state']!='route_unavailable'


def test_vehicle_off_road_cannot_be_excused():
    g=grid();g.data[20*80]=100
    assert evidence(g=g)['state']=='unavailable'


def test_observed_end_is_tracked_when_earlier_free_stretch_leaves_forward_bev():
    check=RoadRouteCheck([(0,0),(100,0)])
    check.observe(0,1,evidence(check))
    check.expire(.2)
    e=check.sample(grid(cut=1),(9,0),0,9)
    assert e['state']=='route_unavailable' and e['blocked_start_m']==10
    check.observe(.2,2,e)
    check.expire(2)
    assert check.sample(grid(cut=1),(9,0),0,9)['state']!='route_unavailable'


def test_brief_supported_patch_does_not_count_as_a_transition():
    assert evidence(g=grid(cut=1))['state']!='route_unavailable'


def test_alternate_grid_convention_matches_planner():
    check=RoadRouteCheck([(0,0),(100,0)])
    e=check.sample(grid(positive=True,value_after=0),(0,0),0,0,True,50)
    assert e['state']=='route_unavailable'


def test_scalar_pose_yaw_and_quaternion_grid_origin_are_independent():
    check=RoadRouteCheck([(20,30),(20,130)])
    assert check.sample(grid(),(20,30),math.pi/2,0)['state']=='route_unavailable'
    g=grid();g.info.origin.orientation=NS(x=0.,y=0.,z=math.sqrt(.5),w=math.sqrt(.5))
    # Rotate grid 90 deg: vehicle-body (x,y)=(-grid_y,grid_x).
    g.info.origin.position=NS(x=10.,y=-1.)
    check=RoadRouteCheck([(0,1),(0,101)])
    assert check.sample(g,(0,1),0,0)['state']=='route_unavailable'


def test_bad_frame_and_nonquaternion_origin_are_unavailable():
    g=grid();g.header.frame_id='map'
    assert evidence(g=g)['state']=='unavailable'
    g=grid();g.info.origin.orientation.w=0
    assert evidence(g=g)['state']=='unavailable'


def test_duplicate_stamps_gaps_and_different_blocks_reset_confirmation():
    check=RoadRouteCheck([(0,0),(100,0)]);e=evidence(check)
    check.observe(0,1,e)
    for t in (.2,.4,.6,.8,1.,2.,3.): check.observe(t,1,e)
    assert not check.confirmed(3)
    check.observe(3,2,e)
    assert check.since==3
    for i in range(1,12): check.observe(3+i*.2,2+i,e)
    check.observe(5.4,20,dict(e,blocked_start_m=30,blocked_end_m=35))
    assert check.since==5.4 and not check.confirmed(5.4)
    check.observe(5.6,21,{'state':'unavailable','reason':'unknown'})
    assert check.since is None


@pytest.mark.parametrize('kwargs',[{'road_confirmation_sec':0},
    {'road_lookahead_m':100},{'road_corridor_half_width_m':11},
    {'road_blocked_span_m':float('nan')},{'road_lookahead_m':4}])
def test_invalid_settings(kwargs):
    with pytest.raises(ValueError): RoadLimits(**kwargs)
