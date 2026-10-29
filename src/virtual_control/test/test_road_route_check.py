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


# Corroborate sparse-waypoint blockage against the current observed-free reference.
def reference_guard_grid(with_bypass=False):
    data=[]
    for iy in range(40):
        y=-10+iy*.5
        for ix in range(80):
            x=ix*.5
            free=x<10 or (with_bypass and abs(y+.25+5)<1.5)
            data.append(0 if free else 100)
    return NS(header=NS(frame_id='Vehicle'),data=data,
              info=NS(width=80,height=40,resolution=.5,
                      origin=NS(position=NS(x=0.,y=-10.),
                                orientation=NS(x=0.,y=0.,z=0.,w=1.))))


def reference_guard_sample(g,reference=None,check=None):
    check=check or RoadRouteCheck([(0,0),(100,0)])
    return check.sample(g,(0,0),0,0,reference_points=reference)


def test_sparse_waypoint_block_with_observed_free_current_bypass_is_map_mismatch():
    g=reference_guard_grid(True);reference=[(0,0),(8,-5),(22,-5)]
    before=reference_guard_sample(g)
    after=reference_guard_sample(g,reference)
    assert before['state']=='route_unavailable'
    assert after['state']=='route_map_mismatch'
    assert after['waypoint_corridor_state']=='route_unavailable'
    assert (after['blocked_start_m'],after['blocked_end_m'])==(before['blocked_start_m'],before['blocked_end_m'])
    assert after['reference_free_samples']>=2


def test_real_road_ending_still_confirms_with_reference_through_blockage():
    check=RoadRouteCheck([(0,0),(100,0)]);g=reference_guard_grid();reference=[(0,0),(25,0)]
    for i in range(16):
        now=i*.2;check.expire(now);e=reference_guard_sample(g,reference,check);check.observe(now,i+1,e)
        assert e['state']=='route_unavailable'
    assert check.confirmed(3.0)


def test_free_reference_ending_before_claimed_block_cannot_override_stop():
    assert reference_guard_sample(reference_guard_grid(True),[(0,0),(8,-5),(12,-5)])['state']=='route_unavailable'


def test_missing_stale_filtered_or_nonfinite_reference_cannot_override_stop():
    # Runner forwards None when stale, wrong-frame, invalid or before mission start.
    for reference in [None,[],[(0,0)],[(0,0),(math.nan,-5)]]:
        assert reference_guard_sample(reference_guard_grid(True),reference)['state']=='route_unavailable'


def test_unknown_or_outside_reference_cannot_override_stop():
    g=reference_guard_grid(True)
    for ix in range(20,80):g.data[10*80+ix]=-1
    assert reference_guard_sample(g,[(0,0),(8,-5),(22,-5)])['state']=='route_unavailable'
    assert reference_guard_sample(reference_guard_grid(True),[(0,0),(8,-5),(12,-12),(22,-12)])['state']=='route_unavailable'


def test_unrelated_free_reference_far_from_ego_cannot_override_stop():
    assert reference_guard_sample(reference_guard_grid(True),[(0,-5),(22,-5)])['state']=='route_unavailable'


def test_same_fixed_block_is_rechecked_before_corroboration_and_history_resets():
    check=RoadRouteCheck([(0,0),(100,0)]);g=reference_guard_grid(True)
    e=reference_guard_sample(g,None,check);check.observe(0,1,e)
    assert check.observations==1
    e=reference_guard_sample(g,[(0,0),(8,-5),(22,-5)],check)
    assert e['state']=='route_map_mismatch'
    check.observe(.2,2,e)
    assert check.since is None and check.block is None and check.observations==0


def test_one_metre_free_channel_is_too_narrow_to_override_stop():
    g=reference_guard_grid()
    for iy in range(g.info.height):
        y=-10+iy*.5+.25
        for ix in range(g.info.width):
            if ix*.5>=10 and abs(y+5)<.5:g.data[iy*80+ix]=0
    assert reference_guard_sample(g,[(0,0),(8,-5),(22,-5)])['state']=='route_unavailable'
