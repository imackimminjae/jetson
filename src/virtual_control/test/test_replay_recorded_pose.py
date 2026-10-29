"""Select real completed zero-bias sessions without starting ROS publishers."""
import json
import os
from pathlib import Path
import sys

import pytest
import numpy as np
from geometry_msgs.msg import PoseStamped

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from replay_recorded_pose import (latest_zero_session,load_truth_data,map_yaw_alignment,
                                  truth_poses,geodetic_enu)


def recording(root,name,bias,scenario=2,completed=True,stamp=1):
    directory=root/name;session=directory/'sessions'/'job';(session/'bag').mkdir(parents=True)
    (session/'bag/metadata.yaml').write_text('')
    (directory/'plan.json').write_text(json.dumps({'bias_degrees':[bias],
        'jobs':[{'id':'job','scenario':f'scenario{scenario}'}]}))
    (directory/'progress.json').write_text(json.dumps({'completed':{'job':str(session)} if completed else {}}))
    os.utime(directory,(stamp,stamp));return session


def test_uses_latest_completed_zero_not_newer_live_or_nonzero_run(tmp_path):
    expected=recording(tmp_path,'manual_bias_old_zero',0,stamp=1)
    expected=recording(tmp_path,'manual_bias_new_zero',0,stamp=2)
    recording(tmp_path,'manual_bias_nonzero',5,stamp=3)
    recording(tmp_path,'manual_bias_incomplete',0,completed=False,stamp=4)
    assert latest_zero_session(tmp_path,2)==expected


def test_scenario_selection_and_missing_recording(tmp_path):
    expected=recording(tmp_path,'manual_bias_s2',0,stamp=1)
    recording(tmp_path,'manual_bias_s8',0,scenario=8,stamp=2)
    assert latest_zero_session(tmp_path,2)==expected
    with pytest.raises(ValueError,match='scenario1'):
        latest_zero_session(tmp_path,1)


def test_wgs84_axes_and_altitude_at_equator():
    # At lat=lon=0, ECEF +Y is east and +Z is north. This oracle does
    # not reuse the coordinate transform's trigonometric rotation matrix.
    radius=6378137.0
    longitude=np.degrees(np.arcsin(10/radius))
    east=geodetic_enu(np.array([0.]),np.array([longitude]),np.array([0.]),[0.,0.,0.])[0]
    assert east[0]==pytest.approx(10.,abs=1e-6)
    assert east[1]==pytest.approx(0.,abs=1e-6)
    up=geodetic_enu(np.array([0.]),np.array([0.]),np.array([7.]),[0.,0.,0.])[0]
    assert up==pytest.approx([0.,0.,7.],abs=1e-6)


def test_truth_keeps_independent_body_yaw_and_one_fixed_position_anchor():
    stamps=np.arange(10.,11.11,.1)
    east=10*(stamps-10)
    data=np.column_stack([stamps,np.zeros(len(stamps)),
                          np.degrees(np.arcsin(east/6378137.0)),
                          np.zeros(len(stamps)),np.zeros(len(stamps))])
    anchor=PoseStamped().pose
    anchor.position.x=100.;anchor.position.y=200.;anchor.position.z=3.
    anchor.orientation.z=-.3  # The output must use SIH body yaw instead.
    poses=truth_poses(data,10.,11.,0.,anchor,np.pi/2)
    assert poses[0][0]==0.
    assert poses[0][1].position.x==pytest.approx(100.)
    assert poses[0][1].position.y==pytest.approx(200.)
    elapsed,p=poses[-1]
    assert p.position.x==pytest.approx(100.,abs=1e-6)
    assert p.position.y==pytest.approx(200.+10*elapsed,abs=1e-6)
    # North-facing body becomes map -X, even though this example moves map +Y.
    assert abs(p.orientation.z)==pytest.approx(np.pi)
    assert p.orientation.w==0.


def test_truth_rejects_partial_coverage_and_gaps():
    data=np.column_stack([np.array([10.,10.1,10.8,10.9,11.]),np.zeros((5,4))])
    anchor=PoseStamped().pose
    with pytest.raises(ValueError,match='cover the full'):
        truth_poses(data,9.9,11.,0.,anchor,0.)
    with pytest.raises(ValueError,match='gap'):
        truth_poses(data,10.2,10.9,0.,anchor,0.)


def test_map_rotation_uses_alignment_for_selected_attempt(tmp_path):
    (tmp_path/'controllers.log').write_text(
        '[INFO] [10.000] [bridge]: Map alignment initialized once: yaw_delta=0.4\n'
        '[INFO] [20.000] [bridge]: Map alignment initialized once: yaw_delta=-1.2\n')
    assert map_yaw_alignment(tmp_path,15.)==.4
    assert map_yaw_alignment(tmp_path,25.)==-1.2
    with pytest.raises(ValueError,match='alignment is missing'):
        map_yaw_alignment(tmp_path,5.)


def test_truth_archive_rejects_duplicate_timestamps(tmp_path):
    path=tmp_path/'truth.npz'
    np.savez(path,message_type='HIL_STATE_QUATERNION',stamp_sec=[1.,1.],
             latitude_deg=[0.,0.],longitude_deg=[0.,0.],altitude_m=[0.,0.],yaw_ned_rad=[0.,0.])
    with pytest.raises(ValueError,match='strictly ordered'):
        load_truth_data(path)
