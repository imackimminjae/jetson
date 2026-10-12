"""Execute candidate state machine without ROS nodes, sockets, services, or vehicle outputs."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace, MethodType
import sys
import pytest
H=Path(__file__).resolve().parent
sys.path.insert(0,'/home/imac/ros2_ws/src/virtual_control/scripts')
spec=importlib.util.spec_from_file_location('candidate_batch',H/'candidate/knu_batch_runner.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
from knu_batch_monitor import Limits,RouteMonitor
from nav_msgs.msg import Odometry,OccupancyGrid
from std_msgs.msg import Bool

@pytest.fixture
def h(tmp_path,monkeypatch):
 clock=[100.];monkeypatch.setattr(m,'time',SimpleNamespace(monotonic=lambda:clock[0]))
 settings=dict(require_matlab_ack=False,pose_fresh_sec=.6,grid_fresh_sec=2.,pose_loss_sec=10.,grid_loss_sec=10.,recorder_timeout_sec=2.,require_recorder=True,stationary_timeout_sec=60.,input_recovery_stable_sec=30.,input_recovery_timeout_sec=120.,route_timeout_sec=180.,setup_timeout_sec=30.)
 r=SimpleNamespace(settings=settings,phase='running',index=0,routes=['scenario1']*3,route='scenario1',token='test:0:scenario1',session='test',phase_sequence=0,phase_stamp_sec=100.,entered=100.,attempt_started=100.,pose_at=100.,grid_at=100.,reference_at=100.,recorder_at=100.,recorder_seen=True,recorder_ok=True,pose=(0.,0.,0.),input_recovery_needed=False,input_recovery_reason='',input_recovery_stable_since=None,stationary_since=None,rows=[],last_result=None,last_status_at=100.,output=tmp_path,limits=Limits(),points={'scenario1':[(0.,0.,0.),(1000.,0.,0.)]},monitor=RouteMonitor([(0,0),(1000,0)],100.),future=None,upper_reset_future=None,logger_required=False)
 r.fields=['route','token','outcome','duration_sec','progress_ratio','best_progress_ratio','cross_track_m','goal_distance_m','detail','stop_status','stop_x','stop_y','stop_goal_distance_m']
 for name,f in m.KnuBatchRunner.__dict__.items():
  if callable(f) and not name.startswith('__'):setattr(r,name,MethodType(f,r))
 permissions=[];statuses=[];logs=[]
 r.get_logger=lambda:SimpleNamespace(info=logs.append,warn=logs.append,error=logs.append)
 r.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=int(clock[0]*1e9)))
 r.permission_pub=SimpleNamespace(publish=lambda msg:permissions.append(msg.data));r.status_pub=SimpleNamespace(publish=lambda msg:statuses.append(json.loads(msg.data)));r.route_pub=SimpleNamespace(publish=lambda msg:None)
 def step(dt=.1,pose=True,grid=True,speed=0.,recorder=True,tick=True):
  clock[0]+=dt
  if pose:
   msg=Odometry();msg.header.frame_id='map';msg.twist.twist.linear.x=speed;r.on_pose(msg)
  if grid:
   msg=OccupancyGrid();msg.info.width=msg.info.height=1;msg.info.resolution=1.;msg.data=[0];r.on_grid(msg)
  if recorder is not None:r.on_recorder(Bool(data=recorder))
  if tick:r.tick()
 def pump(sec,**kwargs):
  for _ in range(round(sec/.1)):step(**kwargs)
 return SimpleNamespace(r=r,clock=clock,step=step,pump=pump,permissions=permissions,statuses=statuses)

def recovering(h):
 h.r.fail_or_finish('pose_timeout');h.r.require_input_recovery('pose stale');h.pump(1.3)
 assert h.r.phase=='recovering';assert h.r.rows[0]['stop_status']=='stationary'

def test_healthy_offroute_retains_existing_next_attempt_behavior(h):
 h.r.fail_or_finish('off_route');h.pump(1.3)
 assert h.r.phase=='stopping' and h.r.index==1
 assert not h.r.input_recovery_needed

def test_pose_outage_is_latched_even_if_callback_recovers_before_tick(h):
 h.step(dt=.8,tick=False)
 assert h.r.input_recovery_needed
 h.r.fail_or_finish('off_route');h.pump(1.3)
 assert h.r.phase=='recovering' and h.r.index==0

def test_thirty_seconds_required_before_next_route_and_permission_stays_false(h):
 recovering(h);h.pump(29.)
 assert h.r.phase=='recovering' and h.r.index==0
 assert not any(h.permissions)
 h.pump(1.3)
 assert h.r.phase=='stopping' and h.r.index==1
 assert not h.r.input_recovery_needed
 assert len(h.r.rows)==1

def test_repeated_pose_outages_restart_window_and_cannot_consume_attempts(h):
 recovering(h)
 for _ in range(3):
  h.pump(19.);h.pump(1.,pose=False);h.step()
  assert h.r.phase=='recovering' and h.r.index==0
 h.pump(29.)
 assert h.r.phase=='recovering'
 h.pump(1.3)
 assert h.r.index==1 and h.r.phase=='stopping'

def test_new_grid_gap_invalidates_stability(h):
 recovering(h);h.pump(25.);h.pump(2.3,grid=False);h.step();h.pump(10.)
 assert h.r.phase=='recovering' and h.r.index==0

def test_motion_invalidates_stationary_recovery_window(h):
 recovering(h);h.pump(25.);h.step(speed=.21);h.pump(10.)
 assert h.r.phase=='recovering' and h.r.index==0

def test_recorder_failure_still_aborts_immediately(h):
 recovering(h);h.step(recorder=False)
 assert h.r.phase=='aborting' and h.r.index==0
 assert 'recorder' in json.loads((h.r.output/'abort.json').read_text())['reason']
 assert not h.permissions[-1]

def test_recovery_deadline_aborts_without_new_alignment_or_attempt(h):
 recovering(h)
 for _ in range(6):
  h.pump(19.);h.pump(1.,pose=False)
 assert h.r.phase=='aborting' and h.r.index==0 and len(h.r.rows)==1
 assert 'input recovery timeout' in json.loads((h.r.output/'abort.json').read_text())['reason']

def test_final_attempt_does_not_wait_for_unneeded_recovery(h):
 h.r.index=2;recover_needed=h.r.require_input_recovery('pose stale');h.r.fail_or_finish('pose_timeout');h.pump(1.3)
 assert h.r.phase=='done' and len(h.r.rows)==1

def test_malformed_pose_cannot_refresh_recovery(h):
 recovering(h);h.pump(25.);h.pump(1.,pose=False)
 bad=Odometry();bad.header.frame_id='map';bad.pose.pose.position.x=float('nan');before=h.r.pose_at;h.r.on_pose(bad)
 assert h.r.pose_at==before
 h.pump(10.);assert h.r.phase=='recovering'

def test_grid_timeout_and_stop_timeout_keep_distinct_outcomes(h):
 h.pump(10.2,grid=False)
 assert h.r.phase=='coasting' and h.r.rows[-1]['outcome']=='grid_timeout'
 h.pump(60.2,pose=False,grid=False)
 assert h.r.phase=='aborted' and h.r.index==0
 assert 'stop_timeout' in json.loads((h.r.output/'abort.json').read_text())['reason']
