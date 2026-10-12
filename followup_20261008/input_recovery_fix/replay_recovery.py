"""Recorded receive times with the candidate state machine. No ROS node or executor."""
from pathlib import Path
import gzip,json,bisect
from types import SimpleNamespace
import pytest
from test_recovery import h
from nav_msgs.msg import Odometry,OccupancyGrid
from std_msgs.msg import Bool
H=Path(__file__).resolve().parent;P=H.parent/'scenario1_detailed_review';s=json.loads((P/'summary.json').read_text());d=json.loads(gzip.decompress((P/'telemetry.json.gz').read_bytes()));z=s['attempts'][3];start=z['end'];out=H/'trace_replay';out.mkdir(exist_ok=True)
mp=pytest.MonkeyPatch();v=h.__wrapped__(out,mp);r=v.r;r.routes=['scenario1']*10;r.index=3;r.token='recorded:3:scenario1';r.attempt_started=100.-(z['end']-z['start']);r.entered=r.attempt_started
od=d['data']['/motive/vehicle/odom_map'];times=[t for t,_ in od];j=bisect.bisect_right(times,start)-1;p=od[j][1];r.pose=(p[0],p[1],(p[3]**2+p[4]**2)**.5);r.pose_at=100.+times[j]-start;r.grid_at=100.;r.fail_or_finish('off_route')
events=[(t,'pose',a) for t,a in od if start<t<start+180];events.extend((t,'grid',None) for t in d['receive_times']['/bev/occupancy_grid'] if start<t<start+180);events.sort(key=lambda q:q[0]);cursor=0;trans=[];last=None;all_false=True
for n in range(1801):
 t=start+n*.1
 while cursor<len(events) and events[cursor][0]<=t:
  et,kind,a=events[cursor];v.clock[0]=100.+et-start
  if kind=='pose':
   msg=Odometry();msg.header.frame_id=a[6];msg.pose.pose.position.x=a[0];msg.pose.pose.position.y=a[1];msg.pose.pose.position.z=a[7];msg.twist.twist.linear.x=a[3];msg.twist.twist.linear.y=a[4];r.on_pose(msg)
  else:
   msg=OccupancyGrid();msg.info.width=msg.info.height=1;msg.info.resolution=1.;msg.data=[0];r.on_grid(msg)
  cursor+=1
 v.clock[0]=100.+n*.1
 # Readiness wasn't recorded. Assume continuously ready to isolate pose/grid gate;
 # a recorder fault would cause the pre-existing earlier abort instead.
 r.on_recorder(Bool(data=True));r.tick();all_false=all_false and not v.permissions[-1]
 key=(r.phase,r.index)
 if key!=last:trans.append(dict(recorded_t=t,seconds_from_previous_outcome=n*.1,phase=r.phase,index=r.index+1,rows=len(r.rows)));last=key
 if r.phase=='aborted':break
assert r.phase=='aborted' and r.index==3 and len(r.rows)==1 and all_false
# Upper bound on eligibility: even if stationary, the pose-only stable window
# after recovery is shorter than 30 seconds for each old attempt 5..9.
rx=d['receive_times']['/motive/vehicle/odom_map'];gaps=[(a,b) for a,b in zip(rx[:-1],rx[1:]) if b-a>.6];rows=[]
for z in s['attempts'][4:]:
 previous=[(a,b) for a,b in gaps if b<z['start']];nextgap=[(a,b) for a,b in gaps if a>z['start']];recovered=previous[-1][1]
 rows.append(dict(attempt=z['attempt'],seconds_since_recovery_at_old_start=z['start']-recovered,pose_continuous_window_until_next_gap_s=nextgap[0][0]-recovered if nextgap else None,eligible_at_old_start=z['start']-recovered>=30))
(H/'recorded_replay.json').write_text(json.dumps(dict(transitions=trans,never_allowed_drive=all_false,attempts_added=0,recorded_start_comparison=rows,assumptions=['Recorder continuously ready (topic absent in original).','All recorded pose/grid times retained after hypothetical hold; not a closed-loop trajectory.','Grid metadata assumed valid, matching original bag extraction.']),indent=2))
print(json.dumps(trans,indent=2));print(rows);mp.undo()
