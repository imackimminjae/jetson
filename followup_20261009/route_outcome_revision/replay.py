"""Streaming counterfactual evidence check; never plays a bag or starts ROS nodes."""
from pathlib import Path
from collections import Counter
import csv,json,sys,time
import yaml
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

R=Path('/home/imac/ros2_ws');O=Path(__file__).resolve().parent
sys.path.insert(0,str(R/'src/virtual_control/scripts'))
from knu_batch_monitor import RouteMonitor,RoadRouteCheck,Limits
B=R/'drive_batches/20261009_070714_all_d8c3cf'
A=list(csv.DictReader((O.parent/'batch_070714_review/tables/per_attempt.csv').open()))
C=yaml.safe_load((B/'effective_config.yaml').read_text())['upper_planner_node']['ros__parameters']
assert not C.get('grid_origin_x_override_enabled',False)
assert not C.get('enable_waypoint_bias',False)
limits=Limits(**{k:yaml.safe_load((B/'effective_config.yaml').read_text())['knu_batch_runner']['ros__parameters'][k] for k in vars(Limits())})
topics=['/motive/vehicle/odom_map','/bev/occupancy_grid']
rd=rosbag2_py.SequentialReader()
rd.open(rosbag2_py.StorageOptions(uri=str(B/'bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
rd.set_filter(rosbag2_py.StorageFilter(topics=topics))
types={x.name:get_message(x.type) for x in rd.get_all_topics_and_types() if x.name in topics}
idx=0;pose=None;monitor=check=None;last_update=last_sample=-float('inf');stats=Counter();first=None;report=[];sample_times=[]
def initialize(a):
    points=[(float(x['x']),float(x['y'])) for x in csv.DictReader((B/'routes'/f'{a["route"]}.csv').open())]
    return RouteMonitor(points,float(a['start_epoch_s']),limits),RoadRouteCheck(points)
def finish(a):
    report.append(dict(token=a['token'],original_scenario=a['route'],original_outcome=a['outcome'],
        first_confirmed_epoch_s=first,first_confirmed_progress_ratio=first_progress,
        evidence_counts=dict(stats),note='Counterfactual classification along recorded positions; no replayed driving and no original relabeling.'))
    print(f"{idx+1}/60 {a['route']} A{a['attempt']}: {a['outcome']}, evidence-confirmed={first is not None}",flush=True)
first_progress=None
while rd.has_next():
    topic,blob,tn=rd.read_next();t=tn/1e9
    if idx>=len(A):break
    a=A[idx]
    while t>=float(a['end_epoch_s']):
        finish(a);idx+=1
        if idx>=len(A):break
        a=A[idx];monitor=check=None;pose=None;last_update=last_sample=-float('inf');stats=Counter();first=first_progress=None
    if idx>=len(A):break
    if t<float(a['start_epoch_s']):continue
    if monitor is None:monitor,check=initialize(a)
    if topic==topics[0]:
        m=deserialize_message(blob,types[topic]);p=m.pose.pose
        pose=(t,m.header.stamp.sec+m.header.stamp.nanosec*1e-9,p.position.x,p.position.y,p.orientation.z)
        if t-last_update>=.1:
            monitor.update(t,pose[2:4]);last_update=t
    elif t-last_sample>=.2 and pose is not None:
        last_sample=t;m=deserialize_message(blob,types[topic]);stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        started=time.perf_counter()
        check.expire(t)
        synchronized=(abs(stamp-pose[1])<=check.limits.road_pose_skew_sec if stamp>0
                      else stamp==0 and t-pose[0]<=check.limits.road_pose_skew_sec)
        if not synchronized or t-pose[0]>.6:
            e=dict(state='unavailable',reason='unsynchronized grid/pose stamps')
        else:
            e=check.sample(m,pose[2:4],pose[4],monitor.progress,C['grid_positive_is_drivable'],C['grid_value_threshold'],C['expected_grid_frame_id'])
        sample_times.append(time.perf_counter()-started)
        stats[e['state']]+=1;stats['reason:'+e['reason']]+=1
        check.observe(t,len(sample_times),e)
        if check.confirmed(t) and first is None:
            first=t;first_progress=monitor.progress/monitor.arc[-1]
if idx<len(A):finish(A[idx])
assert len(report)==60
out=dict(run_id=B.name,method='One grid at a time, sampled <=5Hz; pose scalar yaw, original nominal routes and recorded grid convention. Fixed evidence thresholds; original results untouched.',
    per_attempt=report,confirmed_by_original_outcome=dict(Counter(r['original_outcome'] for r in report if r['first_confirmed_epoch_s'] is not None)),
    samples=len(sample_times),sampling_max_ms=max(sample_times)*1000,confirmed_attempts=sum(r['first_confirmed_epoch_s'] is not None for r in report))
(O/'replay_summary.json').write_text(json.dumps(out,indent=2)+'\n')
print('Confirmed counts:',out['confirmed_by_original_outcome'],'; max sampling ms:',out['sampling_max_ms'],flush=True)
