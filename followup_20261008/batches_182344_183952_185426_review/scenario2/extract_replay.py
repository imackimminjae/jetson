from pathlib import Path
from collections import defaultdict,Counter
import json,gzip,bisect,math,hashlib,os
import numpy as np,yaml,rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
H=Path(__file__).resolve().parent;R=Path('/home/imac/ros2_ws');B=R/'drive_batches/20261008_183952_scenario2_5524df'
os.sched_setaffinity(0,{min(os.sched_getaffinity(0))})
def reader(names):
 r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(B/'bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));r.set_filter(rosbag2_py.StorageFilter(topics=names));return r
names=['/debug/upper_branch_event','/debug/upper_interval_centering_state','/motive/vehicle/odom_map','/debug/lower_mpc_trace','/debug/upper_stage_timing','/knu_batch/status','/navigation/global_path','/rosout','/debug/upper_miqp_trace']
r=reader(names);types={x.name:get_message(x.type) for x in r.get_all_topics_and_types()};d=defaultdict(list);ph=[];es=[];bs=[];odom=[];lower=[];route=None;timings=[]
while r.has_next():
 n,b,tn=r.read_next();t=tn/1e9;m=deserialize_message(b,types[n]);d[n].append(t)
 if n==names[0]:es.append(json.loads(m.data))
 elif n==names[1]:bs.append(list(m.data))
 elif n==names[2]:odom.append([t,m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.orientation.z,math.hypot(m.twist.twist.linear.x,m.twist.twist.linear.y)])
 elif n==names[3]:lower.append(list(m.data))
 elif n==names[4]:timings.append(json.loads(m.data))
 elif n==names[5]:
  s=json.loads(m.data)
  if not ph or (s['index'],s['phase'])!=(ph[-1]['index'],ph[-1]['phase']):ph.append(s)
 elif n==names[6] and route is None:route=[[p.pose.position.x,p.pose.position.y] for p in m.poses]
bs=np.array(bs);odom=np.array(odom);lower=np.array(lower);cycles=[];attempts=[]
for i in [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]:
 ps=[s for s in ph if s['index']==i];ref=next(s['phase_stamp_sec'] for s in ps if s['phase']=='reference');start=next(s['phase_stamp_sec'] for s in ps if s['phase']=='running');end=next(s['phase_stamp_sec'] for s in ps if s['phase']=='coasting');stop=next((s['phase_stamp_sec'] for s in ph if s['index']==i+1),ph[-1]['phase_stamp_sec']);cs=[];prev=False
 for e in es:
  t=e['t_start']
  if not ref<=t<end:continue
  j=int(np.argmin(abs(bs[:,0]-t)));assert abs(bs[j,0]-t)<.1
  oi=bisect.bisect_right(odom[:,0],t)-1
  c=dict(t=t,x=e['x'],y=e['y'],yaw=e['yaw_rad'],v=odom[oi,4],rec_wp0=e['wp0'],rec_wp1=e['wp1'],rec_valid=e['valid_plan'],rec_reason=e['reason'],rec_status=e['status'],rec_planned=e['planned_world'],rec_published=e['published_world'],rec_preview=e['preview_world'],rec_B_before=bs[j,1],rec_B=bs[j,2],rec_guard=e['side_boundary_tail_start'],rec_used_previous=prev,rec_intervals=e['intervals_world'],running=t>=start,attempt=i)
  cs.append(c);cycles.append(c)
  if e['valid_plan']:prev=True
 attempts.append(dict(attempt=i,start=start,end=end,stop=stop,cycles=cs))
req=iter(cycles);cur=next(req,None);r=reader(['/bev/occupancy_grid']);last=None;idx=0;gt=[];geometry=Counter()
with gzip.open(H/'grids.bin.gz','wb',compresslevel=6) as f:
 while r.has_next():
  _,b,tn=r.read_next();t=tn/1e9;gt.append(t)
  while cur is not None and t>cur['t']:
   assert last is not None
   time,blob=last;g=deserialize_message(blob,types['/bev/occupancy_grid']);assert (g.info.width,g.info.height,g.info.origin.position.x,g.info.origin.position.y)==(128,128,0,-20)
   assert abs(g.info.resolution-.3125)<1e-9
   data=np.frombuffer(bytes(g.data),dtype=np.int8).reshape(128,128);f.write(data.tobytes());cur.update(grid_index=idx,grid_t=time,grid_age=cur['t']-time,grid_free=int(np.count_nonzero(data<=1)),grid_near_free=int(np.count_nonzero(data[58:70,:16]<=1)));idx+=1;cur=next(req,None)
  last=(t,b)
assert cur is None
manifest=[]
for a in attempts:
 p=H/f'input_{a["attempt"]}.json';p.write_text(json.dumps(dict(route=route,cycles=a.pop('cycles'))));manifest.append(dict(**a,input=str(p),grids=str(H/'grids.bin.gz')))
(H/'attempts.json').write_text(json.dumps(manifest,indent=2));(H/'phases.json').write_text(json.dumps(ph,indent=2))
with gzip.open(H/'replay_telemetry.json.gz','wt') as f:json.dump(dict(odom=odom.tolist(),lower=lower.tolist(),timings=timings,receive_times={**d,'/bev/occupancy_grid':gt}),f)
cfg=yaml.safe_load((B/'effective_config.yaml').read_text());(H/'config.yaml').write_text(yaml.safe_dump({'upper_planner_node':cfg['upper_planner_node']}))
files=[*list((B/'bag').iterdir()),B/'effective_config.yaml',B/'controllers.log',B/'session_result.json',R/'src/virtual_control/scripts/repeat_drive_session.py',R/'src/virtual_control/scripts/knu_batch_runner.py',R/'src/virtual_control/src/tracking_control.cpp',R/'src/virtual_control/include/virtual_control/tracking_control.hpp',R/'src/virtual_control/include/virtual_control/egocentric_planner_geometry.hpp']
(H/'replay_sources_sha256.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},indent=2));print('cycles',idx,'compressed grids bytes',(H/'grids.bin.gz').stat().st_size)
