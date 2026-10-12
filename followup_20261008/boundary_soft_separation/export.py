from pathlib import Path
import json,bisect,math,gzip,hashlib,yaml
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
H=Path(__file__).resolve().parent;R=H.parents[1];B=R/'drive_batches/20261008_062606_scenario3_5cea23';review=H.parent/'batch_062606_review'
def reader(topics):
 r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(B/'bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));r.set_filter(rosbag2_py.StorageFilter(topics=topics));return r
names=['/debug/upper_branch_event','/debug/upper_interval_centering_state','/motive/vehicle/odom_map','/navigation/global_path','/parameter_events']
r=reader(names);types={x.name:get_message(x.type) for x in r.get_all_topics_and_types()};es=[];bs=[];od=[];route=None;params={}
while r.has_next():
 n,b,t=r.read_next();m=deserialize_message(b,types[n]);t/=1e9
 if n==names[0]:es.append(dict(t=t,**json.loads(m.data)))
 elif n==names[1]:bs.append(list(m.data))
 elif n==names[2]:od.append((t,math.hypot(m.twist.twist.linear.x,m.twist.twist.linear.y)))
 elif n==names[3] and route is None:route=[[p.pose.position.x,p.pose.position.y] for p in m.poses]
 elif n==names[4]:
  for p in [*m.new_parameters,*m.changed_parameters]:
   if m.node=='/upper_planner_node':params[p.name]={1:p.value.bool_value,2:p.value.integer_value,3:p.value.double_value,4:p.value.string_value}.get(p.value.type)
assert es==json.loads((review/'events.json').read_text())
summary=json.loads((review/'summary.json').read_text());bt=[x[0] for x in bs];ot=[x[0] for x in od];allcycles=[];manifest=[]
for a in summary['attempts']:
 cycles=[];previous=[]
 for e in es:
  if not (a['reference_time']<=e['t_start']<a['end']):continue
  t=e['t_start'];bi=min(range(max(0,bisect.bisect_left(bt,t)-1),min(len(bt),bisect.bisect_left(bt,t)+2)),key=lambda i:abs(bt[i]-t));b=bs[bi];assert abs(b[0]-t)<.1
  oi=bisect.bisect_right(ot,t)-1
  c=dict(t=t,x=e['x'],y=e['y'],yaw=e['yaw_rad'],v=od[oi][1],rec_wp0=e['wp0'],rec_wp1=e['wp1'],rec_valid=e['valid_plan'],rec_planned=e['planned_world'],rec_published=e['published_world'],rec_preview=e['preview_world'],rec_B_before=b[1],rec_B=b[2],rec_used_previous=bool(previous),rec_guard=e['side_boundary_tail_start'],rec_intervals=e['intervals_world'],running=t>=a['start'])
  cycles.append(c)
  if e['valid_plan']:previous=e['planned_world']
 assert cycles[0]['rec_B_before']==0 and cycles[0]['rec_wp0']==1
 tag=f'batch{a["attempt"]:02d}';manifest.append(dict(tag=tag,input=str(H/f'input_{tag}.json'),grids=str(H/'batch_grids.bin.gz'),config=str(H/'batch_config.yaml')));allcycles+=cycles
 (H/f'input_{tag}.json').write_text(json.dumps(dict(route=route,cycles=cycles)))
# One streaming map scan; keep only the prior serialized frame. Only solve-time maps persist, compressed.
req=iter(sorted(allcycles,key=lambda c:c['t']));cur=next(req,None);r=reader(['/bev/occupancy_grid']);last=None;idx=0
with gzip.open(H/'batch_grids.bin.gz','wb',compresslevel=9) as f:
 while r.has_next():
  _,b,t=r.read_next();t/=1e9
  while cur is not None and t>cur['t']:
   assert last is not None
   gt,blob=last;g=deserialize_message(blob,types['/bev/occupancy_grid']);assert (g.info.width,g.info.height,g.info.origin.position.x,g.info.origin.position.y)==(128,128,0,-20)
   assert abs(g.info.resolution-.3125)<1e-9
   f.write(bytes(g.data));cur['grid_index']=idx;cur['grid_receive_t']=gt;idx+=1;cur=next(req,None)
  last=(t,b)
assert cur is None
for a in manifest:
 tag=a['tag'];i=int(tag[-2:]);cs=[c for c in allcycles if summary['attempts'][i-1]['reference_time']<=c['t']<summary['attempts'][i-1]['end']];Path(a['input']).write_text(json.dumps(dict(route=route,cycles=cs)))
cfg=yaml.safe_load((B/'effective_config.yaml').read_text());p=cfg['upper_planner_node']['ros__parameters'];(H/'batch_config.yaml').write_text(yaml.safe_dump({'upper_planner_node':{'ros__parameters':p}}))
current=yaml.safe_load((R/'src/virtual_control/config/tracking_control_split.yaml').read_text())['upper_planner_node']['ros__parameters']
audit=dict(raw_events_identical=True,total_raw_events=len(es),replay_cycles=idx,first_states=[dict(attempt=a['attempt'],B=a['first_B_before'],wp=a['first_wp'],revision=a['first_revision']) for a in summary['attempts']],recorded_upper_params=params,config_differences={k:[p.get(k),current.get(k)] for k in p.keys()|current.keys() if p.get(k)!=current.get(k)},raw_hashes={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in [*list((B/'bag').iterdir()),B/'effective_config.yaml',B/'session_result.json']})
assert audit['raw_hashes']==json.loads((review/'sources_sha256.json').read_text())
(H/'input_audit.json').write_text(json.dumps(audit,indent=2));(H/'batch_manifest.json').write_text(json.dumps(manifest,indent=2));print('verified raw events/hashes; replay cycles',idx,'compressed maps bytes',(H/'batch_grids.bin.gz').stat().st_size,'config differences',audit['config_differences'])
