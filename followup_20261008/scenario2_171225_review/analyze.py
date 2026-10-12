from pathlib import Path
from collections import Counter
from datetime import datetime
from zoneinfo import ZoneInfo
import json,os,hashlib
import numpy as np,rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
H=Path(__file__).resolve().parent;B=H.parents[1]/'drive_batches/20261008_171225_scenario2_214e7e'
r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(B/'bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));names=['/motive/vehicle/odom_map','/debug/upper_branch_event','/debug/lower_mpc_trace','/navigation/global_path','/knu_batch/status','/rosout'];r.set_filter(rosbag2_py.StorageFilter(topics=names));types={x.name:get_message(x.type) for x in r.get_all_topics_and_types()};od=[];es=[];lo=[];route=None;ph=[];logs=[]
clock=lambda t:datetime.fromtimestamp(t,ZoneInfo('Asia/Seoul')).strftime('%H:%M:%S.%f')[:-3]
while r.has_next():
 n,b,t=r.read_next();t/=1e9;m=deserialize_message(b,types[n])
 if n==names[0]:od.append([t,m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.orientation.z,m.twist.twist.linear.x,m.twist.twist.linear.y])
 elif n==names[1]:es.append(json.loads(m.data))
 elif n==names[2]:lo.append(list(m.data))
 elif n==names[3] and route is None:route=[[p.pose.position.x,p.pose.position.y] for p in m.poses]
 elif n==names[4]:
  s=json.loads(m.data)
  if not ph or ph[-1]['phase']!=s['phase']:ph.append(s)
 elif n==names[5] and m.level>=30:logs.append(dict(t=t,name=m.name,msg=m.msg))
od=np.array(od);lo=np.array(lo);route=np.array(route);start=next(x['phase_stamp_sec'] for x in ph if x['phase']=='running');end=next(x['phase_stamp_sec'] for x in ph if x['phase']=='coasting');active=od[(od[:,0]>=start)&(od[:,0]<end)];ev=[x for x in es if start<=x['t_start']<end];wp=[]
for e in ev:
 if not wp or wp[-1]['pair']!=[e['wp0'],e['wp1']]:wp.append(dict(time=clock(e['t_start']),t=e['t_start'],pair=[e['wp0'],e['wp1']],xy=[e['x'],e['y']]))
near=[]
for i,p in enumerate(route):
 j=int(np.argmin(np.linalg.norm(active[:,1:3]-p,axis=1)));near.append(dict(waypoint=i,point=p.tolist(),time=clock(active[j,0]),xy=active[j,1:3].tolist(),distance=float(np.linalg.norm(active[j,1:3]-p))))
out=dict(session=str(B),route=route.tolist(),phases=ph,waypoint_changes=wp,nearest_waypoints=near,upper_count=len(ev),upper_invalid=sum(not e['valid_plan'] for e in ev),invalid_reasons=dict(Counter(e['reason'] for e in ev if not e['valid_plan'])),guard_hits=sum(e['side_boundary_tail_start']>=0 for e in ev),last_events=ev[-8:],moving_end=[dict(time=clock(v[0]),xy=v[1:3].tolist(),speed=float(np.hypot(v[4],v[5]))) for v in active[::100]],lower_modes=dict(Counter(int(v[12]) for v in lo if start<=v[0]<end)),result=json.loads((B/'session_result.json').read_text()))
(H/'summary.json').write_text(json.dumps(out,indent=2));(H/'events.json').write_text(json.dumps(es));(H/'warnings.json').write_text(json.dumps(logs,indent=2));(H/'sources_sha256.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [B/'session_result.json',B/'effective_config.yaml',*list((B/'bag').iterdir())]},indent=2))
os.environ['MPLCONFIGDIR']='/tmp/scenario2_171225_plot'
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,axs=plt.subplots(1,2,figsize=(12,5),layout='constrained');ax=axs[0];ax.plot(*route.T,'k--o',ms=4,label='Requested route');ax.plot(active[:,1],active[:,2],color='#16897a',label='Recorded vehicle');
for i,p in enumerate(route):ax.annotate(str(i),p,xytext=(3,6),textcoords='offset points')
ax.scatter(*active[-1,1:3],s=60,c='#c84b36',label='Stopped');ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend();ax.set_title('Scenario 2 - latest single run');ax.set_xlabel('X (m)');ax.set_ylabel('Y (m)')
ax=axs[1];ax.plot(*route.T,'k--o',ms=4);ax.plot(active[:,1],active[:,2],color='#16897a');ax.scatter(*active[-1,1:3],s=60,c='#c84b36');
for e in ev:
 if e['x']>-65 and e['valid_plan'] and e['plan_seq']%4==0:
  p=np.array(e['published_world']);ax.plot(*p.T,alpha=.3,color='#426fa3')
ax.set_xlim(-80,10);ax.set_ylim(50,115);ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_title('Final branch and stopping position');ax.set_xlabel('X (m)');ax.set_ylabel('Y (m)');fig.savefig(H/'track.png',dpi=140)
print(json.dumps({k:v for k,v in out.items() if k not in ['last_events','result','moving_end','phases']},indent=2));print('phases',[(s['phase'],clock(s['phase_stamp_sec'])) for s in ph]);print('lower last',lo[-1].tolist());print('warnings',logs[-8:])
