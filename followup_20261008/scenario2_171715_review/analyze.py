from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
from collections import Counter
import json,csv,os
import numpy as np,rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
H=Path(__file__).resolve().parent;B=H.parents[1]/'drive_batches/20261008_171715_scenario2_592ff2'
r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(B/'bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));topics={x.name:get_message(x.type) for x in r.get_all_topics_and_types()};wanted=['/knu_batch/status','/motive/vehicle/odom_map','/debug/upper_branch_event','/navigation/global_path'];r.set_filter(rosbag2_py.StorageFilter(topics=wanted));ph=[];od=[];es=[];route=None
clock=lambda t:datetime.fromtimestamp(t,ZoneInfo('Asia/Seoul')).strftime('%H:%M:%S.%f')[:-3]
while r.has_next():
 n,b,t=r.read_next();t/=1e9;m=deserialize_message(b,topics[n])
 if n==wanted[0]:
  v=json.loads(m.data)
  if not ph or (v['index'],v['phase'])!=(ph[-1]['index'],ph[-1]['phase']):ph.append(v)
 elif n==wanted[1]:od.append([t,m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.orientation.z,m.twist.twist.linear.x,m.twist.twist.linear.y])
 elif n==wanted[2]:es.append(json.loads(m.data))
 elif n==wanted[3] and route is None:route=[[p.pose.position.x,p.pose.position.y] for p in m.poses]
od=np.array(od);route=np.array(route);rows=list(csv.DictReader(next((B/'results').glob('*/summary.csv')).open()));summary=[]
os.environ['MPLCONFIGDIR']='/tmp/scenario2_171715_plot'
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,ax=plt.subplots(figsize=(10,5),layout='constrained');ax.plot(*route.T,'k--o',label='Requested route',ms=3)
for i,row in enumerate(rows,1):
 start=next(s['phase_stamp_sec'] for s in ph if s['index']==i and s['phase']=='running');end=next(s['phase_stamp_sec'] for s in ph if s['index']==i and s['phase']=='coasting');stop=next((s['phase_stamp_sec'] for s in ph if s['index']==i+1),ph[-1]['phase_stamp_sec']);tr=od[(od[:,0]>=start)&(od[:,0]<stop)];ev=[e for e in es if start<=e['t_start']<end];ax.plot(tr[:,1],tr[:,2],lw=2 if i==7 else 1,alpha=1 if i==7 else .6,label=f'{i}: {row["outcome"]}',color='#c44b35' if i==7 else None)
 z=dict(attempt=i,start=clock(start),end=clock(end),result=row,upper_count=len(ev),invalid=sum(not e['valid_plan'] for e in ev),transitions=[])
 prev=None
 for e in ev:
  key=(e['valid_plan'],e['reason'])
  if key!=prev:z['transitions'].append(dict(time=clock(e['t_start']),xy=[e['x'],e['y']],valid=e['valid_plan'],reason=e['reason'],horizon=e['horizon'],end=e['published_world'][-1:]));prev=key
 if i==7:
  z['branch_events']=[dict(time=clock(e['t_start']),xy=[e['x'],e['y']],yaw=e['yaw_rad'],valid=e['valid_plan'],end=e['published_world'][-1:],counts=e['candidate_counts'],selected=e['selected_corridors']) for e in ev if -170<e['x']<-115]
 summary.append(z)
ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_xlabel('X (m)');ax.set_ylabel('Y (m)');ax.legend(fontsize=8);ax.set_title('Scenario 2: attempts 1-7; batch aborted before attempt 8');fig.savefig(H/'tracks.png',dpi=140)
out=dict(session=str(B),attempts=summary,phases=ph,abort=json.loads(next((B/'results').glob('*/abort.json')).read_text()),diagnostic_topics_present={n:n in topics for n in ['/knu_batch/recorder_ready','/knu_batch/abort']});(H/'summary.json').write_text(json.dumps(out,indent=2));print('PHASES',[(s['index'],s['phase'],clock(s['phase_stamp_sec'])) for s in ph[-12:]]);print('ATTEMPT7',json.dumps(summary[-1],indent=2));print('diagnostic topics',out['diagnostic_topics_present'])
