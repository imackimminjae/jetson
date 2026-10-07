from pathlib import Path
from collections import defaultdict, Counter
from datetime import datetime
from zoneinfo import ZoneInfo
import os,json,csv,hashlib
import numpy as np,yaml,rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.signal import butter,sosfiltfilt
ROOT=Path('/home/imac/ros2_ws');H=Path(__file__).resolve().parent
B=ROOT/'drive_batches/20261008_062606_scenario3_5cea23'
clock=lambda t:datetime.fromtimestamp(float(t),ZoneInfo('Asia/Seoul')).strftime('%H:%M:%S.%f')[:-3]
r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(B/'bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
meta=yaml.safe_load((B/'bag/metadata.yaml').read_text())['rosbag2_bagfile_information']
counts={x['topic_metadata']['name']:x['message_count'] for x in meta['topics_with_message_count']}
types={x.name:get_message(x.type) for x in r.get_all_topics_and_types()};r.set_filter(rosbag2_py.StorageFilter(topics=[n for n in types if n!='/bev/occupancy_grid']))
d=defaultdict(list);params=[]
while r.has_next():
 n,blob,tn=r.read_next();t=tn/1e9;m=deserialize_message(blob,types[n])
 if n in ['/knu_batch/status','/knu_batch/active_route','/debug/upper_branch_event','/debug/upper_stage_timing']:
  try:v=json.loads(m.data)
  except:continue
 elif n=='/motive/vehicle/odom_map':v=[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.orientation.z,m.twist.twist.linear.x,m.twist.twist.linear.y]
 elif n.startswith('/debug/') and hasattr(m,'data'):v=list(m.data)
 elif n=='/navigation/global_path':v=[[p.pose.position.x,p.pose.position.y] for p in m.poses]
 elif n=='/planner/lower_reference_path/with_arclength':v=[[p.pose.position.x,p.pose.position.y] for p in m.path.poses]
 elif n=='/rosout':v=dict(name=m.name,level=m.level,msg=m.msg)
 elif n in ['/planner/goal_reached','/controller/reset']:v=m.data
 elif n=='/parameter_events':
  for p in [*m.new_parameters,*m.changed_parameters]:
   if p.name in ['guard_side_boundary_terminal_loss','enable_batch_mission_reset','reset_upper_each_attempt','upper_preview_steps','goal_stop_distance_m','mavlink_auto_arm','mavlink_force_arm','configure_px4_parameters']:
    params.append(dict(t=t,node=m.node,name=p.name,value={1:p.value.bool_value,2:p.value.integer_value,3:p.value.double_value}.get(p.value.type)))
  continue
 else:continue
 d[n].append((t,v))
phases=[];prev=None
for t,s in d['/knu_batch/status']:
 key=(s['index'],s['phase'],s['sequence'])
 if key!=prev:phases.append(dict(t=s['phase_stamp_sec'],received_t=t,time=clock(s['phase_stamp_sec']),**s));prev=key
route=np.array(d['/navigation/global_path'][0][1]);ot=np.array([x[0] for x in d['/motive/vehicle/odom_map']]);od=np.array([x[1] for x in d['/motive/vehicle/odom_map']]);lt=np.array([x[0] for x in d['/debug/lower_mpc_trace']]);lo=np.array([x[1] for x in d['/debug/lower_mpc_trace']]);es=[dict(t=t,**e) for t,e in d['/debug/upper_branch_event']];bt=np.array([v[0] for _,v in d['/debug/upper_interval_centering_state']]);bs=np.array([v for _,v in d['/debug/upper_interval_centering_state']]);csvrows=list(csv.DictReader(next((B/'results').glob('*/summary.csv')).open()))
def quant(v):return dict(zip(['min','median','p95','max'],map(float,np.quantile(v,[0,.5,.95,1])))) if len(v) else None
rows=[];tracks=[]
for i in range(1,11):
 ps=[p for p in phases if p['index']==i];start=next(p['t'] for p in ps if p['phase']=='running');end=next(p['t'] for p in ps if p['phase']=='coasting');stop=next((p['t'] for p in phases if p['index']==i+1),phases[-1]['t']);ref=next(p['t'] for p in ps if p['phase']=='reference');c=csvrows[i-1]
 tr=lo[(lt>=start)&(lt<end)];ev=[e for e in es if start<=e['t_start']<end];first=next(e for e in es if e['t_start']>=ref);bi=int(np.argmin(abs(bt-first['t_emit'])));active=(lo[:,12]==0)&(lo[:,4]>1)&(lt>=start)&(lt<end);active_tr=lo[active]
 neutral=lo[(lo[:,0]>start+2)&(lo[:,0]<=stop)&(lo[:,12]==4)][0];n=neutral[0];nxy=neutral[1:3];sxy=np.array([float(c['stop_x']),float(c['stop_y'])]);m=(ot>=start)&(ot<n)
 tracks.append(dict(attempt=i,t=ot[m].tolist(),xy=od[m,:2].tolist(),coast_xy=od[(ot>=n)&(ot<stop),:2].tolist()))
 hp=[];inds=np.flatnonzero(active)
 for chunk in np.split(inds,np.flatnonzero((np.diff(inds)>1)|(np.diff(lt[inds])>.3))+1):
  if len(chunk)<20:continue
  tt=np.arange(lt[chunk[0]],lt[chunk[-1]],.1);vv=np.interp(tt,lt[chunk],lo[chunk,26]);hp.extend(sosfiltfilt(butter(2,.3,fs=10,btype='highpass',output='sos'),vv))
 prev=None;shifts=[]
 for e in ev:
  if not e['valid_plan']:continue
  plan=np.array(e['published_world'])
  if len(plan)<2:continue
  if prev is not None and 0<e['t_start']-prev['t_start']<.8:
   old=np.array(prev['published_world']);v=np.diff(old,axis=0);ll=np.linalg.norm(v,axis=1);u=np.clip(np.sum((plan[1]-old[:-1])*v,axis=1)/np.maximum(ll**2,1e-12),0,1);q=old[:-1]+u[:,None]*v;j=int(np.argmin(np.linalg.norm(plan[1]-q,axis=1)));shift=float(np.cross(v[j],plan[1]-q[j])/max(ll[j],1e-12));k=int(np.argmin(abs(bt-e['t_start'])))
   shifts.append(dict(t=e['t_start'],time=clock(e['t_start']),xy=[e['x'],e['y']],lateral_m=shift,B_before=float(bs[k,1]),B=float(bs[k,2])))
  prev=e
 active_widths=bs[(bt>=start)&(bt<end),2];sig=active_tr[abs(active_tr[:,26])>np.deg2rad(1)];flips=int(sum(np.sign(x[26])!=np.sign(y[26]) for x,y in zip(sig[:-1],sig[1:]) if 0<y[0]-x[0]<.7))
 row=dict(attempt=i,outcome=c['outcome'],start=start,start_clock=clock(start),end=end,end_clock=clock(end),reference_time=ref,stop_time=stop,drive_seconds=n-start,first_neutral_time=n,neutral_xy=nxy.tolist(),neutral_speed=float(neutral[4]),stop_goal_m=float(c['stop_goal_distance_m']),goal_at_trigger_m=float(c['goal_distance_m']),coast_seconds=stop-n,coast_displacement_m=float(np.linalg.norm(sxy-nxy)),upper_count=len(ev),upper_valid=sum(e['valid_plan'] for e in ev),upper_failed=sum(not e['valid_plan'] for e in ev),horizons=dict(Counter(e['horizon'] for e in ev)),guard_hits=sum(e.get('side_boundary_tail_start',-1)>=0 for e in ev),first_revision=first['path_revision'],first_wp=[first['wp0'],first['wp1']],first_B_before=float(bs[bi,1]),first_plan_valid=first['valid_plan'],first_xy=[first['x'],first['y']],first_heading_deg=float(np.degrees(first['yaw_rad'])),speed=quant(active_tr[:,4]),lateral_abs=quant(abs(active_tr[:,6])),sat_percent=float(np.mean(abs(active_tr[:,26])>=.29845)*100),highpass_rms_deg=float(np.sqrt(np.mean(np.degrees(hp)**2))),sign_flips=flips,B_range=[float(np.min(active_widths)),float(np.max(active_widths))],largest_k1_shifts=sorted(shifts,key=lambda x:abs(x['lateral_m']),reverse=True)[:4],lower_modes=dict(Counter(int(x[12]) for x in tr)),lower_max_gap=float(np.max(np.diff(lt[(lt>=start)&(lt<end)]))),odom_max_gap=float(np.max(np.diff(ot[(ot>=start)&(ot<end)]))),failure_events=[dict(time=clock(e['t_start']),t=e['t_start'],xy=[e['x'],e['y']],status=e['status']) for e in ev if not e['valid_plan']])
 rows.append(row)
summary=dict(session=str(B),route=route.tolist(),counts=counts,phases=phases,attempts=rows,parameters=params,bag_bytes=sum(p.stat().st_size for p in (B/'bag').iterdir()),total_bytes=sum(p.stat().st_size for p in B.rglob('*') if p.is_file()))
(H/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');(H/'events.json').write_text(json.dumps(es,indent=2)+'\n');(H/'tracks.json').write_text(json.dumps(tracks)+'\n');(H/'rosout.txt').write_text('\n'.join(clock(t)+' '+v['name']+' '+v['msg'] for t,v in d['/rosout']));(H/'sources_sha256.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [*list((B/'bag').iterdir()),B/'effective_config.yaml',B/'session_result.json']},indent=2))
for z in rows:print(z['attempt'],z['outcome'],z['start_clock'],round(z['drive_seconds'],1),'solves',z['upper_valid'],z['upper_count'],'guard',z['guard_hits'],'sat%',round(z['sat_percent'],1),'err95',round(z['lateral_abs']['p95'],3),'HP',round(z['highpass_rms_deg'],2),'B',z['B_range'],'k1',round(z['largest_k1_shifts'][0]['lateral_m'],3),'stop goal',z['stop_goal_m'],'rev',z['first_revision'],'B0',z['first_B_before'])
# Summarized visuals retain no uncompressed grid duplicate.
os.environ['MPLCONFIGDIR']='/tmp/batch062606_plot'
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf');plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False})
fig,axs=plt.subplots(1,3,figsize=(16,6),layout='constrained');ax=axs[0];ax.plot(*route.T,'k--o',ms=3,label='지정 경로')
for z in tracks:
 xy=np.array(z['xy']);col='#c44632' if z['attempt']==5 else '#148676';ax.plot(*xy.T,color=col,alpha=1 if z['attempt']==5 else .45,lw=2 if z['attempt']==5 else 1.2,label=('5회차 경로 이탈' if z['attempt']==5 else '목표 방향 9회' if z['attempt']==1 else None))
ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(fontsize=9);ax.set_title('10회 모두 주행: 9회 정분기 · 5회차 오분기');ax.set_xlabel('X (m)');ax.set_ylabel('Y (m)')
ax=axs[1];xx=np.arange(1,11);ax.bar(xx,[z['sat_percent'] for z in rows],color=['#c44632' if i==5 else '#148676' for i in xx]);ax.set_xticks(xx);ax.set_xlabel('회차');ax.set_ylabel('출력 조향 포화 비율 (%)');ax.set_title('조향 한계에 닿는 구간은 계속 남음');ax.grid(axis='y',alpha=.2)
ax=axs[2];ax.bar(xx,[z['stop_goal_m'] for z in rows],color=['#c44632' if i==5 else '#d39831' for i in xx]);ax.set_xticks(xx);ax.set_xlabel('회차');ax.set_ylabel('최종 정지점과 목표 거리 (m)');ax.set_title('목표 접근과 실제 정지는 별개');ax.grid(axis='y',alpha=.2)
fig.savefig(H/'overview.png',dpi=150);plt.close(fig)
