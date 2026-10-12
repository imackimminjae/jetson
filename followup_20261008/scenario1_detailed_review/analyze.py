from pathlib import Path
from collections import defaultdict,Counter
from datetime import datetime
from zoneinfo import ZoneInfo
import os,json,csv,gzip,hashlib,sys
os.sched_setaffinity(0,{min(os.sched_getaffinity(0))})
os.environ['MPLCONFIGDIR']='/tmp/scenario_detailed_plot'
import numpy as np,yaml,rosbag2_py
from scipy.signal import butter,sosfiltfilt
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
R=Path('/home/imac/ros2_ws'); scenario=int(sys.argv[1]);H=R/f'followup_20261008/scenario{scenario}_detailed_review';B=R/'drive_batches'/({1:'20261008_173306_scenario1_756dad',2:'20261008_171715_scenario2_592ff2'}[scenario])
clock=lambda t:datetime.fromtimestamp(float(t),ZoneInfo('Asia/Seoul')).strftime('%H:%M:%S.%f')[:-3]
def quant(v):
 v=np.asarray(v);v=v[np.isfinite(v)];return dict(zip(['min','median','p95','max'],map(float,np.quantile(v,[0,.5,.95,1])))) if len(v) else None
r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(B/'bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));types={x.name:get_message(x.type) for x in r.get_all_topics_and_types()};d=defaultdict(list);rx=defaultdict(list);gridmeta=Counter();params=[]
while r.has_next():
 n,blob,tn=r.read_next();t=tn/1e9;rx[n].append(t);m=deserialize_message(blob,types[n])
 if n=='/bev/occupancy_grid':
  g=m.info;gridmeta[(g.resolution,g.width,g.height,g.origin.position.x,g.origin.position.y,m.header.frame_id)]+=1;continue
 if n in ['/knu_batch/status','/knu_batch/active_route','/debug/upper_branch_event','/debug/upper_stage_timing']:
  try:v=json.loads(m.data)
  except:continue
 elif n=='/motive/vehicle/odom_map':v=[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.orientation.z,m.twist.twist.linear.x,m.twist.twist.linear.y,m.header.stamp.sec+m.header.stamp.nanosec/1e9,m.header.frame_id,m.pose.pose.position.z]
 elif n.startswith('/debug/') and hasattr(m,'data'):v=list(m.data)
 elif n=='/navigation/global_path':v=[[p.pose.position.x,p.pose.position.y] for p in m.poses]
 elif n=='/rosout':v=dict(name=m.name,level=m.level,msg=m.msg)
 elif n in ['/planner/goal_reached','/controller/reset']:v=m.data
 elif n=='/parameter_events':continue
 else:continue
 d[n].append([t,v])
ph=[];prev=None
for t,s in d['/knu_batch/status']:
 key=(s['index'],s['phase'],s['sequence'])
 if key!=prev:ph.append(dict(received_t=t,time=clock(s['phase_stamp_sec']),**s));prev=key
route=np.array(d['/navigation/global_path'][0][1]);ot=np.array(rx['/motive/vehicle/odom_map']);od=np.array([v[:6]+[v[7]] for _,v in d['/motive/vehicle/odom_map']]);lt=np.array(rx['/debug/lower_mpc_trace']);lo=np.array([v for _,v in d['/debug/lower_mpc_trace']]);bt=np.array([v[0] for _,v in d['/debug/upper_interval_centering_state']]);bs=np.array([v for _,v in d['/debug/upper_interval_centering_state']]);es=[dict(received_t=t,**e) for t,e in d['/debug/upper_branch_event']];csvrows=list(csv.DictReader(next((B/'results').glob('*/summary.csv')).open()));rows=[];tracks=[]
def gaps(tt,start,end):
 tt=np.array(tt);ind=np.flatnonzero((tt[:-1]<end)&(tt[1:]>start));j=ind[np.argmax(np.diff(tt)[ind])] if len(ind) else None
 return dict(seconds=float(tt[j+1]-tt[j]),from_time=clock(tt[j]),to_time=clock(tt[j+1]),from_t=float(tt[j]),to_t=float(tt[j+1])) if j is not None else None
for i,c in enumerate(csvrows,1):
 ps=[p for p in ph if p['index']==i];start=next(p['phase_stamp_sec'] for p in ps if p['phase']=='running');end=next(p['phase_stamp_sec'] for p in ps if p['phase']=='coasting');stop=next((p['phase_stamp_sec'] for p in ph if p['index']==i+1),ph[-1]['phase_stamp_sec']);ref=next(p['phase_stamp_sec'] for p in ps if p['phase']=='reference')
 ev=[e for e in es if start<=e['t_start']<end];first=next(e for e in es if e['t_start']>=ref);bi=int(np.argmin(abs(bt-first['t_start'])));m=(lo[:,0]>=start)&(lo[:,0]<end);active=m&(lo[:,12]==0)&(lo[:,4]>1);atr=lo[active];width=bs[(bt>=start)&(bt<end),2];shifts=[];prior=None
 for e in ev:
  if not e['valid_plan']:continue
  p=np.array(e['published_world'])
  if len(p)<2:continue
  if prior is not None and 0<e['t_start']-prior['t_start']<.8:
   old=np.array(prior['published_world']);v=np.diff(old,axis=0);ll=np.linalg.norm(v,axis=1);u=np.clip(np.sum((p[1]-old[:-1])*v,axis=1)/np.maximum(ll**2,1e-12),0,1);q=old[:-1]+u[:,None]*v;j=int(np.argmin(np.linalg.norm(p[1]-q,axis=1)));shift=float(np.cross(v[j],p[1]-q[j])/max(ll[j],1e-12));k=int(np.argmin(abs(bt-e['t_start'])));shifts.append(dict(time=clock(e['t_start']),t=e['t_start'],xy=[e['x'],e['y']],lateral_m=shift,B_before=float(bs[k,1]),B=float(bs[k,2])))
  prior=e
 hp=[];inds=np.flatnonzero(active)
 for chunk in np.split(inds,np.flatnonzero((np.diff(inds)>1)|(np.diff(lo[inds,0])>.3))+1):
  if len(chunk)<20:continue
  tt=np.arange(lo[chunk[0],0],lo[chunk[-1],0],.1);vv=np.interp(tt,lo[chunk,0],lo[chunk,26]);hp.extend(sosfiltfilt(butter(2,.3,fs=10,btype='highpass',output='sos'),vv))
 sig=atr[abs(atr[:,26])>np.deg2rad(1)];flips=sum(np.sign(a[26])!=np.sign(b[26]) for a,b in zip(sig[:-1],sig[1:]) if 0<b[0]-a[0]<.7)
 lm=lo[m];trans=[];mode=None
 for z in lm:
  if int(z[12])!=mode:trans.append(dict(t=z[0],time=clock(z[0]),mode=int(z[12]),xy=z[1:3].tolist(),speed=z[4]));mode=int(z[12])
 lastdrive=lo[(lo[:,0]>=start)&(lo[:,0]<end)&np.isin(lo[:,12],[0,1,2,5])];tailneutral=lo[(lo[:,0]>(lastdrive[-1,0] if len(lastdrive) else start))&(lo[:,0]<stop)&np.isin(lo[:,12],[3,4])];neutral=tailneutral[0] if len(tailneutral) else None
 sm=(ot>=start)&(ot<stop);tr=od[sm];ts=ot[sm];tracks.append(dict(attempt=i,t=ts.tolist(),xy=tr[:,:2].tolist(),end=end,neutral_t=float(neutral[0]) if neutral is not None else None))
 bad=[e for e in ev if not e['valid_plan']];row=dict(attempt=i,**c,start=start,start_clock=clock(start),end=end,end_clock=clock(end),stop_time=stop,reference_time=ref,upper_count=len(ev),upper_failed=len(bad),upper_failure_reasons=dict(Counter(e['reason'] for e in bad)),horizons=dict(Counter(e['horizon'] for e in ev)),guard_hits=sum(e.get('side_boundary_tail_start',-1)>=0 for e in ev),first_B_before=float(bs[bi,1]),first_wp=[first['wp0'],first['wp1']],first_xy=[first['x'],first['y']],first_yaw_deg=float(np.degrees(first['yaw_rad'])),speed=quant(atr[:,4]),lateral_abs=quant(abs(atr[:,6])),sat_percent=float(np.mean(abs(atr[:,26])>=.29845)*100) if len(atr) else None,highpass_rms_deg=float(np.sqrt(np.mean(np.degrees(hp)**2))) if hp else None,sign_flips=int(flips),B_range=[float(width.min()),float(width.max())] if len(width) else None,largest_k1_shifts=sorted(shifts,key=lambda z:abs(z['lateral_m']),reverse=True)[:8],lower_modes=dict(Counter(int(x[12]) for x in lm)),lower_transitions=trans,active_samples=len(atr),gaps={n:gaps(rx[n],start,end) for n in ['/motive/vehicle/odom_map','/bev/occupancy_grid','/debug/lower_mpc_trace','/knu_batch/status']},first_invalid={k:bad[0][k] for k in ['t_start','x','y','reason','status','horizon']} if bad else None,terminal_neutral=dict(t=neutral[0],time=clock(neutral[0]),xy=neutral[1:3].tolist(),speed=float(neutral[4]),seconds_to_phase_end=float(stop-neutral[0]),displacement_to_stop_m=float(np.linalg.norm(np.array([float(c['stop_x']),float(c['stop_y'])])-neutral[1:3]))) if neutral is not None else None)
 rows.append(row)
summary=dict(scenario=scenario,session=str(B),session_result=json.loads((B/'session_result.json').read_text()),route=route.tolist(),attempts=rows,phases=ph,counts={n:len(tt) for n,tt in rx.items()},grid_metadata=[dict(values=k,count=v) for k,v in gridmeta.items()],total_bytes=sum(p.stat().st_size for p in B.rglob('*') if p.is_file()),odom_invalid_count=sum(v[6].lstrip('/')!='map' or not np.isfinite(np.array(v[:6]+[v[7]],dtype=float)).all() for _,v in d['/motive/vehicle/odom_map']))
(H/'summary.json').write_text(json.dumps(summary,indent=2));(H/'sources_sha256.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [*list((B/'bag').iterdir()),B/'effective_config.yaml',B/'controllers.log',B/'session_result.json']},indent=2))
with gzip.open(H/'telemetry.json.gz','wt') as f:json.dump(dict(data=d,receive_times=rx),f)
with gzip.open(H/'events.json.gz','wt') as f:json.dump(es,f)
with gzip.open(H/'tracks.json.gz','wt') as f:json.dump(tracks,f)
(H/'rosout.txt').write_text('\n'.join(clock(t)+' '+v['name']+' '+v['msg'] for t,v in d['/rosout']))
fields=['attempt','outcome','start_clock','end_clock','duration_sec','progress_ratio','goal_distance_m','stop_goal_distance_m','upper_count','upper_failed','sat_percent','highpass_rms_deg','sign_flips','first_B_before'];outrows=[{**{k:z[k] for k in fields},'lateral_abs_p95_m':z['lateral_abs']['p95'] if z['lateral_abs'] else None,'odom_max_gap_s':z['gaps']['/motive/vehicle/odom_map']['seconds']} for z in rows]
with (H/'per_attempt.csv').open('w') as f:w=csv.DictWriter(f,fieldnames=list(outrows[0]));w.writeheader();w.writerows(outrows)
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf');plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False})
fig,axs=plt.subplots(1,3,figsize=(17,6),layout='constrained');ax=axs[0];ax.plot(*route.T,'k--o',ms=3,label='지정 경로')
colors={i:plt.cm.tab10((i-1)%10) for i in range(1,len(rows)+1)}
for tr,z in zip(tracks,rows):
 xy=np.array(tr['xy']);tt=np.array(tr['t']);parts=np.split(np.arange(len(tt)),np.flatnonzero(np.diff(tt)>.5)+1)
 for j,ind in enumerate(parts):ax.plot(*xy[ind].T,color=colors[z['attempt']],lw=1.3,label=f"{z['attempt']}: {z['outcome']}" if j==0 else None)
 ax.scatter(float(z['stop_x']),float(z['stop_y']),marker='x',color=colors[z['attempt']],s=24)
ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(fontsize=8);ax.set_title(f'시나리오{scenario}: 기록 궤적·정지점 (공백은 연결하지 않음)');ax.set_xlabel('X (m)');ax.set_ylabel('Y (m)')
xx=np.arange(1,len(rows)+1);axs[1].bar(xx,[z['upper_failed'] for z in rows],color=[colors[i] for i in xx]);axs[1].set_title('주행 구간 상위 계획 실패 횟수');axs[1].set_xticks(xx);axs[1].set_xlabel('회차');axs[1].grid(axis='y',alpha=.2)
axs[2].bar(xx,[z['sat_percent'] for z in rows],color=[colors[i] for i in xx]);axs[2].set_title('출력 조향 명령 포화 비율 (실제 바퀴각 아님)');axs[2].set_ylabel('% (mode0·속도>1m/s 표본)');axs[2].set_xticks(xx);axs[2].set_xlabel('회차');axs[2].grid(axis='y',alpha=.2);fig.savefig(H/'overview.png',dpi=140);plt.close(fig)
for z in outrows:print(z)
print('SAVED',H,flush=True)
