from pathlib import Path
from collections import defaultdict, Counter
from datetime import datetime
from zoneinfo import ZoneInfo
import os,sys,json,csv,hashlib
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.signal import butter,sosfiltfilt
ROOT=Path('/home/imac/ros2_ws');H=Path(__file__).resolve().parent
B=ROOT/'drive_batches/20261008_055617_scenario3_73cb2c'
clock=lambda t:datetime.fromtimestamp(float(t),ZoneInfo('Asia/Seoul')).strftime('%H:%M:%S.%f')[:-3]
r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(B/'bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
types={x.name:get_message(x.type) for x in r.get_all_topics_and_types()};d=defaultdict(list);counts=Counter();rawbytes=Counter();grids=[]
while r.has_next():
 n,blob,tn=r.read_next();t=tn/1e9;counts[n]+=1;rawbytes[n]+=len(blob);m=deserialize_message(blob,types[n])
 if n=='/bev/occupancy_grid':
  a=np.array(m.data);grids.append(dict(t=t,free=int(np.sum(a==0)),unknown=int(np.sum(a<0)),positive=int(np.sum(a>0)),shape=[m.info.width,m.info.height],res=m.info.resolution,origin=[m.info.origin.position.x,m.info.origin.position.y],frame=m.header.frame_id,sha=hashlib.sha256(a.tobytes()).hexdigest()));continue
 if n in ['/knu_batch/status','/knu_batch/active_route','/debug/upper_branch_event','/debug/upper_stage_timing']:
  try:v=json.loads(m.data)
  except:continue
 elif n=='/motive/vehicle/odom_map':v=[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.orientation.z,m.twist.twist.linear.x,m.twist.twist.linear.y]
 elif n.startswith('/debug/') and hasattr(m,'data'):v=list(m.data)
 elif n=='/navigation/global_path':v=[[p.pose.position.x,p.pose.position.y] for p in m.poses]
 elif n=='/planner/lower_reference_path/with_arclength':v=[[p.pose.position.x,p.pose.position.y] for p in m.path.poses]
 elif n=='/rosout':v=dict(name=m.name,level=m.level,msg=m.msg)
 elif n in ['/planner/goal_reached','/controller/reset']:v=m.data
 else:continue
 d[n].append((t,v))
phases=[];prev=None
for t,s in d['/knu_batch/status']:
 key=(s['index'],s['phase'],s['sequence'])
 if key!=prev:phases.append(dict(t=s['phase_stamp_sec'],received_t=t,time=clock(s['phase_stamp_sec']),**s));prev=key
print('PHASES',[(s['index'],s['phase'],s['time']) for s in phases])
route=np.array(d['/navigation/global_path'][0][1]);ot=np.array([x[0] for x in d['/motive/vehicle/odom_map']]);od=np.array([x[1] for x in d['/motive/vehicle/odom_map']]);lt=np.array([x[0] for x in d['/debug/lower_mpc_trace']]);lo=np.array([x[1] for x in d['/debug/lower_mpc_trace']]);gt=np.array([x['t'] for x in grids]);es=d['/debug/upper_branch_event']
def quant(v):return dict(zip(['min','median','p95','max'],map(float,np.quantile(v,[0,.5,.95,1])))) if len(v) else None
rows=[]
for i in range(1,11):
 ss=[x for x in phases if x['index']==i and x['phase']!='done'];a=min(x['t'] for x in ss);b=max(x['t'] for x in ss);nxt=[x['t'] for x in phases if x['index']==i+1];b=min(nxt) if nxt else phases[-1]['t'];running=[x['t'] for x in ss if x['phase']=='running'];coast=[x['t'] for x in ss if x['phase']=='coasting'];aa=running[0] if running else a;bb=coast[0] if coast else b
 ev=[e for t,e in es if aa<=t<bb];tr=lo[(lt>=aa)&(lt<bb)];og=od[(ot>=aa)&(ot<bb)];gg=[g for g in grids if aa<=g['t']<bb];rr=[t for t,_ in d['/planner/lower_reference_path/with_arclength'] if aa<=t<bb]
 row=dict(attempt=i,start=clock(a),end=clock(b),running=bool(running),running_start=clock(aa) if running else None,run_seconds=bb-aa if running else 0,upper_count=len(ev),upper_valid=sum(e['valid_plan'] for e in ev),horizons=dict(Counter(e['horizon'] for e in ev)),wp_pairs=dict(Counter(str((e.get('wp0'),e.get('wp1'))) for e in ev)),upper_reasons=dict(Counter(e['reason'] for e in ev)),upper_statuses=dict(Counter(e['status'] for e in ev)),guard_hits=sum(e.get('side_boundary_tail_start',-1)>=0 for e in ev),refs=len(rr),lower_modes=dict(Counter(int(x[12]) for x in tr)),grid_count=len(gg),grid_free=quant([g['free'] for g in gg]),grid_max_gap=float(np.max(np.diff([g['t'] for g in gg]))) if len(gg)>1 else None,odom_max_gap=float(np.max(np.diff(ot[(ot>=aa)&(ot<bb)]))) if len(og)>1 else None,xy_span=np.ptp(og[:,:2],axis=0).tolist() if len(og) else None)
 if running:
  active=tr[(tr[:,12]==0)&(tr[:,4]>1)];row.update(speed=quant(active[:,4]),lateral_abs=quant(abs(active[:,6])),steering_sat=float(np.mean(abs(active[:,26])>=.29845)),steering_peak_deg=float(np.max(abs(np.degrees(active[:,26])))),lower_max_gap=float(np.max(np.diff(lt[(lt>=aa)&(lt<bb)]))),first_invalid=[dict(time=clock(t),reason=e['reason'],status=e['status'],xy=[e['x'],e['y']]) for t,e in es if aa<=t<bb and not e['valid_plan']][:5])
  indices=np.flatnonzero((lt>=aa)&(lt<bb)&(lo[:,12]==0)&(lo[:,4]>1));hp=[]
  for chunk in np.split(indices,np.flatnonzero((np.diff(indices)>1)|(np.diff(lt[indices])>.3))+1):
   if len(chunk)<20:continue
   tt=np.arange(lt[chunk[0]],lt[chunk[-1]],.1);vv=np.interp(tt,lt[chunk],lo[chunk,26]);hp.extend(sosfiltfilt(butter(2,.3,fs=10,btype='highpass',output='sos'),vv))
  row['steering_highpass_rms_deg']=float(np.sqrt(np.mean(np.degrees(hp)**2)))
  row['coast_seconds']=b-bb;row['coast_displacement_m']=float(np.linalg.norm(od[np.argmin(abs(ot-b)),:2]-od[np.argmin(abs(ot-bb)),:2]))
 rows.append(row)
# Event schema and first/second attempt evidence.
print('EVENT KEYS',list(es[0][1]));print('ROUTE',route.tolist())
print(json.dumps(rows,indent=2))
summary=dict(session=str(B),counts=counts,raw_serialized_bytes=rawbytes,bag_bytes=sum(p.stat().st_size for p in (B/'bag').iterdir()),total_bytes=sum(p.stat().st_size for p in B.rglob('*') if p.is_file()),phases=phases,attempts=rows,global_path_publications=len(d['/navigation/global_path']),global_paths_identical=all(v==d['/navigation/global_path'][0][1] for _,v in d['/navigation/global_path']),route=route.tolist(),grid_geometry=dict(Counter(str((g['shape'],g['res'],g['origin'],g['frame'])) for g in grids)),grid_unknown_frames=sum(g['unknown']>0 for g in grids))
(H/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');(H/'events.json').write_text(json.dumps([{'t':t,**e} for t,e in es],indent=2)+'\n');(H/'rosout.txt').write_text('\n'.join(clock(t)+' '+v['name']+' '+v['msg'] for t,v in d['/rosout']))
(H/'sources_sha256.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [*list((B/'bag').iterdir()),B/'effective_config.yaml',B/'session_result.json']},indent=2))
# Figures: first drive/coast, commands, then initialization failure after map reset.
os.environ['MPLCONFIGDIR']='/tmp/batch055617_plot'
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf');plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False})
fig,axs=plt.subplots(2,2,figsize=(13,9),layout='constrained');ax=axs[0,0]
s1=[s for s in phases if s['index']==1];a=next(s['t'] for s in s1 if s['phase']=='running');b=next(s['t'] for s in s1 if s['phase']=='coasting');end=next(s['t'] for s in phases if s['index']==2)
ax.plot(*route.T,'k--o',ms=3,label='지정 경로');m=(ot>=a)&(ot<=b);ax.plot(*od[m,:2].T,color='#078c79',label='1회차 주행');m=(ot>=b)&(ot<end);ax.plot(*od[m,:2].T,color='#d57b22',label='중립 명령 이후 관성 주행');ax.scatter(*od[np.argmin(abs(ot-b)),:2],marker='x',s=70,color='red',label='목표 영역 접근 판정');ax.scatter(*od[np.searchsorted(ot,end)-1,:2],s=40,color='black',label='정지');ax.set_aspect('equal');ax.legend(fontsize=8);ax.grid(alpha=.2);ax.set_title('실제 주행은 1회: 정분기 방향 진입 후 관성 이동');ax.set_xlabel('X (m)');ax.set_ylabel('Y (m)')
ax=axs[0,1];m=(lt>=a)&(lt<end);ax.plot(lt[m]-a,np.degrees(lo[m,26]),label='출력 조향 (deg)',color='#147da4');ax.axvline(b-a,c='red',ls='--',label='주행 종료 판정');ax2=ax.twinx();ax2.plot(lt[m]-a,lo[m,4],c='#b98120',alpha=.7,label='속도');ax2.set_ylabel('속도 (m/s)');ax.set_xlabel('1회차 출발 후 (s)');ax.set_ylabel('출력 조향 (deg)');ax.legend(fontsize=8);ax.grid(alpha=.2);ax.set_title('목표 접근 뒤에도 즉시 멈추지 않음')
ax=axs[1,0];st=phases[0]['t'];ax.plot(ot-st,od[:,0],label='차량 X');ax.plot(ot-st,od[:,1],label='차량 Y');ax.set_xlabel('세션 경과 (s)');ax.set_ylabel('맵 위치 (m)');ax.set_title('2~10회는 출발점에서 경로 준비 시간 초과');ax.legend();ax.grid(alpha=.2)
ax=axs[1,1];et=np.array([t for t,e in es]);ax.scatter(et-st,[e['valid_plan'] for t,e in es],s=6,color='#147da4');ax.set_yticks([0,1],['상위 계산 실패','상위 계산 성공']);ax.set_xlabel('세션 경과 (s)');ax.set_title('새 맵이 들어와도 후속 회차의 경로 발행이 없음');ax.grid(alpha=.2)
fig.savefig(H/'overview.png',dpi=150);plt.close(fig)
