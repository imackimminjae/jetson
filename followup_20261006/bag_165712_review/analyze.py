from pathlib import Path
import json,pickle,hashlib,csv,os
from collections import Counter
from datetime import datetime
from zoneinfo import ZoneInfo
import numpy as np
from scipy.signal import butter,sosfiltfilt,find_peaks
os.environ['MPLCONFIGDIR']='/tmp/codex_mpl_grid'
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib import font_manager
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf');plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False})
h=Path(__file__).resolve().parent;root=h.parents[1];bag=root/'drive_debug_20261006_165712_light';d=pickle.load((bag/'analysis/decoded.pkl').open('rb'))
g=d['/bev/occupancy_grid'];es=[x['data'] for x in d['/debug/upper_branch_event']];tr=np.array([x['data'] for x in d['/debug/lower_mpc_trace']]);od=d['/motive/vehicle/odom_map'];ot=np.array([x['t'] for x in od]);xy=np.array([x['xy'] for x in od]);gp=d['/navigation/global_path'][0]['xy'];gt=np.array([x['t'] for x in g]);et=np.array([e['t_start'] for e in es]);t0=gt[0];goal=next(x['t'] for x in d['/planner/goal_reached'] if x['data'])
def clock(t):return datetime.fromtimestamp(float(t),ZoneInfo('Asia/Seoul')).strftime('%H:%M:%S.%f')[:-3]
def quant(a):return dict(zip(['min','p50','p95','max'],map(float,np.quantile(a,[0,.5,.95,1])))) if len(a) else {}
def project(p,line):
 v=np.diff(line,axis=0);l=np.linalg.norm(v,axis=1);u=np.clip(np.sum((p[:,None,:]-line[None,:-1,:])*v,axis=2)/(l*l),0,1);q=line[None,:-1,:]+u[:,:,None]*v;dist=np.linalg.norm(p[:,None,:]-q,axis=2);j=np.argmin(dist,axis=1);idx=np.arange(len(p));s=np.r_[0,np.cumsum(l)];return s[j]+u[idx,j]*l[j],dist[idx,j]
rows=[]
for m in g:
 a=m['grid'];yy,xx=np.where(a==0);ky,kx=np.where(a>=0);r=m['res'];ox,oy=m['origin']
 row={'time':clock(m['t']),'t':m['t'],'seconds':m['t']-t0,'known':len(kx),'unknown':int((a<0).sum()),'free':len(xx),'known_forward_min':ox+kx.min()*r,'known_forward_end':ox+(kx.max()+1)*r,'free_forward_min_effective0':xx.min()*r if len(xx) else None,'free_forward_end_effective0':(xx.max()+1)*r if len(xx) else None,'free_first9m':int((a[:,:int(9/r)]==0).sum()),'free_at_k7_forward_column':int((a[:,int(31.5/r)]==0).sum()),'free_y0_cells':int((a[int(-oy/r),:]==0).sum()),'stamp_zero':m['stamp']==0}
 rows.append(row)
with (h/'all_grid_frames.csv').open('w') as f:w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
near=[];prev=None
for e in es:
 pl=np.array(e['planned_world']);ref=np.array(e['preview_world']);step=pl[1]-pl[0];tangent=ref[1]-ref[0];normal=np.array([-tangent[1],tangent[0]])/np.linalg.norm(tangent);corr=float((pl[1]-ref[1])@normal);shift=None
 if prev is not None:
  vv=np.diff(prev,axis=0);ll=np.sum(vv*vv,axis=1);uu=np.clip(np.sum((pl[1]-prev[:-1])*vv,axis=1)/ll,0,1);q=prev[:-1]+uu[:,None]*vv;j=np.argmin(np.linalg.norm(pl[1]-q,axis=1));shift=float(np.cross(vv[j],pl[1]-q[j])/np.sqrt(ll[j]))
 near.append({'t':e['t_start'],'time':clock(e['t_start']),'horizon':e['horizon'],'k1_reference_correction':corr,'k1_previous_plan_shift':shift,'k1_relative_heading_deg':float(np.degrees(np.arctan2(np.sin(np.arctan2(step[1],step[0])-e['yaw_rad']),np.cos(np.arctan2(step[1],step[0])-e['yaw_rad'])))),'max_candidates':e['max_candidates']});prev=pl
active=tr[:,12]==0;segments={}
for title,aa,zz in [('전체 정상 추종',tr[active,0][0],goal),('초반 직선',tr[active,0][0],1791273445),('곡선·분기 구간',1791273445,1791273461),('후반 종점 접근',1791273461,goal)]:
 x=tr[active&(tr[:,0]>=aa)&(tr[:,0]<zz)];ang=np.degrees(x[:,8]);tm=x[:,0];sig=np.sign(ang[np.abs(ang)>2]);peaks=find_peaks(ang,prominence=3)[0];troughs=find_peaks(-ang,prominence=3)[0]
 segments[title]={'n':len(x),'start':clock(tm[0]),'end':clock(tm[-1]),'speed_median':float(np.median(x[:,4])),'steering_deg':quant(ang),'sat_fraction':float(np.mean(abs(x[:,8])>=.29845)),'local_lateral_abs_m':quant(abs(x[:,6])),'heading_abs_deg':quant(abs(np.degrees(x[:,7]))),'sign_changes_above_2deg':int(np.sum(np.diff(sig)!=0)),'extrema':[{'t':clock(tm[i]),'deg':float(ang[i])} for i in sorted(set(peaks)|set(troughs))]}
geom=Counter((x['width'],x['height'],x['res'],tuple(x['origin']),tuple(x['q'])) for x in g);pre=[r for r in rows if r['t']<goal];post=[r for r in rows if r['t']>=goal]
summ={'bag':str(bag),'time_window':[clock(gt[0]),clock(gt[-1])],'grid_count':len(g),'grid_hz':float((len(g)-1)/(gt[-1]-gt[0])),'grid_max_gap_sec':float(max(np.diff(gt))),'geometry_patterns':{str(k):v for k,v in geom.items()},'known_counts':dict(Counter(r['known'] for r in rows)),'unknown_counts':dict(Counter(r['unknown'] for r in rows)),'zero_stamp_count':sum(r['stamp_zero'] for r in rows),'free_far_effective0_pre_goal':quant([r['free_forward_end_effective0'] for r in pre]),'free_far_effective0_post_goal':quant([r['free_forward_end_effective0'] for r in post]),'near9m_empty_before_goal':sum(r['free_first9m']==0 for r in pre),'near9m_empty_after_goal':sum(r['free_first9m']==0 for r in post),'goal_time':clock(goal),'goal_last_track_distance_m':float(np.linalg.norm(xy[-1]-gp[-1])),'goal_first_track_distance_m':float(np.linalg.norm(xy[np.searchsorted(ot,goal)]-gp[-1])),'upper_events':len(es),'upper_valid':sum(e['valid_plan'] for e in es),'horizons':dict(Counter(e['horizon'] for e in es)),'upper_max_gap_s':float(max(np.diff(et))),'lower_modes':dict(Counter(map(int,tr[:,12]))),'lower_max_gap_s':float(max(np.diff(tr[:,0]))),'segments':segments,'k1_largest_shifts':sorted([r for r in near if r['k1_previous_plan_shift'] is not None],key=lambda r:abs(r['k1_previous_plan_shift']),reverse=True)[:8]}
(h/'summary.json').write_text(json.dumps(summ,ensure_ascii=False,indent=2));(h/'plan_changes.json').write_text(json.dumps(near,ensure_ascii=False,indent=2));(h/'upper_events.json').write_text(json.dumps(es,ensure_ascii=False,indent=2));(h/'rosout.txt').write_text('\n'.join(clock(m['t'])+' '+m['name']+' '+m['msg'] for m in d['/rosout']))
manifest={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [bag/'bag/bag_0.db3',bag/'bag/metadata.yaml',bag/'config_snapshot.yaml']};(h/'source_sha256.json').write_text(json.dumps(manifest,indent=2));print(json.dumps(summ,ensure_ascii=False,indent=2))
fig,axs=plt.subplots(2,2,figsize=(13,9),layout='constrained');ax=axs[0,0];ax.plot(*gp.T,'k--o',label='지정 경로');ax.plot(*xy[ot<goal].T,label='목표 판정 전 궤적');ax.plot(*xy[ot>=goal].T,':',color='red',label='목표 판정 뒤 궤적');ax.set_aspect('equal');ax.set_title('기록된 주행 궤적');ax.legend(fontsize=8);ax.set_xlabel('지도 X (m)');ax.set_ylabel('지도 Y (m)')
ax=axs[0,1];ax.plot(gt-t0,[r['known_forward_end']-2.95 for r in rows],label='미관측 제외 범위 (원점 0 환산)');ax.plot(gt-t0,[r['free_forward_end_effective0'] for r in rows],label='도로 셀의 가장 먼 위치');ax.step(et-t0,[e['horizon']*4.5 for e in es],where='post',label='실제 사용 N × 4.5m');ax.set_ylim(0,43);ax.set_ylabel('전방 거리 (m)');ax.set_title('전체 1,114프레임: 범위와 실제 계획 길이');ax.legend(fontsize=8)
ax=axs[1,0];ax.plot(tr[:,0]-t0,np.degrees(tr[:,8]),label='ROS 조향 명령');ax.plot(tr[:,0]-t0,np.degrees(tr[:,19]),label='제어기 내부 유효 조향 추정',alpha=.7);ax.set_ylabel('조향 (도)');ax.set_title('후반에도 조향 왕복은 남아 있음');ax.legend(fontsize=8)
ax=axs[1,1];ax.plot(et[1:]-t0,[r['k1_previous_plan_shift'] for r in near[1:]],'.-',label='새 k1의 직전 계획 대비 횡이동');ax.plot(tr[active,0]-t0,tr[active,6],label='지역 경로 횡오차',alpha=.7);ax.set_ylabel('횡방향 (m)');ax.set_title('계획 갱신과 하위 추종 오차');ax.legend(fontsize=8)
for ax in [axs[0,1],*axs[1,:]]:ax.axvline(goal-t0,c='red',ls=':',label='goal');ax.set_xlabel('첫 맵 기록 이후 시간 (초)');ax.grid(alpha=.2)
fig.suptitle('16:57:12 로스백 · 상위 67/67 성공 · goal 이후에도 차량 이동 기록',fontsize=14);fig.savefig(h/'run_summary.png',dpi=150);plt.close(fig)
fig,axs=plt.subplots(2,3,figsize=(13,9),layout='constrained')
for ax,tm in zip(axs.flat,[gt[0],1791273449.5,1791273455.5,1791273463.5,1791273467.06,gt[-1]]):
 j=max(0,np.searchsorted(gt,tm,side='right')-1);m=g[j];a=m['grid'];ax.imshow(a,origin='lower',extent=[0,40,-20,20],cmap=ListedColormap(['#3a91ca','#e8b1b1']),vmin=0,vmax=100,interpolation='nearest');idx=np.searchsorted(et,m['t'],side='right')-1
 if idx>=0 and m['t']-et[idx]<.6:
  e=es[idx];pl=np.array(e['planned_world'])-np.array([e['x'],e['y']]);yaw=e['yaw_rad'];rot=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]]);body=pl@rot;ax.plot(*body.T,'k.-',ms=3,label='근접 시각 계획 (차량상태 차이 있음)')
 ax.set_title(clock(m['t'])+(' · goal 이후' if m['t']>=goal else ''));ax.set_xlabel('플래너 적용 전방 x (m), 원점 0');ax.set_ylabel('횡방향 y (m)');ax.axhline(0,c='gray',ls=':',lw=.7)
fig.suptitle('주행 중 맵은 도로가 이어짐 · 파랑=도로(0), 분홍=비도로(100)\n수신 원점 2.95m를 플래너가 0m로 덮어쓴 좌표로 표시; 선은 인접 시각 계획',fontsize=13);fig.savefig(h/'grid_timeline.png',dpi=150);plt.close(fig)
