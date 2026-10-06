from pathlib import Path
from datetime import datetime
from collections import Counter
import pickle,json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.font_manager import FontProperties
plt.rcParams.update({'font.family':FontProperties(fname='/usr/share/fonts/truetype/nanum/NanumGothic.ttf').get_name(),'axes.unicode_minus':False})
p=Path(__file__).resolve().parent;d=pickle.load(open(p/'decoded.pkl','rb'));r=json.loads((p/'telemetry.json').read_text());lo=np.array([a['data'] for a in d['/debug/lower_mpc_trace']]);es=[a['data'] for a in d['/debug/upper_branch_event']];features=json.loads((p/'features.json').read_text());o=d['/motive/vehicle/odom_map'];ot=np.array([a['t'] for a in o]);xy=np.array([a['xy'] for a in o]);start=ot[0];end=ot[-1];valid=lo[:,10]==1;clock=lambda t:datetime.fromtimestamp(t).strftime('%H:%M:%S.%f')[:-3];quant=lambda x:np.quantile(x,[.5,.95,1]).tolist()
h=[a for a in r if a['mavpackettype']=='HIL_STATE_QUATERNION'];ht=np.array([a['t'] for a in h]);hv=np.array([[a['vx'],a['vy']] for a in h])*.01;lp=[a for a in r if a['mavpackettype']=='LOCAL_POSITION_NED'];lt=np.array([a['t'] for a in lp]);lv=np.array([[a['vx'],a['vy']] for a in lp]);mask=(ht>=start)&(ht<=end);error=np.linalg.norm(np.column_stack([np.interp(ht,lt,lv[:,j]) for j in range(2)])-hv,axis=1);status=[a['data'] for a in d['/px4_ekf_bridge/status']];est=[a for a in r if a['mavpackettype']=='ESTIMATOR_STATUS' and start<=a['t']<=end];fails=[e for e in es if not e['valid_plan']]
metrics={'bag':str(p.parent),'start':clock(start),'end':clock(end),'grid_count':len(d['/bev/occupancy_grid']),'received_grid_origins':list({tuple(g['origin']) for g in d['/bev/occupancy_grid']}),'grid_max_gap_s':float(max(np.diff([g['t'] for g in d['/bev/occupancy_grid']]))),'pose_max_gap_s':float(max(np.diff(ot))),'upper_success':sum(e['valid_plan'] for e in es),'upper_fail':len(fails),'lower_valid_total':[int(sum(valid)),len(lo)],'lower_applied_saturation_count':int(sum(abs(lo[valid,26])>=.299)),'lower_error_abs_median_p95_max_m':quant(abs(lo[valid,6])),'lower_solve_ms_median_p95_max':quant(lo[:,11]),'truth_speed_median_p95_max_m_s':quant(np.linalg.norm(hv[mask],axis=1)),'ekf_velocity_vector_error_median_p95_max_m_s':quant(error[mask]),'heading_error_median_p95_max_deg':quant([abs(a['ekf_minus_sih_yaw_deg']) for a in status]),'estimator_flags':dict(Counter(a['flags'] for a in est)),'horizontal_valid_status_count':sum(bool(a['flags']&2) and bool(a['flags']&24) for a in est),'status_count':len(est),'stale_intervals':[],'failures':[]}
stale=lo[:,12]==3
for i in np.flatnonzero(stale&~np.r_[False,stale[:-1]]):
 k=i
 while k+1<len(stale) and stale[k+1]:k+=1
 metrics['stale_intervals'].append({'start':clock(lo[i,0]),'last':clock(lo[k,0]),'resumed':clock(lo[k+1,0]) if k+1<len(stale) else None,'samples':k-i+1})
feas=json.loads((p/'feasibility_results.json').read_text())['failures']
for e,ff in zip(fails,feas):
 info=min(d['/debug/upper_interval_info'],key=lambda a:abs(a['t']-e['t_emit']))['data'].reshape(-1,12)
 metrics['failures'].append(dict(ff,horizon=e['horizon'],first_step_details=info[info[:,0]==1].tolist(),solve_ms=e['solver_time_ms']))
(p/'latest_metrics.json').write_text(json.dumps(metrics,indent=2,default=lambda a:a.item()));print(json.dumps(metrics,indent=2,default=lambda a:a.item()))
fig=plt.figure(figsize=(14,10));gs=fig.add_gridspec(3,2,width_ratios=[1,1.4]);ax=fig.add_subplot(gs[:,0]);route=d['/navigation/global_path'][0]['xy'];ax.plot(route[:,0],route[:,1],'--o',color='#a58300',label='전역 경로');ax.plot(xy[:,0],xy[:,1],color='#087e8b',lw=2,label='차량 자취 (EKF)')
for i in [7,9,12,15,18,22]:
 e=es[i];pp=np.array(e['planned_world']);ax.plot(pp[:,0],pp[:,1],lw=1,alpha=.65,label='상위 계획' if i==7 else None)
for e in fails:
 ax.scatter(e['x'],e['y'],c='#c62828',s=40);ax.annotate(clock(e['t_start'])[:8],(e['x'],e['y']),xytext=(5,8),textcoords='offset points',fontsize=9,color='#c62828')
ax.set_aspect('equal');ax.set(title='목표 경로와 계획·자취\n빨간 점: 상위 계획 실패',xlabel='map X (m)',ylabel='map Y (m)');ax.grid(alpha=.2);ax.legend(fontsize=9)
a=fig.add_subplot(gs[0,1]);a.plot(ht-start,np.linalg.norm(hv,axis=1),label='SIH 실제 속력',color='#087e8b');a.plot(lt-start,np.linalg.norm(lv,axis=1),label='EKF 속력',color='#e67e22');a.set(ylabel='속력 (m/s)',title='실제 속도와 EKF는 대체로 일치');a.legend(fontsize=9)
a2=fig.add_subplot(gs[1,1],sharex=a);a2.plot(lo[:,0]-start,np.degrees(lo[:,26]),c='#c62828',label='적용 조향');a2.axhline(np.degrees(.3),c='gray',ls='--');a2.axhline(-np.degrees(.3),c='gray',ls='--');a2.set(ylabel='조향 (도)',title='유효 제어 중 약 29%가 조향 한계 ±17.2도');a2.legend(fontsize=9)
a3=fig.add_subplot(gs[2,1],sharex=a);a3.plot(lo[valid,0]-start,lo[valid,6],c='#7b1fa2',label='하위 MPC 횡오차');a3.axhline(0,c='gray',lw=.8);a3.set(ylabel='경로 횡오차 (m)',xlabel='기록 시작부터 경과 시간 (초)',title='추종 오차 최대 2.39m');a3.legend(fontsize=9)
for aa in [a,a2,a3]:
 aa.set_xlim(0,end-start);aa.grid(alpha=.2)
 for e in fails:aa.axvline(e['t_start']-start,c='#c62828',ls=':',alpha=.7)
 for i in np.flatnonzero(stale):aa.axvspan(lo[i,0]-start,lo[i,0]-start+.1,color='gray',alpha=.15)
fig.suptitle('최신 기록 10월 3일 05:07:33~05:07:58 · 원본 지도 x=0, 추가 이동=0m',fontsize=15);fig.tight_layout();fig.savefig(p/'latest_diagnosis.png',dpi=160)
fig,axes=plt.subplots(1,3,figsize=(15,7))
for ax,e in zip(axes,[es[12],es[13],es[19]]):
 g=min(d['/bev/occupancy_grid'],key=lambda a:abs(a['t']-e['t_start']));yaw=e['yaw_rad'];R=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]]);pos=np.array([e['x'],e['y']]);body=lambda pts:(np.array(pts)-pos)@R;ox,oy=g['origin'];grid=np.where(g['grid']<0,0,np.where(g['grid']==0,2,1));ax.imshow(grid,origin='lower',extent=[ox,ox+g['width']*g['res'],oy,oy+g['height']*g['res']],cmap=ListedColormap(['#616976','#b7bdc5','#fafafa']),vmin=0,vmax=2,interpolation='nearest');first=True
 for inter in e['intervals_world']:
  ends=body(np.array(inter[2:6]).reshape(2,2));mid=ends.mean(axis=0);dire=(ends[1]-ends[0])/np.linalg.norm(ends[1]-ends[0]);raw=np.array([mid-dire*inter[6]/2,mid+dire*inter[6]/2]);ax.plot(raw[:,0],raw[:,1],c='#287ac0',lw=2,label='관측 단면' if first else None);ax.plot(ends[:,0],ends[:,1],c='#ef6c00',lw=4,label='축소된 허용 단면' if first else None);first=False
 ref=body(e['preview_world']);ax.plot(ref[:,0],ref[:,1],'x--',c='#8e44ad',label='기준 경로')
 if e['valid_plan']:pp=body(e['planned_world']);label='새 계획'
 else:old=max((v for v in es if v['valid_plan'] and v['t_emit']<e['t_start']),key=lambda v:v['t_emit']);pp=body(old['planned_world']);label='유지된 이전 계획'
 ax.plot(pp[:,0],pp[:,1],'o-',ms=3,c='#198754' if e['valid_plan'] else '#c62828',label=label);ax.scatter(0,0,marker='>',c='black',s=80,label='차량');ax.set(xlim=(-1,25),ylim=(-14,12),title=clock(e['t_start'])[:8]+(' 성공' if e['valid_plan'] else ' 실패'),xlabel='전방 (m)',ylabel='왼쪽 (m)');ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(fontsize=8,loc='lower right')
fig.suptitle('실패 구간: 47초는 축소 제약 충돌, 53초는 원래 관측 단면으로도 불가능\n흰색=주행 가능, 회색=비주행, 짙은색=미관측 · 지도는 최근접 수신 시각으로 대응',fontsize=13);fig.tight_layout();fig.savefig(p/'latest_failure_grid.png',dpi=160)
