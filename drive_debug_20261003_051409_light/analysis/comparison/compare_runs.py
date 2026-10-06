from pathlib import Path
import json,pickle,numpy as np,yaml
from collections import Counter
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
plt.rcParams.update({'font.family':FontProperties(fname='/usr/share/fonts/truetype/nanum/NanumGothic.ttf').get_name(),'axes.unicode_minus':False})
p=Path(__file__).resolve().parent;ws=p.parents[2];runs=['20261002_041101','20261002_041646','20261002_043054','20261003_051409'];results={};datasets={}
def project(path,point):
 v=np.diff(path,axis=0);length=np.linalg.norm(v,axis=1);s=np.r_[0,np.cumsum(length)];u=np.clip(np.sum((point-path[:-1])*v,axis=1)/np.maximum(length**2,1e-12),0,1);q=path[:-1]+u[:,None]*v;i=np.argmin(np.linalg.norm(q-point,axis=1));return s[i]+u[i]*length[i],s
for run in runs:
 root=ws/('drive_debug_'+run+'_light');d=pickle.load(open(root/'analysis/decoded.pkl','rb'));datasets[run]=d;lo=np.array([a['data'] for a in d['/debug/lower_mpc_trace']]);es=[a['data'] for a in d['/debug/upper_branch_event']];o=d['/motive/vehicle/odom_map'];grid=d['/bev/occupancy_grid'];valid=lo[:,10]==1;section=valid&(lo[:,2]>=-100)&(lo[:,2]<=-30);et=np.array([e['t_start'] for e in es]);cfg=yaml.safe_load((root/'config_snapshot.yaml').read_text());r={'events':dict(Counter(e['reason'] for e in es)),'actual_plan_period_median_max_s':[float(np.median(np.diff(et))),float(max(np.diff(et)))],'grid_origins':list({tuple(g['origin']) for g in grid}),'config':{k:cfg['upper_planner_node']['ros__parameters'].get(k) for k in ['upper_planner_rate_hz','upper_r_dpsi','preview_interval_soft_ratio','grid_origin_x_override_enabled']},'lower_rd':cfg['lower_tracking_mpc_node']['ros__parameters']['lower_rd_steering_rate'],'sections':{}}
 for name,mask in [('all_valid',valid),('same_approach',section)]:
  x=lo[mask];dt=np.diff(x[:,0]);ok=(dt>0)&(dt<.3);angle=np.degrees(x[:,26]);r['sections'][name]={'samples':len(x),'speed_median':float(np.median(x[:,4])),'steering_variation_deg_s':float(np.sum(abs(np.diff(angle))[ok])/np.sum(dt[ok])),'steering_saturation_percent':float(np.mean(abs(x[:,26])>.299)*100),'lat_error_abs_p95_m':float(np.quantile(abs(x[:,6]),.95)),'lat_error_abs_max_m':float(max(abs(x[:,6])))}
 changes=[];old=None
 for e in es:
  if not e['valid_plan']:continue
  new=np.array(e['planned_world'])
  if old is not None and -100<=e['y']<=-30:
   progress,s=project(old,new[0]);ns=np.r_[0,np.cumsum(np.linalg.norm(np.diff(new,axis=0),axis=1))];off=np.array([4.5,9.]);a=np.column_stack([np.interp(progress+off,s,old[:,j]) for j in [0,1]]);b=np.column_stack([np.interp(off,ns,new[:,j]) for j in [0,1]]);ha=np.arctan2(*(a[1]-a[0])[::-1]);hb=np.arctan2(*(b[1]-b[0])[::-1]);delta=np.degrees(np.arctan2(np.sin(hb-ha),np.cos(hb-ha)));lat=float(np.mean((b-a)@np.array([-np.sin(ha),np.cos(ha)])));changes.append([e['t_start'],e['y'],float(delta),lat])
  old=new
 r['path_change_same_approach']={'n':len(changes),'heading_abs_p95_deg':float(np.quantile(np.abs(np.array(changes)[:,2]),.95)),'lateral_abs_p95_m':float(np.quantile(np.abs(np.array(changes)[:,3]),.95)),'data':changes}
 tel=json.loads((root/'analysis/telemetry.json').read_text());st=[a for a in tel if a['mavpackettype']=='ESTIMATOR_STATUS' and lo[valid,0][0]<=a['t']<=lo[valid,0][-1]];r['estimator_flags_during_valid_control']=dict(Counter(a['flags'] for a in st));r['gps_texts']=[a.get('text') for a in tel if a['mavpackettype']=='STATUSTEXT' and 'GPS' in a.get('text','')];results[run]=r
(p/'run_comparison.json').write_text(json.dumps(results,indent=2));print(json.dumps({k:{q:v for q,v in r.items() if q!='path_change_same_approach'} for k,r in results.items()},indent=2))
fig,axs=plt.subplots(2,2,figsize=(13,10));colors=['#7570b3','#087e8b','#c62828'];chosen=['20261002_041101','20261002_041646','20261003_051409'];labels=['어제 1Hz 04:11','어제 1Hz 04:16','최신 1Hz 05:14']
for run,label,c in zip(chosen,labels,colors):
 d=datasets[run];lo=np.array([a['data'] for a in d['/debug/lower_mpc_trace']]);xy=np.array([a['xy'] for a in d['/motive/vehicle/odom_map']]);mask=(lo[:,10]==1)&(lo[:,2]>=-100)&(lo[:,2]<=-30);x=lo[mask];axs[0,0].plot(xy[:,0],xy[:,1],c=c,label=label);axs[0,1].plot(x[:,2],np.degrees(x[:,26]),c=c,label=label);axs[1,0].plot(x[:,2],x[:,6],c=c,label=label);ch=np.array(results[run]['path_change_same_approach']['data']);axs[1,1].plot(ch[:,1],ch[:,2],'o-',c=c,label=label)
route=datasets[chosen[0]]['/navigation/global_path'][0]['xy'];axs[0,0].plot(route[:,0],route[:,1],'--o',c='#a58300',label='동일한 전역 경로');axs[0,0].set_aspect('equal');axs[0,0].set(title='전체 자취',xlabel='map X (m)',ylabel='map Y (m)');axs[0,1].set(title='같은 접근 구간의 조향',ylabel='조향 출력 (도)');axs[1,0].set(title='같은 접근 구간의 추종 오차',ylabel='경로 횡오차 (m)');axs[1,1].set(title='차량 진행량을 보정한 계획 경로 변경',ylabel='이전 계획 대비 방향 변화 (도)')
for ax in [axs[0,1],axs[1,0],axs[1,1]]:ax.set_xlabel('map Y (m), -100 → -30 진행');ax.set_xlim(-100,-30)
for ax in axs.flat:ax.grid(alpha=.2);ax.legend(fontsize=9)
fig.suptitle('실측 계획 주기는 모두 1초 · 같은 경로/접근 구간 비교',fontsize=15);fig.tight_layout();fig.savefig(p/'same_route_comparison.png',dpi=160)
