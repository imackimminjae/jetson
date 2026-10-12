from pathlib import Path
import os,json,gzip,ast,itertools,re
os.sched_setaffinity(0,{min(os.sched_getaffinity(0))});os.environ['MPLCONFIGDIR']='/tmp/scenario_detail_diagnostic'
import numpy as np
from scipy.optimize import linprog
H=Path(__file__).resolve().parent;P=H.parent/'boundary_soft_separation';R=H.parents[1]
src=ast.parse((P/'feasibility.py').read_text());exec(compile(ast.Module(body=[n for n in src.body if isinstance(n,ast.FunctionDef)],type_ignores=[]),'lp','exec'))
s=json.loads((H/'summary.json').read_text());d=json.loads(gzip.decompress((H/'telemetry.json.gz').read_bytes()));cs={};rs={};checks={};tests=[]
for i in [1,4,10]:
 cs[i]=json.loads((H/f'input_{i}.json').read_text())['cycles'];rs[i]=json.loads(gzip.decompress((H/f'result_{i}.json.gz').read_bytes()));base=rs[i]['fixed'];errs=[]
 for a,b in zip(cs[i],base):
  if a['rec_valid'] and b['valid'] and np.array(a['rec_planned']).shape==np.array(b['planned']).shape:errs.append(float(np.linalg.norm(np.array(a['rec_planned'])-b['planned'],axis=1).max()))
 checks[i]=dict(cycles=len(base),valid_match=sum(a['rec_valid']==b['valid'] for a,b in zip(cs[i],base)),valid_comparable=len(errs),plan_within_1mm=sum(e<.001 for e in errs),plan_max_error_m=max(errs),mode_failures={mode:sum(not a['valid'] for a,c in zip(z,cs[i]) if c['running']) for mode,z in rs[i].items()})
for i,inds in [(4,[54,55,58,59,60]),(10,[65])]:
 for j in inds:
  c=cs[i][j];base=rs[i]['fixed'][j];yaw=c['yaw'];rot=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]]);refbody=(np.array(base['reference'])-np.array([c['x'],c['y']]))@rot
  row=dict(attempt=i,index=j,t=c['t'],xy=[c['x'],c['y']],yaw_deg=float(np.degrees(yaw)),ref_body=refbody.tolist(),B_before=c['rec_B_before'],B=c['rec_B'],counts=base['counts'],modes={})
  for mode in ['fixed','hold_B','guard_off','raw_diagnostic']:
   z=rs[i][mode][j];ft=run(z,c);row['modes'][mode]=dict(valid=z['valid'],selected=z['selected'],plan=z.get('planned'),reference=z['reference'],feasibility=ft,intervals=z['intervals'],turn_steps=z['turn_steps'],turn_targets=z['turn_targets'])
  row['drop_k6_feasibility']=run(base,c,(6,))
  if i==10:row['feasible_drop_single_stage']={str(k):any(z['feasible'] for z in run(base,c,(k,))) for k in range(1,7)}
  tests.append(row)
(H/'offline_validation.json').write_text(json.dumps(dict(checks=checks,cases=tests),indent=2))
# Exact source outage windows from map odometry receive timestamps, with independent log evidence.
tt=np.array(d['receive_times']['/motive/vehicle/odom_map']);logs=d['data']['/rosout'];loss=[]
for j in np.flatnonzero(np.diff(tt)>1):
 a,b=tt[j:j+2];rows=[z['attempt'] for z in s['attempts'] if a<z['stop_time'] and b>z['start']]
 loss.append(dict(start=float(a),end=float(b),seconds=float(b-a),attempts=rows,logs=[dict(received_t=t,**v) for t,v in logs if a-1<t<b+1 and ('heartbeat' in v['msg'] or 'heading check' in v['msg'] or 'connection regained' in v['msg'])]))
(H/'pose_outages.json').write_text(json.dumps(loss,indent=2))
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf');plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False})
fig,ax=plt.subplots(1,2,figsize=(14,6),layout='constrained');route=np.array(s['route'])
for a in ax:a.plot(*route.T,'k--o',ms=3,label='지정 경로');a.grid(alpha=.2);a.set_aspect('equal');a.set_xlabel('X (m)');a.set_ylabel('Y (m)')
tr=json.loads(gzip.decompress((H/'tracks.json.gz').read_bytes()))
for i,col in [(1,'#148676'),(4,'#c44632'),(10,'#cc9922')]:
 q=tr[i-1];xy=np.array(q['xy']);time=np.array(q['t']);idx=(time<q['end'])&(xy[:,1]>-40)&(xy[:,0]<160);ax[0].plot(*xy[idx].T,color=col,label=f'{i}회 실측 궤적')
for j,col in [(58,'#148676'),(59,'#c44632')]:
 z=rs[4]['fixed'][j];c=cs[4][j];p=np.array(c['rec_published']);ax[0].plot(*p.T,'o-',ms=3,color=col,lw=2,label=f'4회 계획 {j-58}: {c["t"]%60:.3f}초');q=np.array(z['reference']);ax[0].plot(*q.T,':',color=col)
ax[0].set_xlim(103,151);ax[0].set_ylim(-35,25);ax[0].legend(fontsize=8);ax[0].set_title('4회: 동쪽 계획에서 북쪽 계획으로 전환')
# Map-frame view at wrong-branch trigger: compressed selected snapshot only.
c=cs[4][59];g=np.frombuffer(gzip.decompress((H/'grids.bin.gz').read_bytes()),dtype=np.int8).reshape(-1,128,128)[c['grid_index']];ax[1].clear();ax[1].imshow(g,origin='lower',extent=(0,40,-20,20),cmap='gray_r',vmin=0,vmax=100,interpolation='nearest');z=rs[4]['fixed'][59];yaw=c['yaw'];rot=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]]);origin=np.array([c['x'],c['y']])
for pts,style,col,label in [(z['reference'],'--o','#008bbb','기록 참조'),(c['rec_published'],'-o','#ee5522','발행 계획')]:
 a=(np.array(pts)-origin)@rot;ax[1].plot(*a.T,style,color=col,ms=4,label=label)
for k in [4,5,6]:
 for q in z['cands'][k]:
  a=(np.array(q).reshape(2,2)-origin)@rot;ax[1].plot(*a.T,color='#22ab55',lw=3)
ax[1].set_xlim(0,40);ax[1].set_ylim(-24,20);ax[1].axhline(-20,c='k',ls=':');ax[1].set_aspect('equal');ax[1].legend(fontsize=8);ax[1].set_title('4회 17:36:56.898: k6 참조가 측면 지도 밖');ax[1].set_xlabel('차량 전방 (m)');ax[1].set_ylabel('차량 왼쪽 (m)');fig.savefig(H/'branch_detail.png',dpi=150);plt.close(fig)
fig,axs=plt.subplots(2,1,figsize=(13,6),layout='constrained');t0=s['attempts'][0]['start']
for z in s['attempts']:axs[0].plot([(z['start']-t0)/60,(z['end']-t0)/60],[z['attempt']]*2,lw=7,color='#c44632' if z['outcome']=='pose_timeout' else '#148676')
for q in loss:
 for a in axs:a.axvspan((q['start']-t0)/60,(q['end']-t0)/60,color='#eaaa44',alpha=.3)
axs[0].set_yticks(range(1,11));axs[0].set_ylabel('회차');axs[0].set_title('시나리오1: 노란 영역은 실제 위치 메시지 공백')
lo=np.array([v for _,v in d['data']['/debug/lower_mpc_trace']]);axs[1].plot((lo[:,0]-t0)/60,lo[:,12],lw=.8);axs[1].set_ylabel('lower mode');axs[1].set_yticks([0,3,4]);axs[1].set_xlabel('첫 주행 시작 이후 (분)');axs[1].set_title('mode4는 목표 도달 또는 배치 허가 중단; 공백 속 위치/속도는 마지막 값')
fig.savefig(H/'input_outages.png',dpi=140);plt.close(fig)
print('CHECKS',checks)
for q in tests:
 print('CASE',q['attempt'],q['index'],'refbodylast',q['ref_body'][-1],'feasible',q['modes']['fixed']['feasibility'],'drop6',q['drop_k6_feasibility'])
