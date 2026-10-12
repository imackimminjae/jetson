from pathlib import Path
import json,pickle,hashlib,datetime,os,numpy as np
os.environ['MPLCONFIGDIR']='/tmp/codex_mpl_grid'
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib import font_manager
from matplotlib.patches import Polygon
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf');plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False})
H=Path(__file__).resolve().parent;R=H.parents[1];out={};manifest={};fig,axs=plt.subplots(1,3,figsize=(14,6),layout='constrained')
for tag in ['184813','184905','185025','185147']:
 cs=json.loads((H/f'input_{tag}.json').read_text())['cycles'];ds=json.loads((H/f'replay_{tag}.json').read_text());d=pickle.load((R/f'drive_debug_20261006_{tag}_light/analysis/decoded.pkl').open('rb'));es=[e['data'] for e in d['/debug/upper_branch_event']];first=next(e for e in es if not e['valid_plan']);j=next(i for i,c in enumerate(cs) if abs(c['t']-first['t_start'])<.01);errs=[];mismatch=[]
 for i,(c,z) in enumerate(zip(cs,ds['fixed_295'])):
  if c['rec_valid']!=z['valid']:mismatch.append(i)
  if c['rec_valid'] and z['valid'] and len(c['rec_planned'])==len(z['planned']):errs.append({'i':i,'error_m':float(np.max(np.linalg.norm(np.array(c['rec_planned'])-z['planned'],axis=1)))})
 z=ds['fixed_295'][j];c=cs[j];a=c['yaw'];rot=np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]]);ref=(np.array(z['reference'])-[c['x'],c['y']])@rot;iv=next(v for v in first['intervals_world'] if v[0]==1);ab=(np.array([iv[2:4],iv[4:6]])-[c['x'],c['y']])@rot;u=(ab[1]-ab[0])/np.linalg.norm(ab[1]-ab[0]);mid=ab.mean(0);raw=np.array([mid-u*iv[6]/2,mid+u*iv[6]/2]);out[tag]={'recorded_upper_total':len(es),'recorded_valid':sum(e['valid_plan'] for e in es),'goals_true':sum(v['data'] for v in d['/planner/goal_reached']),'replay_cycles':len(cs),'baseline_validity_mismatch_indices':mismatch,'baseline_valid_plan_count':len(errs),'baseline_under1mm':sum(v['error_m']<.001 for v in errs),'baseline_plan_residuals':[v for v in errs if v['error_m']>=.001],'variant_valid_counts':{k:sum(v['valid'] for v in rs) for k,rs in ds.items()},'first_failure':{'t':c['t'],'xy':[c['x'],c['y']],'k1_reference_body':ref[1].tolist(),'k1_raw_width':iv[6],'k1_processed_width':iv[7],'k1_raw_body_endpoints':raw.tolist(),'counterfactual_validity':{k:rs[j]['valid'] for k,rs in ds.items()}}}
 for f in [R/f'drive_debug_20261006_{tag}_light/bag/bag_0.db3',R/f'drive_debug_20261006_{tag}_light/config_snapshot.yaml']:manifest[str(f)]=hashlib.sha256(f.read_bytes()).hexdigest()
 if tag=='184813':continue
 ax=axs[['184905','185025','185147'].index(tag)];g=d['/bev/occupancy_grid'][c['grid_record_index']];x,y=np.meshgrid(2.95+np.arange(129)*.3125,-20+np.arange(129)*.3125);ax.pcolormesh(x,y,g['grid'],cmap=ListedColormap(['#ddeddd','#f1d9d9']),vmin=0,vmax=100);ax.axvspan(0,2.95,color='#eeeeee');ax.axvline(2.95,color='black',ls='--',label='지도 시작 2.95 m');ax.plot(*raw.T,'-',c='#286fa4',lw=5,label='관측된 원단면');ax.plot(*ab.T,'-',c='#d02030',lw=3,label='축소된 허용 구간');ax.plot(*ref[1],'o',c='#222222',label='이전 계획의 k1 참조');ax.plot(0,0,'k^',ms=8)
 h=np.arctan2(ref[1,1],ref[1,0]);v=np.array([np.cos(h),np.sin(h)]);n=np.array([-np.sin(h),np.cos(h)]);phi=np.tan(.3)*.75/2.8;poly=[]
 for dv,sign in [(-.75,-1),(.75,-1),(.75,1),(-.75,1)]:poly.append(ref[1]+.75*v*dv+4.5*n*(sign*phi*(6+dv)-h))
 ax.add_patch(Polygon(poly,closed=True,fc='#e6bb36',ec='#967419',alpha=.4,label='상위 모델의 k1 도달 범위'));ax.set(xlim=(0,8),ylim=(-7,7),xlabel='차량 전방 X (m)',ylabel='차량 좌측 Y (m)',title=datetime.datetime.fromtimestamp(c['t']).strftime('%H:%M:%S.%f')[:-3]+f"\n관측 폭 {iv[6]:.2f} → 허용 폭 {iv[7]:.2f} m");ax.set_aspect('equal');ax.legend(fontsize=7,loc='upper right')
fig.suptitle('원점 변경 후 세 주행의 첫 계산 실패 · k1 단면의 지도 앞쪽 잘림\n빨간 허용 구간과 노란 도달 범위가 겹치지 않음 · 선형화된 상위 모델 기준',fontsize=12);fig.savefig(H/'first_failure_geometry.png',dpi=150);plt.close(fig)
(H/'summary.json').write_text(json.dumps(out,ensure_ascii=False,indent=2));(H/'sources_sha256.json').write_text(json.dumps(manifest,indent=2));print(json.dumps({k:{'recorded_valid':v['recorded_valid'],'total':v['recorded_upper_total'],'baseline_mismatch':v['baseline_validity_mismatch_indices']} for k,v in out.items()},indent=2))
