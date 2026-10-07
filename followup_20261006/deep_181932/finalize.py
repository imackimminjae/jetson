from pathlib import Path
import json,pickle,hashlib,os,numpy as np
os.environ['MPLCONFIGDIR']='/tmp/codex_mpl_grid'
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib import font_manager
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf');plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False})
H=Path(__file__).resolve().parent;R=H.parents[1];C=json.loads((H/'input.json').read_text())['cycles'];D=json.loads((H/'result.json').read_text());d=pickle.load((R/'drive_debug_20261006_181932_light/analysis/decoded.pkl').open('rb'));i=32;c=C[i];g=d['/bev/occupancy_grid'][c['grid_record_index']];r=g['res'];x,y=np.meshgrid(np.arange(129)*r,np.arange(129)*r-20);a=c['yaw'];gx=c['x']+np.cos(a)*x-np.sin(a)*y;gy=c['y']+np.sin(a)*x+np.cos(a)*y
fig,axs=plt.subplots(1,2,figsize=(13,6),layout='constrained')
for ax,k,title in zip(axs,['baseline','fresh_ref_recorded_B'],['기존 참조 + B 6.2 m','새 참조 + 동일 B 6.2 m']):
 z=D[k][i];ax.pcolormesh(gx,gy,g['grid'],cmap=ListedColormap(['#e1f0e4','#f0d8d8']),vmin=0,vmax=100,rasterized=True);gp=d['/navigation/global_path'][0]['xy'];ax.plot(*gp.T,'k--o',ms=3,label='지정 경로');ax.plot(*np.array(z['reference']).T,':',c='#bc7f21',lw=2,label='단면 추출용 참조');ax.plot(*np.array(z['planned']).T,'o-',c='#1963bc',ms=4,lw=2,label='계산된 계획')
 for k2 in range(1,len(z['cands'])):
  for iv in z['cands'][k2]:ax.plot([iv[0],iv[2]],[iv[1],iv[3]],c='#666666',lw=.8,alpha=.7)
 ax.plot(c['x'],c['y'],'ko',ms=7,label='동일한 차량 위치');ax.set(xlim=(238,287),ylim=(119,176),xlabel='지도 X (m)',ylabel='지도 Y (m)',title=title);ax.set_aspect('equal');ax.legend(fontsize=8,loc='upper left')
fig.suptitle('최신 18:19:52.566 · 같은 지도와 B에서 참조만 변경한 대조\n초록: 도로 셀 / 분홍: 비도로 셀 / 회색: 허용 단면 / 실제 주행 성공을 뜻하지 않음');fig.savefig(H/'reference_comparison.png',dpi=150);plt.close(fig)
errors=[]
for j,(cc,z) in enumerate(zip(C,D['baseline'])):
 if cc['rec_valid'] and z['valid'] and len(cc['rec_planned'])==len(z['planned']):errors.append({'i':j,'max_m':float(np.linalg.norm(np.array(cc['rec_planned'])-np.array(z['planned']),axis=1).max())})
s={'cycles':len(C),'skipped_recorded_horizon_below2':51-len(C),'validity_matches':sum(cc['rec_valid']==z['valid'] for cc,z in zip(C,D['baseline'])),'valid_plans':len(errors),'valid_plans_under_1mm':sum(e['max_m']<.001 for e in errors),'residuals_over_1mm':[e for e in errors if e['max_m']>=.001],'valid_by_variant':{k:sum(z['valid'] for z in v) for k,v in D.items()},'critical_cycle':c,'critical_variant_results':{k:{'B':v[i]['B'],'end':v[i]['planned'][-1],'selected_nominal_fallback':[iv['k'] for iv in v[i]['intervals'] if iv['fallback'] and v[i]['selected'][iv['k']]==iv['i']]} for k,v in D.items()}}
(H/'summary.json').write_text(json.dumps(s,ensure_ascii=False,indent=2))
files=[R/'drive_debug_20261006_181932_light/bag/bag_0.db3',R/'drive_debug_20261006_181932_light/config_snapshot.yaml',R/'src/virtual_control/src/tracking_control.cpp',R/'src/virtual_control/include/virtual_control/egocentric_planner_geometry.hpp',H.parent/'deep_180803/replay',H/'input.json',H/'grids.bin',H/'config.yaml'];(H/'sources_sha256.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},indent=2));print(json.dumps(s,ensure_ascii=False,indent=2))
