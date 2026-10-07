from pathlib import Path
import os,json,pickle,numpy as np,yaml
os.environ['MPLCONFIGDIR']='/tmp/codex_mpl_grid'
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib import font_manager
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf');plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False})
H=Path(__file__).resolve().parent;R=H.parents[1];tags=['032453','181932'];fig,axs=plt.subplots(1,2,figsize=(12,7),layout='constrained');rows=[];samples={}
for ax,tag in zip(axs,tags):
 d=pickle.load((R/f'drive_debug_20261006_{tag}_light/analysis/decoded.pkl').open('rb'));e=min([v['data'] for v in d['/debug/upper_branch_event']],key=lambda e:abs(e['x']-232));g=[g for g in d['/bev/occupancy_grid'] if g['t']<=e['t_start']][-1];yaw=e['yaw_rad'];r=g['res'];x,y=np.meshgrid(np.arange(129)*r,np.arange(129)*r-20);X=e['x']+np.cos(yaw)*x-np.sin(yaw)*y;Y=e['y']+np.sin(yaw)*x+np.cos(yaw)*y
 ax.pcolormesh(X,Y,g['grid'],cmap=ListedColormap(['#eeeeee','#d4eada','#eed3d3']),vmin=-1,vmax=100) if False else ax.pcolormesh(X,Y,np.where(g['grid']<0,0,np.where(g['grid']==0,1,2)),cmap=ListedColormap(['#eeeeee','#d4eada','#eed3d3']),vmin=0,vmax=2)
 ax.plot(*d['/navigation/global_path'][0]['xy'].T,'k--o',ms=3,label='지정 경로');ax.plot(*np.array(e['preview_world']).T,':',c='#b7801c',label='기록 참조');ax.plot(*np.array(e['planned_world']).T,'o-',c='#1963bc',ms=3,label='기록 계획');ax.plot(e['x'],e['y'],'ko');ax.set(xlim=(227,283),ylim=(120,172),xlabel='지도 X(m)',ylabel='지도 Y(m)',title=('확장 전 성공 03:24:53' if tag=='032453' else '확장 후 실패 18:19:32')+f"\n차량 ({e['x']:.2f},{e['y']:.2f}), 방향 {np.degrees(yaw):.1f}°");ax.set_aspect('equal');ax.legend(fontsize=8)
 rows.append({'tag':tag,'pose':[e['x'],e['y'],yaw],'grid_age_sec':e['t_start']-g['t'],'received_origin':g['origin'],'effective_origin':[0,-20],'counts':e['candidate_counts'],'planned':e['planned_world'],'reference':e['preview_world']});samples[tag]=(e,g)
fig.suptitle('비슷한 진입 위치의 입력 비교 · 플래너 원점 0 적용\n녹색=도로, 분홍=비도로, 회색=미관측 · 서로 다른 주행', fontsize=11);fig.savefig(H/'matched_approach.png',dpi=150)
# Fixed world cross sections: compare perceived road intervals, not physical ground truth.
for row in rows:
 e,g=samples[row['tag']];yaw=e['yaw_rad'];row['road_intervals_by_world_y']={}
 for wy in [135,140,145,150]:
  wx=np.arange(240,280,.05);dx=wx-e['x'];dy=wy-e['y'];bx=np.cos(yaw)*dx+np.sin(yaw)*dy;by=-np.sin(yaw)*dx+np.cos(yaw)*dy;ix=np.floor(bx/g['res']).astype(int);iy=np.floor((by+20)/g['res']).astype(int);inside=(ix>=0)&(ix<128)&(iy>=0)&(iy<128);val=np.full(len(wx),-1);val[inside]=g['grid'][iy[inside],ix[inside]];ind=np.flatnonzero(val==0);parts=np.split(ind,np.flatnonzero(np.diff(ind)>1)+1) if len(ind) else [];row['road_intervals_by_world_y'][wy]=[[float(wx[v[0]]),float(wx[v[-1]]+.05)] for v in parts]
(H/'comparison.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2));print([(r['tag'],r['road_intervals_by_world_y']) for r in rows])
