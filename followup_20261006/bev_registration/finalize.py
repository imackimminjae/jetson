from pathlib import Path
import json,pickle,numpy as np,os,hashlib
os.environ['MPLCONFIGDIR']='/tmp/codex_mpl_grid'
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib import font_manager
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf');plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False})
H=Path(__file__).resolve().parent;R=H.parents[1];fig,axs=plt.subplots(1,3,figsize=(15,6.5),layout='constrained')
for ax,tag,ox,title in zip(axs,['032453','181932','181932'],[0,0,2.95],['확장 전 기록','확장 후 · 현재 원점 0','확장 후 · 수신 원점 2.95']):
 d=pickle.load((R/f'drive_debug_20261006_{tag}_light/analysis/decoded.pkl').open('rb'));e=min([v['data'] for v in d['/debug/upper_branch_event']],key=lambda e:abs(e['x']-232));g=[g for g in d['/bev/occupancy_grid'] if g['t']<=e['t_start']][-1];a=e['yaw_rad'];x,y=np.meshgrid(np.arange(129)*.3125+ox,np.arange(129)*.3125-20);X=e['x']+np.cos(a)*x-np.sin(a)*y;Y=e['y']+np.sin(a)*x+np.cos(a)*y;val=np.where(g['grid']<0,0,np.where(g['grid']==0,1,2));ax.pcolormesh(X,Y,val,cmap=ListedColormap(['#eeeeee','#d4eada','#eed3d3']),vmin=0,vmax=2);ax.plot(*d['/navigation/global_path'][0]['xy'].T,'k--o',ms=3,label='동일 지정 경로');ax.plot(e['x'],e['y'],'ko',label='차량 위치');ax.plot(245,138.5,'s',c='#146bcd',ms=6,label='비교 지점');ax.set(xlim=(227,283),ylim=(120,172),xlabel='지도 X(m)',ylabel='지도 Y(m)',title=title);ax.set_aspect('equal');ax.legend(fontsize=8,loc='upper left')
fig.suptitle('수신 원점을 적용하면 도로와 지정 경로의 상대 위치가 달라짐\n오른쪽 두 그림은 동일 BEV 셀 · 좌표 해석만 변경 / 운영에는 미적용',fontsize=12);fig.savefig(H/'origin_comparison.png',dpi=150);plt.close(fig)
summary={'replay':{},'source_integrity':{},'user_confirmed_sender_range':[2.95,42.95]}
for tag in ['180803','181932']:
 d=json.loads((H/f'replay_{tag}.json').read_text());c=json.loads((H.parent/f'deep_{tag}/input.json').read_text())['cycles'];summary['replay'][tag]={'cycles':len(c),'valid_counts':{k:sum(r['valid'] for r in v) for k,v in d.items()},'focus':{k:[{'t':cc['t'],'xy':[cc['x'],cc['y']],'end':r.get('planned',[None])[-1],'B':r['B']} for cc,r in zip(c,v) if 237<cc['x']<251] for k,v in d.items()}}
for k,v in json.loads((H.parent/'deep_180803/sources_sha256.json').read_text()).items():summary['source_integrity'][k]=hashlib.sha256(Path(k).read_bytes()).hexdigest()==v
(H/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2));print('production source/bag integrity',all(summary['source_integrity'].values()))
