from pathlib import Path
import pickle,numpy as np,os,json
os.environ['MPLCONFIGDIR']='/tmp/codex_mpl_grid'
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib import font_manager
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf');plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False})
H=Path(__file__).resolve().parent;R=H.parents[1];fig,axs=plt.subplots(1,2,figsize=(13,6),layout='constrained');info=[]
for ax,tag,target in zip(axs,['171507','174526'],[1791274530.691,1791276387.941]):
 d=pickle.load((R/f'drive_debug_20261006_{tag}_light/analysis/decoded.pkl').open('rb'));e=min([v['data'] for v in d['/debug/upper_branch_event']],key=lambda e:abs(e['t_start']-target));gg=[m for m in d['/bev/occupancy_grid'] if m['t']<=e['t_start']];g=gg[-1];r=g['res'];x,y=np.meshgrid(np.arange(129)*r,np.arange(129)*r-20);yaw=e['yaw_rad'];X=e['x']+np.cos(yaw)*x-np.sin(yaw)*y;Y=e['y']+np.sin(yaw)*x+np.cos(yaw)*y
 ax.pcolormesh(X,Y,g['grid'],cmap=ListedColormap(['#acd7ef','#f0d4d4']),vmin=0,vmax=100,rasterized=True);gp=d['/navigation/global_path'][0]['xy'];ax.plot(*gp.T,'k--',label='지정 경로');pl=np.array(e['planned_world']);ref=np.array(e['preview_world']);ax.plot(*ref.T,'o-',c='#aa6e00',ms=3,label='이전 계획에서 이어진 참조');ax.plot(*pl.T,'o-',c='#2060bb',ms=3,label='새 계획')
 for iv in e['intervals_world']:
  ax.plot([iv[2],iv[4]],[iv[3],iv[5]],c='#888888',lw=.6)
 ax.plot(e['x'],e['y'],'ko',ms=5);ax.set_xlim(225,287);ax.set_ylim(116,173);ax.set_aspect('equal');ax.set_title(tag+' · '+('17:15:30.691' if tag=='171507' else '17:46:27.941'));ax.legend(fontsize=8);ax.set_xlabel('지도 X (m)');ax.set_ylabel('지도 Y (m)');info.append({'tag':tag,'event':e['t_start'],'grid_age_sec':e['t_start']-g['t'],'limits':'Nearest prior bag reception; not proof of exact grid consumed. Effective origin0, event pose; received origin2.95 remains overridden.'})
fig.suptitle('분기 직전 맵·참조·계획 대조 (인접 수신 맵, 플래너 원점 0 적용)\n파랑=도로 셀, 분홍=비도로 셀, 회색=허용 단면',fontsize=13);fig.savefig(H/'branch_maps.png',dpi=145);(H/'branch_map_alignment.json').write_text(json.dumps(info,indent=2))
