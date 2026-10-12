from pathlib import Path
import pickle,json,numpy as np,os
os.environ['MPLCONFIGDIR']='/tmp/codex_side_live_plot'
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.colors import ListedColormap,BoundaryNorm
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf');plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False})
H=Path(__file__).resolve().parent;R=H.parents[1];data={t:pickle.load(open(R/f'drive_debug_20261008_{t}_light/analysis/decoded.pkl','rb')) for t in ['045146','052329','052412']};route=data['052329']['/navigation/global_path'][0]['xy'];fig,axs=plt.subplots(1,2,figsize=(12,7),layout='constrained');ax=axs[0];ax.plot(*route.T,'k--o',ms=3,label='지정 경로')
for t,col,name in [('045146','#a5adae','이전 정분기'),('052329','#c8522e','05:23:29 오분기'),('052412','#138c74','05:24:12 정분기 방향 / 기록 종료')]:
 d=data[t];xy=np.array([z['xy'] for z in d['/motive/vehicle/odom_map']]);ot=np.array([z['t'] for z in d['/motive/vehicle/odom_map']]);goals=[z['t'] for z in d.get('/planner/goal_reached',[]) if z['data']]
 if goals:xy=xy[ot<=goals[0]]
 ax.plot(*xy.T,c=col,lw=1.7,label=name)
 if t=='052412':ax.scatter(*xy[-1],c=col,marker='x',s=70)
ax.set_aspect('equal');ax.set_title('신규 2회: 보호 조건은 모두 미발동');ax.set_xlabel('지도 X (m)');ax.set_ylabel('지도 Y (m)');ax.grid(alpha=.2);ax.legend(fontsize=8)
ax=axs[1];d=data['052329'];e=d['/debug/upper_branch_event'][36]['data'];gr=d['/bev/occupancy_grid'];gt=np.array([z['t'] for z in gr]);g=gr[np.searchsorted(gt,e['t_start'])-1];p=np.array([e['x'],e['y']]);yaw=e['yaw_rad'];rot=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]]);xx,yy=np.meshgrid(np.arange(129)*g['res'],np.arange(129)*g['res']-20);wx=p[0]+np.cos(yaw)*xx-np.sin(yaw)*yy;wy=p[1]+np.sin(yaw)*xx+np.cos(yaw)*yy;ax.set_facecolor('#c4d1e0');ax.pcolormesh(wx,wy,g['grid'],cmap=ListedColormap(['#c4d1e0','white','#787f84']),norm=BoundaryNorm([-2,-.5,.5,101],3),shading='flat');ax.plot(*route.T,'k--',lw=1,label='지정 경로');ref=np.array(e['preview_world']);ax.plot(*ref.T,':',c='#8d57a5',label='참조');plan=np.array(e['planned_world']);ax.plot(*plan.T,'-o',c='#c8522e',ms=3,label='실제 발행 계획');diag=json.loads((H/'diagnostic_result.json').read_text())['force_terminal'][0];q=np.array(diag['published']);ax.plot(*q.T,'-o',c='#138c74',ms=3,label='k6 제약 강제 제외 진단');edge=np.array([[0,20],[40,20]])@rot.T+p;ax.plot(*edge.T,'--',c='#e09c23',lw=2,label='지도 왼쪽 경계');rb=(ref[-1]-p)@rot;point=np.array([rb[0],20])@rot.T+p;ax.plot([ref[-1,0],point[0]],[ref[-1,1],point[1]],c='#8d57a5');ax.scatter(*ref[-1],c='#8d57a5',marker='x',s=60);ax.annotate('참조점~경계 5.55 m\n허용 조건 4.81 m 초과',ref[-1],xytext=(242,157),arrowprops=dict(arrowstyle='->'),fontsize=9)
ax.set_xlim(232,282);ax.set_ylim(120,166);ax.set_aspect('equal');ax.set_title('05:23:49.089 — 참조점 거리 조건이 보호를 차단');ax.set_xlabel('지도 X (m)');ax.legend(loc='lower left',fontsize=8)
fig.savefig(H/'comparison.png',dpi=150);plt.close(fig)
