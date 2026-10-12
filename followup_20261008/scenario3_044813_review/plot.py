from pathlib import Path
import pickle,json,numpy as np,os
os.environ['MPLCONFIGDIR']='/tmp/codex_scenario3_plot'
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.colors import ListedColormap,BoundaryNorm
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False})
H=Path(__file__).resolve().parent;R=H.parents[1];d=pickle.load(open(R/'drive_debug_20261008_044813_light/analysis/decoded.pkl','rb'));es=[z['data'] for z in d['/debug/upper_branch_event']];tr=np.array([z['data'] for z in d['/debug/lower_mpc_trace']]);xy=np.array([z['xy'] for z in d['/motive/vehicle/odom_map']]);ot=np.array([z['t'] for z in d['/motive/vehicle/odom_map']]);route=d['/navigation/global_path'][0]['xy'];cs=np.array([z['data'] for z in d['/debug/upper_interval_centering_state']]);base=es[0]['t_start']-16.996;tg=next(z['t'] for z in d['/planner/goal_reached'] if z['data']);j=np.argmin(abs(ot-tg));di=np.linalg.norm(xy-route[-1],axis=1)
fig=plt.figure(figsize=(13,7),layout='constrained');gs=fig.add_gridspec(3,2,width_ratios=[1,1.3]);ax=fig.add_subplot(gs[:,0]);ax.plot(*route.T,'k--o',ms=4,label='지정 경로');ax.plot(*xy[:j+1].T,color='#008d71',lw=2,label='N6 실제 주행');ax.plot(*xy[j:].T,color='#be533d',lw=2,label='도착 신호 이후');ax.scatter(*route[-1],marker='*',s=140,c='black');
for i in [25,31]:
 e=es[i];ax.scatter(e['x'],e['y'],c='#c85326',s=40);ax.annotate('%.1f초'%(e['t_start']-base),(e['x'],e['y']),xytext=(e['x']-9,e['y']-13),arrowprops=dict(arrowstyle='->'),fontsize=9)
ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(fontsize=9);ax.set_xlabel('지도 X (m)');ax.set_ylabel('지도 Y (m)');ax.set_title('시나리오 3: 지정 갈래를 따라 목표 근처 통과')
a=fig.add_subplot(gs[0,1]);a.plot(tr[:,0]-base,np.rad2deg(tr[:,26]),lw=1.4);a.axhline(17.19,color='gray',ls=':',lw=.8);a.axhline(-17.19,color='gray',ls=':',lw=.8);a.set_ylabel('출력 조향 (도)');a.set_title('04:48:29~35의 큰 조향 반전')
b=fig.add_subplot(gs[1,1],sharex=a);b.plot(tr[:,0]-base,tr[:,6],color='#a44d21');b.set_ylabel('지역 경로 횡오차 (m)')
c=fig.add_subplot(gs[2,1],sharex=a);c.plot(cs[:,0]-base,cs[:,2],color='#8f5bbb',drawstyle='steps-post',label='B');c.set_ylabel('B (m)');c.set_xlabel('04:48 이후 시간 (초)')
for ax in [a,b,c]:
 ax.axvspan(29.497,35,color='#faebce',alpha=.6)
 for s in [29.497,32.497]:ax.axvline(s,color='#bd6232',ls='--',lw=.7)
 ax.axvline(tg-base,color='#1b8d69',ls=':',label='도착 신호');ax.grid(alpha=.2);ax.set_xlim(16,43.2)
fig.savefig(H/'overview.png',dpi=150);plt.close(fig)
fig,axs=plt.subplots(1,3,figsize=(15,5.5),layout='constrained');gr=d['/bev/occupancy_grid'];gt=np.array([z['t'] for z in gr]);cmap=ListedColormap(['#c4d1e0','white','#767c83']);norm=BoundaryNorm([-2,-.5,.5,101],3)
for ax,i in zip(axs,[24,25,43]):
 e=es[i];g=gr[np.searchsorted(gt,e['t_start'])-1];p=np.array([e['x'],e['y']]);yaw=e['yaw_rad'];rot=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]]);xx,yy=np.meshgrid(np.arange(129)*g['res']+g['origin'][0],np.arange(129)*g['res']+g['origin'][1]);wx=p[0]+np.cos(yaw)*xx-np.sin(yaw)*yy;wy=p[1]+np.sin(yaw)*xx+np.cos(yaw)*yy;ax.set_facecolor('#c4d1e0');ax.pcolormesh(wx,wy,g['grid'],cmap=cmap,norm=norm,shading='flat');ax.plot(*route.T,'k--',lw=1,label='지정 경로');plan=np.array(e['planned_world']);ref=np.array(e['preview_world']);ax.plot(*ref.T,':',c='#8c4fab',label='참조');ax.plot(*plan.T,'-o',ms=3,c='#bc492b',label='계산된 계획');ax.scatter(*p,c='black',marker='^');
 for iv in e['intervals_world']:
  if iv[0]==1:ax.plot([iv[2],iv[4]],[iv[3],iv[5]],color='#03adbc',lw=4,label='k1 허용 구간' if iv[1]==0 else None)
 if i<40:ax.set_xlim(216,257);ax.set_ylim(129,169)
 else:ax.set_xlim(250,295);ax.set_ylim(145,191)
 ax.set_aspect('equal');ax.set_title('04:48:%.3f / N%d'%(e['t_start']-base,e['horizon']));ax.set_xlabel('지도 X (m)');ax.legend(fontsize=8)
fig.suptitle('좌/중: k1 후보가 2개→1개로 바뀌는 시점 · 우: 끝부분 비도로 입력으로 N5 축소\n흰색=도로(0), 회색=비도로(100), 파란 배경=지도 밖',fontsize=12);fig.savefig(H/'map_diagnostics.png',dpi=150);plt.close(fig)
extra={'closest_goal_m':float(di.min()),'closest_goal_xy':xy[di.argmin()].tolist(),'last_speed_mps':float(tr[-1,4]),'record_after_goal_s':float(ot[-1]-tg),'last_goal_distance_m':float(di[-1]),'max_lateral_error_time_seconds':float(tr[np.argmax(abs(tr[:,6])),0]-base),'config_identical_to_043557':(R/'drive_debug_20261008_043557_light/config_snapshot.yaml').read_bytes()==(R/'drive_debug_20261008_044813_light/config_snapshot.yaml').read_bytes()};(H/'details.json').write_text(json.dumps(extra,indent=2));print(extra)
