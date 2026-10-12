from pathlib import Path
import json,pickle,numpy as np,os
os.environ['MPLCONFIGDIR']='/tmp/codex_scenario3_repeat_plot'
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.colors import ListedColormap,BoundaryNorm
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf');plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False})
H=Path(__file__).resolve().parent;R=H.parents[1];tags=['044813','045057','045146'];data={t:pickle.load(open(R/f'drive_debug_20261008_{t}_light/analysis/decoded.pkl','rb')) for t in tags};colors=['#87949b','#c64d2d','#148d73'];names=['04:48:13 이전 정분기','04:50:57 추가 1: 오분기','04:51:46 추가 2: 정분기'];route=data[tags[0]]['/navigation/global_path'][0]['xy']
def project(xy):
 v=np.diff(route,axis=0);l=np.linalg.norm(v,axis=1);u=np.clip(np.sum((xy[:,None,:]-route[None,:-1,:])*v,axis=2)/l**2,0,1);q=route[None,:-1,:]+u[:,:,None]*v;j=np.argmin(np.linalg.norm(xy[:,None,:]-q,axis=2),axis=1);return np.r_[0,np.cumsum(l)][j]+u[np.arange(len(xy)),j]*l[j]
fig=plt.figure(figsize=(13,7),layout='constrained');gs=fig.add_gridspec(2,2);ax=fig.add_subplot(gs[:,0]);a=fig.add_subplot(gs[0,1]);b=fig.add_subplot(gs[1,1]);ax.plot(*route.T,'k--o',ms=3,label='지정 경로')
for tag,col,name in zip(tags,colors,names):
 d=data[tag];xy=np.array([z['xy'] for z in d['/motive/vehicle/odom_map']]);ot=np.array([z['t'] for z in d['/motive/vehicle/odom_map']]);gg=[z['t'] for z in d.get('/planner/goal_reached',[]) if z['data']];end=np.searchsorted(ot,gg[0]) if gg else len(xy);ax.plot(*xy[:end].T,color=col,lw=1.7,label=name)
 if gg:ax.plot(*xy[end-1:].T,color=col,ls=':',lw=1.5)
 tr=np.array([z['data'] for z in d['/debug/lower_mpc_trace']]);pr=project(tr[:,1:3]);m=(tr[:,12]==0)&(pr>=65)&(pr<=110);a.plot(pr[m],np.rad2deg(tr[m,26]),color=col,lw=1.4,label=name);es=[z['data'] for z in d['/debug/upper_branch_event']];cs=np.array([z['data'] for z in d['/debug/upper_interval_centering_state']]);p=project(np.array([[e['x'],e['y']] for e in es]));bb=np.array([cs[np.argmin(abs(cs[:,0]-e['t_start'])),2] for e in es]);m=(p>=65)&(p<=110);b.plot(p[m],bb[m],color=col,drawstyle='steps-post')
e=data['045057']['/debug/upper_branch_event'][36]['data'];ax.scatter(e['x'],e['y'],marker='x',s=70,c='#b22d22');ax.annotate('04:51:19.524\n오분기 계획 시작',(e['x'],e['y']),xytext=(216,116),arrowprops=dict(arrowstyle='->'),fontsize=9);ax.scatter(*route[-1],marker='*',s=100,c='black');ax.set_aspect('equal');ax.set_xlabel('지도 X (m)');ax.set_ylabel('지도 Y (m)');ax.legend(fontsize=8);ax.grid(alpha=.2);ax.set_title('추가 2회: 오분기 1회 / 정분기 1회\n점선 주행 궤적은 도착 신호 이후')
a.set_ylabel('출력 조향 (도)');a.set_title('동일 경로 진행량으로 비교: 큰 조향 변화가 반복');b.set_ylabel('B (m)');b.set_xlabel('지정 경로상 진행량 (m)')
for aa in [a,b]:aa.axvspan(87,90,color='#f5d9aa',alpha=.5);aa.grid(alpha=.2);aa.set_xlim(65,110)
fig.savefig(H/'comparison.png',dpi=150);plt.close(fig)
fig,axs=plt.subplots(1,3,figsize=(15,6),layout='constrained');cmap=ListedColormap(['#c4d1e0','white','#797e84']);norm=BoundaryNorm([-2,-.5,.5,101],3)
for ax,tag,i,title in zip(axs,['045057','045057','045146'],[35,36,40],['실패 주행: 04:51:19.024','실패 주행: 04:51:19.524','성공 주행: 04:52:07.354']):
 d=data[tag];e=d['/debug/upper_branch_event'][i]['data'];gs0=d['/bev/occupancy_grid'];gt=np.array([z['t'] for z in gs0]);g=gs0[np.searchsorted(gt,e['t_start'])-1];p=np.array([e['x'],e['y']]);yaw=e['yaw_rad'];rot=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]]);xx,yy=np.meshgrid(np.arange(129)*g['res']+g['origin'][0],np.arange(129)*g['res']+g['origin'][1]);wx=p[0]+np.cos(yaw)*xx-np.sin(yaw)*yy;wy=p[1]+np.sin(yaw)*xx+np.cos(yaw)*yy;ax.set_facecolor('#c4d1e0');ax.pcolormesh(wx,wy,g['grid'],cmap=cmap,norm=norm,shading='flat');edge=np.array([[0,20],[40,20]])@rot.T+p;ax.plot(*edge.T,'--',c='#e09e28',lw=2,label='지도 왼쪽 경계 y=+20m');ax.plot(*route.T,'k--',lw=1,label='지정 경로');ref=np.array(e['preview_world']);pl=np.array(e['planned_world']);ax.plot(*ref.T,':',c='#874caf',label='참조');ax.plot(*pl.T,'-o',ms=3,c='#c54330',label='선택된 계획');ax.scatter(*p,marker='^',c='black')
 for iv in e['intervals_world']:
  if iv[0]==6:ax.plot([iv[2],iv[4]],[iv[3],iv[5]],c='#00a4b7',lw=4,label='k6 후보' if iv[1]==0 else None)
 ax.set_xlim(232,282);ax.set_ylim(117,164);ax.set_aspect('equal');ax.set_title(title);ax.set_xlabel('지도 X (m)');ax.legend(loc='lower left',fontsize=7)
fig.suptitle('N6에서도 정분기 쪽 마지막 후보가 지도 왼쪽 경계에서 소실\n성공 주행은 k6 정분기 후보가 유지됨 · 흰색=도로, 회색=비도로, 파랑=지도 밖',fontsize=13);fig.savefig(H/'branch_boundary.png',dpi=150);plt.close(fig)
