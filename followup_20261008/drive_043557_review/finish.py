from pathlib import Path
import pickle,json,numpy as np,os
os.environ['MPLCONFIGDIR']='/tmp/codex_drive043557_plot'
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False})
H=Path(__file__).resolve().parent;R=H.parents[1]
d=pickle.load(open(R/'drive_debug_20261008_043557_light/analysis/decoded.pkl','rb'));old=pickle.load(open(R/'drive_debug_20261008_042308_light/analysis/decoded.pkl','rb'))
route=d['/navigation/global_path'][0]['xy'];xy=np.array([z['xy'] for z in d['/motive/vehicle/odom_map']]);ts=np.array([z['t'] for z in d['/motive/vehicle/odom_map']]);ox=np.array([z['xy'] for z in old['/motive/vehicle/odom_map']]);tr=np.array([z['data'] for z in d['/debug/lower_mpc_trace']]);tg=next(z['t'] for z in d['/planner/goal_reached'] if z['data']);j=np.argmin(abs(ts-tg));near=np.linalg.norm(xy-route[-1],axis=1);moving=tr[(tr[:,0]>=tg)&(tr[:,4]<.2)];stop=moving[0];post=xy[(ts>=tg)&(ts<=stop[0])];travel=float(np.linalg.norm(np.diff(post,axis=0),axis=1).sum())
extra={'closest_goal_distance_m':float(min(near)),'goal_time':float(tg),'goal_trace_speed_mps':float(tr[np.argmin(abs(tr[:,0]-tg)),4]),'first_below_0_2_mps_after_goal_s':float(stop[0]-tg),'travel_until_below_0_2_mps_m':travel,'stop_xy':stop[1:3].tolist(),'virtual_brake_goal_mode_unique':np.unique(tr[tr[:,12]==4,15]).tolist(),'mavlink_throttle_goal_mode_unique':np.unique(tr[tr[:,12]==4,16]).tolist()}
(H/'stop_detail.json').write_text(json.dumps(extra,indent=2))
fig,ax=plt.subplots(figsize=(8,8),layout='constrained');ax.plot(*route.T,'k--o',ms=4,label='지정 경로');ax.plot(*ox.T,color='#bb7755',label='직전 N7: 북쪽 오분기');ax.plot(*xy[:j+1].T,color='#008d71',lw=2,label='최신 N6: 동쪽 정분기');ax.plot(*xy[j:].T,color='#8d5bc0',lw=2,label='도착 신호 이후 진행');ax.scatter(*route[-1],marker='*',s=150,c='black');ax.annotate('목표 근처 통과\n최소 거리 0.35 m',route[-1],xytext=(176,-40),arrowprops=dict(arrowstyle='->'));ax.annotate('최종 정지 위치\n목표에서 약 50 m',xy[-1],xytext=(188,22),arrowprops=dict(arrowstyle='->'));ax.set_aspect('equal');ax.set_xlabel('지도 X (m)');ax.set_ylabel('지도 Y (m)');ax.set_title('04:35:57 최신 기록 — 두 분기 정분기, 목표 이후 정지는 별도 문제');ax.legend(loc='lower right',fontsize=9);ax.grid(alpha=.2);fig.savefig(H/'comparison.png',dpi=150);plt.close(fig)
print(json.dumps(extra,indent=2))
