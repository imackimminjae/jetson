from pathlib import Path
import json,pickle,numpy as np,os
os.environ['MPLCONFIGDIR']='/tmp/codex_latest_drive_plot'
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap,BoundaryNorm
from matplotlib import font_manager
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False})
H=Path(__file__).resolve().parent;R=H.parents[1]
d=pickle.load((R/'drive_debug_20261008_042308_light/analysis/decoded.pkl').open('rb'))
old=pickle.load((R/'drive_debug_20261006_050627_light/analysis/decoded.pkl').open('rb'))
route=d['/navigation/global_path'][0]['xy'];xy=np.array([z['xy'] for z in d['/motive/vehicle/odom_map']]);oldxy=np.array([z['xy'] for z in old['/motive/vehicle/odom_map']]);es=[z['data'] for z in d['/debug/upper_branch_event']];inp=json.loads((H/'input_042308.json').read_text());rs=json.loads((H/'replay_042308.json').read_text())['fixed'];n6=json.loads((H/'first_wrong_N6_result.json').read_text())['fixed'][0]
fig,ax=plt.subplots(figsize=(7,8),layout='constrained');ax.plot(*route.T,'k--o',ms=4,label='지정 경로');ax.plot(*oldxy.T,c='#22a579',lw=1.5,label='10/06 05:06 기존 정분기');ax.plot(*xy.T,c='#c85c30',lw=2,label='10/08 04:23 최신 주행');ax.scatter([es[59]['x']],[es[59]['y']],marker='*',s=130,c='red',label='04:23:43.582 오분기 계획 시작');goal=json.loads((H/'summary.json').read_text())['20261008_042308']['goal'];ax.scatter(*goal['xy'],marker='x',s=90,c='purple');ax.annotate('도착 신호 발생\n실제 목표까지 69.93 m',goal['xy'],xytext=(112,55),arrowprops=dict(arrowstyle='->'),fontsize=10);ax.set_aspect('equal');ax.set_xlabel('지도 X (m)');ax.set_ylabel('지도 Y (m)');ax.set_title('첫 분기 통과 → 두 번째 북쪽 오분기\n상위 계산은 92/92 성공');ax.grid(alpha=.2);ax.legend(loc='lower left',fontsize=9);fig.savefig(H/'latest_track.png',dpi=150);plt.close(fig)
fig,axes=plt.subplots(1,2,figsize=(13,6),layout='constrained')
for ax,i,title in zip(axes,[58,59],['04:23:43.082 — 정분기 k7 원폭 0.10 m','04:23:43.582 — k7에 북쪽 후보만 남음']):
 c=inp['cycles'][i];g=d['/bev/occupancy_grid'][c['grid_record_index']];a=g['grid'];yaw=c['yaw'];rot=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]]);pos=np.array([c['x'],c['y']]);xx,yy=np.meshgrid(np.arange(129)*g['res']+g['origin'][0],np.arange(129)*g['res']+g['origin'][1]);wx=pos[0]+np.cos(yaw)*xx-np.sin(yaw)*yy;wy=pos[1]+np.sin(yaw)*xx+np.cos(yaw)*yy
 ax.set_facecolor('#c9d5e3');ax.pcolormesh(wx,wy,a,cmap=ListedColormap(['#c9d5e3','white','#777d83']),norm=BoundaryNorm([-2,-.5,.5,101],3),rasterized=True,shading='flat')
 edge=np.array([[0,-20],[40,-20]])@rot.T+pos;ax.plot(*edge.T,c='#e59317',ls='--',lw=2,label='지도 오른쪽 경계 y=-20 m')
 r=rs[i];ref=np.array(r['reference']);plan=np.array(r['planned']);ax.plot(*route[2:].T,'k--',lw=1,label='지정 경로');ax.plot(*ref.T,':',c='#804cb8',lw=2,label='사용된 참조');ax.plot(*plan.T,'-o',ms=4,c='#bd342c',label='실제 N7 계획')
 for iv in r['cands'][7]:q=np.array(iv).reshape(2,2);ax.plot(*q.T,c='#00a9bb',lw=4)
 ax.scatter(*ref[7],s=100,marker='x',c='#804cb8',zorder=10)
 if i==59:
  q=np.array(n6['planned']);ax.plot(*q.T,'-o',ms=3,c='#138c62',label='동일 시점 N6 재계산');ax.annotate('k7 참조가 지도 밖\n차량좌표 y=-20.012 m',ref[7],xytext=(134,-23),arrowprops=dict(arrowstyle='->'),fontsize=9)
 else:ax.annotate('남은 정분기 허용폭 0.04 m',plan[-1],xytext=(124,-25),arrowprops=dict(arrowstyle='->'),fontsize=9)
 ax.plot(*pos,'k^',ms=8);ax.set_xlim(102,149);ax.set_ylim(-30,26);ax.set_aspect('equal');ax.set_xlabel('지도 X (m)');ax.set_ylabel('지도 Y (m)');ax.set_title(title);ax.legend(fontsize=8,loc='upper left')
fig.suptitle('전방 0~40 m 지도에서도 회전 중 오른쪽 갈래가 가로 범위 밖으로 나감\n파란 배경=지도 밖 · N6 결과는 한 시점의 오프라인 계산이며 실주행 검증 아님',fontsize=13);fig.savefig(H/'k7_side_boundary.png',dpi=150);plt.close(fig)
