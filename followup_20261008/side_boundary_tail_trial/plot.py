from pathlib import Path
import json,pickle,numpy as np,os
os.environ['MPLCONFIGDIR']='/tmp/codex_boundary_trial_plot'
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.colors import ListedColormap,BoundaryNorm
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf');plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False})
H=Path(__file__).resolve().parent;R=H.parents[1];fig,axs=plt.subplots(1,2,figsize=(12,6),layout='constrained')
for ax,tag,i,title in zip(axs,['20261008_042308','20261008_045057'],[59,36],['시나리오 1: 오른쪽 경계 / k7','시나리오 3: 왼쪽 경계 / k6']):
 c=json.loads((H/f'input_{tag}.json').read_text())['cycles'][i];rs=json.loads((H/f'result_{tag}.json').read_text());old=rs['fixed_off'][i];new=rs['fixed_on'][i];d=pickle.load(open(R/f'drive_debug_{tag}_light/analysis/decoded.pkl','rb'));g=d['/bev/occupancy_grid'][c['grid_record_index']];p=np.array([c['x'],c['y']]);yaw=c['yaw'];rot=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]]);xx,yy=np.meshgrid(np.arange(129)*g['res']+g['origin'][0],np.arange(129)*g['res']+g['origin'][1]);wx=p[0]+np.cos(yaw)*xx-np.sin(yaw)*yy;wy=p[1]+np.sin(yaw)*xx+np.cos(yaw)*yy;ax.set_facecolor('#c4d1e0');ax.pcolormesh(wx,wy,g['grid'],cmap=ListedColormap(['#c4d1e0','white','#777d83']),norm=BoundaryNorm([-2,-.5,.5,101],3),shading='flat');route=d['/navigation/global_path'][0]['xy'];ax.plot(*route.T,'k--',lw=1,label='지정 경로');ref=np.array(old['reference']);ax.plot(*ref.T,':',color='#925baa',label='변경하지 않은 참조');q=np.array(old['planned']);ax.plot(*q.T,'-o',color='#c65035',ms=3,label='기존: 오분기');q=np.array(new['planned']);ax.plot(*q[-2:].T,':',color='#168c6b',lw=2,label='계산만 유지 / 발행 제외');q=np.array(new['published']);ax.plot(*q.T,'-o',color='#168c6b',lw=2,ms=4,label='수정: 정분기 쪽 발행 경로');edge=np.array([[0,new['side_boundary_side']*20],[40,new['side_boundary_side']*20]])@rot.T+p;ax.plot(*edge.T,'--',c='#dc9a21',lw=1.7,label='지도 좌우 경계');ax.scatter(*p,marker='^',c='black');ax.set_aspect('equal');ax.set_title(title);ax.set_xlabel('지도 X (m)');ax.legend(fontsize=8)
 if i==59:ax.set_xlim(106,147);ax.set_ylim(-29,19)
 else:ax.set_xlim(235,280);ax.set_ylim(117,163)
fig.suptitle('동일한 지도·차량 위치·참조에서의 오프라인 비교\n전체 주행 성공 검증은 아니며, 앞쪽 도로 제약과 차량 입력 한계는 유지',fontsize=12);fig.savefig(H/'fixed_input_comparison.png',dpi=150);plt.close(fig)
