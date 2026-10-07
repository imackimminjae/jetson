from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager,transforms
from matplotlib.colors import ListedColormap,BoundaryNorm
here=Path(__file__).resolve().parent
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False,'font.size':11})
inp=json.loads((here/'replay_050627.json').read_text());rs=json.loads((here/'result_050627_recorded.json').read_text());i=47;c=inp['cycles'][i]
grids=np.fromfile(here/'replay_050627_grids.bin',np.int8).reshape(-1,128,128);g=grids[c['grid_index']]
fig=plt.figure(figsize=(12,9));gs=fig.add_gridspec(2,2,height_ratios=[3.2,1.5],hspace=.35)
for k,title,col in [(1,'참조·B를 기록값으로 고정: 두 안 모두 동쪽',0),(0,'이전 계획을 다음 주기로 전달: 상한 안은 북쪽',1)]:
 ax=fig.add_subplot(gs[0,col]);tr=transforms.Affine2D().rotate(c['yaw']).translate(c['x'],c['y'])+ax.transData
 ax.imshow(g,origin='lower',extent=(0,40,-20,20),transform=tr,cmap=ListedColormap(['#ccd1d8','#fafafa','#a5a9b0']),norm=BoundaryNorm([-1.5,-.5,.5,101],3),interpolation='nearest',alpha=.75)
 route=np.array(inp['route']);ax.plot(*route.T,'--',color='#648b71',lw=1.4,label='지정 웨이포인트 경로')
 rec=np.array(c['rec_planned']);ax.plot(*rec.T,':',color='black',lw=1.4,label='당시 기록된 상위 계획')
 for cap,color,label in [(0,'#2477bc','기존'),(12,'#dd7324','12 m 상한')]:
  a=np.array(rs[f'{k}_{cap:.6f}'][i]['planned']);ax.plot(*a.T,'o-',color=color,lw=2.4,ms=3.5,label=label)
 ax.scatter(c['x'],c['y'],c='black',marker='^',s=65,zorder=10);ax.text(c['x']-1,c['y']-3,'차량 자세 고정',fontsize=9)
 ax.set_title(title,fontsize=12);ax.set_aspect('equal');ax.set_xlim(105,149);ax.set_ylim(-24,21);ax.set_xlabel('지도 x (m)');ax.set_ylabel('지도 y (m)');ax.legend(fontsize=9,loc='upper left');ax.grid(alpha=.15)
ax=fig.add_subplot(gs[1,:]);t=np.array([c['t'] for c in inp['cycles']]);base=1791230808.;mask=(t>=base)&(t<=base+6)
for key,color,label in [('0_0.000000','#2477bc','기존'),('0_12.000000','#dd7324','12 m 상한')]:
 ax.plot((t-base)[mask],[r['B'] for r,m in zip(rs[key],mask) if m],'o-',color=color,label=label)
ax.axvline(c['t']-base,color='#888',ls='--');ax.set_xticks(range(7),[f'05:06:{48+j:02d}' for j in range(7)]);ax.set_ylabel('再計算 B (m)'.replace('再計算','재계산된'));ax.set_title('계획이 달라지면 다음 단면과 B도 달라짐 — 이전 계획을 전달한 재생',fontsize=12);ax.grid(alpha=.25);ax.legend(loc='upper right')
fig.suptitle('12 m soft 폭 상한 시험 · 05:06:51.906의 상위 계획 비교\n주행 당시 설정 사용 / 기록된 차량 궤적·BEV 고정 / 실주행 결과가 아님',fontsize=14,y=.98)
fig.savefig(here/'cap12_regression.png',dpi=160,bbox_inches='tight');plt.close(fig)
