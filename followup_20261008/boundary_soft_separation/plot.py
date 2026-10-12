from pathlib import Path
import json,gzip,os
import numpy as np
os.environ['MPLCONFIGDIR']='/tmp/boundary_soft_plot'
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf');plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False})
H=Path(__file__).resolve().parent;read=lambda p:json.loads(gzip.decompress(p.read_bytes())) if p.suffix=='.gz' else json.loads(p.read_text())
r=read(H/'result_batch05.json.gz');q=read(H/'refine_batch05.json.gz');inp=read(H/'input_batch05.json');route=np.array(inp['route'])
fig,axs=plt.subplots(1,3,figsize=(17,5.7),layout='constrained')
for ax,j,title in [(axs[0],46,'5회차 06:30:31.369\n동일 참조·B에서 잘린 쪽 축소만 변경'),(axs[1],49,'5회차 06:30:32.869\n이전 계획을 이어받은 뒤 후보가 달라짐')]:
 c=inp['cycles'][j];ax.plot(*route.T,'k--',lw=1,label='지정 경로')
 for z,col,label in [(r['base_carried'][j],'#c24a39','기존 계산'),(q['gated_skip_carried'][j],'#138979','조건부 후보')]:
  plan=np.array(z['planned']);ref=np.array(z['reference']);ax.plot(*plan.T,'-o',color=col,ms=3,label=label);ax.plot(*ref.T,':',color=col,alpha=.7)
  for a in z['cands'][-1]:
   a=np.array(a).reshape(2,2);ax.plot(*a.T,color=col,lw=4,alpha=.4)
  if j==46:
   R=np.array([[np.cos(c['yaw']),-np.sin(c['yaw'])],[np.sin(c['yaw']),np.cos(c['yaw'])]]);d=next(x for x in z['raw_detail'] if x['k']==6);ab=np.array(d['raw_body']).reshape(2,2)@R.T+[c['x'],c['y']];ax.plot(*ab.T,'k:',lw=2)
 ax.scatter(c['x'],c['y'],marker='^',color='black');ax.set_xlim(231,272);ax.set_ylim(128,159);ax.set_aspect('equal');ax.set_xlabel('세계 X (m)');ax.set_ylabel('세계 Y (m)');ax.set_title(title);ax.legend(fontsize=9,loc='upper left');ax.grid(alpha=.2)
b=read(H/'B_jump_causal.json');ax=axs[2];x=np.arange(1,11);ax.bar(x-.18,[abs(z['same_old_plan_shift_updated_B']) for z in b],.36,label='기록의 갱신 후 B',color='#c24a39');ax.bar(x+.18,[abs(z['same_old_plan_shift_previous_B']) for z in b],.36,label='갱신 전 B 대입 진단',color='#3777a4');ax.set_xticks(x);ax.set_xlabel('회차');ax.set_ylabel('직전 기록 계획 대비 k1 횡이동 절대값 (m)');ax.set_title('동일 참조에서 B만 바꾼 즉시 효과\n단순 지연은 다음 주기에 급변을 옮길 수 있음');ax.legend(fontsize=9);ax.grid(axis='y',alpha=.2)
fig.suptitle('기록 차량·지도 고정 재계산 — 실제 폐루프 주행/조향 개선을 보장하지 않음',fontsize=13);fig.savefig(H/'comparison.png',dpi=150);plt.close(fig)
