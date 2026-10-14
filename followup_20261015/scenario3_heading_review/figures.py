from pathlib import Path
import json,pickle,os
import numpy as np
os.environ['MPLCONFIGDIR']='/tmp/codex_s3_heading_mpl'
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt,font_manager
from matplotlib.backends.backend_pdf import PdfPages
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False,'font.size':10})
P=Path(__file__).resolve().parent;RR=json.loads((P/'metrics.json').read_text());R=[RR[0],RR[2]];C=pickle.loads((P/'curves.pkl').read_bytes());colors=['#2563eb','#ea580c'];runcolors=['#0072B2','#D55E00','#009E73','#CC79A7','#E69F00'];labels=['기존 하위 헤딩 0.5','이번 하위 헤딩 0.3'];figs=[]
fig,axes=plt.subplots(1,4,figsize=(14,4.8))
for ax,field,title,source in zip(axes,['variation_deg_per_100m','rate_p95_degps','yaw_accel_rms_degps2','rms_spread_m'],['조향 보정량\n도 / 100m','조향 변화속도 P95\n도 / 초','차량 헤딩 가속도 RMS\n도 / 초²','회차 간 궤적 편차 RMS\nm'],['steering','steering','attempt','spatial']):
 vals=[]
 for j,r in enumerate(R):
  yy=[a['steering'][field] if source=='steering' else a[field] for a in r['attempts']] if source!='spatial' else None
  v=float(np.median(yy)) if yy is not None else r['spatial'][field];vals.append(v);ax.bar(j,v,color=colors[j],width=.6,alpha=.8)
  if yy is not None:ax.scatter(j+np.linspace(-.13,.13,len(yy)),yy,color='#1f2937',s=23,zorder=3)
  ax.text(j,v*.4,f'{v:.3f}' if source=='spatial' else f'{v:.1f}',ha='center',color='white',weight='bold')
 ax.set_title(title);ax.set_xticks([0,1],['하위 0.5','하위 0.3']);ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
fig.suptitle('시나리오 3 · 각 5회 · 상위 헤딩 비용은 둘 다 0.6 / 플래너 5Hz / 제어 10Hz',fontsize=13)
fig.text(.5,.02,'조향/차량 지표는 회차 중앙값(점: 개별 회차), 궤적 편차는 공통 진행 위치 5~154.5m에서 계산. 낮을수록 좋음.',ha='center',fontsize=9);fig.tight_layout(rect=[0,.07,1,.91]);fig.savefig(P/'overview.png',dpi=170);figs.append(fig)
route=np.asarray(pickle.loads((P/f"decoded_{R[0]['id']}.pkl").read_bytes())['route'])
fig,axes=plt.subplots(3,2,figsize=(12,12))
views=[((180,285),(128,225)),((185,225),(150,190)),((235,285),(130,178))]
for j,r in enumerate(R):
 for ax in axes[:,j]:ax.plot(route[:,0],route[:,1],'k--o',lw=1,ms=3,alpha=.5)
 for k,c in enumerate(C[r['id']]):
  od=c['odom']
  for ax in axes[:,j]:ax.plot(od[:,1],od[:,2],color=runcolors[k],lw=1.5,label=f'{k+1}회')
 for i,(ax,view) in enumerate(zip(axes[:,j],views)):
  ax.set_xlim(*view[0]);ax.set_ylim(*view[1]);ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_xlabel('지도 x (m)');ax.set_ylabel('지도 y (m)');ax.set_title(labels[j] if i==0 else ['첫 회전','두 번째 회전 / 출구'][i-1])
 axes[0,j].legend(fontsize=8)
fig.suptitle('실제 궤적 · 목표 접근 각 5/5\n점선은 웨이포인트 연결선이며 도로 중심선 정답이 아님',fontsize=14);fig.tight_layout(rect=[0,0,1,.93]);fig.savefig(P/'trajectories.png',dpi=160);figs.append(fig)
fig,axes=plt.subplots(3,2,figsize=(13,10),sharex=True,sharey='row')
for j,r in enumerate(R):
 z=np.load(P/f"spatial_{r['id']}.npz");s=z['s']
 for k in range(5):
  axes[0,j].plot(s,z['steering'][k],color=runcolors[k],lw=1,alpha=.8);axes[1,j].plot(s,z['heading'][k],color=runcolors[k],lw=1,alpha=.8)
 axes[2,j].plot(s,z['spread'],color=colors[j]);axes[0,j].set_title(labels[j]);axes[2,j].set_xlabel('웨이포인트 진행 거리 (m)')
 for ax in axes[:,j]:ax.axvspan(35,80,color='#cbd5e1',alpha=.25);ax.axvspan(105,145,color='#cbd5e1',alpha=.25);ax.grid(alpha=.2)
for ax,l in zip(axes[:,0],['출력 조향 (도)','계획 경로 대비 헤딩 오차 (도)','회차 간 궤적 편차 (m)']):ax.set_ylabel(l)
fig.suptitle('같은 진행 위치에서 비교 · 회색: 첫 회전 / 두 번째 회전',fontsize=14);fig.tight_layout(rect=[0,0,1,.94]);fig.savefig(P/'spatial_signals.png',dpi=170);figs.append(fig)
with PdfPages(P/'scenario3_heading_comparison.pdf') as pdf:
 for f in figs:pdf.savefig(f,bbox_inches='tight');plt.close(f)
print('3 figures and PDF saved')
