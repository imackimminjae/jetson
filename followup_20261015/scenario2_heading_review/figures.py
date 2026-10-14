from pathlib import Path
import os,json,pickle
import numpy as np
os.environ['MPLCONFIGDIR']='/tmp/codex_scenario2_heading_mpl'
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt,font_manager
from matplotlib.backends.backend_pdf import PdfPages
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False,'font.size':10})
P=Path(__file__).resolve().parent;R=json.loads((P/'metrics.json').read_text());C=pickle.loads((P/'curves.pkl').read_bytes());labels=['이전 P2Hz / 상위 0.6','이번 P5Hz / 상위 0.3'];col=['#2563eb','#ea580c'];runcol=['#0072B2','#D55E00','#009E73','#CC79A7','#E69F00'];figs=[]
route=np.asarray(pickle.loads((P/f"decoded_{R[0]['id']}.pkl").read_bytes())['route'])
fig,axes=plt.subplots(1,4,figsize=(14,5.2))
for ax,key,title,source in zip(axes,['variation_deg_per_100m','rate_p95_degps','yaw_accel_rms_degps2','rms_spread_m'],['조향 보정량\n도 / 100m','조향 변화속도 P95\n도 / 초','차량 헤딩 가속도 RMS\n도 / 초²','완주 4회 궤적 편차 RMS\nm'],['steering','steering','attempt','spatial']):
 for j,r in enumerate(R):
  yy=[a['steering'][key] if source=='steering' else a[key] for a in r['attempts']] if source!='spatial' else None
  v=float(np.median(yy)) if yy is not None else r['spatial'][key];ax.bar(j,v,color=col[j],width=.6,alpha=.8)
  if yy is not None:
   for k,a in enumerate(r['attempts']):ax.scatter(j+(k-2)*.065,yy[k],s=25,marker='x' if a['outcome']!='completed' else 'o',color='#b91c1c' if a['outcome']!='completed' else '#1f2937',zorder=3)
  ax.text(j,v*.4,f'{v:.3f}' if source=='spatial' else f'{v:.1f}',ha='center',color='white',weight='bold')
 ax.set_title(title);ax.set_xticks([0,1],['2Hz / 0.6','5Hz / 0.3']);ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
fig.suptitle('시나리오 2 · 목표 접근 이전 4/5, 이번 4/5 · 제어10Hz / 하위 헤딩0.5',fontsize=14)
fig.text(.5,.025,'조향/차량 지표: 5회 중앙값, 빨간 ×: 이탈 회차. 전체 길이 궤적 편차: 완주 4회만. 주파수와 비용이 함께 바뀐 비교.',ha='center',fontsize=9);fig.tight_layout(rect=[0,.08,1,.91]);fig.savefig(P/'overview.png',dpi=175);figs.append(fig)
fig,axes=plt.subplots(3,2,figsize=(13,11));views=[((-259,-68),(27,108)),((-228,-184),(65,94)),((-157,-95),(27,70))]
for j,r in enumerate(R):
 for ax in axes[:,j]:ax.plot(route[:,0],route[:,1],'k--o',lw=1,ms=3,alpha=.4)
 for k,c in enumerate(C[r['id']]):
  od=c['odom'];bad=c['outcome']!='completed'
  for ax in axes[:,j]:ax.plot(od[:,1],od[:,2],lw=2 if bad else 1.3,color='#dc2626' if bad else runcol[k],ls='--' if bad else '-',label=f'{k+1}회'+(' 이탈' if bad else ''))
 for i,(ax,vw) in enumerate(zip(axes[:,j],views)):
  ax.set_xlim(*vw[0]);ax.set_ylim(*vw[1]);ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_xlabel('지도 x (m)');ax.set_ylabel('지도 y (m)');ax.set_title(labels[j] if i==0 else ['첫 회전','두 번째 갈림길'][i-1])
 axes[0,j].legend(fontsize=8,ncol=2)
fig.suptitle('각 5회 실제 궤적 · 두 그룹 모두 갈림길에서 1회 이탈\n점선 기준은 성긴 웨이포인트 연결선이며 도로 중심선 정답이 아님',fontsize=14);fig.tight_layout(rect=[0,0,1,.93]);fig.savefig(P/'trajectories.png',dpi=165);figs.append(fig)
fig,axes=plt.subplots(3,2,figsize=(13,10),sharex=True,sharey='row')
for j,r in enumerate(R):
 z=np.load(P/f"spatial_{r['id']}.npz");s=z['s']
 for k,i in enumerate(r['spatial']['completed_attempts']):axes[0,j].plot(s,z['steering'][k],lw=1,color=runcol[i-1],label=f'{i}회');axes[1,j].plot(s,z['lateral'][k],lw=1,color=runcol[i-1])
 axes[2,j].plot(s,z['spread'],color=col[j]);axes[0,j].set_title(labels[j]+' · 완주 4회');axes[0,j].legend(fontsize=8,ncol=4);axes[2,j].set_xlabel('웨이포인트 진행 거리 (m)')
 for ax in axes[:,j]:ax.axvspan(30,75,color='#cbd5e1',alpha=.25);ax.axvspan(115,150,color='#cbd5e1',alpha=.25);ax.grid(alpha=.2)
for ax,label in zip(axes[:,0],['출력 조향 (도)','계획 경로 대비 횡오차 (m)','회차 간 궤적 편차 (m)']):ax.set_ylabel(label)
fig.suptitle('완주 회차의 같은 진행 위치 비교 · 실패는 앞선 그림과 전체 지표에 보존',fontsize=14);fig.tight_layout(rect=[0,0,1,.94]);fig.savefig(P/'spatial_signals.png',dpi=165);figs.append(fig)
f=C[R[1]['id']][2];st=f['start'];l=f['lower'];od=f['odom'];ev=f['events']
fig=plt.figure(figsize=(13,9));gs=fig.add_gridspec(3,2,width_ratios=[1.1,1]);ax=fig.add_subplot(gs[:,0]);ax.plot(route[:,0],route[:,1],'k--o',alpha=.4,label='웨이포인트')
for c in C[R[1]['id']]:
 if c['outcome']=='completed':ax.plot(c['odom'][:,1],c['odom'][:,2],color='#94a3b8',lw=1,alpha=.6)
ax.plot(od[:,1],od[:,2],color='#dc2626',lw=2,label='3회차 이탈')
for et,color in [(22.06,'#2563eb'),(23.06,'#d97706'),(24.06,'#7c3aed')]:
 e=min(ev,key=lambda e:abs(e['t_start']-st-et));pp=np.array(e['published_world']);ax.plot(pp[:,0],pp[:,1],'.-',color=color,lw=2,label=f"{e['t_start']-st:.1f}초 계획")
ax.set_xlim(-155,-82);ax.set_ylim(28,72);ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_xlabel('지도 x (m)');ax.set_ylabel('지도 y (m)');ax.legend(fontsize=9);ax.set_title('목표 출구는 오른쪽 위\n계획과 실제 궤적은 오른쪽 아래로 진행')
axes=[fig.add_subplot(gs[i,1]) for i in range(3)]
axes[0].plot(od[:,0]-st,np.degrees(od[:,3]),color='#dc2626');axes[0].axhline(np.degrees(np.arctan2(11,57)),color='#64748b',ls='--',label='출구 웨이포인트 방향');axes[0].set_ylabel('차량 헤딩 (도)');axes[0].legend(fontsize=8)
axes[1].plot(l[:,0]-st,np.degrees(-l[:,17]*l[:,31]),color='#2563eb');axes[1].set_ylabel('출력 조향 (도)')
axes[2].step(l[:,0]-st,l[:,12],where='post',color='#059669',label='하위 모드 (0=정상 QP)');axes[2].scatter([e['t_start']-st for e in ev if not e['valid_plan']],[1 for e in ev if not e['valid_plan']],color='#dc2626',marker='x',label='상위 QP 불가능');axes[2].set_ylim(-.4,4.5);axes[2].set_yticks([0,1,3,4]);axes[2].legend(fontsize=8);axes[2].set_xlabel('running 시작 후 시간 (초)')
for ax in axes:ax.set_xlim(21,32.6);ax.axvspan(23,25,color='#fef3c7',alpha=.5);ax.grid(alpha=.2)
fig.suptitle('이번 3회차 이탈 · 주행 중 하위 QP 제어 지속\nBEV 최대 간격0.231초 · 하위 제어 최대 간격0.104초 · 상위 계획 실패1회',fontsize=14);fig.tight_layout(rect=[0,0,1,.93]);fig.savefig(P/'failure_detail.png',dpi=175);figs.append(fig)
with PdfPages(P/'scenario2_analysis.pdf') as pdf:
 for fig in figs:pdf.savefig(fig,bbox_inches='tight');plt.close(fig)
print('4 figures and PDF saved')
