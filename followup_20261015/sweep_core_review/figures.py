from pathlib import Path
import os,json,pickle,csv
import numpy as np
os.environ['MPLCONFIGDIR']='/tmp/codex_sweep_review_mpl'
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt,font_manager
from matplotlib.backends.backend_pdf import PdfPages
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False,'font.size':10})
P=Path(__file__).resolve().parent;R=json.loads((P/'metrics.json').read_text());C=pickle.loads((P/'curves.pkl').read_bytes());S=json.loads((P/'summary.json').read_text())
labels=['기준','플래너 2Hz','플래너 10Hz','제어 20Hz\n예측 3초','헤딩 비용\n0.3','헤딩 비용\n1.2','헤딩 보정 비용\n6','상위 조향 한계\n0.25rad','조향 변화 비용\n300']
colors=['#64748b','#6b8e23','#2563eb','#ea580c','#059669','#9333ea','#dc2626','#a16207','#db2777']
route=np.array(pickle.loads((P/'decoded_01_baseline_scenario1.pkl').read_bytes())['route'])
figs=[]
fig,axes=plt.subplots(1,3,figsize=(15,6.3));xx=np.arange(9)
for ax,field,title,unit in [(axes[0],'variation_deg_per_100m','조향 보정량 (실제 출력 명령)','도 / 100m'),(axes[1],'yaw_accel_rms_degps2','실제 차량 헤딩의 변화 거칠기','도 / 초²')]:
 for j,r in enumerate(R):
  yy=[a['steering'][field] if field in a['steering'] else a[field] for a in r['attempts']];med=np.median(yy)
  ax.bar(j,med,color=colors[j],alpha=.7)
  for k,a in enumerate(r['attempts']):ax.scatter(j+(k-1)*.12,yy[k],marker='x' if a['outcome']!='completed' else 'o',color='#111827' if a['outcome']=='completed' else '#b91c1c',s=24,zorder=4)
  ax.text(j,med+max(yy)*.045,f'{med:.1f}',ha='center',fontsize=9)
 baseline=np.median([a['steering'][field] if field in a['steering'] else a[field] for a in R[0]['attempts']]);ax.axhline(baseline,ls='--',color='#475569',lw=1)
 ax.set_title(title+'\n낮을수록 부드러움');ax.set_ylabel(unit);ax.set_ylim(0,ax.get_ylim()[1]*1.12)
ax=axes[2]
for j,r in enumerate(R):
 val=r['spatial']['rms_spread_m'];ax.bar(j,val,color=colors[j],alpha=.7,hatch='//' if len(r['spatial']['completed_attempts'])<3 else None);ax.text(j,val+.015,f'{val:.3f}',ha='center',fontsize=9)
ax.set_title('같은 진행 위치에서 회차 간 궤적 편차\n낮을수록 반복 궤적이 일정함');ax.set_ylabel('m RMS');ax.set_ylim(0,.60)
for ax in axes:ax.set_xticks(xx,labels,rotation=65,ha='right');ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
fig.suptitle('시나리오 1 · 9세트 × 3회 = 27회 · 목표 접근 판정 26회 / 경로 이탈 1회',fontsize=15)
fig.text(.5,.025,'조향/헤딩 지표: 3회 중앙값, 점은 개별 회차, 빨간 ×는 이탈 회차. 궤적 편차: 공통 5~179.5m, 세트 7은 완주한 2회만(사선).',ha='center',fontsize=10)
fig.subplots_adjust(top=.82,bottom=.30,left=.06,right=.98,wspace=.32);fig.savefig(P/'overview.png',dpi=180);figs.append(fig)
fig,axes=plt.subplots(3,3,figsize=(13,12),sharex=True,sharey=True)
for j,(r,ax) in enumerate(zip(R,axes.flat)):
 ax.plot(route[:,0],route[:,1],'k--',alpha=.35,label='웨이포인트 연결선')
 for k,c in enumerate(C[r['id']]):
  od=c['odom'];ax.plot(od[:,1],od[:,2],color='#dc2626' if c['outcome']!='completed' else plt.get_cmap('viridis')(.2+k*.3),lw=1.6,ls='-' if c['outcome']=='completed' else '--',label=f'{k+1}회'+(' 이탈' if c['outcome']!='completed' else ''))
 ax.set_title(f'{j+1}. '+labels[j].replace('\n',' ') +f" ({S[j]['successes']}/3)");ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(fontsize=8,loc='lower right')
 ax.set_xlim(101,171);ax.set_ylim(-64,18)
 for x,y in route[2:]:ax.scatter(x,y,s=12,color='#333333')
for ax in axes[-1]:ax.set_xlabel('지도 x (m)')
for ax in axes[:,0]:ax.set_ylabel('지도 y (m)')
fig.suptitle('회전과 출구 구간 실제 궤적 · 27회 모두 표시\n점선 기준은 성긴 웨이포인트 연결선이며 도로 중심선 정답이 아님',fontsize=14);fig.tight_layout(rect=[0,0,1,.94]);fig.savefig(P/'trajectories.png',dpi=170);figs.append(fig)
fig,axes=plt.subplots(3,1,figsize=(13,10),sharex=True)
for j in [0,2,3,4,8]:
 r=R[j];z=np.load(P/f"spatial_{r['id']}.npz");sg=z['s'];steer=z['steering'];axes[0].plot(sg,np.mean(steer,axis=0),label=labels[j].replace('\n',' '),color=colors[j]);axes[0].fill_between(sg,steer.min(axis=0),steer.max(axis=0),alpha=.08,color=colors[j]);axes[1].plot(sg,z['spread'],color=colors[j],label=labels[j].replace('\n',' '))
for j,r in enumerate(R):
 aa=[a for a in r['attempts'] if a['outcome']=='completed'];vals=[np.median([a['segments'][seg]['variation_deg_per_100m'] for a in aa]) for seg in ['approach','turn','exit']]
 axes[2].plot([50,122.5,165],vals,marker='o',color=colors[j],label=labels[j].replace('\n',' '))
axes[0].set_title('진행 위치를 맞춘 출력 조향 · 선: 평균, 음영: 회차 범위');axes[0].set_ylabel('조향 명령 (도)');axes[0].legend(ncol=3,fontsize=9)
axes[1].set_title('회차 간 궤적 편차');axes[1].set_ylabel('m');axes[2].set_title('구간별 조향 보정량 · 완주 회차 중앙값');axes[2].set_ylabel('도 / 100m');axes[2].set_xlabel('웨이포인트 연결선을 따라 측정한 진행 거리 (m)');axes[2].legend(ncol=3,fontsize=8)
for ax in axes:ax.axvspan(90,155,color='#cbd5e1',alpha=.20);ax.grid(alpha=.2)
fig.tight_layout();fig.savefig(P/'spatial_signals.png',dpi=180);figs.append(fig)
f=C['07_upper_heading_smooth_scenario1'][0];st=f['start'];l=f['lower'];od=f['odom'];events=f['events']
fig=plt.figure(figsize=(13,9));gs=fig.add_gridspec(3,2,width_ratios=[1.1,1]);ax=fig.add_subplot(gs[:,0]);ax.plot(route[:,0],route[:,1],'k--',alpha=.4,label='웨이포인트');
for c in C['07_upper_heading_smooth_scenario1'][1:]:ax.plot(c['odom'][:,1],c['odom'][:,2],color='#94a3b8',alpha=.7,lw=1,label='같은 세트 완주 궤적' if c['attempt']==2 else None)
ax.plot(od[:,1],od[:,2],color='#dc2626',lw=2,label='1회차 이탈 궤적')
for et,col in [(25.71,'#2563eb'),(26.11,'#d97706'),(27.71,'#7c3aed')]:
 e=min(events,key=lambda e:abs(e['t_start']-st-et));pp=np.array(e['published_world']);ax.plot(pp[:,0],pp[:,1],'.-',color=col,lw=2,label=f"{e['t_start']-st:.1f}초 새 계획")
ax.set_xlim(103,174);ax.set_ylim(-35,28);ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_xlabel('지도 x (m)');ax.set_ylabel('지도 y (m)');ax.legend(fontsize=9);ax.set_title('출구는 오른쪽으로 이어지지만\n계획과 실제 궤적이 위쪽으로 진행')
axes=[fig.add_subplot(gs[i,1]) for i in range(3)]
axes[0].plot(od[:,0]-st,np.degrees(od[:,3]),color='#dc2626');axes[0].set_ylabel('차량 헤딩 (도)');axes[0].set_title('이탈 당시 제어·계획 상태')
axes[1].plot(l[:,0]-st,np.degrees(-l[:,17]*l[:,31]),color='#2563eb');axes[1].set_ylabel('출력 조향 (도)')
axes[2].step(l[:,0]-st,l[:,12],color='#059669',where='post',label='하위 모드 (0 = 정상 계산)');axes[2].scatter([e['t_start']-st for e in events if not e['valid_plan']],[1 for e in events if not e['valid_plan']],marker='x',color='#dc2626',label='상위 해 불가능');axes[2].set_yticks([0,1,3,4]);axes[2].set_ylim(-.4,4.5);axes[2].legend(fontsize=8);axes[2].set_xlabel('회차 시작 후 시간 (초)')
for a in axes:a.set_xlim(22,33.2);a.axvspan(26.0,28.3,color='#fef3c7',alpha=.5);a.grid(alpha=.2)
fig.suptitle('세트 7 · 상위 헤딩 보정 비용 6 · 1회차 실패 분석\nBEV 최대 수신 간격 0.161초 · 하위 제어 최대 간격 0.104초 · 회전 중 정상 제어 지속',fontsize=14);fig.tight_layout(rect=[0,0,1,.92]);fig.savefig(P/'failure_detail.png',dpi=180);figs.append(fig)
with PdfPages(P/'sweep_comparison.pdf') as pdf:
 for fig in figs:pdf.savefig(fig,bbox_inches='tight');plt.close(fig)
print('saved overview, trajectories, spatial signals, failure detail, PDF')
