from pathlib import Path
import os,json,gzip,math
os.sched_setaffinity(0,{min(os.sched_getaffinity(0))});os.environ['MPLCONFIGDIR']='/tmp/all_attempts_plot'
import numpy as np
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib import font_manager
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf');plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False,'font.size':10})
ROOT=Path('/home/imac/ros2_ws/followup_20261008/batches_182344_183952_185426_review')
labels={'completed':'목표 접근','off_route':'경로 이탈','pose_timeout':'위치 입력 중단','stopped_10s':'10초 이동 정체'}
def segmented(ax,t,y,gap=.5,**kw):
 if not len(t):return
 for j,ii in enumerate(np.split(np.arange(len(t)),np.flatnonzero(np.diff(t)>gap)+1)):
  k=dict(kw)
  if j:k.pop('label',None)
  ax.plot(t[ii],np.asarray(y)[ii],**k)
def xyplot(ax,t,xy,mask,col,label):
 ids=np.flatnonzero(mask)
 if not len(ids):return
 for j,ii in enumerate(np.split(ids,np.flatnonzero((np.diff(t[ids])>.5)|(np.diff(ids)>1))+1)):
  ax.plot(*xy[ii].T,color=col,lw=1.6,label=label if j==0 else None)
def mapplot(ax,z,tr,route,limits,small=False):
 xy=np.array(tr['xy']);t=np.array(tr['t']);neutral=tr['neutral_t'];cut=neutral if neutral is not None else z['end']
 ax.plot(*route.T,'--',color='#555555',lw=1,label='지정 경로');ax.scatter(*route[-1],marker='*',s=75,c='#333333',label='목표')
 xyplot(ax,t,xy,t<cut,'#147d92','중립 전 관측 궤적');xyplot(ax,t,xy,t>=cut,'#d89020','중립 이후 관측 궤적')
 if len(xy):ax.scatter(*xy[0],marker='o',s=22,c='#147d92',label='출발')
 ax.scatter(float(z['stop_x']),float(z['stop_y']),marker='X',s=45,c='#b33340',label='최종 정지점')
 # Mark last known pose at outcome, not an interpolated position through outages.
 ii=np.flatnonzero(t<=z['end'])
 if len(ii):ax.scatter(*xy[ii[-1]],marker='s',s=30,facecolors='none',edgecolors='#a63bb0',label='판정 전 마지막 위치')
 for j in np.flatnonzero(np.diff(t)>.5):ax.scatter(*xy[j],marker='|',s=50,c='#e34b37')
 ax.set_xlim(*limits[0]);ax.set_ylim(*limits[1]);ax.set_aspect('equal');ax.grid(alpha=.18);ax.set_xlabel('X (m)');ax.set_ylabel('Y (m)')
 ax.set_title(f"{z['attempt']}회 · {labels[z['outcome']]}\n상위 실패 {z['upper_failed']}/{z['upper_count']} · 정지 목표거리 {float(z['stop_goal_distance_m']):.1f}m",fontsize=10 if small else 12)
for sc in [1,2,3]:
 H=ROOT/f'scenario{sc}';V=H/'all_attempts';V.mkdir(exist_ok=True);s=json.loads((H/'summary.json').read_text());tr=json.loads(gzip.decompress((H/'tracks.json.gz').read_bytes()));d=json.loads(gzip.decompress((H/'telemetry.json.gz').read_bytes()))['data'];es=json.loads(gzip.decompress((H/'events.json.gz').read_bytes()));route=np.array(s['route']);lo=np.array([v for _,v in d['/debug/lower_mpc_trace']]);bs=np.array([v for _,v in d['/debug/upper_interval_centering_state']]);ot=np.array([t for t,_ in d['/motive/vehicle/odom_map']]);ov=np.array([math.hypot(v[3],v[4]) for _,v in d['/motive/vehicle/odom_map']]);points=np.vstack([route,*[np.array(x['xy']) for x in tr]]);mn=points.min(axis=0)-8;mx=points.max(axis=0)+8;limits=[(mn[0],mx[0]),(mn[1],mx[1])]
 # All ten requested slots, including unexecuted slots for scenario 2.
 nr,nc=(2,5) if sc==1 else (5,2);fig,aa=plt.subplots(nr,nc,figsize=(19,11) if sc==1 else (15,18),layout='constrained')
 for i,ax in enumerate(aa.flat):
  if i<len(tr):mapplot(ax,s['attempts'][i],tr[i],route,limits,True)
  else:ax.set_axis_off();ax.text(.5,.5,f'{i+1}회 · 미실행\n7회 후 전체 배치 중단',ha='center',va='center',transform=ax.transAxes,color='#666666',fontsize=13)
 fig.get_layout_engine().set(rect=(0,.065,1,.935));handles,names=aa.flat[0].get_legend_handles_labels();fig.legend(handles,names,loc='lower center',ncol=6,fontsize=10);fig.suptitle(f'시나리오{sc} 전 회차 궤적 — 같은 좌표 범위로 비교\n위치 공백은 연결하지 않음 · completed는 목표 접근이며 목표 정차 성공이 아님',fontsize=15);fig.savefig(V/'all_tracks.png',dpi=145,bbox_inches='tight');plt.close(fig)
 with PdfPages(V/f'scenario{sc}_all_attempts.pdf') as pdf:
  for z,track in zip(s['attempts'],tr):
   start,end,stop=z['start'],z['end'],z['stop_time'];fig=plt.figure(figsize=(16,11),layout='constrained');gs=fig.add_gridspec(5,2,width_ratios=[1,1.5]);ax=fig.add_subplot(gs[:,0]);mapplot(ax,z,track,route,limits);ax.legend(loc='best',fontsize=8)
   axs=[fig.add_subplot(gs[j,1]) for j in range(5)];lm=lo[(lo[:,0]>=start)&(lo[:,0]<stop)];tt=lm[:,0]-start;om=(ot>=start)&(ot<stop);ev=[e for e in es if start<=e['t_start']<stop];bt=bs[(bs[:,0]>=start)&(bs[:,0]<stop)]
   segmented(axs[0],ot[om]-start,ov[om],color='#147d92',label='受信 위치 메시지의 속도'.replace('受信','수신'))
   axs[0].set_ylabel('속도 (m/s)');axs[0].legend(fontsize=8,loc='upper right')
   segmented(axs[1],tt,np.degrees(lm[:,26]),gap=.3,color='#276bb0',label='출력 조향 명령');axs[1].axhline(np.degrees(.3),c='#888888',ls=':',lw=.8);axs[1].axhline(-np.degrees(.3),c='#888888',ls=':',lw=.8);axs[1].set_ylabel('조향 명령 (도)');axs[1].set_ylim(-19,19);axs[1].legend(fontsize=8,loc='upper right')
   # Zero-filled invalid error is not measured perfect tracking: mask non-mode0.
   err=lm[:,6].copy();err[lm[:,12]!=0]=np.nan;segmented(axs[2],tt,err,gap=.3,color='#8e4d9a',label='mode0 지역참조 횡오차');axs[2].set_ylabel('횡오차 (m)');axs[2].legend(fontsize=8,loc='upper right')
   if len(bt):segmented(axs[3],bt[:,0]-start,bt[:,2],gap=.8,color='#147d92',label='B (m)')
   axs[3].set_ylabel('B (m)');axs[3].legend(fontsize=8,loc='upper right')
   for valid,col,label in [(True,'#148676','유효 계획'),(False,'#c44632','계획 실패')]:
    ee=[e for e in ev if e['valid_plan']==valid];axs[4].scatter([e['t_start']-start for e in ee],[e['horizon'] for e in ee],s=13,color=col,label=label)
   axs[4].set_ylabel('상위 지평 N');axs[4].set_yticks(range(7));axs[4].set_ylim(-.4,6.6);axs[4].legend(fontsize=8,loc='upper right');axs[4].set_xlabel('running 시작 이후 (초)')
   for a in axs:
    a.axvline(end-start,color='#a63bb0',ls='--',lw=1)
    if track['neutral_t'] is not None:a.axvline(track['neutral_t']-start,color='#d89020',ls=':',lw=1.3)
    for j in np.flatnonzero(np.diff(ot)>.5):
     left,right=max(start,ot[j]),min(stop,ot[j+1])
     if right>left:a.axvspan(left-start,right-start,color='#f1bc59',alpha=.23)
    a.set_xlim(0,stop-start);a.grid(alpha=.18)
   fig.suptitle(f"시나리오{sc} · {z['attempt']:02}회 · {z['outcome']} | {z['start_clock']} 시작\n출력 포화 {z['sat_percent']:.2f}% · 지역참조 |횡오차| p95 {z['lateral_abs']['p95']:.3f}m",fontsize=15)
   fig.supxlabel('보라 점선: 회차 판정 / 주황 점선: 최종 중립 전환 / 노란 영역: 위치 수신 공백\n조향은 명령값이며 실제 바퀴각 아님 · 입력 공백 속도는 보간하지 않음 · 판정 후 계획 계산도 별도 표시',fontsize=10)
   fig.savefig(V/f'attempt_{z["attempt"]:02}.png',dpi=135);pdf.savefig(fig);plt.close(fig);print('saved',sc,z['attempt'],flush=True)
 index={'scenario':sc,'executed_attempts':len(tr),'requested_attempts':10,'not_executed':list(range(len(tr)+1,11)),'pdf':str(V/f'scenario{sc}_all_attempts.pdf'),'overview':str(V/'all_tracks.png'),'images':[str(V/f'attempt_{z["attempt"]:02}.png') for z in s['attempts']]};(V/'index.json').write_text(json.dumps(index,indent=2));(V/'README.txt').write_text(f'시나리오{sc}: 실행한 전 {len(tr)}회 개별 그림 및 PDF. all_tracks.png는 요청10회 슬롯을 모두 표시.\n각 개별 그림: 궤적·정지점·속도·출력조향명령·지역참조횡오차·B·상위계획지평과실패.\n위치 수신 공백>0.5초는 궤적 연결/속도보간 없음. 기록 위치 상태는 실제 바퀴각이 아님.\n회차 판정 뒤 정지 확인까지 포함하며 주행/판정 이후 구간은 보라선으로 구분.\n원본은 상위폴더의 압축 분석 자료 재사용. 운영 변경/재주행 없음.\n')
 with (H/'RESULT.txt').open('a') as f:f.write(f'\n전 회차 필수 시각화 추가: all_attempts/all_tracks.png, scenario{sc}_all_attempts.pdf, attempt_01~{len(tr):02}.png.\n')
