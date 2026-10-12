from pathlib import Path
import json, gzip, csv, os
os.environ['MPLCONFIGDIR']='/tmp/scenario1_sway_plot'
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False,'font.size':10})
ROOT=Path('/home/imac/ros2_ws')
SRC=ROOT/'followup_20261008/batches_182344_183952_185426_review/scenario1'
OUT=Path(__file__).resolve().parent
s=json.loads((SRC/'summary.json').read_text())
d=json.load(gzip.open(SRC/'telemetry.json.gz','rt'))['data']
events=json.load(gzip.open(SRC/'events.json.gz','rt'))
lo=np.array([v for _,v in d['/debug/lower_mpc_trace']])
bs=np.array([v for _,v in d['/debug/upper_interval_centering_state']])
info=d['/debug/upper_interval_info']; it=np.array([t for t,_ in info])
colors={1:'#c0392b',3:'#dd7f24',8:'#b54387',5:'#238443',6:'#1489ac',10:'#454fbc',2:'#aaa',4:'#999',7:'#888',9:'#555'}
def lateral(pt,path):
    v=np.diff(path,axis=0); ll=np.sum(v*v,axis=1)
    u=np.clip(np.sum((pt-path[:-1])*v,axis=1)/np.maximum(ll,1e-12),0,1)
    q=path[:-1]+u[:,None]*v; j=np.argmin(np.linalg.norm(pt-q,axis=1))
    return float((v[j,0]*(pt-q[j])[1]-v[j,1]*(pt-q[j])[0])/max(np.sqrt(ll[j]),1e-9))
rows=[]; details={}; sample_rows=[]
for a in s['attempts']:
    n=a['attempt']; l=lo[(lo[:,0]>=a['start'])&(lo[:,0]<a['end'])&(lo[:,12]==0)&(lo[:,4]>1)]
    ee=[e for e in events if a['start']<=e['t_start']<a['end']]
    prev=None; ev=[]
    for e in ee:
        b=bs[np.argmin(abs(bs[:,0]-e['t_start']))]
        iv=np.array(info[np.argmin(abs(it-e['t_emit']))][1]).reshape(-1,12)
        p=np.array(e['published_world']); ref=np.array(e['preview_world'])
        q=dict(t=e['t_start'],x=e['x'],y=e['y'],yaw_deg=float(np.degrees(e['yaw_rad'])),B=float(b[2]),B_before=float(b[1]),candidate_step=int(b[5]),valid=e['valid_plan'],reason=e['reason'],k1_shift=None,k1_vs_preview=None,intervals=iv.tolist())
        if e['valid_plan'] and len(p)>1:
            q['k1_shift']=lateral(p[1],prev) if prev is not None else None
            q['k1_vs_preview']=lateral(p[1],ref)
            prev=p
        ev.append(q)
    details[n]=dict(lower=l,events=ev)
    masks={'between':(l[:,2]>=-35)&(l[:,2]<-15),'turn':(l[:,2]>=-15)&(l[:,1]<125),'exit':(l[:,1]>=125)&(l[:,1]<=160)&(l[:,2]>-15)}
    for region,mask in masks.items():
        # Attempt 9 follows a different branch; no exit comparison against successful route.
        if n==9 and region!='between':continue
        q=l[mask]
        if not len(q):continue
        v=[e for e in ev if q[0,0]<=e['t']<=q[-1,0]]
        rows.append(dict(attempt=n,region=region,samples=len(q),speed_median=float(np.median(q[:,4])),sat_percent=float(np.mean(abs(q[:,26])>=.29845)*100),lateral_abs_p95=float(np.quantile(abs(q[:,6]),.95)),command_min_deg=float(np.degrees(q[:,26].min())),command_max_deg=float(np.degrees(q[:,26].max())),upper_failed=sum(not e['valid'] for e in v),upper_count=len(v),B_min=min(e['B'] for e in v),B_max=max(e['B'] for e in v)))
    candidates=[e for e in ev if -24<e['y']<-21]
    q=min(candidates,key=lambda e:abs(e['y']+22.5))
    k1=next(i for i in q['intervals'] if i[0]==1 and i[1]==0)
    sample_rows.append(dict(attempt=n,**{k:q[k] for k in ['t','x','y','yaw_deg','B','B_before','candidate_step','k1_shift','k1_vs_preview']},k1_raw=k1[2],k1_processed=k1[3],k1_soft=bool(k1[4]),k1_clipped=bool(k1[7])))
with (OUT/'section_metrics.csv').open('w') as f:
    w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
(OUT/'observations.json').write_text(json.dumps(dict(source=s['session'],section_definitions={'between':'-35<=Y<-15','turn':'Y>=-15 and X<125; attempt9 excluded','exit':'125<=X<=160 and Y>-15; attempt9 excluded'},matched_samples=sample_rows,events={n:x['events'] for n,x in details.items()}),indent=2))

# All attempts: zoomed trajectories and command against Y in the inter-branch area.
fig,axs=plt.subplots(2,5,figsize=(19,11),layout='constrained')
route=np.array(s['route'])
for n,ax in enumerate(axs.flat,1):
    l=details[n]['lower']; m=(l[:,2]>=-40)&(l[:,1]<165)
    ax.plot(*route.T,'--',color='#999',lw=1)
    ax.plot(l[m,1],l[m,2],color=colors[n],lw=2)
    ax.axhline(-15,c='#888',ls=':',lw=1);ax.axhline(-35,c='#888',ls=':',lw=1)
    ax.set(xlim=(104,164),ylim=(-40,10),xlabel='X (m)',ylabel='Y (m)',title=f'{n}회'+(' · 오분기 (별도)' if n==9 else ''))
    ax.set_aspect('equal');ax.grid(alpha=.2)
fig.suptitle('시나리오1 전 10회 확대 궤적 — 점선 Y=-35~-15: 두 분기 사이 비교 구간\n붉은 계열: 흔들림 큰 1·3·8회 / 청록·푸른 계열: 부드러운 5·6·10회 / 9회는 오분기',fontsize=15)
fig.savefig(OUT/'all10_zoom.png',dpi=145);plt.close(fig)

fig,aa=plt.subplots(3,2,figsize=(15,13),layout='constrained')
for n in [1,3,8,5,6,10]:
    l=details[n]['lower'];ev=details[n]['events'];m=(l[:,2]>=-30)&(l[:,2]<=-12)&(l[:,1]<120);q=l[m];label=f'{n}회';c=colors[n]
    aa[0,0].plot(q[:,2],np.degrees(q[:,3]),color=c,label=label)
    aa[0,1].plot(q[:,2],np.degrees(q[:,26]),color=c,label=label)
    e=[e for e in ev if -30<=e['y']<=-12 and e['x']<120]
    aa[1,0].step([v['y'] for v in e],[v['B'] for v in e],where='post',color=c,label=label)
    aa[1,1].plot([v['y'] for v in e],[v['k1_shift'] for v in e],'.-',color=c,label=label)
    m=(l[:,1]>=125)&(l[:,1]<=160)&(l[:,2]>-15);q=l[m]
    aa[2,0].plot(q[:,1],q[:,2],color=c,label=label)
    aa[2,1].plot(q[:,1],np.degrees(q[:,26]),color=c,label=label)
for ax,title,y in [(aa[0,0],'분기 직전 차량 방향 (0° 동쪽 / 90° 북쪽)','scalar yaw (도)'),(aa[0,1],'분기 직전 출력 조향 명령','명령 환산값 (도)'),(aa[1,0],'B가 낮아지는 위치 비교','B (m)'),(aa[1,1],'새 계획 k1의 직전 발행 계획 대비 횡이동','왼쪽 + / 오른쪽 - (m)'),(aa[2,0],'두 번째 분기 통과 후 궤적','Y (m)'),(aa[2,1],'통과 후 출력 조향 명령','명령 환산값 (도)')]:
    ax.set_title(title);ax.set_ylabel(y);ax.grid(alpha=.2)
for ax in aa[:2].flat:ax.set_xlabel('차량 Y (m), 위쪽으로 진행');ax.set_xlim(-30,-12)
for ax in aa[2]:ax.set_xlabel('차량 X (m), 동쪽으로 진행');ax.set_xlim(125,160)
for ax in [aa[0,1],aa[2,1]]:
    ax.axhline(np.degrees(.3),c='#888',ls=':');ax.axhline(-np.degrees(.3),c='#888',ls=':');ax.set_ylim(-19,19)
aa[1,1].axhline(0,c='#888',lw=.8);aa[0,0].legend(ncol=3)
fig.suptitle('흔들림 큰 1·3·8회와 부드러운 5·6·10회 — 같은 위치 구간 비교\nB·계획 변화와 차량 반응의 동시 관측이며, B 단독 인과 검증은 아님',fontsize=15)
fig.savefig(OUT/'comparison.png',dpi=145);plt.close(fig)

# Concrete near-position example, chosen using the same target Y for both runs.
fig,axs=plt.subplots(1,2,figsize=(13,6),layout='constrained')
for n,ax in zip([1,5],axs):
    q=next(r for r in sample_rows if r['attempt']==n);e=min([e for e in events if s['attempts'][n-1]['start']<=e['t_start']<s['attempts'][n-1]['end']],key=lambda e:abs(e['t_start']-q['t']))
    ref=np.array(e['preview_world']);p=np.array(e['published_world'])
    ax.plot(*ref.T,'--o',c='#777',label='이어받은 참조',ms=4);ax.plot(*p.T,'-o',c=colors[n],label='새 발행 계획',ms=4)
    for k,j,x0,y0,x1,y1,*rest in e['intervals_world']:
        if k<=3:ax.plot([x0,x1],[y0,y1],color='#5fac9e',alpha=.7,lw=3,label='축소 후 후보 구간' if k==1 and j==0 else None)
    ax.scatter(e['x'],e['y'],s=65,c='black',marker='^',label='차량 위치')
    ax.set(xlim=(102,138),ylim=(-26,2),xlabel='X (m)',ylabel='Y (m)',title=f"{n}회: B={q['B']:.1f}m / k1 폭 {q['k1_raw']:.1f}→{q['k1_processed']:.2f}m\nk1 횡이동 {q['k1_shift']:+.2f}m / soft {'ON' if q['k1_soft'] else 'OFF'}")
    ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(fontsize=9)
fig.suptitle('Y≈-22.5m에서 1회와 5회 기록 비교\n차량 위치·방향·참조·지도 단면이 서로 다른 관측 — 조건을 고정한 실험이 아님',fontsize=14)
fig.savefig(OUT/'plan_example.png',dpi=145);plt.close(fig)
print(json.dumps(sample_rows,indent=2))
