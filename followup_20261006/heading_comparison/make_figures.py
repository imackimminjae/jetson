"""Offline native-NED EKF/SIH heading comparison. No control or live-log access."""
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import json,pickle,hashlib,csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager,dates as mdates
from matplotlib.backends.backend_pdf import PdfPages

HERE=Path(__file__).resolve().parent;KST=ZoneInfo('Asia/Seoul')
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False,'font.size':10,
    'axes.spines.top':False,'axes.spines.right':False,'figure.facecolor':'#ffffff'})
manifest=json.loads((HERE/'inputs_manifest.json').read_text());sources={}
for name,m in manifest.items():
    p=HERE/m['snapshot'];assert hashlib.sha256(p.read_bytes()).hexdigest()==m['sha256']
    sources[name]=pickle.load(p.open('rb')) if p.suffix=='.pkl' else json.loads(p.read_text())
def wrap(a):return np.arctan2(np.sin(a),np.cos(a))
def date(t):return datetime.fromtimestamp(float(t),KST)
def clock(t):return date(t).strftime('%m/%d %H:%M:%S')
def stamp(m):return m.get('_t',m.get('t'))
def qyaw(q):
    q=np.asarray(q,float);q=q/np.linalg.norm(q,axis=-1,keepdims=True)
    w,x,y,z=q.T;return np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))
assert np.isclose(np.degrees(wrap(np.radians(179-(-179)))),-2)
assert np.isclose(qyaw(np.array([[1,0,0,0]]))[0],0)

def align(data):
    att=[m for m in data['ATTITUDE'] if tuple(m.get('_src',(1,1)))==(1,1)]
    truth=[m for m in data['HIL_STATE_QUATERNION'] if tuple(m.get('_src',(1,1)))==(1,1)]
    a=np.array([[stamp(m),m['yaw']] for m in att]);h=np.array([[stamp(m),*m['attitude_quaternion'],np.hypot(m['vx'],m['vy'])/100] for m in truth])
    a=a[np.all(np.isfinite(a),axis=1)];h=h[np.all(np.isfinite(h),axis=1)]
    norms=np.linalg.norm(h[:,1:5],axis=1);h=h[abs(norms-1)<.05]
    a=a[np.argsort(a[:,0],kind='stable')];h=h[np.argsort(h[:,0],kind='stable')]
    a=a[np.r_[True,np.diff(a[:,0])>0]];h=h[np.r_[True,np.diff(h[:,0])>0]]
    hs=qyaw(h[:,1:5]);unwrapped=np.unwrap(hs)
    j=np.searchsorted(h[:,0],a[:,0],side='right');inside=(j>0)&(j<len(h))
    j=np.clip(j,1,len(h)-1);left=j-1;right=j
    gap=h[right,0]-h[left,0];nearest=np.minimum(abs(a[:,0]-h[left,0]),abs(a[:,0]-h[right,0]))
    good=inside&(gap<=.15)&(nearest<=.075)
    t=a[good,0];ekf=a[good,1];sih=np.interp(t,h[:,0],unwrapped)
    error=np.degrees(wrap(ekf-sih))
    return dict(t=t,ekf=ekf,sih=sih,error=error,speed=np.interp(t,h[:,0],h[:,5]),
        nearest_ms=nearest[good]*1000,gap_ms=gap[good]*1000,
        rejected=int(len(a)-np.sum(good)),attitude_count=len(a))

def stats(values):
    if len(values)==0:return None
    return dict(n=len(values),median=float(np.median(values)),p05=float(np.quantile(values,.05)),
        p95=float(np.quantile(values,.95)),abs_p95=float(np.quantile(abs(values),.95)),max_abs=float(max(abs(values))))

paired={};runs=[]
for name in ['1003','1005','0137','0141_0143','0214_0215','0323_0332']:
    raw=sources[name];data=raw.get('data',raw);paired[name]=align(data)
    commands=sorted([m for m in data['COMMAND_LONG'] if m['command']==187 and m['param1']>0],key=stamp)
    groups=[]
    for m in commands:
        if not groups or stamp(m)-stamp(groups[-1][-1])>8:groups.append([])
        groups[-1].append(m)
    for g in groups:
        on,off=stamp(g[0]),stamp(g[-1])
        if len(g)<20 or off-on<3:continue
        d=paired[name];t=d['t'];moving=(t>=on)&(t<=off)&(d['speed']>.5)
        pre=(t>=on-5)&(t<on)&(d['speed']<.3)
        s=stats(d['error'][moving]);assert s
        is_initial=name=='1005' and date(on).hour==23 and date(on).minute==42
        label=clock(on)+('  재부팅 후 첫 주행' if is_initial else '')
        runs.append(dict(source=name,start=on,end=off,label=label,initial=is_initial,stats=s,
            before_stopped=stats(d['error'][pre]),nearest_dt_p95_ms=float(np.quantile(d['nearest_ms'][moving],.95))))
runs.sort(key=lambda r:r['start']);assert len(runs)==20
np.savez_compressed(HERE/'aligned_series.npz',**{name+'_'+k:v for name,d in paired.items() for k,v in d.items() if isinstance(v,np.ndarray)})

pdf=PdfPages(HERE/'heading_comparison_all.pdf')
def save(fig,name):
    fig.savefig(HERE/(name+'.png'),dpi=180,bbox_inches='tight',facecolor='white');pdf.savefig(fig,bbox_inches='tight');plt.close(fig)

# Page 1: directly comparable paired-error definition across all saved drive windows.
fig,(ax,bx)=plt.subplots(1,2,figsize=(14,10),gridspec_kw={'width_ratios':[1.65,1]},layout='constrained')
y=np.arange(len(runs));labels=[]
for i,r in enumerate(runs):
    s=r['stats'];col='#d24a43' if r['initial'] else '#167ca6'
    ax.plot([s['p05'],s['p95']],[i,i],color=col,lw=5,alpha=.45,solid_capstyle='round')
    ax.scatter(s['median'],i,color=col,s=34,zorder=3)
    bx.barh(i,s['abs_p95'],color=col,alpha=.85,height=.56)
    bx.text(s['abs_p95']+.10,i,f"{s['abs_p95']:.2f}°",va='center',fontsize=9,color=col)
    labels.append(r['label'])
ax.axvline(0,color='#555',lw=1);ax.set_yticks(y,labels);ax.invert_yaxis();ax.grid(axis='x',alpha=.18)
ax.set_xlabel('EKF - SIH (°)');ax.set_title('점: 부호 있는 중앙값  /  선: 5~95백분위',pad=12)
bx.set_yticks(y,[]);bx.set_ylim(ax.get_ylim());bx.set_xlim(0,max(r['stats']['abs_p95'] for r in runs)*1.2)
bx.set_xlabel('|EKF - SIH| (°)');bx.set_title('절대 헤딩 차이의 95백분위',pad=12);bx.grid(axis='x',alpha=.18)
fig.suptitle('과거 20개 주행 구간의 EKF–SIH 헤딩 차이\n2026-10-03 ~ 10-06 03:32 · 표시 시각은 첫 양(+) 스로틀 명령(KST)',fontsize=16)
fig.supxlabel('이동 중(SIH 속도 > 0.5 m/s)만 집계 · 두 신호 모두 NED · 고정 오프셋 제거 없음\nATTITUDE 수신시각에 SIH 각도를 보간. 서로 다른 경로·속도·설정이 포함되어 성능 순위로 해석하지 않음.',fontsize=10)
save(fig,'01_heading_summary')

# Page 2: same error scale in every run, including the stopped pre-start interval.
fig,axes=plt.subplots(5,4,figsize=(17,15),layout='constrained')
all_values=[]
for r in runs:
    d=paired[r['source']];m=(d['t']>=r['start']-5)&(d['t']<=r['end']+1);all_values.extend(d['error'][m])
ymin=min(-15.,np.floor(min(all_values))-1);ymax=max(5.,np.ceil(max(all_values))+1)
for ax,r in zip(axes.flat,runs):
    d=paired[r['source']];m=(d['t']>=r['start']-5)&(d['t']<=r['end']+1)
    col='#d24a43' if r['initial'] else '#167ca6'
    ax.axvspan(-5,0,color='#eee');ax.axvline(0,color='#888',ls=':',lw=.8);ax.axhline(0,color='#666',lw=.7)
    ax.plot(d['t'][m]-r['start'],d['error'][m],color=col,lw=1)
    ax.set_xlim(-5,r['end']-r['start']+1);ax.set_ylim(ymin,ymax);ax.grid(alpha=.15)
    ax.set_title(clock(r['start'])+('\n재부팅 후 첫 주행' if r['initial'] else ''),fontsize=11,color=col)
    ax.text(.97,.07,f"이동 중 |차이| P95 {r['stats']['abs_p95']:.2f}°",transform=ax.transAxes,ha='right',fontsize=8)
    ax.set_xlabel('첫 양(+) 스로틀 명령 후 (s)',fontsize=8);ax.set_ylabel('EKF - SIH (°)',fontsize=8)
fig.suptitle('주행별 시간 변화 — 모든 패널 동일한 세로축\n회색: 출발 명령 전 5초 · 오프셋 보정 및 시간 지연 맞춤 없음',fontsize=16)
save(fig,'02_heading_timelines')

# Page 3: the initial run where the estimator heading changes relative to SIH.
r=next(r for r in runs if r['initial']);d=paired['1005']
m=(d['t']>=r['start']-10)&(d['t']<=r['end']+2);t=d['t'][m];dt=[date(x) for x in t]
ekf=np.unwrap(d['ekf'][m]);sih=d['sih'][m];sih=sih+2*np.pi*np.round((ekf[0]-sih[0])/(2*np.pi))
fig,axes=plt.subplots(3,1,figsize=(13,9),sharex=True,layout='constrained',gridspec_kw={'height_ratios':[1.25,1,0.7]})
axes[0].plot(dt,np.degrees(ekf),label='EKF (ATTITUDE)',color='#d24a43',lw=2)
axes[0].plot(dt,np.degrees(sih),label='SIH 실제 상태',color='#167ca6',lw=2)
axes[0].set_ylabel('NED 헤딩 (°)');axes[0].legend(loc='best');axes[0].set_title('회전에 따른 ±180° 표시 끊김만 연결; 두 신호 사이의 오프셋은 그대로 유지',fontsize=10)
axes[1].plot(dt,d['error'][m],color='#d24a43',lw=2);axes[1].axhline(0,color='#555',lw=.8);axes[1].set_ylabel('EKF - SIH (°)')
idx=np.argmin(d['error'][m]);axes[1].annotate(f"최소 {d['error'][m][idx]:.2f}°",(dt[idx],d['error'][m][idx]),xytext=(15,18),textcoords='offset points',arrowprops={'arrowstyle':'->','color':'#555'})
axes[2].plot(dt,d['speed'][m],color='#4b646b',lw=1.5);axes[2].set_ylabel('SIH 속도 (m/s)')
for ax in axes:
    ax.axvline(date(r['start']),color='#777',ls='--',lw=1);ax.grid(alpha=.2)
axes[2].xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S',tz=KST));axes[2].set_xlabel('2026-10-05 KST · 점선: 첫 양(+) 스로틀 명령')
fig.suptitle('재부팅 후 첫 주행: 헤딩 차이가 커졌다가 주행 중 수렴\n23:41 실행 / 실제 양(+) 스로틀 시작 '+date(r['start']).strftime('%H:%M:%S'),fontsize=16)
save(fig,'03_first_run_detail')

# Page 4: older diagnostic captures, clearly separated from the 20 drive summary.
old=sources['0923_before'];a=align({'ATTITUDE':old['attitude'],'HIL_STATE_QUATERNION':old['sih_state']})
repair=sources['0923_after'];b=align({kind:[m for m in repair if m['mavpackettype']==kind] for kind in ['ATTITUDE','HIL_STATE_QUATERNION']})
oldpairs=sources['1002_stationary']['pairs'];good=[p for p in oldpairs if 0<=p['dt']<=.075]
c=dict(t=np.array([p['t'] for p in good]),error=np.array([p['error'] for p in good]),speed=np.array([p['truth_speed'] for p in good]))
assert np.allclose(c['error'],np.degrees(wrap(np.radians([p['ekf_deg']-p['truth_deg'] for p in good]))),atol=1e-8)
history=[('09/23 21:50 · 큰 불일치 기록',a),('09/23 22:13 · 재부팅 이후 별도 기록',b),('10/02 02:31 · 정지 상태 진단',c)]
fig,axes=plt.subplots(3,1,figsize=(13,9),layout='constrained')
historical=[]
for ax,(title,s) in zip(axes,history):
    st=stats(s['error']);historical.append(dict(label=title,start=clock(s['t'][0]),end=clock(s['t'][-1]),stats=st))
    ax.plot([date(x) for x in s['t']],s['error'],color='#b86739',lw=1.5)
    ax.set_title(title+f"  |  중앙값 {st['median']:+.2f}° / 범위 {min(s['error']):+.2f}~{max(s['error']):+.2f}°",loc='left',fontsize=12)
    ax.set_ylabel('EKF - SIH (°)');ax.grid(alpha=.2);ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S',tz=KST))
    if title.startswith('09/23 21'):ax.set_ylim(0,150)
    elif title.startswith('09/23 22'):ax.set_ylim(-2,2)
    else:ax.set_ylim(-15,0)
axes[-1].set_xlabel('기록 시각 (KST)')
fig.suptitle('더 오래된 진단 기록 — 위 20개 주행 통계와 별도\n패널마다 세로축 범위가 다름 · 당시 설정 변경과 현재 제어기 성능을 직접 비교하지 않음',fontsize=15)
save(fig,'04_older_diagnostics');pdf.close()

summary=dict(definition='wrap(EKF NED yaw - SIH NED yaw), degrees; no fitted offset or lag',
    runs=runs,historical=historical,source_pairing={name:{k:v for k,v in d.items() if not isinstance(v,np.ndarray)} for name,d in paired.items()},
    drive_count=len(runs),input_sources=manifest)
(HERE/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n')
with (HERE/'heading_metrics.csv').open('w',encoding='utf-8-sig',newline='') as f:
    writer=csv.DictWriter(f,fieldnames=['start_KST','source','duration_s','moving_samples','median_deg','abs_p95_deg','max_abs_deg','stationary_before_median_deg','nearest_pair_dt_p95_ms']);writer.writeheader()
    for r in runs:
        s=r['stats'];pre=r['before_stopped'];writer.writerow(dict(start_KST=clock(r['start']),source=r['source'],duration_s=r['end']-r['start'],moving_samples=s['n'],median_deg=s['median'],abs_p95_deg=s['abs_p95'],max_abs_deg=s['max_abs'],stationary_before_median_deg=pre['median'] if pre else '',nearest_pair_dt_p95_ms=r['nearest_dt_p95_ms']))
for r in runs:print(r['label'],'median/p95abs/max',*[round(r['stats'][k],2) for k in ['median','abs_p95','max_abs']])
print('Saved four PNG figures, four-page PDF, numerical CSV and provenance.')
