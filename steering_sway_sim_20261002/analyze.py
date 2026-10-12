from pathlib import Path
import json,csv
import numpy as np
from scipy.signal import butter,sosfiltfilt,find_peaks,welch
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager

p=Path(__file__).resolve().parent
runs=[json.loads(x) for f in ['results.jsonl','injected_results.jsonl','refine_results.jsonl'] for x in (p/f).read_text().splitlines()]
key=lambda r:(r['scene'],r['plant']['name'],r['config']['name'])
bykey={key(r):r for r in runs}
def highpass(a,f):return sosfiltfilt(butter(2,f,btype='highpass',fs=10,output='sos'),a)
rows=[]
for r in runs:
    a=r['states'];t=np.array([z['t'] for z in a]);cmd=np.degrees([z['command'] for z in a]);yawrate=np.degrees([z['yaw_rate'] for z in a]);err=np.array([z['original_ref_error'] for z in a])
    if len(a)<30:raise RuntimeError(f'insufficient run: {key(r)}')
    mask=(t>=4)&(t<=t[-1]-.5);duration=t[mask][-1]-t[mask][0]
    hp03=highpass(cmd,.3);hp07=highpass(cmd,.7)
    peaks=find_peaks(cmd[mask],prominence=2,distance=3)[0];troughs=find_peaks(-cmd[mask],prominence=2,distance=3)[0]
    rows.append(dict(scene=r['scene'],plant=r['plant']['name'],config=r['config']['name'],outcome=r['outcome'],
        duration=float(t[-1]),hp03_rms_deg=float(np.sqrt(np.mean(hp03[mask]**2))),hp07_rms_deg=float(np.sqrt(np.mean(hp07[mask]**2))),
        yawrate_hp03_rms_degps=float(np.sqrt(np.mean(highpass(yawrate,.3)[mask]**2))),
        reversals_2deg_per10sec=float((len(peaks)+len(troughs))*10/duration),
        steering_tv_degps=float(np.sum(abs(np.diff(cmd[mask])))/duration),
        tracking_rms_m=float(np.sqrt(np.mean(err[mask]**2))),tracking_p95_m=float(np.quantile(err[mask],.95)),tracking_max_m=float(max(err[mask])),
        saturated_percent=float(np.mean(abs(cmd[mask])>17.18)*100),
        road_center_rms_m=float(np.sqrt(np.mean(np.array([z['route_distance'] for z in a])[mask]**2))),
        road_center_max_m=float(max(np.array([z['route_distance'] for z in a])[mask])),
        lower_failures=sum(not z['lower_valid'] for z in a),upper_failures=sum(not z['valid'] for z in r['plans'])))
lookup={(r['scene'],r['plant'],r['config']):r for r in rows}
rank=[]
names=list(dict.fromkeys(r['config'] for r in rows if r['scene']=='recorded_front'))
for name in names:
    a=[r for r in rows if r['scene']=='recorded_front' and r['config']==name]
    b=[lookup[(r['scene'],r['plant'],'baseline')] for r in a]
    hp03=np.mean([r['hp03_rms_deg'] for r in a]);hp07=np.mean([r['hp07_rms_deg'] for r in a]);base03=np.mean([r['hp03_rms_deg'] for r in b]);base07=np.mean([r['hp07_rms_deg'] for r in b])
    guard=all(r['tracking_p95_m']<=z['tracking_p95_m']+.1 and r['tracking_max_m']<=z['tracking_max_m']+.2 for r,z in zip(a,b))
    rank.append(dict(config=name,hp03_change_percent=100*(hp03/base03-1),hp07_change_percent=100*(hp07/base07-1),
        mean_tracking_p95_m=float(np.mean([r['tracking_p95_m'] for r in a])),mean_tracking_max_m=float(np.mean([r['tracking_max_m'] for r in a])),
        tracking_guard_pass=guard,score=float(.5*(hp03/base03+hp07/base07))))
rank.sort(key=lambda x:x['score'])
summary={'runs':len(runs),'metrics':rows,'replay_ranking':rank,
    'metric_definition':'0.3 / 0.7 Hz high-pass steering RMS, 2nd-order zero-phase Butterworth; t>=4s and final 0.5s excluded. Proxy for oscillation, not an exact removal of all normal cornering.',
    'tracking_guard':'Each plant: original unfiltered reference error P95 <= baseline+0.10m and maximum <= baseline+0.20m; analysis screen, not a validated safety threshold.'}
(p/'metrics.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False))
with (p/'metrics.csv').open('w') as f:
    writer=csv.DictWriter(f,list(rows[0]));writer.writeheader();writer.writerows(rows)
for r in rank:print(json.dumps(r))
print('LIVE PARAMETERS')
for r in rows:
    if r['scene']=='live_curve':print(r['plant'],r['config'],round(r['hp03_rms_deg'],3),round(r['hp07_rms_deg'],3),round(r['tracking_p95_m'],3),'fail',r['upper_failures'],r['outcome'])

font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False,'font.size':10})
best=next(r['config'] for r in rank if r['tracking_guard_pass'] and r['config']!='baseline')
best_unrestricted=rank[0]['config'];names=list(dict.fromkeys(['baseline',best,best_unrestricted,'filter5']))
colors=['#333333','#147fbf','#e48a26','#a85aaf']
fig,axs=plt.subplots(2,2,figsize=(15,10),layout='constrained')
for name,color in zip(names,colors):
    r=bykey[('recorded_front','perturbed',name)];a=r['states'];t=np.array([z['t'] for z in a]);cmd=np.degrees([z['command'] for z in a]);err=np.array([z['original_ref_error'] for z in a])
    axs[0,0].plot(t,cmd,c=color,label=name,lw=1.5)
    axs[0,1].plot(t,highpass(cmd,.3),c=color,label=name,lw=1.5)
    axs[1,0].plot(t,err,c=color,label=name,lw=1.5)
for ax in [axs[0,0],axs[0,1],axs[1,0]]:ax.set_xlim(4,16.4);ax.set_xlabel('가상 시간 (s)');ax.grid(alpha=.2);ax.legend(fontsize=8)
axs[0,0].set_title('실제 앞쪽 계획 경로 → 가상 차량 조향');axs[0,0].set_ylabel('조향 명령 (deg)')
axs[0,1].set_title('0.3 Hz 이상 조향 성분 — 반복 흔들림 지표');axs[0,1].set_ylabel('조향 성분 (deg)')
axs[1,0].set_title('평활화 전 원래 계획과의 거리도 함께 확인');axs[1,0].set_ylabel('거리 (m)')
ax=axs[1,1]
for r in rank:
    ax.scatter(r['mean_tracking_p95_m'],r['hp03_change_percent'],c='#187fbd' if r['tracking_guard_pass'] else '#df7953',s=45)
    ax.annotate(r['config'],(r['mean_tracking_p95_m'],r['hp03_change_percent']),xytext=(4,3),textcoords='offset points',fontsize=8)
ax.axhline(0,c='#888',lw=1);ax.set_xlabel('원래 계획 거리 오차 P95 / 두 응답 조건 평균 (m)');ax.set_ylabel('0.3 Hz 이상 조향 RMS 변화 (%)');ax.set_title('떨림 감소와 추종 오차의 관계');ax.grid(alpha=.2)
fig.suptitle('조향 떨림 중심의 가상 시험 — 분기 도달률로 순위를 매기지 않음',fontsize=16)
fig.supxlabel('실제 로그의 계획을 동일하게 입력하고 가상 차량은 자유롭게 이동. 고역 성분은 떨림의 근사 지표이며 정상 회전을 완전히 분리하지는 못함.',fontsize=10)
fig.savefig(p/'steering_sway_findings.png',dpi=150);plt.close(fig)
print('PLOTTED',names)
