from pathlib import Path
import json,pickle,hashlib,os
import numpy as np
os.environ['MPLCONFIGDIR']='/tmp/codex_controller_rate_mpl'
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt,font_manager
from matplotlib.backends.backend_pdf import PdfPages
from PIL import Image
OUT=Path(__file__).resolve().parent
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False,'font.size':10})
reports=json.loads((OUT/'metrics.json').read_text());summaries=json.loads((OUT/'summary.json').read_text())
colors=['#0072B2','#D55E00','#009E73','#CC79A7','#E69F00'];ratecolors=['#4477aa','#cc6677'];figs=[]
fig,axs=plt.subplots(2,2,figsize=(12,9))
for j,r in enumerate(reports):
    d=pickle.load((OUT/f"decoded_{r['batch']}.pkl").open('rb'));route=np.array(d['route']);odom=np.array(d['data']['/motive/vehicle/odom_map']);low=d['data']['/debug/lower_mpc_trace']
    for ax in axs[:,j]:ax.plot(route[:,0],route[:,1],'k--o',lw=1,ms=3)
    for i,a in enumerate(r['attempts']):
        od=odom[(odom[:,0]>=a['start'])&(odom[:,0]<a['end'])]
        for ax in axs[:,j]:ax.plot(od[:,1],od[:,2],color=colors[i],lw=1.5,label=f'{i+1}회')
        for mode,style in [(4,'o'),(3,'x')]:
            bad=np.array([v for t,v in low if a['start']<=t<a['end'] and v[4]>1 and v[12]==mode])
            if len(bad):
                for ax in axs[:,j]:
                    if mode==4:ax.scatter(bad[:,1],bad[:,2],s=9,facecolors='none',edgecolors='#e54b4b',lw=.7,zorder=4)
                    else:ax.scatter(bad[:,1],bad[:,2],s=8,color='#d63434',marker='x',lw=.8,zorder=4)
    axs[0,j].set_title(f"컨트롤러 {r['rate_hz']:g} Hz · 예측 {r['prediction_horizon_sec']:g}초\n평가 {len(r['attempts'])}회 · 궤적 편차 RMS {r['spatial']['rms_spread_m']:.2f} m");axs[0,j].legend(fontsize=8)
    for ax,view in zip(axs[:,j],[((95,180),(-145,10)),((102,153),(-35,8))]):
        ax.set_xlim(*view[0]);ax.set_ylim(*view[1]);ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_xlabel('map X [m]');ax.set_ylabel('map Y [m]')
    axs[1,j].set_title('회전 구간 확대')
fig.suptitle('시나리오1 · 플래너 5 Hz 고정 · 붉은 원: 보호 정지 / 붉은 ×: 실패 중립',fontsize=14)
fig.tight_layout();fig.savefig(OUT/'trajectories.png',dpi=160);figs.append(fig)

fig,axs=plt.subplots(4,2,figsize=(13,12),sharex='col')
for j,r in enumerate(reports):
    z=np.load(OUT/f"spatial_{r['rate_hz']:g}hz.npz");s=z['s']
    for i in range(len(r['attempts'])):
        for k,name in enumerate(['steering','lateral','heading']):axs[k,j].plot(s,z[name][i],lw=1,color=colors[i],alpha=.85)
    axs[3,j].plot(s,z['spread'],color='#333333');axs[0,j].set_title(f"컨트롤러 {r['rate_hz']:g} Hz · 예측 {r['prediction_horizon_sec']:g}초")
    for k,lim in enumerate([(-19,19),(-2.2,2.2),(-45,45),(0,2.5)]):
        axs[k,j].set_ylim(*lim);axs[k,j].grid(alpha=.2);axs[k,j].axvspan(90,155,color='#aaa',alpha=.12)
    axs[3,j].set_xlabel('웨이포인트 경로 진행거리 [m]')
for ax,label in zip(axs[:,0],['송신 조향 명령 [도 환산]','MPC 횡오차 [m]','MPC 방향오차 [도]','회차 궤적 편차 [m]']):ax.set_ylabel(label)
fig.suptitle('플래너 5 Hz 고정 · 동일 진행 위치에서 컨트롤러 비교',fontsize=15)
fig.tight_layout();fig.savefig(OUT/'spatial_signals.png',dpi=150);figs.append(fig)

fig,axs=plt.subplots(1,4,figsize=(14,4.5))
for ax,key,title,steer in zip(axs,['variation_deg_per_100m','rms_spread_m','limit_ratio_pct','yaw_accel_rms_degps2'],['누적 조향 보정량\n[도 / 100 m]','회차별 궤적 편차 RMS\n[m]','조향 한계 근접 시간비\n[%]','차량 yaw 가속도 RMS\n[도 / 초²]'],[True,None,True,False]):
    values=[s[key]['median'] if steer is not None else s['repeatability'][key] for s in summaries];ax.bar([0,1],values,color=ratecolors,width=.55)
    if steer is not None:
        for j,r in enumerate(reports):
            vs=[a['steering'][key] if steer else a[key] for a in r['attempts']];ax.scatter(j+np.linspace(-.1,.1,len(r['attempts'])),vs,c='#333',s=20,zorder=3)
    ax.set_xticks([0,1],['10 Hz / 3초 예측','20 Hz / 1.5초 예측']);ax.set_title(title);ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
    for x,v in enumerate(values):ax.text(x,v*.4,f'{v:.2f}' if v<10 else f'{v:.1f}',color='white',ha='center',fontweight='bold')
fig.suptitle('시나리오1 · 10 Hz 5회 / 20 Hz 4회 · 20 Hz 마지막 회차 제외 · 플래너 5 Hz 고정',fontsize=13)
fig.tight_layout();fig.savefig(OUT/'overview.png',dpi=165);figs.insert(0,fig)
with PdfPages(OUT/'controller_comparison.pdf') as pdf:
    for f in figs:pdf.savefig(f)

sensitivity={'official_exclusion':True,'controller_rate_hz':20,'excluded_attempt':5,'reason':'User reported a brief BEV interruption and requested exclusion','included_attempts':[1,2,3,4],'original_logs_preserved':True}
(OUT/'exclusion.json').write_text(json.dumps(sensitivity,ensure_ascii=False,indent=2)+'\n')
(OUT/'sensitivity.json').write_text(json.dumps(sensitivity,ensure_ascii=False,indent=2)+'\n')
# Compute actual durations of non-tracking command modes from native20Hz samples.
details=[]
for r in reports:
    D=pickle.load((OUT/f"decoded_{r['batch']}.pkl").open('rb'))['data']
    for a in r['attempts']:
        l=np.array([v for t,v in D['/debug/lower_mpc_trace'] if a['start']<=t<a['end']]);episodes=[];run=[];last_mode=None
        for row in l:
            mode=int(row[12]);bad=mode in [3,4] and row[4]>1
            if bad and mode==last_mode:run.append(row)
            else:
                if run:episodes.append(dict(mode=last_mode,elapsed_start_sec=float(run[0][0]-a['start']),duration_sec=float(run[-1][0]-run[0][0]+1/r['rate_hz']),samples=len(run),first_pose=run[0][1:5].tolist()))
                run=[row] if bad else []
            last_mode=mode if bad else None
        if run:episodes.append(dict(mode=last_mode,elapsed_start_sec=float(run[0][0]-a['start']),duration_sec=float(run[-1][0]-run[0][0]+1/r['rate_hz']),samples=len(run),first_pose=run[0][1:5].tolist()))
        details.append(dict(controller_rate_hz=r['rate_hz'],attempt=a['attempt'],episodes=episodes))
(OUT/'non_tracking_episodes.json').write_text(json.dumps(details,indent=2)+'\n')
for p,h in json.loads((OUT/'sources_sha256.json').read_text()).items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==h
for p in OUT.glob('*.png'):
    with Image.open(p) as im:im.verify()
v=json.loads((OUT/'validation.json').read_text());v.update(original_hashes_rechecked=True,png_verified=True,all_attempts_completed=9,steering_metric_uses_native_updates=True,rapid_command_metric='Common100Hz zero-order-held command and0.5-second moving-average residual',protective_mode4_is_not_qp_failure=True,sensitivity=sensitivity)
(OUT/'validation.json').write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n')
print('Figures and validation saved; sensitivity',sensitivity)
print('Non-tracking episodes',[e for e in details if e['episodes']])
