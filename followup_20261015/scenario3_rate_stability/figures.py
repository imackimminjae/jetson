from pathlib import Path
import json,pickle,hashlib,os
import numpy as np
os.environ['MPLCONFIGDIR']='/tmp/codex_scenario3_rate_mpl'
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt,font_manager
from matplotlib.backends.backend_pdf import PdfPages
from PIL import Image
OUT=Path(__file__).resolve().parent
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False,'font.size':10})
reports=json.loads((OUT/'metrics.json').read_text());summaries=json.loads((OUT/'summary.json').read_text())
colors=['#0072B2','#D55E00','#009E73','#CC79A7','#E69F00'];ratecolors=['#4477aa','#009988']
figs=[]
fig,axs=plt.subplots(3,2,figsize=(12,13))
for j,r in enumerate(reports):
    d=pickle.load((OUT/f"decoded_{r['batch']}.pkl").open('rb'));route=np.array(d['route']);odom=np.array(d['data']['/motive/vehicle/odom_map']);low=d['data']['/debug/lower_mpc_trace']
    for ax in axs[:,j]:ax.plot(route[:,0],route[:,1],'k--o',lw=1,ms=3)
    for i,a in enumerate(r['attempts']):
        od=odom[(odom[:,0]>=a['start'])&(odom[:,0]<a['end'])]
        for ax in axs[:,j]:ax.plot(od[:,1],od[:,2],color=colors[i],lw=1.5,label=f'{i+1}회')
        neutral=np.array([v for t,v in low if a['start']<=t<a['end'] and v[4]>1 and v[10]==0 and v[27]==0])
        if len(neutral):
            for ax in axs[:,j]:ax.scatter(neutral[:,1],neutral[:,2],s=9,facecolors='none',edgecolors='#e54b4b',lw=.7,zorder=4)
    axs[0,j].set_title(f"{r['rate_hz']:g} Hz · 목표 접근 5/5\n전체 궤적 편차 RMS {r['spatial']['rms_spread_m']:.2f} m");axs[0,j].legend(fontsize=8)
    for ax,view,title in zip(axs[:,j],[((180,285),(128,225)),((185,225),(150,190)),((235,285),(130,175))],['전체','첫 회전 확대','두 번째 회전 확대']):
        ax.set_xlim(*view[0]);ax.set_ylim(*view[1]);ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_xlabel('map X [m]');ax.set_ylabel('map Y [m]')
        if title!='전체':ax.set_title(title)
fig.suptitle('시나리오3 · 2 / 5 Hz · 붉은 빈 원: 주행 중 제어 중립 구간',fontsize=15)
fig.tight_layout();fig.savefig(OUT/'trajectories.png',dpi=155);figs.append(fig)

fig,axs=plt.subplots(4,2,figsize=(13,12),sharex='col')
for j,r in enumerate(reports):
    z=np.load(OUT/f"spatial_{r['rate_hz']:g}hz.npz");s=z['s']
    for i in range(5):
        for k,name in enumerate(['steering','lateral','heading']):axs[k,j].plot(s,z[name][i],lw=1,color=colors[i],alpha=.85)
    axs[3,j].plot(s,z['spread'],color='#333333')
    axs[0,j].set_title(f"{r['rate_hz']:g} Hz")
    for k,lim in enumerate([(-19,19),(-.8,.8),(-40,40),(0,1.8)]):
        axs[k,j].set_ylim(*lim);axs[k,j].grid(alpha=.2);axs[k,j].axvspan(35,80,color='#aaa',alpha=.12);axs[k,j].axvspan(105,145,color='#aaa',alpha=.12)
    axs[3,j].set_xlabel('웨이포인트 경로 진행거리 [m]')
for ax,label in zip(axs[:,0],['송신 조향 명령 [도 환산]','MPC 횡오차 [m]','MPC 방향오차 [도]','회차 궤적 편차 [m]']):ax.set_ylabel(label)
fig.suptitle('시나리오3 · 동일 진행 위치 비교 · 회색: 첫 회전 / 두 번째 회전',fontsize=15)
fig.tight_layout();fig.savefig(OUT/'spatial_signals.png',dpi=150);figs.append(fig)

fig,axs=plt.subplots(1,4,figsize=(14,4.5))
for ax,key,title,steer in zip(axs,['variation_deg_per_100m','rms_spread_m','rapid_residual_rms_deg','yaw_accel_rms_degps2'],['누적 조향 보정량\n[도 / 100 m]','회차별 궤적 편차 RMS\n[m]','빠른 조향 변화 RMS\n[도 환산]','차량 yaw 가속도 RMS\n[도 / 초²]'],[True,None,True,False]):
    values=[s[key]['median'] if steer is not None else s['repeatability'][key] for s in summaries];ax.bar([0,1],values,color=ratecolors,width=.55)
    if steer is not None:
        for j,r in enumerate(reports):
            vs=[a['steering'][key] if steer else a[key] for a in r['attempts']];ax.scatter(j+np.linspace(-.1,.1,5),vs,c='#333',s=20,zorder=3)
    ax.set_xticks([0,1],['2 Hz','5 Hz']);ax.set_title(title);ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
    for x,v in enumerate(values):ax.text(x,v*.4,f'{v:.2f}' if v<10 else f'{v:.1f}',color='white',ha='center',fontweight='bold')
fig.suptitle('시나리오3 · 각 5회 · 점: 개별 회차 / 궤적 편차: 5회 종합 / 나머지 막대: 중앙값',fontsize=12)
fig.tight_layout();fig.savefig(OUT/'overview.png',dpi=165);figs.insert(0,fig)
with PdfPages(OUT/'scenario3_comparison.pdf') as pdf:
    for f in figs:pdf.savefig(f)

# Spatial sensitivity: an uninterrupted-control 2 Hz subset, dropping attempt2.
z=np.load(OUT/'spatial_2hz.npz');xy=np.delete(z['xy'],1,axis=0);spread=np.sqrt(np.var(xy[:,:,0],axis=0)+np.var(xy[:,:,1],axis=0))
sensitivity={'excluded_rate_hz':2,'excluded_attempt':2,'reason':'7.5 seconds neutral output in late turn; success by goal proximity does not imply uninterrupted controlled tracking','remaining_attempts':[1,3,4,5],'remaining_spread_rms_m':float(np.sqrt(np.mean(spread**2))),'five_hz_all5_spread_rms_m':reports[1]['spatial']['rms_spread_m']}
(OUT/'sensitivity.json').write_text(json.dumps(sensitivity,ensure_ascii=False,indent=2)+'\n')
for p,h in json.loads((OUT/'sources_sha256.json').read_text()).items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==h
for p in OUT.glob('*.png'):
    with Image.open(p) as im:im.verify()
v=json.loads((OUT/'validation.json').read_text());v.update(original_hashes_rechecked=True,png_verified=True,all_attempts_completed=10,spatial_comparison='Same progress5..154.5m at0.5m intervals',source_is_sent_steering_not_measured_actuator=True,sensitivity=sensitivity)
(OUT/'validation.json').write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n')
print('Figures and source validation saved; sensitivity',sensitivity)
