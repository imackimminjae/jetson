from pathlib import Path
import json,pickle,hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt,font_manager
OUT=Path(__file__).resolve().parent
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False})
reports=json.loads((OUT/'metrics.json').read_text())
fig,axs=plt.subplots(3,3,figsize=(15,10),sharex='col')
checks=[]
for j,(ri,ai) in enumerate([(0,0),(1,1),(3,1)]):
    r=reports[ri];a=r['attempts'][ai];D=pickle.load((OUT/f"decoded_{r['batch']}.pkl").open('rb'))['data'];start,end=a['start'],a['end']
    g=np.asarray(D['/bev/occupancy_grid'])[:,0];sample=np.arange(start,end,.025);ix=np.maximum(0,np.searchsorted(g,sample,side='right')-1);age=sample-g[ix]
    axs[0,j].plot(sample-start,age,color='#0072B2',lw=1);axs[0,j].axhline(3,color='gray',ls='--');axs[0,j].set_ylim(0,3.2)
    axs[0,j].set_title(('이전 10/13' if ri==0 else '이번 기록')+f" · {r['rate']:g} Hz · {a['attempt']}회\n"+('목표 접근' if a['outcome']=='completed' else '경로 이탈'))
    cy=[v for t,v in D['/debug/upper_stage_timing'] if start<=v['t_start']<end]
    axs[1,j].plot([v['t_start']-start for v in cy],[v['wall_ms'] for v in cy],color='#009E73');axs[1,j].set_ylim(0,26)
    ev=[v for t,v in D['/debug/upper_branch_event'] if start<=v['t_start']<end];ts=[v['t_start']-start for v in ev]
    axs[2,j].plot(ts,np.degrees([v['yaw_rad'] for v in ev]),color='#333333',label='차량 yaw')
    for v in ev:
        if not v['valid_plan']:
            for ax in axs[:,j]:ax.axvline(v['t_start']-start,color='#D55E00',alpha=.1,lw=2)
    axs[2,j].set_ylim(-10,155);axs[2,j].set_xlabel('주행 시작 후 시간 [초]')
    if ri==1:
        axs[2,j].annotate('22.36초부터 계획 불가',xy=(22.36,137),xytext=(24,145),fontsize=9,arrowprops={'arrowstyle':'->'})
    for ax in axs[:,j]:ax.set_xlim(0,end-start);ax.grid(alpha=.2)
    gaps=[dict(elapsed_s=float(g[k]-start),gap_s=float(g[k+1]-g[k])) for k in np.flatnonzero((np.diff(g)>1)&(g[:-1]>=start)&(g[:-1]<end))]
    checks.append(dict(batch=r['batch'],attempt=a['attempt'],gaps_over_1s=gaps))
axs[0,0].set_ylabel('직전 BEV 수신 이후 [초]\n녹화 시각 기준 추정')
axs[1,0].set_ylabel('전체 플래너 계산 [ms]')
axs[2,0].set_ylabel('차량 yaw [도]')
fig.suptitle('계산은 제때 끝나지만 BEV 수신이 반복 중단 · 주황선: 계획 발행 실패',fontsize=15)
fig.text(.02,.005,'BEV 헤더 시각은 전부 0이고 실제 사용 grid ID가 없어, 수신 간격과 사용 지도 나이는 구분해야 합니다.',fontsize=9)
fig.tight_layout(rect=(0,.025,1,.96));fig.savefig(OUT/'input_timeline.png',dpi=150)
(OUT/'selected_gap_evidence.json').write_text(json.dumps(checks,indent=2)+'\n')
hashes={}
for r in reports:
    b=OUT.parents[1]/'drive_batches'/r['batch']
    for p in b.rglob('*'):
        if p.is_file():hashes[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
(OUT/'input_sha256.json').write_text(json.dumps(hashes,indent=2)+'\n')
print('Timeline and input hashes saved')
