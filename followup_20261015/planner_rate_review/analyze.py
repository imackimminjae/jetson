from pathlib import Path
from collections import Counter
import csv, json, pickle, math
import numpy as np
import yaml

OUT = Path(__file__).resolve().parent
ALL = []

def stats(values):
    a = np.asarray([v for v in values if v is not None and math.isfinite(v)])
    return dict(min=float(a.min()), median=float(np.median(a)), p95=float(np.percentile(a,95)), max=float(a.max())) if len(a) else {}

def gap(records):
    return stats(np.diff([r[0] for r in records]))

for f in sorted(OUT.glob('decoded_*.pkl')):
    d = pickle.load(f.open('rb')); D = d['data']
    p = yaml.safe_load(d['config'])['upper_planner_node']['ros__parameters']
    report = dict(batch=d['batch'], rate=p['upper_planner_rate_hz'], attempts=[])
    phase = [v for t,v in D['/knu_batch/status']]
    events = [dict(received=t, **v) for t,v in D['/debug/upper_branch_event']]
    timing = [dict(received=t, **v) for t,v in D['/debug/upper_stage_timing']]
    for i,row in enumerate(d['summary'],1):
        start = next(v['phase_stamp_sec'] for v in phase if v['index']==i and v['phase']=='running')
        end = next(v['phase_stamp_sec'] for v in phase if v['index']==i and v['phase']=='coasting')
        ref = next(v['phase_stamp_sec'] for v in phase if v['index']==i and v['phase']=='reference')
        ev = [v for v in events if start<=v['t_start']<end]
        cy = [v for v in timing if start<=v['t_start']<end]
        low = [(t,v) for t,v in D['/debug/lower_mpc_trace'] if start<=t<end]
        b = [(t,v) for t,v in D['/debug/upper_interval_centering_state'] if start<=t<end]
        od = [v for v in D['/motive/vehicle/odom_map'] if start<=v[0]<end]
        grid = [v for v in D['/bev/occupancy_grid'] if start<=v[0]<end]
        logs = [v for v in D['/rosout'] if start<=v[0]<end and v[2]>=30]
        failures = []; run=[]
        for v in ev:
            if not v['valid_plan']: run.append(v)
            elif run:
                failures.append(dict(first_s=run[0]['t_start']-start,duration_s=v['t_start']-run[0]['t_start'],n=len(run),x=run[0]['x'],y=run[0]['y'],reason=run[0]['reason'],status=run[0]['status']))
                run=[]
        if run: failures.append(dict(first_s=run[0]['t_start']-start,duration_s=end-run[0]['t_start'],n=len(run),x=run[0]['x'],y=run[0]['y'],reason=run[0]['reason'],status=run[0]['status']))
        samples=[]
        for x,y in [(110,-29),(114,-18),(120,-10),(130,-5)]:
            e=min(ev,key=lambda v:(v['x']-x)**2+(v['y']-y)**2)
            bt=min(b,key=lambda v:abs(v[0]-e['t_emit']))[1]
            ut=min(D['/debug/upper_miqp_trace'],key=lambda v:abs(v[0]-e['t_emit']))[1]
            samples.append(dict(target=[x,y],elapsed=e['t_start']-start,pose=[e['x'],e['y'],math.degrees(e['yaw_rad'])],B=bt[2],valid=e['valid_plan'],status=e['status'],branch_step=e['branch_step'],corridors=e['selected_corridors'],candidates=e['candidate_counts'],end=(e['published_world'] or e['retained_world'])[-1] if (e['published_world'] or e['retained_world']) else [],previous_reference=ut[-1],heading_target_deg=math.degrees(ut[12]+ut[3]),event=e))
        init=min(D['/motive/vehicle/odom_map'],key=lambda v:abs(v[0]-start))
        first=next((v for v in events if ref<=v['t_start']<start),None)
        a=dict(attempt=i,outcome=row['outcome'],start=start,end=end,duration=end-start,progress=float(row['progress_ratio']),initial_pose=list(init[1:4]),upper_cycles=len(cy),upper_unpublished=sum(not v['valid_plan'] for v in ev),failure_reasons=dict(Counter(v['reason'] for v in ev if not v['valid_plan'])),solver_status=dict(Counter(v['status'] for v in ev)),upper_wall_ms=stats([v['wall_ms'] for v in cy]),upper_cpu_ms=stats([v['cpu_ms'] for v in cy]),upper_start_interval_ms=stats([v['start_interval_ms'] for v in cy]),upper_over_period=sum(v['wall_ms']>1000/report['rate'] for v in cy),upper_early_returns=sum(v['plan_seq']==0 for v in cy),upper_early_outcomes=dict(Counter(v['outcome'] for v in cy if v['plan_seq']==0)),lower_wall_ms=stats([v[11] for t,v in low]),lower_invalid=sum(v[10]==0 for t,v in low),lower_fallback=dict(Counter(int(v[12]) for t,v in low)),lower_receive_gap_s=gap(low),grid_receive_gap_s=gap(grid),odom_receive_gap_s=gap(od),B=stats([v[2] for t,v in b]),initial_B=min(D['/debug/upper_interval_centering_state'],key=lambda v:abs(v[0]-first['t_emit']))[1][2] if first else None,steer_saturation_ratio=float(np.mean([abs(v[9])>=.99 for t,v in low])),failed_runs=failures,samples=samples,warning_counts=dict(Counter(v[3] for v in logs)),max_wall_stage=max(cy,key=lambda v:v['wall_ms']))
        report['attempts'].append(a)
        print(d['batch'],i,row['outcome'],'wall',a['upper_wall_ms'],'over',a['upper_over_period'],'fail',a['failure_reasons'],'lowfail',a['lower_invalid'],'B',a['B'],'gaps',a['grid_receive_gap_s'].get('max'),flush=True)
    ALL.append(report)

(OUT/'metrics.json').write_text(json.dumps(ALL,ensure_ascii=False,indent=2)+'\n')
with (OUT/'per_attempt.csv').open('w',newline='') as f:
    rows=[dict(batch=r['batch'],rate=r['rate'],attempt=a['attempt'],outcome=a['outcome'],upper_wall_p95_ms=a['upper_wall_ms']['p95'],upper_wall_max_ms=a['upper_wall_ms']['max'],over_period=a['upper_over_period'],upper_unpublished=a['upper_unpublished'],lower_invalid=a['lower_invalid'],grid_gap_max_s=a['grid_receive_gap_s']['max'],initial_B=a['initial_B'],B_min=a['B']['min'],failure_reasons=json.dumps(a['failure_reasons'])) for r in ALL for a in r['attempts']]
    w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)

import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt, font_manager
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False})
fig,axs=plt.subplots(2,4,figsize=(17,9))
colors=['#0072B2','#D55E00','#009E73','#CC79A7','#E69F00']
for j,r in enumerate(ALL):
    d=pickle.load((OUT/f"decoded_{r['batch']}.pkl").open('rb'));od=np.asarray(d['data']['/motive/vehicle/odom_map']);route=np.asarray(d['route'])
    for ax in axs[:,j]:ax.plot(route[:,0],route[:,1],'k--o',ms=3,lw=1)
    for a,c in zip(r['attempts'],colors):
        tr=od[(od[:,0]>=a['start']) & (od[:,0]<a['end'])]
        for ax in axs[:,j]:
            ax.plot(tr[:,1],tr[:,2],color=c,lw=1.4,label=f"{a['attempt']}회: "+('목표 접근' if a['outcome']=='completed' else '이탈'))
            ax.plot(tr[-1,1],tr[-1,2],'x',color=c)
    axs[0,j].set_title(('이전 10/13' if j==0 else '이번 기록')+f" · {r['rate']:g} Hz\n목표 접근 {sum(a['outcome']=='completed' for a in r['attempts'])}/5")
    axs[0,j].set_xlim(65,190);axs[0,j].set_ylim(-145,50);axs[0,j].legend(fontsize=8)
    axs[1,j].set_xlim(95,155);axs[1,j].set_ylim(-35,25);axs[1,j].set_title('분기·회전 구간 확대')
    for ax in axs[:,j]:ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_xlabel('map X [m]');ax.set_ylabel('map Y [m]')
fig.suptitle('시나리오1 · 7스텝 / 예측 간격 0.5초 · 주행 중 궤적 비교',fontsize=16)
fig.tight_layout();fig.savefig(OUT/'trajectories.png',dpi=155)
print('Saved',OUT)
