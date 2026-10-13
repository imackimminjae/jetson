"""Compare cached previous analysis with the latest local extracted signals."""
from pathlib import Path
from collections import Counter
import csv
import gzip
import hashlib
import json
import os
os.environ['MPLCONFIGDIR'] = '/tmp/s812_dt05_compare_mpl'
import numpy as np
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt, font_manager
from PIL import Image

OUT = Path(__file__).resolve().parent
OLD = OUT.parent/'scenarios_7_8_12_review'
BATCHES = {8:'20261013_022929_scenario8_50338f',12:'20261013_023722_scenario12_6d8899'}
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False})
colors=['#0072B2','#D55E00','#009E73','#CC79A7','#E69F00']
fig,axes=plt.subplots(2,2,figsize=(14,9))
comparison=[]
timelines=[]
for row,n in enumerate(BATCHES):
    route=np.array([[float(r['x']),float(r['y'])] for r in csv.DictReader(
        Path(f'/home/imac/ros2_ws/drive_batches/{BATCHES[n]}/routes/scenario{n}.csv').open())])
    result={'scenario':n,'configuration':{'prediction_dt_sec':.5,'horizon':7,'spacing_m':3,'nominal_preview_m':21,'bev_forward_m':40}}
    for col,folder in enumerate([OLD,OUT]):
        ax=axes[row,col]
        data=np.load(folder/f'scenario{n}/signals.npz')['odom']
        rows=list(csv.DictReader((folder/f'scenario{n}/per_attempt.csv').open()))
        completed=sum(a['outcome']=='completed' for a in rows)
        ax.plot(route[:,0],route[:,1],'k--o',lw=1.2,ms=3)
        ax.plot(*route[-1],'k*',ms=10)
        for a,color in zip(rows,colors):
            start,end,stop=map(float,[a['start_epoch_s'],a['outcome_epoch_s'],a['stop_phase_epoch_s']])
            tr=data[(data[:,0]>=start)&(data[:,0]<stop)];run=tr[:,0]<end
            label=f'{a["attempt"]}회 '+('목표 접근' if a['outcome']=='completed' else 'off_route')
            ax.plot(tr[run,1],tr[run,2],color=color,lw=1.5,label=label)
            ax.plot(tr[~run,1],tr[~run,2],':',color=color,lw=1)
            ax.plot(float(a['stop_x']),float(a['stop_y']),'s',mfc='white',color=color,ms=4)
        bounds=((-164,110),(-88,112)) if n==8 else ((150,334),(100,214))
        ax.set(xlim=bounds[0],ylim=bounds[1],xlabel='map X [m]',ylabel='map Y [m]',
               title=f'시나리오{n} · '+('이전 dt0.75/6단계' if col==0 else '이번 dt0.5/7단계')+f' · 목표 접근{completed}/5')
        ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(fontsize=7,loc='best')
        total=sum(int(a['upper_planning_cycles']) for a in rows)
        failed=sum(int(a['upper_unpublished']) for a in rows)
        result['previous' if col==0 else 'current']={'completed':completed,'attempts':5,'upper_cycles':total,
            'upper_unpublished':failed,'upper_unpublished_pct':100*failed/total}
    events=json.load(gzip.open(OUT/f'scenario{n}/events.json.gz','rt'))
    extra=[]
    for a in rows:
        start,end=float(a['start_epoch_s']),float(a['outcome_epoch_s'])
        ev=[e for e in events if start<=e['t_start']<end]
        runs=[];run=[]
        for e in ev:
            if not e['valid_plan']:run.append(e)
            else:
                if run:runs.append({'cycles':len(run),'first_elapsed_s':run[0]['t_start']-start,
                    'last_elapsed_s':run[-1]['t_start']-start,'recovery_elapsed_s':e['t_start']-start,
                    'until_recovery_s':e['t_start']-run[0]['t_start']});run=[]
        assert not run,'No recovered plan before outcome'
        extra.append({'attempt':int(a['attempt']),'horizon_min':min(e['horizon'] for e in ev),
            'horizon_max':max(e['horizon'] for e in ev),'failure_reasons':dict(Counter(e['reason'] for e in ev if not e['valid_plan'])),
            'consecutive_failures_max':max([r['cycles'] for r in runs],default=0),'failed_runs':runs})
        for e in ev:
            timelines.append({'scenario':n,'attempt':a['attempt'],'elapsed_s':e['t_start']-start,
                **{k:e.get(k) for k in ['x','y','yaw_rad','wp0','wp1','horizon','valid_plan','reason','retained_previous_path','retained_path_age_sec']}})
    result['current_attempt_diagnostics']=extra
    comparison.append(result)
fig.suptitle('시나리오8·12 · 변경 전후5회 비교\nBEV40×40m 유지 · 실선 running · 점선 관성 이동 · □ 정지 · ★ 목표',fontsize=13)
fig.tight_layout(rect=(0,0,1,.91))
fig.savefig(OUT/'scenarios8_12_before_after.png',dpi=170)
fig.savefig(OUT/'scenarios8_12_before_after.pdf')
plt.close(fig)
(OUT/'comparison.json').write_text(json.dumps(comparison,ensure_ascii=False,indent=2)+'\n')
with (OUT/'branch_timeline.csv').open('w',newline='') as f:
    w=csv.DictWriter(f,fieldnames=list(timelines[0]));w.writeheader();w.writerows(timelines)
for name,expected in json.loads((OUT/'sources_sha256.json').read_text()).items():
    h=hashlib.sha256()
    with Path(name).open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    assert h.hexdigest()==expected,name
for image in OUT.glob('*.png'):
    with Image.open(image) as im:im.verify()
selected=json.loads((OUT/'selected_branch_cycles.json').read_text())
assert len(selected)==2
v=json.loads((OUT/'validation.json').read_text())
v.update(grid_decoded=2,selected_grid_frames=2,grid_scope='selected single frames only; no accumulated map cache',
    all_original_hashes_rechecked=True,previous_comparison_source='already extracted earlier analysis; no new replay of previous bags',
    selected_frame_age_max_ms=max(c['grid_before_plan_ms'] for c in selected))
(OUT/'validation.json').write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n')
print('전후 궤적·분기 진단 저장, 원본 해시와 PNG 검증 완료')
