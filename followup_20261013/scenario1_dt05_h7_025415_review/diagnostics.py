"""Save current attempt diagnostics and verify original files without ROS nodes."""
from pathlib import Path
from collections import Counter
import csv
import gzip
import hashlib
import json
from PIL import Image

OUT=Path(__file__).resolve().parent
rows=list(csv.DictReader((OUT/'per_attempt.csv').open()))
events=json.load(gzip.open(OUT/'scenario1/events.json.gz','rt'))
diagnostics=[]
timeline=[]
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
        timeline.append({'attempt':int(a['attempt']),'elapsed_s':e['t_start']-start,
            **{k:e.get(k) for k in ['x','y','yaw_rad','wp0','wp1','horizon','valid_plan','reason','retained_previous_path','retained_path_age_sec']}})
    if run:runs.append({'cycles':len(run),'first_elapsed_s':run[0]['t_start']-start,
        'last_elapsed_s':run[-1]['t_start']-start,'recovery_elapsed_s':None,'until_recovery_s':None})
    diagnostics.append({'attempt':int(a['attempt']),'outcome':a['outcome'],
        'horizon_min':min(e['horizon'] for e in ev),'horizon_max':max(e['horizon'] for e in ev),
        'failure_reasons':dict(Counter(e['reason'] for e in ev if not e['valid_plan'])),'failed_runs':runs})
(OUT/'attempt_diagnostics.json').write_text(json.dumps(diagnostics,ensure_ascii=False,indent=2)+'\n')
with (OUT/'branch_timeline.csv').open('w',newline='') as f:
    w=csv.DictWriter(f,fieldnames=list(timeline[0]));w.writeheader();w.writerows(timeline)
for name,expected in json.loads((OUT/'sources_sha256.json').read_text()).items():
    h=hashlib.sha256()
    with Path(name).open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    assert h.hexdigest()==expected,name
for image in OUT.glob('*.png'):
    with Image.open(image) as im:im.verify()
selected=json.loads((OUT/'selected_branch_cycles.json').read_text())
assert len(selected)==4
assert len(list((OUT/'scenario1').glob('case_*_grid.npz')))==4
v=json.loads((OUT/'validation.json').read_text())
v.update(grid_decoded=4,selected_grid_frames=4,grid_scope='four selected single frames only; no accumulated map cache',
    all_original_hashes_rechecked=True,previous_comparison_source='GitHub archive reports, see historical_sources.json',
    selected_frame_age_max_ms=max(c['grid_before_plan_ms'] for c in selected))
(OUT/'validation.json').write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n')
print('5회 주기 진단 저장, 원본 해시와 PNG 검증 완료')
