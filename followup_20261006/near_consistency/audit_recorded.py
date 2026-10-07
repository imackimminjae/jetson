"""Audit the actual executed planner trace, independently of counterfactual replay."""
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import json
import pickle
import numpy as np

here = Path(__file__).resolve().parent
root = here.parents[1]
all_results = {}
for tag, folder in [('S1','drive_debug_20261006_014143_light'),('S2','drive_debug_20261006_014303_light')]:
    data = pickle.load((root/folder/'analysis/decoded.pkl').open('rb'))
    results = []
    for event in data['/debug/upper_branch_event']:
        e = event['data']
        if not (190 <= e['x'] <= 245 and 140 <= e['y'] <= 180) or not e['valid_plan']:
            continue
        preview = np.array(e['preview_world'])
        plan = np.array(e['planned_world'])
        row = dict(time=datetime.fromtimestamp(e['t_start'],ZoneInfo('Asia/Seoul')).isoformat(),
                   x=e['x'],y=e['y'],selected=e['selected_corridors'][1:3],steps=[])
        for k in [1,2]:
            distances=[]
            raw_distances=[]
            for c in e['intervals_world']:
                if c[0]!=k:
                    continue
                a,b=np.array(c[2:4]),np.array(c[4:6])
                axis=(b-a)/np.linalg.norm(b-a)
                q=float((preview[k]-a)@axis)
                gap=max(0.,-q,q-np.linalg.norm(b-a))
                distances.append(float(gap))
                inset=(float(c[6])-float(c[7]))/2
                raw_distances.append(float(max(0.,-q-inset,q-np.linalg.norm(b-a)-inset)))
            row['steps'].append(dict(k=k,candidate_gaps_m=distances,
                                     raw_candidate_gaps_m=raw_distances,
                                     min_raw_gap_m=min(raw_distances),
                                     min_forced_lateral_change_m=min(distances),
                                     actual_change_m=float(np.linalg.norm(plan[k]-preview[k]))))
        results.append(row)
    all_results[tag]=results
    print(tag, 'cycles',len(results))
    for r in results:
        print(r['time'][11:19], 'selected',r['selected'], 'forced',
              [round(x['min_forced_lateral_change_m'],2) for x in r['steps']],
              'actual',[round(x['actual_change_m'],2) for x in r['steps']])
(here/'recorded_constraint_audit.json').write_text(json.dumps(all_results,indent=2)+'\n')
