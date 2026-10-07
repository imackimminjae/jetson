from pathlib import Path
from collections import defaultdict
import json
import numpy as np

here = Path(__file__).resolve().parent

def branch_result(r):
    name=r['scene']
    if name.startswith('replica_'):
        wanted='east' if 'east' in name else 'north'
        for s in r['states']:
            q=np.array([s['x'],s['y']])-np.array([121.,-7.])
            if np.linalg.norm(q)>18 and q[1]>-3:
                actual='east' if np.degrees(np.arctan2(q[1],q[0]))<42 else 'north'
                return actual==wanted
        return False
    if name.startswith('Y'):
        wanted=name.split('_')[2]
        straight,diverging=(12,-40) if name.startswith('Yright') else (-12,40)
        for s in r['states']:
            q=np.array([s['x'],s['y']])-np.array([80.,0.])
            if np.linalg.norm(q)>20 and q[0]>0:
                ang=np.degrees(np.arctan2(q[1],q[0]))
                actual='diverging' if abs(ang-diverging)<abs(ang-straight) else 'continuing'
                return actual==wanted and r['outcome']=='goal_reached'
        return False
    return r['outcome']=='goal_reached'

summary={}
baseline_checks=[]
for group in ('synthetic','replica','ysynthetic','robust'):
    rows=[json.loads(l) for l in (here/f'results_{group}.jsonl').read_text().splitlines()]
    expected=defaultdict(list)
    for l in (here.parent/'verify'/f'results_{group}_prod.jsonl').read_text().splitlines():
        r=json.loads(l)
        if r['config']['name']=='installed_yaml':
            expected[(r['scene'],r['plant']['name'])]=r
    for r in rows:
        key=f"{group}/{r['config']['name']}"
        if key not in summary:
            summary[key]=dict(correct=0,total=0,upper_failures=0,bad_cases=[],saturation_percent=[],lateral_error_p95_m=[])
        a=summary[key];ok=branch_result(r)
        a['correct']+=int(ok);a['total']+=1
        a['upper_failures']+=sum(not p['valid'] for p in r['plans'])
        cmd=np.array([s['command'] for s in r['states']])
        a['saturation_percent'].append(float(100*np.mean(abs(cmd)>=.298)))
        ey=[abs(s['ey']) for s in r['states'] if s['lower_valid']]
        a['lateral_error_p95_m'].append(float(np.quantile(ey,.95)) if ey else None)
        if not ok:
            a['bad_cases'].append(dict(scene=r['scene'],plant=r['plant']['name'],outcome=r['outcome']))
        if r['config']['near_weight']==0:
            old=expected[(r['scene'],r['plant']['name'])]
            same=r['outcome']==old['outcome'] and len(r['states'])==len(old['states'])
            same=same and all(np.allclose([s[k] for k in ('x','y','yaw','command')],
                                         [t[k] for k in ('x','y','yaw','command')],rtol=0,atol=1e-10)
                             for s,t in zip(r['states'],old['states']))
            baseline_checks.append(dict(group=group,scene=r['scene'],plant=r['plant']['name'],same=same))
for key,a in summary.items():
    a['mean_saturation_percent']=float(np.mean(a['saturation_percent']))
    a['mean_lateral_error_p95_m']=float(np.mean(a['lateral_error_p95_m']))
    print(key,f"{a['correct']}/{a['total']}", 'failures',a['upper_failures'],
          'sat%',round(a['mean_saturation_percent'],2),'bad',a['bad_cases'])
(here/'simulation_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
(here/'baseline_comparison.json').write_text(json.dumps(baseline_checks,indent=2)+'\n')
print('Zero-weight closed-loop matches production:',sum(r['same'] for r in baseline_checks),'/',len(baseline_checks))
