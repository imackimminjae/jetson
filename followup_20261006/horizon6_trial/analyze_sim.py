from pathlib import Path
from collections import defaultdict
import json,numpy as np
from scipy.signal import butter,sosfiltfilt
h=Path(__file__).resolve().parent
# Preserve the pre-existing scene-specific branch/goal criterion.
s=(h.parent/'near_consistency/analyze_sim.py').read_text();exec(s[s.index('def branch_result'):s.index('\nsummary={}')])
summary={};pairs=[]
for group in ['synthetic','replica','ysynthetic','robust']:
 p=h/f'results_{group}.jsonl'
 if not p.exists():continue
 rows=[json.loads(l) for l in p.read_text().splitlines()];bycase=defaultdict(dict)
 for r in rows:
  name=r['config']['name'];bycase[(r['scene'],r['plant']['name'])][name]=r;k=f'{group}/{name}';a=summary.setdefault(k,dict(correct=0,total=0,upper_failures=0,bad_cases=[]));ok=branch_result(r);a['correct']+=int(ok);a['total']+=1;a['upper_failures']+=sum(not q['valid'] for q in r['plans'])
  if not ok:a['bad_cases'].append(dict(scene=r['scene'],plant=r['plant']['name'],outcome=r['outcome'],final_xy=r['final_xy']))
  gaps=np.diff([q['t'] for q in r['plans']]);assert np.allclose(gaps,.5),gaps
 for (scene,plant),rs in bycase.items():
  if set(rs)!=set(['h7','h6']):continue
  ranges={k:[q['progress'] for q in r['states'] if q['t']>=4] for k,r in rs.items()}
  if not all(ranges.values()):continue
  lo=max(min(v) for v in ranges.values());hi=min(max(v) for v in ranges.values());row=dict(group=group,scene=scene,plant=plant,common_progress=[lo,hi])
  for k,r in rs.items():
   t=np.array([q['t'] for q in r['states']]);prog=np.array([q['progress'] for q in r['states']]);cmd=np.array([q['command'] for q in r['states']]);ey=np.array([q['ey'] for q in r['states']]);v=np.array([q['v'] for q in r['states']]);m=(t>=4)&(prog>=lo)&(prog<=hi);hp=sosfiltfilt(butter(2,.3,fs=10,btype='high',output='sos'),np.degrees(cmd));sign=np.sign(cmd[m][abs(cmd[m])>np.radians(1)])
   row[k]=dict(correct=branch_result(r),outcome=r['outcome'],samples=int(sum(m)),sat_fraction=float(np.mean(abs(cmd[m])>=.298)),steering_highpass_rms_deg=float(np.sqrt(np.mean(hp[m]**2))),local_lateral_p95_m=float(np.quantile(abs(ey[m]),.95)),speed_median_mps=float(np.median(v[m])),sign_changes_above_1deg=int(np.sum(np.diff(sign)!=0)))
  pairs.append(row)
for k,v in summary.items():print(k,v)
for row in pairs:
 if row['group']=='synthetic':print('METRICS',row)
(h/'simulation_summary.json').write_text(json.dumps(summary,indent=2));(h/'simulation_pairs.json').write_text(json.dumps(pairs,indent=2))
