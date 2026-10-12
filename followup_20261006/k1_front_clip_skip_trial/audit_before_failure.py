"""Check whether removing clipped k1 constraints changes plans BEFORE failure.
This is a same-state planning counterfactual, not a vehicle counterfactual.
"""
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import json,numpy as np
H=Path(__file__).resolve().parent
out=[]
for tag in ['184905','185025','185147']:
 cs=json.loads((H.parent/'origin295_drive_review'/f'input_{tag}.json').read_text())['cycles']
 ds=json.loads((H/f'result_{tag}.json').read_text())
 fail=next(i for i,c in enumerate(cs) if not c['rec_valid'])
 row=dict(tag=tag,first_failure_index=fail,first_failure_time=cs[fail]['t'],tests={})
 for prefix in ['fixed','carried']:
  changes=[];max_pre_delta=0.;validity_changes=[]
  for i in range(fail):
   a,b=ds[prefix+'_m0'][i],ds[prefix+'_m1'][i]
   if a['valid']!=b['valid']:validity_changes.append(i)
   if not a['valid'] or not b['valid']:continue
   if len(a['planned'])!=len(b['planned']):
    changes.append(dict(i=i,dimension_change=True));continue
   delta=np.array(b['planned'])-a['planned']
   maxd=float(np.linalg.norm(delta,axis=1).max());max_pre_delta=max(max_pre_delta,maxd)
   if maxd<=1e-6:continue
   yaw=cs[i]['yaw'];axis=np.array([-np.sin(yaw),np.cos(yaw)])
   changes.append(dict(i=i,time=datetime.fromtimestamp(cs[i]['t'],ZoneInfo('Asia/Seoul')).isoformat(),seconds_before_failure=cs[fail]['t']-cs[i]['t'],k1_lateral_delta_m=float(delta[1]@axis),max_plan_delta_m=maxd,k1_skip_mask=b['k1_skip_mask'],recorded_yaw_deg=float(np.rad2deg(yaw))))
  row['tests'][prefix]=dict(max_pre_failure_plan_delta_m=max_pre_delta,validity_changes=validity_changes,changes=changes)
 out.append(row)
(H/'before_failure_audit.json').write_text(json.dumps(out,indent=2))
for r in out:
 print(r['tag'],[(k,v['max_pre_failure_plan_delta_m'],len(v['changes'])) for k,v in r['tests'].items()])
