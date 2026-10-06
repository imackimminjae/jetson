import pickle,json
import numpy as np
from pathlib import Path
from collections import Counter
from datetime import datetime
from zoneinfo import ZoneInfo
root=Path(__file__).resolve().parent;d=pickle.load(open(root/'decoded.pkl','rb'))
clock=lambda t:datetime.fromtimestamp(t,ZoneInfo('Asia/Seoul')).strftime('%H:%M:%S.%f')[:-3]
events=[x['data'] for x in d['/debug/upper_branch_event']];states=[x['data'] for x in d['/debug/upper_interval_centering_state']];details=[x['data'].reshape(-1,12) for x in d['/debug/upper_interval_info']]
assert len(events)==len(states)==len(details)
counts=Counter();selected=Counter();checks=Counter();cycles=[];prev=None;switches=[]
for e,s,rows in zip(events,states,details):
 assert all(abs(r[10]-s[2])<1e-8 for r in rows)
 for inter in e['intervals_world']:
  matches=[r for r in rows if int(r[0])==inter[0] and int(r[1])==inter[1]];assert len(matches)==1
  assert np.allclose(matches[0][2:4],inter[6:8])
 candidates=[]
 for k in sorted(set(rows[:,0])):
  inside=[r for r in rows if r[0]==k and r[9]==1]
  if len(inside)==1:
   r=inside[0]
   if r[8]==0 and r[5]==1 and r[6]==1 and r[2]>0:candidates.append(r)
 candidate=min(candidates,key=lambda r:r[2]) if candidates else None
 if candidate is not None:checks['candidate_mismatch']+=int(abs(candidate[2]-s[4])>1e-8)
 else:checks['candidate_mismatch']+=int(not np.isnan(s[4]))
 expectedB=s[4] if np.isfinite(s[4]) and (s[1]<=0 or s[4]<=1.5*s[1]) else s[1]
 checks['B_update_mismatch']+=int(abs(expectedB-s[2])>1e-8)
 cur={};off=[]
 for r in rows:
  k,j,raw,processed,on=r[:5];mode='ON' if on else 'OFF';counts[mode]+=1
  threshold=9 if r[8] else 1.5*s[2] if s[2]>0 else np.nan
  checks['decision_mismatch']+=int(bool(on)!=(raw<=threshold))
  expected=.4*raw if on else raw-2*min(2,.45*raw)
  checks['length_mismatch']+=int(abs(expected-processed)>1e-8)
  key=(int(k),int(j));cur[key]=bool(on)
  if not on:off.append(dict(k=int(k),raw=raw,processed=processed,extra_vs_on=processed-.4*raw))
  sel=e['selected_corridors']
  if e['valid_plan'] and 0<k<len(sel) and sel[int(k)]==j:selected[mode]+=1
  if prev and key in prev and prev[key]!=bool(on):switches.append(dict(t=e['t_start'],k=int(k),candidate=int(j),mode=mode,raw=raw,B=s[2]))
 cycles.append(dict(t=e['t_start'],clock=clock(e['t_start']),seq=e['plan_seq'],N=e['horizon'],valid=e['valid_plan'],B_before=s[1],B=s[2],candidate=s[4],candidate_step=s[5],off=off))
 prev=cur
print('events',len(events),'outcomes',Counter(e['reason'] for e in events),'N',Counter(e['horizon'] for e in events))
print('all k including k0',counts,'selected successful k>=1',selected,'checks',checks)
print('B range',min(s[2] for s in states),max(s[2] for s in states),'updates',sum(abs(s[1]-s[2])>1e-8 for s in states),'holds',sum(np.isfinite(s[4]) and s[4]>1.5*s[1] for s in states))
print('cycles withOFF',sum(bool(x['off']) for x in cycles),'switches',len(switches))
for c in cycles:
 if c['off'] or not c['valid']:print('cycle',c)
print('STALLS')
for m in d['/debug/upper_stage_timing']:
 e=m['data']
 if e['wall_ms']>100 or e['start_interval_ms']>800:print(clock(e['t_start']),e['outcome'],'wall',e['wall_ms'],'interval',e['start_interval_ms'],'stages',sorted(e['stages'],key=lambda s:s['wall_ms'],reverse=True)[:3])
lower=np.array([m['data'] for m in d['/debug/lower_mpc_trace']]);active=lower[:,10]==1
for idx,name in [(6,'laterror'),(7,'headingerror'),(9,'steernorm')]:print(name,'med/p95/max',np.quantile(abs(lower[active,idx]),[.5,.95,1]))
(root/'soft_ratio_results.json').write_text(json.dumps(dict(counts=dict(counts),selected=dict(selected),checks=dict(checks),cycles=cycles,switches=switches),indent=2))
