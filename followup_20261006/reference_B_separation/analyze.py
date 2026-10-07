from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import json,numpy as np,pickle
here=Path(__file__).resolve().parent;old=here.parent/'soft_cap12_trial'
clock=lambda t:datetime.fromtimestamp(t,ZoneInfo('Asia/Seoul')).strftime('%H:%M:%S.%f')[:-3]
def classify(r):
 if not r['valid']:return 'failed'
 q=np.array(r['planned'][-1])-np.array([121.,-7.])
 if np.linalg.norm(q)<8 or q[1]<-4:return 'before'
 return 'E' if np.degrees(np.arctan2(q[1],q[0]))<42 else 'N'
def err(a,b):
 if a['valid']!=b['valid']:return float('inf')
 if not a['valid']:return 0.
 if len(a['planned'])!=len(b['planned']):return float('inf')
 return float(np.max(np.linalg.norm(np.array(a['planned'])-b['planned'],axis=1)))
def metric(rows):
 vals=[]
 for r in rows:
  if not r['valid']:continue
  p=np.array(r['planned']);ref=np.array(r['reference'])
  if len(p)>2:vals.append(float(max(np.linalg.norm(p[1:3]-ref[1:3],axis=1))))
 return dict(failures=sum(not r['valid'] for r in rows),near_correction_p95_m=float(np.quantile(vals,.95)))
summary={}; checks={};critical={}
for tag in ['050627','050536','055105']:
 p=here/f'diagnostic_{tag}_recorded.json'
 if not p.exists():continue
 D=json.loads(p.read_text());C=json.loads((here/f'replay_{tag}.json').read_text())['cycles']
 if tag!='055105':
  previous=json.loads((old/f'result_{tag}_recorded.json').read_text())
  checks[tag]={key:max(err(a,b) for a,b in zip(D[key],previous[oldkey])) for key,oldkey in [('legacy_carried','0_0.000000'),('legacy_fixed','1_0.000000')]}
 match=[err(a,dict(valid=c['rec_valid'],planned=c['rec_planned'])) for a,c in zip(D['legacy_fixed'],C)]
 checks[tag+'/record']={'matched_1mm':sum(x<.001 for x in match),'cycles':len(match),'max_m':max(match),'valid_match':sum(a['valid']==c['rec_valid'] for a,c in zip(D['legacy_fixed'],C))}
 summary[tag+'/diagnostic']={k:metric(rows) for k,rows in D.items()}
 table=[]
 for i,c in enumerate(C):
  time=clock(c['t'])
  if tag=='050627' and '05:06:49'<=time<='05:06:53.999':
   row={'time':time}
   for k,rs in D.items():
    r=rs[i];row[k]=dict(choice=classify(r),B=r['B'],B_candidate=r['B_candidate'],candidate_step=r['B_candidate_step'],end=r.get('planned',[[None,None]])[-1],selected=r['selected'],turn_steps=r['turn_steps'])
   table.append(row)
 critical[tag]=table
 if table:
  for r in table:print(r['time'],{k:(v['choice'],round(v['B'],2)) for k,v in r.items() if k!='time'})
for p in sorted(here.glob('candidate_*_current.json')):
 D=json.loads(p.read_text());tag=p.name.split('_')[1];C=json.loads((here/f'replay_{tag}.json').read_text())['cycles'];stats={}
 for k,rs in D.items():
  st=metric(rs);st['vs_baseline_changed_cycles']=sum(err(a,b)>.001 for a,b in zip(rs,D['baseline']))
  st['E_to_N']=[]
  if tag in ['050536','050627','055105']:
   for r,b,c in zip(rs,D['baseline'],C):
    if 110<c['x']<135 and -23<c['y']<5 and classify(b)=='E' and classify(r)=='N':st['E_to_N'].append(clock(c['t']))
  stats[k]=st
 summary[tag+'/candidate']=stats
 print(tag,stats)
(here/'summary.json').write_text(json.dumps(summary,indent=2));(here/'validation.json').write_text(json.dumps(checks,indent=2));(here/'critical.json').write_text(json.dumps(critical,indent=2))
print('VALIDATION',checks)
