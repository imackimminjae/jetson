from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import json,numpy as np
h=Path(__file__).resolve().parent;old=h.parent/'reference_B_separation'
clock=lambda t:datetime.fromtimestamp(t,ZoneInfo('Asia/Seoul')).strftime('%H:%M:%S.%f')[:-3]
def choice(r,tag):
 if not r['valid']:return 'failed'
 p=np.array(r['planned'][-1])
 if tag in ['050627','050536','055105']:
  q=p-[121,-7]
  if np.linalg.norm(q)<8 or q[1]<-4:return 'before'
  return 'E' if np.degrees(np.arctan2(q[1],q[0]))<42 else 'N'
 return 'NE' if p[0]>260 and p[1]>145 else ('E' if p[0]>265 and p[1]<140 else 'other')
def stats(rs):
 v=[]
 for r in rs:
  if r['valid'] and len(r['planned'])>2:v.append(max(np.linalg.norm(np.array(r['planned'])[1:3]-np.array(r['reference'])[1:3],axis=1)))
 return {'failures':sum(not r['valid'] for r in rs),'near_correction_p95_m':float(np.quantile(v,.95)) if v else None,'cycles':len(rs)}
summary={};details={};validation={}
for tag in ['050627','050536','055105','014143','014303','045314','044738']:
 C=json.loads((old/f'replay_{tag}.json').read_text())['cycles']
 for cfg in ['recorded','current']:
  for seed in ['recorded','fresh']:
   D={str(n):json.loads((h/f'result_{tag}_{cfg}_h{n}_{seed}.json').read_text())['carried'] for n in [7,6]}
   assert all(len(r['reference'])<=7 for r in D['6'])
   key=f'{tag}/{cfg}/{seed}';summary[key]={k:stats(rs) for k,rs in D.items()};rows=[]
   for i,c in enumerate(C):
    focus=(110<c['x']<132 and -22<c['y']<6) if tag in ['050627','050536','055105'] else (241<c['x']<263 and 130<c['y']<155)
    if not focus:continue
    row=dict(time=clock(c['t']),xy=[c['x'],c['y']],record_end=c['rec_planned'][-1] if c['rec_valid'] else None)
    for k,rs in D.items():
     r=rs[i];row[k]=dict(choice=choice(r,tag),end=r.get('planned',[[None,None]])[-1],turn_steps=r['turn_steps'],B=r['B'],selected=r['selected'],counts=r['counts'])
    rows.append(row)
   details[key]=rows;summary[key]['correct_to_wrong']=[r['time'] for r in rows if r['7']['choice'] in ['E' if tag.startswith('05') else 'NE'] and r['6']['choice'] in ['N' if tag.startswith('05') else 'E']]
   if seed=='recorded':
    basepath=old/f'candidate_{tag}_current.json' if cfg=='current' else old/f'diagnostic_{tag}_recorded.json'
    if basepath.exists():base=json.loads(basepath.read_text())['baseline' if cfg=='current' else 'legacy_carried']
    else:base=json.loads((h.parent/'soft_cap12_trial'/f'result_{tag}_recorded.json').read_text())['0_0.000000']
    errors=[]
    for a,b in zip(D['7'],base):
     assert a['valid']==b['valid']
     if a['valid']:errors.append(float(max(np.linalg.norm(np.array(a['planned'])-b['planned'],axis=1))))
    validation[key]=max(errors,default=0);assert validation[key]<1e-10
   print(key,'fails',summary[key]['7']['failures'],summary[key]['6']['failures'],'regress',summary[key]['correct_to_wrong'])
(h/'summary.json').write_text(json.dumps(summary,indent=2));(h/'decisions.json').write_text(json.dumps(details,indent=2));(h/'validation.json').write_text(json.dumps(validation,indent=2))
for key,rows in details.items():
 if key.endswith('/recorded') and key.startswith(('050627','055105','014303')):
  print(key)
  for r in rows:print(r['time'],{k:(r[k]['choice'],r[k]['end'],r[k]['turn_steps']) for k in ['7','6']})
