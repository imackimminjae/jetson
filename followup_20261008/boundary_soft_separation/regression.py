from pathlib import Path
import json,gzip,csv
H=Path(__file__).resolve().parent
load=lambda p:json.loads(gzip.decompress(p.read_bytes())) if p.suffix=='.gz' else json.loads(p.read_text())
rows=[];check=[]
for kind in ['batch','history']:
 for a in load(H/f'manifest_{kind}.json'):
  tag=a['tag'];inp=load(Path(a['input']));cs=inp['cycles'];r=load(H/f'result_{tag}.json.gz');s=load(H/f'source_{tag}.json.gz')['candidate']
  rows.append(dict(tag=tag,scope='recorded_config',cycles=sum(c.get('running',True) for c in cs),base_fail=sum(not z['valid'] for c,z in zip(cs,r['base_carried']) if c.get('running',True)),candidate_fail=sum(not z['valid'] for c,z in zip(cs,s) if c.get('running',True))))
  if kind=='history':
   cur=load(H/f'current_{tag}.json.gz');rows.append(dict(tag=tag,scope='current_config_carried',cycles=len(cs),base_fail=sum(not z['valid'] for z in cur['base_carried']),candidate_fail=sum(not z['valid'] for z in cur['candidate_carried'])))
  sc3=max(p[1] for p in inp['route'])>100
  for j,c in enumerate(cs):
   region=(225<c['x']<275 and 125<c['y']<160) if sc3 else (100<c['x']<140)
   if region:check.append(dict(tag=tag,index=j,xy=[c['x'],c['y']],t=c['t'],recorded=c['rec_planned'][-1:],base=r['base_carried'][j].get('planned',[])[-1:],candidate=s[j].get('planned',[])[-1:],base_counts=r['base_carried'][j]['counts'],candidate_counts=s[j]['counts']))
with (H/'regression.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
(H/'branch_regression.json').write_text(json.dumps(check,indent=2));print('regression rows',len(rows),'branch checkpoints',len(check))
