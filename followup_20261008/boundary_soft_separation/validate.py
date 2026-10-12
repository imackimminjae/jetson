from pathlib import Path
import json,gzip,numpy as np,hashlib,subprocess
H=Path(__file__).resolve().parent;R=H.parents[1]
load=lambda p:json.loads(gzip.decompress(p.read_bytes())) if p.suffix=='.gz' else json.loads(p.read_text())
summary={};max_violation=0.;worst=None;candidate_max=0.;candidate_worst=None;total_solves=0
for which in ['batch','history']:
 for a in load(H/f'manifest_{which}.json'):
  tag=a['tag'];cs=load(Path(a['input']))['cycles'];base=load(H/f'result_{tag}.json.gz');refine=load(H/f'refine_{tag}.json.gz');source=load(H/f'source_{tag}.json.gz');groups={**base,**refine,**source};summary[tag]={}
  for name,zs in groups.items():
   assert len(zs)==len(cs);total_solves+=len(zs);errs=[];changes=[];first_change=None
   for j,(c,z,b) in enumerate(zip(cs,zs,base['base_carried'])):
    changed=z['valid']!=b['valid'] or len(z.get('planned',[]))!=len(b.get('planned',[]))
    if z['valid'] and b['valid'] and len(z['planned'])==len(b['planned']):
     err=float(np.max(np.linalg.norm(np.array(z['planned'])-np.array(b['planned']),axis=1)));errs.append(err);changed|=err>1e-7
    if changed and first_change is None:first_change=j
    if not z['valid']:continue
    for k in range(1,len(z['published'])):
     cand=np.array(z['cands'][k][z['selected'][k]]).reshape(2,2);ang=z['psirk'][k]+c['yaw'];u=np.array([-np.sin(ang),np.cos(ang)]);lo,hi=sorted(cand@u);pos=np.array(z['published'][k])@u;violation=max(float(lo-pos),float(pos-hi))
     if violation>max_violation:max_violation=violation;worst=dict(tag=tag,mode=name,index=j,k=k,violation_m=violation)
     if name=='candidate' and violation>candidate_max:candidate_max=violation;candidate_worst=dict(tag=tag,index=j,k=k)
   active=[z for c,z in zip(cs,zs) if c.get('running',True)]
   summary[tag][name]=dict(n=len(active),fail=sum(not z['valid'] for z in active),first_change_vs_base_carried=first_change,max_plan_difference_vs_base_carried_m=max(errs,default=0))
assert max_violation<1e-3,(max_violation,worst)
for p in H.glob('current_*.json.gz'):total_solves+=sum(len(zs) for zs in load(p).values())
total_solves+=sum(len(zs) for zs in load(H/'crossover_result.json').values())
# Operational content and prior work are read-only during the trial.
hashes=load(H/'baseline_sha256.json');changed=[p for p,sha in hashes.items() if not (R/p).exists() or hashlib.sha256((R/p).read_bytes()).hexdigest()!=sha];assert not changed,changed
input_hashes={}
for which in ['batch','history']:
 for a in load(H/f'manifest_{which}.json'):
  for field in ['input','grids','config']:
   p=Path(a[field]);input_hashes[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
(H/'reused_inputs_sha256.json').write_text(json.dumps(input_hashes,indent=2));(H/'final_summary.json').write_text(json.dumps(summary,indent=2))
v=dict(operational_files_unchanged=len(hashes),changed_files=changed,upper_solve_calls=total_solves,max_published_constraint_violation_m=max_violation,worst_constraint=worst,candidate_max_published_constraint_violation_m=candidate_max,candidate_worst_constraint=candidate_worst,compressed_map_bytes=(H/'batch_grids.bin.gz').stat().st_size,source_validation=load(H/'candidate_source_validation.json'))
(H/'final_validation.json').write_text(json.dumps(v,indent=2));(H/'git_status_after.txt').write_text(subprocess.check_output(['git','status','--short'],cwd=R,text=True));print({k:v for k,v in v.items() if k!='source_validation'})
