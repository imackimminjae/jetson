from pathlib import Path
import json,numpy as np,hashlib
H=Path(__file__).resolve().parent;R=H.parents[1];out={};max_lane=0.;total=0;matches=0;valid_matches=0;valid_total=0
for tag in json.loads((H/'tags.json').read_text()):
 data=json.loads((H/f'result_{tag}.json').read_text());cs=json.loads((H/f'input_{tag}.json').read_text())['cycles'];row={};errors=[]
 for c,z in zip(cs,data['fixed_off']):
  valid_matches+=z['valid']==c['rec_valid'];total+=1
  if z['valid'] and c['rec_valid']:
   err=float(np.max(np.linalg.norm(np.array(z['planned'])-np.array(c['rec_planned']),axis=1)));errors.append(err);matches+=err<.001;valid_total+=1
 row['fixed_baseline_max_error_m']=max(errors,default=0);row['fixed_baseline_within_1mm']=sum(x<.001 for x in errors)
 for mode,rs in data.items():
  hits=[]
  for i,(c,z) in enumerate(zip(cs,rs)):
   if z['side_boundary_tail_start']>=0:hits.append(dict(i=i,valid=z['valid'],tail=z['side_boundary_tail_start'],published_end=z['published'][-1:] ))
   if not z['valid']:continue
   start=z['side_boundary_tail_start'];last=len(z['reference'])-1 if start<0 else start-1
   assert len(z['published'])==last+1
   for k in range(1,last+1):
    q=np.array(z['cands'][k][z['selected'][k]]).reshape(2,2);angle=z['psirk'][k]+c['yaw'];axis=np.array([-np.sin(angle),np.cos(angle)]);lo,hi=sorted(q@axis);proj=axis@z['planned'][k];max_lane=max(max_lane,float(lo-proj),float(proj-hi))
  row[mode]=dict(valid=sum(z['valid'] for z in rs),hits=hits)
 for a,b in [('fixed_off','fixed_on'),('carried_off','carried_on')]:
  row[a+'_vs_'+b+'_changed']=[i for i,(x,y) in enumerate(zip(data[a],data[b])) if x['valid']!=y['valid'] or x.get('planned')!=y.get('planned')]
 out[tag]=row
out['validation']=dict(cycles=total,solves=total*4,validity_matches=valid_matches,baseline_valid=valid_total,baseline_valid_within_1mm=matches,max_published_lane_violation_m=max_lane)
assert max_lane<.001
for name,sha in json.loads((H/'baseline_sha256.json').read_text()).items():assert hashlib.sha256((R/name).read_bytes()).hexdigest()==sha,name
for name,sha in json.loads((H/'inputs_sha256.json').read_text()).items():assert hashlib.sha256(Path(name).read_bytes()).hexdigest()==sha,name
(H/'summary.json').write_text(json.dumps(out,indent=2));print(json.dumps(out['validation'],indent=2))
w=json.loads((H/'result_s1_window.json').read_text());print('anchored s1')
for m,rs in w.items():print(m,'hits',[(i+57,r['side_boundary_tail_start']) for i,r in enumerate(rs) if r['side_boundary_tail_start']>=0],'event59',rs[2].get('planned',[])[-1:])
