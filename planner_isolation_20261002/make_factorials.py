import json,itertools
from pathlib import Path
p=Path(__file__).resolve().parent;d=json.loads((p/'inputs.json').read_text());matches=json.loads((p/'grid_matches.json').read_text());jobs=[]
for i in d['selected']:
 assert matches[str(i)][0]<1e-8
 jobs.append({'id':f'pipeline_{i}','mode':'pipeline','s':i,'grid':matches[str(i)][1]})
 jobs.append({'id':f'prune_{i}','mode':'recorded','s':i,'prune_unselected_below':1.0})
for a,b in [(83,84),(84,85),(92,93),(94,95)]:
 factors=['pose','grid','previous','B']+(['wp'] if d['snapshots'][a]['event']['wp0']!=d['snapshots'][b]['event']['wp0'] else [])
 for bits in itertools.product([0,1],repeat=len(factors)):
  choose={key:(b if bit else a) for key,bit in zip(factors,bits)}
  jobs.append({'id':f'factor_{a}_{b}_'+''.join(map(str,bits)),'mode':'pipeline','s':a,'pose_s':choose['pose'],'grid':matches[str(choose['grid'])][1],'grid_frame_s':choose['grid'],'previous_s':choose['previous'],'B_s':choose['B'],'wp_s':choose.get('wp',a),'factors':factors})
# Same recorded constraints for all cycles; do not mistake this for a new closed-loop rollout.
for i in range(len(d['snapshots'])):
 for name,kw in [('turn0',{'turn_weight':0.0}),('dpsi2',{'dpsi_weight':2.0}),('dpsi5',{'dpsi_weight':5.0})]:
  jobs.append({'id':f'sweep_{name}_{i}','mode':'recorded','s':i,**kw})
d['jobs']=jobs;(p/'factorials.json').write_text(json.dumps(d));print('factorial and sweep jobs',len(jobs))
