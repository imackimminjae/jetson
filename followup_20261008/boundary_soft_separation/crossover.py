from pathlib import Path
import json,gzip,subprocess,os,copy
import numpy as np
H=Path(__file__).resolve().parent
cs=json.loads((H/'input_batch05.json').read_text());res=json.loads(gzip.decompress((H/'result_batch05.json.gz').read_bytes()));rows=[]
for j in range(46,51):
 for refname in ['base_carried','edge_carried']:
  for bname in ['base_carried','edge_carried']:
   c=copy.deepcopy(cs['cycles'][j]);c['rec_preview']=res[refname][j]['reference'];c['rec_B']=res[bname][j]['B'];c['label']=dict(index=j,ref=refname,B=bname);rows.append(c)
p=H/'crossover_input.json';p.write_text(json.dumps(dict(route=cs['route'],cycles=rows)))
sp=H/'crossover_specs.json';sp.write_text(json.dumps([dict(name='base_treatment',fixed_ref=True,fixed_B=True,fixed_wp=True,guard=True)]))
env=dict(os.environ,ROS_DOMAIN_ID='177',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(H/'ros_logs'));out=H/'crossover_result.json'
with (H/'crossover.log').open('w') as log:subprocess.run([str(H/'replay'),str(p),'/dev/stdin',str(H/'batch_config.yaml'),str(out),str(sp)],input=gzip.decompress((H/'batch_grids.bin.gz').read_bytes()),env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
r=json.loads(out.read_text())['base_treatment'];small=[]
for c,z in zip(rows,r):
 a=dict(**c['label'],t=c['t'],B_value=z['B'],end=z.get('planned',[])[-1:],valid=z['valid'],counts=z['counts']);small.append(a)
(H/'crossover_summary.json').write_text(json.dumps(small,indent=2));print(*[x for x in small if x['index']==49],sep='\n')
