from pathlib import Path
import json,gzip,os,subprocess,numpy as np
H=Path(__file__).resolve().parent
env=dict(os.environ,ROS_DOMAIN_ID='177',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(H/'ros_logs'))
sp=H/'specs_source_candidate.json';sp.write_text(json.dumps([dict(name='candidate',fixed_wp=True,guard=True)]));validation=[]
for which in ['batch','history']:
 for a in json.loads((H/f'manifest_{which}.json').read_text()):
  g=Path(a['grids']);data=gzip.decompress(g.read_bytes()) if g.suffix=='.gz' else None;out=H/f'source_{a["tag"]}.json'
  with (H/f'source_{a["tag"]}.log').open('w') as log:subprocess.run([str(H/'candidate_replay'),a['input'],'/dev/stdin' if data is not None else str(g),a['config'],str(out),str(sp)],input=data,env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
  r=json.loads(out.read_text())['candidate'];q=json.loads(gzip.decompress((H/f'refine_{a["tag"]}.json.gz').read_bytes()))['gated_skip_carried'];errors=[]
  for x,y in zip(r,q):
   assert x['valid']==y['valid'] and x['counts']==y['counts'] and x['selected']==y['selected'],(a['tag'],x['t'])
   if x['valid']:errors.append(float(np.max(np.linalg.norm(np.array(x['planned'])-np.array(y['planned']),axis=1))))
  assert max(errors,default=0)<1e-8,(a['tag'],max(errors))
  validation.append(dict(tag=a['tag'],n=len(r),max_error_m=max(errors,default=0)))
  with gzip.open(str(out)+'.gz','wb') as f:f.write(out.read_bytes())
  out.unlink();print(a['tag'],'source validated',flush=True)
(H/'candidate_source_validation.json').write_text(json.dumps(validation,indent=2))
