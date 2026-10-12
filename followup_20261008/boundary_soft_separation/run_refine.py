from pathlib import Path
import json,gzip,os,subprocess
H=Path(__file__).resolve().parent
env=dict(os.environ,ROS_DOMAIN_ID='177',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(H/'ros_logs'))
specs=[dict(name=policy+'_'+mode,edge_policy=policy,fixed_ref=mode=='fixed',fixed_B=mode=='fixed',fixed_wp=True,guard=True) for policy in ['gated_skip','clamp'] for mode in ['fixed','carried']]
sp=H/'specs_refine.json';sp.write_text(json.dumps(specs,indent=2))
for which in ['batch','history']:
 for a in json.loads((H/f'manifest_{which}.json').read_text()):
  g=Path(a['grids']);data=gzip.decompress(g.read_bytes()) if g.suffix=='.gz' else None;out=H/f'refine_{a["tag"]}.json'
  with (H/f'refine_{a["tag"]}.log').open('w') as log:
   subprocess.run([str(H/'refine_replay'),a['input'],'/dev/stdin' if data is not None else str(g),a['config'],str(out),str(sp)],input=data,env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
  with gzip.open(str(out)+'.gz','wb') as f:f.write(out.read_bytes())
  out.unlink();print(a['tag'],flush=True)
