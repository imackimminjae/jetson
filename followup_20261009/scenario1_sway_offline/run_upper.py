from pathlib import Path
import json,gzip,subprocess,os
import numpy as np
H=Path(__file__).resolve().parent;R=H.parents[1]
os.sched_setaffinity(0,{min(os.sched_getaffinity(0))})
peer=json.loads((H/'input_5.json').read_text())['cycles']
peer=[c for c in peer if -32<c['y']<-10 and c['x']<121]
for n in range(1,11):
    p=H/f'input_{n}.json';data=json.loads(p.read_text())
    for c in data['cycles']:
        c['peer_B']=float(np.interp(c['y'],[z['y'] for z in peer],[z['rec_B'] for z in peer])) if -26<=c['y']<=-18 and c['x']<120 else c['rec_B']
    p.write_text(json.dumps(data))
base=dict(fixed_ref=True,fixed_B=True,fixed_wp=True,guard=True,dense_output=True)
specs=[dict(base,name='fixed'),dict(base,name='peer_B',peer_B=True),dict(base,name='B7_4',force_B=7.4),dict(base,name='B14',force_B=14.0),dict(base,name='soft_k1',soft_mask=2),dict(base,name='soft_k123',soft_mask=14),dict(base,name='soft_all',soft_mask=126)]
(H/'specs.json').write_text(json.dumps(specs,indent=2))
env=dict(os.environ,ROS_DOMAIN_ID='177',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(H/'ros_logs'),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
for n in range(1,11):
    out=H/f'upper_{n}.json'
    with (H/f'upper_{n}.log').open('w') as log:
        subprocess.run([str(H/'upper_replay'),str(H/f'input_{n}.json'),'/dev/stdin',str(H/'config.yaml'),str(out),str(H/'specs.json')],input=gzip.decompress((H/'grids.bin.gz').read_bytes()),env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
    with gzip.open(str(out)+'.gz','wb') as f:f.write(out.read_bytes())
    out.unlink();print('upper',n,flush=True)
