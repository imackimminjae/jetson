from pathlib import Path
import subprocess,json,os,hashlib
h=Path(__file__).resolve().parent;old=h.parent/'reference_B_separation'
for n,v in json.loads((h/'input_source_sha256.json').read_text()).items():assert hashlib.sha256(Path(n).read_bytes()).hexdigest()==v,n
env=dict(os.environ,ROS_DOMAIN_ID='177',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(h/'ros_logs'))
for tag in ['050627','050536','055105','014143','014303','045314','044738']:
 for cfg in ['recorded','current']:
  for seed in ['recorded','fresh']:
   for horizon in [7,6]:
    name=f'{tag}_{cfg}_h{horizon}_{seed}'
    with (h/f'{name}.log').open('w') as log:
     subprocess.run([str(old/'replay'),str(h/f'input_{tag}_h{horizon}_{seed}.json'),str(old/f'replay_{tag}_grids.bin'),str(h/f'config_{tag}_{cfg}_h{horizon}.yaml'),str(h/f'result_{name}.json'),str(h/'specs.json')],env=env,check=True,stdout=log,stderr=subprocess.STDOUT)
  print('Replayed',tag,cfg,flush=True)
