from pathlib import Path
import subprocess,os,sys
here=Path(__file__).resolve().parent
env=dict(os.environ,ROS_DOMAIN_ID='177',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(here/'ros_logs'))
mode=sys.argv[1]
if mode=='diagnostic':jobs=[(t,'recorded','diagnostic') for t in ['050627','050536','055105']]
else:jobs=[(t,'current','candidate') for t in ['050627','050536','055105','014143','014303','045314','044738']]
for tag,cfg,spec in jobs:
 conf=here/f'config_{tag if cfg=="recorded" else "current"}.yaml'
 with (here/f'{spec}_{tag}_{cfg}.log').open('w') as log:
  subprocess.run([str(here/'replay'),str(here/f'replay_{tag}.json'),str(here/f'replay_{tag}_grids.bin'),str(conf),str(here/f'{spec}_{tag}_{cfg}.json'),str(here/f'{spec}_specs.json')],env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
 print(spec,tag,cfg,flush=True)
