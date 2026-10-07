from pathlib import Path
import subprocess,os
here=Path(__file__).resolve().parent
env=dict(os.environ,ROS_DOMAIN_ID='177',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(here/'ros_logs'))
for tag in ['050536','050627','014143','014303','045314','044738']:
 for config in ['recorded','current']:
  cfg=here/f'config_{tag if config=="recorded" else "current"}.yaml'
  with (here/f'replay_{tag}_{config}.log').open('w') as log:
   subprocess.run([str(here/'replay'),str(here/f'replay_{tag}.json'),str(here/f'replay_{tag}_grids.bin'),str(cfg),str(here/f'result_{tag}_{config}.json')],env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
  print('Replayed',tag,config,flush=True)
