from pathlib import Path
import os,json,subprocess
H=Path(__file__).resolve().parent
env=os.environ.copy();env.update(ROS_DOMAIN_ID='177',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(H/'ros_logs'))
for tag in json.loads((H/'tags.json').read_text()):
 with (H/f'result_{tag}.log').open('w') as log:
  subprocess.run([str(H.parent/'side_boundary_tail_trial/production_replay'),str(H/f'input_{tag}.json'),str(H/f'grids_{tag}.bin'),str(H/f'config_{tag}.yaml'),str(H/f'result_{tag}.json'),str(H/'specs.json')],env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
 print('Finished',tag,flush=True)
