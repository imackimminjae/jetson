from pathlib import Path
import json,subprocess,os
H=Path(__file__).resolve().parent
env=dict(os.environ,ROS_DOMAIN_ID='177',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(H/'ros_logs'))
for m in json.loads((H/'manifest.json').read_text()):
 tag,kind=m['tag'],m['config']
 with (H/f'result_{tag}_{kind}.log').open('w') as log:
  subprocess.run([str(H.parent/'k1_front_clip_skip_trial/replay'),str(H/f'input_{tag}.json'),str(H/f'grids_{tag}.bin'),str(H/f'config_{tag}_{kind}.yaml'),str(H/f'result_{tag}_{kind}.json'),str(H/f'specs_{tag}_{kind}.json')],env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
 print('Finished',tag,kind,flush=True)
