from pathlib import Path
import os,subprocess
H=Path(__file__).resolve().parent
env=os.environ.copy()
env.update(ROS_DOMAIN_ID='177',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(H/'ros_logs'))
for tag in ['032453','050627','165712','184813','184905','185025','185147']:
 old=tag in ['032453','050627','165712']
 p=H.parent/('front_clip_trial' if old else 'origin295_drive_review')
 with (H/f'production_result_{tag}.log').open('w') as log:
  subprocess.run([str(H/'production_replay'),str(p/f'input_{tag}.json'),str(p/f'grids_{tag}.bin'),str(p/f'config_{tag}.yaml'),str(H/f'production_result_{tag}.json'),str(H/f'production_specs_{0 if old else 2.95}.json')],env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
 print('Finished',tag,flush=True)
