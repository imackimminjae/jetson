from pathlib import Path
import subprocess,os,sys
h=Path(__file__).resolve().parent;env=dict(os.environ,ROS_DOMAIN_ID='177',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(h/'ros_logs'))
for group in sys.argv[1:]:
 with (h/f'sim_{group}.log').open('w') as log:
  subprocess.run([str(h/'sim'),str(h/f'input_{group}.json'),str(h/f'results_{group}.jsonl')],env=env,check=True,stdout=log,stderr=subprocess.STDOUT)
 print('Completed',group,flush=True)
