from pathlib import Path
import os,json,subprocess
here=Path(__file__).resolve().parent
env=dict(os.environ,ROS_DOMAIN_ID='177',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(here/'ros_logs'))
for group in ['synthetic','replica','ysynthetic','robust']:
    d=json.loads((here.parent/'verify'/f'input_{group}_prod.json').read_text())
    d['configs']=[dict(name=name,interval_mode=i) for i,name in [(0,'baseline'),(1,'continuous_min'),(2,'fixed_margin')]]
    d['live_extra_configs']=[]
    (here/f'input_{group}.json').write_text(json.dumps(d))
    with (here/f'sim_{group}.log').open('w') as log:
        subprocess.run([str(here/'sim'),str(here/f'input_{group}.json'),str(here/f'results_{group}.jsonl')],env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
    print('Completed',group,flush=True)
