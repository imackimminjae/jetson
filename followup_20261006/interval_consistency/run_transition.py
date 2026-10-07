from pathlib import Path
import subprocess,json,os
here=Path(__file__).resolve().parent
env=dict(os.environ,ROS_DOMAIN_ID='177',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(here/'ros_logs'))
for group in ['synthetic','replica','ysynthetic','robust']:
    d=json.loads((here.parent/'verify'/f'input_{group}_prod.json').read_text())
    d['configs']=[dict(name='delay_near_tightening',interval_mode=4)];d['live_extra_configs']=[]
    (here/f'input_{group}_transition.json').write_text(json.dumps(d))
    with (here/f'sim_{group}_transition.log').open('w') as log:
        subprocess.run([str(here/'sim'),str(here/f'input_{group}_transition.json'),str(here/f'results_{group}_transition.jsonl')],env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
    print('Completed transition',group,flush=True)
