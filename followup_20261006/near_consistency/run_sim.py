from pathlib import Path
import json
import os
import subprocess

here=Path(__file__).resolve().parent
env=dict(os.environ,ROS_DOMAIN_ID='177',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(here/'ros_logs'))
for group in ('synthetic','replica','ysynthetic','robust'):
    data=json.loads((here.parent/'verify'/f'input_{group}_prod.json').read_text())
    data['configs']=[dict(name=f'near{w:g}',near_weight=w) for w in [0,.03,.1,.3,1.,3.]]
    data['live_extra_configs']=[]
    (here/f'input_{group}.json').write_text(json.dumps(data))
    with (here/f'sim_{group}.log').open('w') as log:
        subprocess.run([str(here/'sim'),str(here/f'input_{group}.json'),
                        str(here/f'results_{group}.jsonl')],env=env,check=True,
                       stdout=log,stderr=subprocess.STDOUT)
    print('Completed',group,flush=True)
