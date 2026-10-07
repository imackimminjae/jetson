from pathlib import Path
import subprocess,os,json
here=Path(__file__).resolve().parent;root=here.parents[1]
env=dict(os.environ,ROS_DOMAIN_ID='177',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(here/'ros_logs'))
for config in json.loads((here/'configs.json').read_text()):
    name=config['name']
    for tag in ['S1','S2','R3','A']:
        folder=here.parent/('sc3_0141_0143' if tag.startswith('S') else 'verify')
        with (here/f'replay_{name}_{tag}.log').open('w') as log:
            subprocess.run([str(here/'replay'),str(folder/f'replay_{tag}.json'),str(folder/f'replay_{tag}_grids.bin'),
                str(here/(name+'.yaml')),str(here/f'replay_{name}_{tag}.json')],env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
    print('Replayed',name,flush=True)
