from pathlib import Path
import subprocess,os
here=Path(__file__).resolve().parent;root=here.parents[1]
env=dict(os.environ,ROS_DOMAIN_ID='177',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(here/'ros_logs'))
for tag in ['S2','S1','R3','A']:
    folder=here.parent/('sc3_0141_0143' if tag.startswith('S') else 'verify')
    with (here/f'probe_{tag}.log').open('w') as log:
        subprocess.run([str(here/'probe'),str(folder/f'replay_{tag}.json'),str(folder/f'replay_{tag}_grids.bin'),
            str(root/'src/virtual_control/config/tracking_control_split.yaml'),str(here/f'probe_{tag}.json')],
            env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
    print('Completed independent-reference probes:',tag,flush=True)
