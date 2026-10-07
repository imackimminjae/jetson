from pathlib import Path
import os
import subprocess

here = Path(__file__).resolve().parent
root = here.parents[1]
env = dict(os.environ, ROS_DOMAIN_ID='177', ROS_LOCALHOST_ONLY='1', ROS_LOG_DIR=str(here / 'ros_logs'))
config = root / 'src/virtual_control/config/tracking_control_split.yaml'
for tag in ('S1', 'S2', 'R3', 'A'):
    folder = here.parent / ('sc3_0141_0143' if tag.startswith('S') else 'verify')
    with (here / f'replay_{tag}.log').open('w') as log:
        subprocess.run([str(here / 'replay'), str(folder / f'replay_{tag}.json'),
                        str(folder / f'replay_{tag}_grids.bin'), str(config),
                        str(here / f'replay_{tag}_out.json')], env=env, check=True,
                       stdout=log, stderr=subprocess.STDOUT)
    print('Replayed', tag, flush=True)
