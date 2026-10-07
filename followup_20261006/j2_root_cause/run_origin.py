from pathlib import Path
import subprocess,os,sys
here=Path(__file__).resolve().parent
for tag in sys.argv[1:]:
    for ox in os.environ.get('OXS','0,2.95').split(','):
        env=dict(os.environ,ROS_DOMAIN_ID='177',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(here/'ros_logs'),GRID_OX=ox)
        with (here/f'replay_{tag}_ox{ox}.log').open('w') as log:
            subprocess.run(['nice','-n','19',str(here/'replay'),str(here/f'replay_{tag}.json'),str(here/f'replay_{tag}_grids.bin'),str(here/f'config_{tag}.yaml'),str(here/f'result_{tag}_ox{ox}.json')],env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
    print('replayed',tag,flush=True)
