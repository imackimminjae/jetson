# Replay each bag with its own snapshot config, changing ONLY preview_interval_soft_ratio.
from pathlib import Path
import subprocess,os,re,sys
here=Path(__file__).resolve().parent
env=dict(os.environ,ROS_DOMAIN_ID='177',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(here/'ros_logs'))
for tag in sys.argv[1:]:
    base=(here/f'config_{tag}.yaml').read_text()
    for soft in ('0.7','0.8'):
        cfg=here/f'config_{tag}_soft{soft}.yaml'
        new,n=re.subn(r'(preview_interval_soft_ratio:\s*)[0-9.]+',r'\g<1>'+soft,base); assert n==1
        cfg.write_text(new)
        with (here/f'replay_{tag}_soft{soft}.log').open('w') as log:
            subprocess.run([str(here/'replay'),str(here/f'replay_{tag}.json'),str(here/f'replay_{tag}_grids.bin'),str(cfg),str(here/f'result_{tag}_soft{soft}.json')],env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
        print('replayed',tag,soft,flush=True)
