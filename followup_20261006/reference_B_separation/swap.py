"""Single-cycle 2x2 reference / updated-B crossovers, both with/without cap.
Inputs come from the carried baseline and cap trajectories at exactly the same pose/BEV/time.
"""
from pathlib import Path
import json,copy,os,subprocess
here=Path(__file__).resolve().parent
D=json.loads((here/'diagnostic_050627_recorded.json').read_text());inp=json.loads((here/'replay_050627.json').read_text())
spec=[dict(name='cap0',cap=0,fixed_ref=True,fixed_B=True,fixed_wp=True),dict(name='cap12',cap=12,fixed_ref=True,fixed_B=True,fixed_wp=True)]
(here/'swap_specs.json').write_text(json.dumps(spec))
env=dict(os.environ,ROS_DOMAIN_ID='177',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(here/'ros_logs'))
from datetime import datetime
from zoneinfo import ZoneInfo
cases=[];keys=[]
for i,c in enumerate(inp['cycles']):
 ts=datetime.fromtimestamp(c['t'],ZoneInfo('Asia/Seoul')).strftime('%H:%M:%S.%f')[:-3]
 if not '05:06:49'<=ts<='05:06:52.999':continue
 for r in [0,12]:
  for b in [0,12]:
   cc=copy.deepcopy(c);rr=D[f'cap{r}_ref0_B0'][i];bb=D[f'cap{b}_ref0_B0'][i]
   cc['rec_preview']=rr['reference'];cc['rec_B_before']=bb['B_before'];cc['rec_B']=bb['B']
   cases.append(cc);keys.append(dict(time=ts,reference_source=r,B_source=b))
(here/'swap_input.json').write_text(json.dumps(dict(route=inp['route'],cycles=cases)))
with (here/'swap.log').open('w') as log:
 subprocess.run([str(here/'replay'),str(here/'swap_input.json'),str(here/'replay_050627_grids.bin'),str(here/'config_050627.yaml'),str(here/'swap_result.json'),str(here/'swap_specs.json')],env=env,check=True,stdout=log,stderr=subprocess.STDOUT)
(here/'swap_keys.json').write_text(json.dumps(keys,indent=2))
print('Completed',len(cases)*2,'single-cycle crossovers')
