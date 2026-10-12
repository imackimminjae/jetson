from pathlib import Path
import os,json,gzip,ast,subprocess,re
os.sched_setaffinity(0,{min(os.sched_getaffinity(0))})
import numpy as np
from scipy.optimize import linprog
import itertools
H=Path(__file__).resolve().parent;P=H.parent/'boundary_soft_separation';R=H.parents[1]
src=ast.parse((P/'feasibility.py').read_text());exec(compile(ast.Module(body=[n for n in src.body if isinstance(n,ast.FunctionDef)],type_ignores=[]),'lp','exec'))
c=json.loads((H/'input_7.json').read_text());r=json.loads(gzip.decompress((H/'result_7.json.gz').read_bytes()));out=[]
for j in [48,49,50,51]:
 for mode in ['fixed','raw_diagnostic']:
  z=r[mode][j];tests=run(z,c['cycles'][j]);out.append(dict(index=j,mode=mode,selected=z['selected'],tests=tests))
(H/'branch_feasibility.json').write_text(json.dumps(out,indent=2))
sp=[dict(name='B_6p1',fixed_ref=True,fixed_wp=True,guard=True,force_B=6.1),dict(name='B_6p1_raw',fixed_ref=True,fixed_wp=True,guard=True,force_B=6.1,raw_corridors=True)]
(H/'forced_B_specs.json').write_text(json.dumps(sp));env=dict(os.environ,ROS_DOMAIN_ID='177',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(H/'ros_logs'))
with (H/'forced_B.log').open('w') as f:subprocess.run([str(P/'replay'),str(H/'input_7.json'),'/dev/stdin',str(H/'config.yaml'),str(H/'forced_B.json'),str(H/'forced_B_specs.json')],input=gzip.decompress((H/'grids.bin.gz').read_bytes()),env=env,stdout=f,stderr=f,check=True)
q=json.loads((H/'forced_B.json').read_text());brief=[]
for j in [48,49,50,51]:
 brief.append(dict(index=j,modes={m:dict(selected=z[j]['selected'],end=z[j]['planned'][-1:] if z[j]['planned'] else [],valid=z[j]['valid']) for m,z in {**r,**q}.items()}))
(H/'branch_comparison.json').write_text(json.dumps(brief,indent=2))
with gzip.open(H/'forced_B.json.gz','wb') as f:f.write((H/'forced_B.json').read_bytes())
(H/'forced_B.json').unlink()
d=json.loads(gzip.decompress((H/'telemetry.json.gz').read_bytes()));tt=np.array(d['receive_times']['/bev/occupancy_grid']);rot=[]
for s in re.findall(r'\[(179\d+\.\d+)\].*Writing remaining', (R/'drive_batches/20261008_171715_scenario2_592ff2/recorder.log').read_text()):
 t=float(s);v=tt[(tt>t-3)&(tt<t+4)];rot.append(dict(t=t,max_grid_gap=float(np.diff(v).max()) if len(v)>1 else None))
(H/'rotation_gaps.json').write_text(json.dumps(rot,indent=2))
print(json.dumps(out));print(json.dumps(brief));print(rot)
