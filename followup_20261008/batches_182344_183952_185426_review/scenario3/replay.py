from pathlib import Path
import json,gzip,subprocess,os,hashlib
H=Path(__file__).resolve().parent;R=Path('/home/imac/ros2_ws');P=R/'followup_20261008/boundary_soft_separation'
os.sched_setaffinity(0,{min(os.sched_getaffinity(0))})
baseline=json.loads((P/'baseline_sha256.json').read_text())
for rel in ['src/virtual_control/src/tracking_control.cpp','src/virtual_control/include/virtual_control/tracking_control.hpp','src/virtual_control/include/virtual_control/egocentric_planner_geometry.hpp']:
 assert hashlib.sha256((R/rel).read_bytes()).hexdigest()==baseline[rel]
specs=[dict(name='fixed',fixed_ref=True,fixed_B=True,fixed_wp=True,guard=True),dict(name='natural_B',fixed_ref=True,fixed_wp=True,guard=True),dict(name='hold_B',fixed_ref=True,fixed_wp=True,guard=True,hold_update=True),dict(name='guard_off',fixed_ref=True,fixed_B=True,fixed_wp=True,guard=False),dict(name='raw_diagnostic',fixed_ref=True,fixed_B=True,fixed_wp=True,guard=True,raw_corridors=True)]
sp=H/'specs.json';sp.write_text(json.dumps(specs,indent=2));env=dict(os.environ,ROS_DOMAIN_ID='177',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(H/'ros_logs'),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
for a in json.loads((H/'attempts.json').read_text()):
 out=H/f'result_{a["attempt"]}.json'
 with (H/f'replay_{a["attempt"]}.log').open('w') as log:subprocess.run([str(P/'replay'),a['input'],'/dev/stdin',str(H/'config.yaml'),str(out),str(sp)],input=gzip.decompress((H/'grids.bin.gz').read_bytes()),env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
 with gzip.open(str(out)+'.gz','wb',compresslevel=6) as f:f.write(out.read_bytes())
 out.unlink();print('replayed',a['attempt'],flush=True)
