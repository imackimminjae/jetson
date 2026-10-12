from pathlib import Path
import json,subprocess,shlex,os
H=Path(__file__).resolve().parent;R=H.parents[1];T=H.parent/'side_boundary_tail_trial'
s=(T/'replay.cpp').read_text().replace('   n.applyTerminalPositionTargets(pc,','   if(spec.value("force_terminal_lane_omission",false))pc.side_boundary_tail_start=pc.N_pred;\n   n.applyTerminalPositionTargets(pc,')
(H/'diagnostic_replay.cpp').write_text(s);flags=[]
for line in (R/'build/virtual_control/CMakeFiles/upper_planner_node.dir/flags.make').read_text().splitlines():
 if line.startswith(('CXX_DEFINES =','CXX_INCLUDES =','CXX_FLAGS =')):flags+=shlex.split(line.split('=',1)[1])
flags=[f for f in flags if f!='-O3']+['-O1'];link=shlex.split((R/'build/virtual_control/CMakeFiles/upper_planner_node.dir/link.txt').read_text());libs=link[link.index('upper_planner_node')+1:]
with (H/'diagnostic_build.log').open('w') as log:
 subprocess.run(['/usr/bin/c++',*flags,'-c',str(H/'diagnostic_replay.cpp'),'-o',str(H/'diagnostic_replay.o')],check=True,stdout=log,stderr=subprocess.STDOUT)
 subprocess.run(['/usr/bin/c++',str(H/'diagnostic_replay.o'),str(R/'build/virtual_control/CMakeFiles/test_lower_path_generation.dir/src/tracking_control.cpp.o'),'-o',str(H/'diagnostic_replay'),*libs],check=True,stdout=log,stderr=subprocess.STDOUT)
tag='20261008_052329';data=json.loads((H/f'input_{tag}.json').read_text());data['cycles']=[data['cycles'][36]];(H/'first_wrong.json').write_text(json.dumps(data));(H/'diagnostic_specs.json').write_text(json.dumps([dict(name='current',legacy=1,guard=True),dict(name='force_terminal',legacy=1,guard=True,force_terminal_lane_omission=True)]))
env=os.environ.copy();env.update(ROS_DOMAIN_ID='177',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(H/'ros_logs'))
with (H/'diagnostic_run.log').open('w') as log:subprocess.run([str(H/'diagnostic_replay'),str(H/'first_wrong.json'),str(H/f'grids_{tag}.bin'),str(H/f'config_{tag}.yaml'),str(H/'diagnostic_result.json'),str(H/'diagnostic_specs.json')],env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
for k,v in json.loads((H/'diagnostic_result.json').read_text()).items():print(k,v[0]['valid'],v[0].get('planned',[])[-1:],v[0]['published'][-1:])
