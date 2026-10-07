from pathlib import Path
import shlex,subprocess
p=Path(__file__).resolve().parent;root=Path('/home/imac/ros2_ws')
flags=[]
for line in (root/'build/virtual_control/CMakeFiles/lower_tracking_mpc_node.dir/flags.make').read_text().splitlines():
    if line.startswith(('CXX_DEFINES =','CXX_INCLUDES =','CXX_FLAGS =')):flags+=shlex.split(line.split('=',1)[1])
flags=[x for x in flags if x!='-O3']+['-O2']
link=shlex.split((root/'build/virtual_control/CMakeFiles/upper_planner_node.dir/link.txt').read_text());libs=link[link.index('upper_planner_node')+1:]
with (p/'build.log').open('w') as log:
    subprocess.run(['/usr/bin/c++',*flags,'-DVIRTUAL_CONTROL_UPPER_PLANNER_NO_MAIN','-c',str(p/'tracking_control_variant.cpp'),'-o',str(p/'planner_variant.o')],stdout=log,stderr=subprocess.STDOUT,check=True)
    subprocess.run(['/usr/bin/c++',*flags,'-c',str(p/'sim_branch.cpp'),'-o',str(p/'sim_branch.o')],stdout=log,stderr=subprocess.STDOUT,check=True)
    subprocess.run(['/usr/bin/c++',str(p/'sim_branch.o'),str(p/'planner_variant.o'),'-o',str(p/'sim_branch'),*libs],stdout=log,stderr=subprocess.STDOUT,check=True)
print('built')
