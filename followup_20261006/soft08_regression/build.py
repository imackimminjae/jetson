# Build sim_rate from the CURRENT production tracking_control.cpp (unmodified) + harness copy.
from pathlib import Path
import shlex,subprocess
p=Path(__file__).resolve().parent;root=Path('/home/imac/ros2_ws')
flags=[]
for line in (root/'build/virtual_control/CMakeFiles/lower_tracking_mpc_node.dir/flags.make').read_text().splitlines():
    if line.startswith(('CXX_DEFINES =','CXX_INCLUDES =','CXX_FLAGS =')):flags+=shlex.split(line.split('=',1)[1])
flags=[x for x in flags if x!='-O3']+['-O2']
link=shlex.split((root/'build/virtual_control/CMakeFiles/upper_planner_node.dir/link.txt').read_text());libs=link[link.index('upper_planner_node')+1:]
with (p/'build.log').open('w') as log:
    subprocess.run(['/usr/bin/c++',*flags,'-DVIRTUAL_CONTROL_UPPER_PLANNER_NO_MAIN','-c',str(root/'src/virtual_control/src/tracking_control.cpp'),'-o',str(p/'planner_now.o')],stdout=log,stderr=subprocess.STDOUT,check=True)
    subprocess.run(['/usr/bin/c++',*flags,'-c',str(p/'sim_rate.cpp'),'-o',str(p/'sim_rate.o')],stdout=log,stderr=subprocess.STDOUT,check=True)
    subprocess.run(['/usr/bin/c++',str(p/'sim_rate.o'),str(p/'planner_now.o'),'-o',str(p/'sim_rate'),*libs],stdout=log,stderr=subprocess.STDOUT,check=True)
print('built')
