from pathlib import Path
import subprocess,shlex
H=Path(__file__).resolve().parent;R=H.parents[1];flags=[]
for line in (R/'build/virtual_control/CMakeFiles/upper_planner_node.dir/flags.make').read_text().splitlines():
 if line.startswith(('CXX_DEFINES =','CXX_INCLUDES =','CXX_FLAGS =')):flags+=shlex.split(line.split('=',1)[1])
flags=[f for f in flags if f!='-O3']+['-O2']
link=shlex.split((R/'build/virtual_control/CMakeFiles/upper_planner_node.dir/link.txt').read_text());libs=link[link.index('upper_planner_node')+1:]
obj=R/'build/virtual_control/CMakeFiles/test_lower_path_generation.dir/src/tracking_control.cpp.o'
with (H/'production_replay_build.log').open('w') as log:
 subprocess.run(['/usr/bin/c++',*flags,'-c',str(H/'production_replay.cpp'),'-o',str(H/'production_replay.o')],check=True,stdout=log,stderr=subprocess.STDOUT)
 subprocess.run(['/usr/bin/c++',str(H/'production_replay.o'),str(obj),'-o',str(H/'production_replay'),*libs],check=True,stdout=log,stderr=subprocess.STDOUT)
print('Built production replay')
