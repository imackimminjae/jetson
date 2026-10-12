from pathlib import Path
import shlex,subprocess
here=Path(__file__).resolve().parent;root=here.parents[1]
flags=[]
for line in (root/'build/virtual_control/CMakeFiles/lower_tracking_mpc_node.dir/flags.make').read_text().splitlines():
    if line.startswith(('CXX_DEFINES =','CXX_INCLUDES =','CXX_FLAGS =')):
        flags+=shlex.split(line.split('=',1)[1])
flags=[f for f in flags if f!='-O3']+['-O2']
link=shlex.split((root/'build/virtual_control/CMakeFiles/upper_planner_node.dir/link.txt').read_text())
libs=link[link.index('upper_planner_node')+1:]
with (here/'build.log').open('w') as log:
    for name in ['tracking_control_trial','replay']:
        source=here/f'{name}.cpp';obj=here/f'{name}.o'
        if not obj.exists() or source.stat().st_mtime>obj.stat().st_mtime:
            subprocess.run(['/usr/bin/c++',*flags,'-DVIRTUAL_CONTROL_UPPER_PLANNER_NO_MAIN','-c',str(source),'-o',str(obj)],check=True,stdout=log,stderr=subprocess.STDOUT)
        if name!='tracking_control_trial':
            subprocess.run(['/usr/bin/c++',str(here/f'{name}.o'),str(here/'tracking_control_trial.o'),'-o',str(here/name),*libs],check=True,stdout=log,stderr=subprocess.STDOUT)
            print('Built',name,flush=True)
