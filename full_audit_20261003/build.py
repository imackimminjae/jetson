from pathlib import Path
import shlex,subprocess,hashlib,json
p=Path(__file__).resolve().parent;root=p.parent
flags=[]
for line in (root/'build/virtual_control/CMakeFiles/lower_tracking_mpc_node.dir/flags.make').read_text().splitlines():
    if line.startswith(('CXX_DEFINES =','CXX_INCLUDES =','CXX_FLAGS =')):flags+=shlex.split(line.split('=',1)[1])
flags=[x for x in flags if x!='-O3']+['-O2']
link=shlex.split((root/'build/virtual_control/CMakeFiles/upper_planner_node.dir/link.txt').read_text());libs=link[link.index('upper_planner_node')+1:]
sources=[root/'src/virtual_control/src/tracking_control.cpp',root/'src/virtual_control/src/lower_tracking_mpc_node.cpp',root/'src/virtual_control/config/tracking_control_split.yaml']
(p/'production_hashes.json').write_text(json.dumps({str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in sources},indent=2))
with (p/'build.log').open('w') as log:
    subprocess.run(['/usr/bin/c++',*flags,'-DVIRTUAL_CONTROL_UPPER_PLANNER_NO_MAIN','-c',str(sources[0]),'-o',str(p/'planner.o')],stdout=log,stderr=subprocess.STDOUT,check=True)
    subprocess.run(['/usr/bin/c++',*flags,'-c',str(p/'sim.cpp'),'-o',str(p/'sim.o')],stdout=log,stderr=subprocess.STDOUT,check=True)
    subprocess.run(['/usr/bin/c++',str(p/'sim.o'),str(p/'planner.o'),'-o',str(p/'sim'),*libs],stdout=log,stderr=subprocess.STDOUT,check=True)
print('Built offline upper/lower numerical simulator')
