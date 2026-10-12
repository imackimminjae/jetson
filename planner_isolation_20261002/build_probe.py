from pathlib import Path
import shlex,subprocess,json,hashlib
p=Path(__file__).resolve().parent;root=p.parent
f=(root/'build/virtual_control/CMakeFiles/upper_planner_node.dir/flags.make').read_text()
flags=[]
for line in f.splitlines():
 if line.startswith(('CXX_DEFINES =','CXX_INCLUDES =','CXX_FLAGS =')):flags+=shlex.split(line.split('=',1)[1])
link=shlex.split((root/'build/virtual_control/CMakeFiles/upper_planner_node.dir/link.txt').read_text())
libs=link[link.index('upper_planner_node')+1:]
source=root/'src/virtual_control/src/tracking_control.cpp'
metadata={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in [source,root/'src/virtual_control/include/virtual_control/tracking_control.hpp',root/'install/virtual_control/lib/virtual_control/upper_planner_node']}
(p/'source_hashes.json').write_text(json.dumps(metadata,indent=2))
with (p/'build.log').open('w') as log:
 for name,src,extra in [('planner',source,['-DVIRTUAL_CONTROL_UPPER_PLANNER_NO_MAIN']),('probe',p/'probe.cpp',[])]:
  if name=='planner' and (p/'planner.o').exists() and (p/'planner.o').stat().st_mtime>src.stat().st_mtime:
   continue
  subprocess.run(['/usr/bin/c++',*flags,*extra,'-c',str(src),'-o',str(p/(name+'.o'))],stdout=log,stderr=subprocess.STDOUT,check=True)
 subprocess.run(['/usr/bin/c++',str(p/'probe.o'),str(p/'planner.o'),'-o',str(p/'probe'),*libs],stdout=log,stderr=subprocess.STDOUT,check=True)
print('built isolated probe; production binaries untouched')
