from pathlib import Path
import shlex,subprocess,hashlib,json
here=Path(__file__).resolve().parent;root=here.parents[1]
flags=[]
for line in (root/'build/virtual_control/CMakeFiles/lower_tracking_mpc_node.dir/flags.make').read_text().splitlines():
    if line.startswith(('CXX_DEFINES =','CXX_INCLUDES =','CXX_FLAGS =')):
        flags+=shlex.split(line.split('=',1)[1])
flags=[f for f in flags if f!='-O3']+['-O2']
link=shlex.split((root/'build/virtual_control/CMakeFiles/upper_planner_node.dir/link.txt').read_text())
libs=link[link.index('upper_planner_node')+1:]
obj=root/'followup_20261006/verify/planner_prod.o'
(here/'linked_object_sha256.txt').write_text(hashlib.sha256(obj.read_bytes()).hexdigest()+'  '+str(obj)+'\n')
with (here/'build.log').open('w') as log:
    subprocess.run(['/usr/bin/c++',*flags,str(here/'replay.cpp'),str(obj),'-o',str(here/'replay'),*libs],check=True,stdout=log,stderr=subprocess.STDOUT)
print('Built parameter-only replay.')
