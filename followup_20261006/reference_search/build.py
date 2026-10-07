from pathlib import Path
import shlex,subprocess,hashlib,json
here=Path(__file__).resolve().parent;root=here.parents[1]
for name,h in json.loads((here/'baseline/sha256.json').read_text()).items():
    assert hashlib.sha256((root/name).read_bytes()).hexdigest()==h,name
flags=[]
for line in (root/'build/virtual_control/CMakeFiles/lower_tracking_mpc_node.dir/flags.make').read_text().splitlines():
    if line.startswith(('CXX_DEFINES =','CXX_INCLUDES =','CXX_FLAGS =')):
        flags+=shlex.split(line.split('=',1)[1])
flags=[f for f in flags if f!='-O3']+['-O2']
link=shlex.split((root/'build/virtual_control/CMakeFiles/upper_planner_node.dir/link.txt').read_text())
libs=link[link.index('upper_planner_node')+1:]
obj=root/'followup_20261006/interval_consistency/tracking_control_experiment.o'
(here/'linked_object_sha256.txt').write_text(hashlib.sha256(obj.read_bytes()).hexdigest()+'  '+str(obj)+'\n')
with (here/'build.log').open('w') as log:
    subprocess.run(['/usr/bin/c++',*flags,str(here/'probe.cpp'),str(obj),'-o',str(here/'probe'),*libs],check=True,stdout=log,stderr=subprocess.STDOUT)
print('Built isolated probe; operational files unchanged.')
