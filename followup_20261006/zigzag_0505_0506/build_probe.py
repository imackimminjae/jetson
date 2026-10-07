from pathlib import Path
import shlex,subprocess,hashlib,json
here=Path(__file__).resolve().parent;root=here.parents[1]
flags=[]
for line in (root/'build/virtual_control/CMakeFiles/lower_tracking_mpc_node.dir/flags.make').read_text().splitlines():
    if line.startswith(('CXX_DEFINES =','CXX_INCLUDES =','CXX_FLAGS =')):flags+=shlex.split(line.split('=',1)[1])
flags=[x for x in flags if x!='-O3']+['-O1']
link=shlex.split((root/'build/virtual_control/CMakeFiles/lower_tracking_mpc_node.dir/link.txt').read_text());libs=link[link.index('lower_tracking_mpc_node')+1:]
src=root/'src/virtual_control/src/lower_tracking_mpc_node.cpp'
(here/'probe_source_sha256.txt').write_text(hashlib.sha256(src.read_bytes()).hexdigest()+'  '+str(src)+'\n')
with (here/'build_probe.log').open('w') as log:
    subprocess.run(['/usr/bin/c++',*flags,str(here/'lower_probe.cpp'),'-o',str(here/'lower_probe'),*libs],check=True,stdout=log,stderr=subprocess.STDOUT)
print('built isolated shadow solve')
