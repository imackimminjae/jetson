from pathlib import Path
import shlex,subprocess
p=Path(__file__).resolve().parent;root=p.parent;flags=[]
for line in (root/'build/virtual_control/CMakeFiles/lower_tracking_mpc_node.dir/flags.make').read_text().splitlines():
 if line.startswith(('CXX_DEFINES =','CXX_INCLUDES =','CXX_FLAGS =')):flags+=shlex.split(line.split('=',1)[1])
flags=[x for x in flags if x!='-O3']+['-O2'];link=shlex.split((root/'build/virtual_control/CMakeFiles/lower_tracking_mpc_node.dir/link.txt').read_text());libs=link[link.index('lower_tracking_mpc_node')+1:]
with (p/'build.log').open('w') as f:
 subprocess.run(['/usr/bin/c++',*flags,str(p/'lower_probe.cpp'),'-o',str(p/'lower_probe'),*libs],stdout=f,stderr=subprocess.STDOUT,check=True)
print('built offline lower probe')
