from pathlib import Path
import shlex,subprocess
H=Path(__file__).resolve().parent;R=H.parents[1];flags=[]
for line in (R/'build/virtual_control/CMakeFiles/lower_tracking_mpc_node.dir/flags.make').read_text().splitlines():
    if line.startswith(('CXX_DEFINES =','CXX_INCLUDES =','CXX_FLAGS =')):flags+=shlex.split(line.split('=',1)[1])
flags=[x for x in flags if x!='-O3']+['-O1']
link=shlex.split((R/'build/virtual_control/CMakeFiles/lower_tracking_mpc_node.dir/link.txt').read_text());libs=link[link.index('lower_tracking_mpc_node')+1:]
with (H/'build_lower.log').open('w') as log:
    subprocess.run(['/usr/bin/c++',*flags,str(H/'lower_probe.cpp'),'-o',str(H/'lower_probe'),*libs],check=True,stdout=log,stderr=subprocess.STDOUT)
print('lower built',flush=True)
