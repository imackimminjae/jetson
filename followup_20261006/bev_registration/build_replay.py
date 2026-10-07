from pathlib import Path
import shlex,subprocess
h=Path(__file__).resolve().parent;r=h.parents[1];f=[]
for l in (r/'build/virtual_control/CMakeFiles/lower_tracking_mpc_node.dir/flags.make').read_text().splitlines():
 if l.startswith(('CXX_DEFINES =','CXX_INCLUDES =','CXX_FLAGS =')):f+=shlex.split(l.split('=',1)[1])
f=[x for x in f if x!='-O3']+['-O2'];link=shlex.split((r/'build/virtual_control/CMakeFiles/upper_planner_node.dir/link.txt').read_text());libs=link[link.index('upper_planner_node')+1:]
with (h/'build.log').open('w') as log:
 subprocess.run(['/usr/bin/c++',*f,'-DVIRTUAL_CONTROL_UPPER_PLANNER_NO_MAIN','-c',str(h/'replay.cpp'),'-o',str(h/'replay.o')],stdout=log,stderr=subprocess.STDOUT,check=True)
 subprocess.run(['/usr/bin/c++',str(h/'replay.o'),str(h.parent/'deep_180803/tracking_control_trial.o'),'-o',str(h/'replay'),*libs],stdout=log,stderr=subprocess.STDOUT,check=True)
