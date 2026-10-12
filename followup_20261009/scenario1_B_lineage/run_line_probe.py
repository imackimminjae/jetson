from pathlib import Path
import json,shlex,subprocess,gzip,os
H=Path(__file__).resolve().parent;R=H.parents[1];P=H.parent/'scenario1_sway_offline'
flags=[]
for line in (R/'build/virtual_control/CMakeFiles/upper_planner_node.dir/flags.make').read_text().splitlines():
    if line.startswith(('CXX_DEFINES =','CXX_INCLUDES =','CXX_FLAGS =')):flags+=shlex.split(line.split('=',1)[1])
flags=[x for x in flags if x!='-O3']+['-O1','-DVIRTUAL_CONTROL_UPPER_PLANNER_NO_MAIN']
link=shlex.split((R/'build/virtual_control/CMakeFiles/upper_planner_node.dir/link.txt').read_text());libs=link[link.index('upper_planner_node')+1:]
with (H/'build.log').open('w') as f:
    subprocess.run(['/usr/bin/c++',*flags,'-c',str(H/'line_probe.cpp'),'-o',str(H/'line_probe.o')],check=True,stdout=f,stderr=subprocess.STDOUT)
    subprocess.run(['/usr/bin/c++',str(H/'line_probe.o'),str(P/'tracking_diagnostic.o'),'-o',str(H/'line_probe'),*libs],check=True,stdout=f,stderr=subprocess.STDOUT)
env=dict(os.environ,ROS_DOMAIN_ID='177',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(H/'ros_logs'),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
for a in json.loads((H/'manifest.json').read_text()):
    tag=a['tag'];out=H/(tag+'_result.json')
    with (H/(tag+'.log')).open('w') as f:
        subprocess.run([str(H/'line_probe'),str(H/(tag+'.json')),'/dev/stdin',str(P/'config.yaml'),str(out),str(H/'specs.json')],input=gzip.decompress((P/'grids.bin.gz').read_bytes()),env=env,stdout=f,stderr=subprocess.STDOUT,check=True)
    print(tag,{k:(v[0]['B'],v[0]['B_candidate_step']) for k,v in json.loads(out.read_text()).items()},flush=True)
