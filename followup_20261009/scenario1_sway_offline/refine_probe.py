from pathlib import Path
import json,shlex,subprocess,gzip,os
H=Path(__file__).resolve().parent;R=H.parents[1]
src=(H/'upper_replay.cpp').read_text()
needle='   n.applyTerminalPositionTargets(pc,'
assert src.count(needle)==1
src=src.replace(needle,'''   if(spec.value("relax_only",false)) {
    // Diagnostic monotone relaxation: keep narrow-corridor insets as recorded.
    for(size_t k=1;k<pc.pmk.size();++k)for(size_t j=0;j<pc.pmk[k].size();++j){
      auto&info=pc.interval_info[k][j];
      if(!info.treatment.centering_on)continue;
      double margin=std::min(n.preview_interval_boundary_margin_m_,0.45*info.observation.length);
      double delta=info.treatment.inset-margin;
      if(delta<=1e-9)continue;
      V u=(pc.pMk[k][j]-pc.pmk[k][j]).normalized();
      pc.pmk[k][j]-=delta*u;pc.pMk[k][j]+=delta*u;
      info.treatment.inset=margin;info.treatment.centering_on=false;
      info.processed_length=(pc.pMk[k][j]-pc.pmk[k][j]).norm();
    }
   }
'''+needle)
(H/'upper_refine.cpp').write_text(src)
ff=[]
for line in (R/'build/virtual_control/CMakeFiles/upper_planner_node.dir/flags.make').read_text().splitlines():
    if line.startswith(('CXX_DEFINES =','CXX_INCLUDES =','CXX_FLAGS =')):ff+=shlex.split(line.split('=',1)[1])
ff=[f for f in ff if f!='-O3']+['-O1','-DVIRTUAL_CONTROL_UPPER_PLANNER_NO_MAIN']
link=shlex.split((R/'build/virtual_control/CMakeFiles/upper_planner_node.dir/link.txt').read_text());libs=link[link.index('upper_planner_node')+1:]
with (H/'build_refine.log').open('w') as f:
    subprocess.run(['/usr/bin/c++',*ff,'-c',str(H/'upper_refine.cpp'),'-o',str(H/'upper_refine.o')],check=True,stdout=f,stderr=subprocess.STDOUT)
    subprocess.run(['/usr/bin/c++',str(H/'upper_refine.o'),str(H/'tracking_diagnostic.o'),'-o',str(H/'upper_refine'),*libs],check=True,stdout=f,stderr=subprocess.STDOUT)
print('refine built',flush=True)
os.sched_setaffinity(0,{min(os.sched_getaffinity(0))})
env=dict(os.environ,ROS_DOMAIN_ID='177',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(H/'ros_logs'),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
base=dict(fixed_ref=True,fixed_B=True,fixed_wp=True,guard=True)
specs=[dict(base,name='relax_only',relax_only=True),dict(base,name='soft_k15',soft_mask=34),dict(base,name='soft_k12345',soft_mask=62)]
manifest=[dict(tag=f'sc1_{n}',input=str(H/f'input_{n}.json'),grids=str(H/'grids.bin.gz'),config=str(H/'config.yaml')) for n in range(1,11)]
manifest+=json.loads((H/'regression_manifest.json').read_text())
for a in manifest:
    if a['tag'].startswith('reg_'):
        # Match guard setting used for that historical baseline.
        guard=json.loads((H/(a['tag']+'_specs.json')).read_text())[0]['guard']
        specs=[dict(base,name='relax_only',relax_only=True,**{})]
        specs[0]['guard']=guard
    sp=H/'refine_specs.json';sp.write_text(json.dumps(specs));out=H/('refine_'+a['tag']+'.json');gp=Path(a['grids'])
    with (H/('refine_'+a['tag']+'.log')).open('w') as f:
        subprocess.run([str(H/'upper_refine'),a['input'],'/dev/stdin' if gp.suffix=='.gz' else str(gp),a['config'],str(out),str(sp)],input=gzip.decompress(gp.read_bytes()) if gp.suffix=='.gz' else None,env=env,stdout=f,stderr=subprocess.STDOUT,check=True)
    with gzip.open(str(out)+'.gz','wb') as f:f.write(out.read_bytes())
    out.unlink();print('refine',a['tag'],flush=True)
