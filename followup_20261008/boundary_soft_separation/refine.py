from pathlib import Path
import shlex,subprocess
H=Path(__file__).resolve().parent;R=H.parents[1]
r=(H/'replay.cpp').read_text();needle='   if(spec.value("raw_corridors",false)){'
r=r.replace(needle,'''   // Narrow follow-up candidates applied before the unchanged target/cost/QP calculation.
   if(spec.contains("edge_policy")){
    for(const auto&d:experiment_intervals){
     const size_t k=d["k"],i=d["i"]; if(k>=pc.pmk.size() || i>=pc.pmk[k].size())continue;
     auto& info=pc.interval_info[k][i];
     if(!info.treatment.centering_on || !info.observation.reference_inside)continue;
     V a(d["raw_body"][0].get<double>(),d["raw_body"][1].get<double>()),b(d["raw_body"][2].get<double>(),d["raw_body"][3].get<double>());
     V u=(b-a).normalized();double from_start=(pc.prk[k]-a).dot(u),from_end=(b-pc.prk[k]).dot(u),inset=info.treatment.inset;
     double si=inset,ei=inset;const bool skip=spec["edge_policy"]=="gated_skip";
     if(d["side_clip"][0].get<bool>() && from_start<inset)si=skip?0.0:std::max(0.0,from_start);
     if(d["side_clip"][1].get<bool>() && from_end<inset)ei=skip?0.0:std::max(0.0,from_end);
     pc.pmk[k][i]=a+si*u;pc.pMk[k][i]=b-ei*u;info.processed_length=(pc.pMk[k][i]-pc.pmk[k][i]).norm();pc.processed_lengths[k][i]=info.processed_length;
    }
   }
'''+needle)
(H/'refine_replay.cpp').write_text(r)
flags=['-I'+str(H/'trial/src/virtual_control/include')]
for l in (R/'build/virtual_control/CMakeFiles/upper_planner_node.dir/flags.make').read_text().splitlines():
 if l.startswith(('CXX_DEFINES =','CXX_INCLUDES =','CXX_FLAGS =')):flags+=shlex.split(l.split('=',1)[1])
flags=[f for f in flags if f!='-O3']+['-O2','-DVIRTUAL_CONTROL_UPPER_PLANNER_NO_MAIN']
link=shlex.split((R/'build/virtual_control/CMakeFiles/upper_planner_node.dir/link.txt').read_text());libs=link[link.index('upper_planner_node')+1:]
with (H/'refine_build.log').open('w') as log:
 subprocess.run(['/usr/bin/c++',*flags,'-c',str(H/'refine_replay.cpp'),'-o',str(H/'refine_replay.o')],check=True,stdout=log,stderr=subprocess.STDOUT)
 subprocess.run(['/usr/bin/c++',str(H/'refine_replay.o'),str(H/'tracking_control.o'),'-o',str(H/'refine_replay'),*libs],check=True,stdout=log,stderr=subprocess.STDOUT)
print('Built refined offline harness')
