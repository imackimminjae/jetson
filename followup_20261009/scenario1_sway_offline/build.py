from pathlib import Path
import json,hashlib,shlex,subprocess
H=Path(__file__).resolve().parent;R=H.parents[1];P=R/'followup_20261008/boundary_soft_separation'
old=json.loads((P/'baseline_sha256.json').read_text())
for rel in ['src/virtual_control/src/tracking_control.cpp','src/virtual_control/include/virtual_control/tracking_control.hpp','src/virtual_control/include/virtual_control/egocentric_planner_geometry.hpp']:
    assert hashlib.sha256((R/rel).read_bytes()).hexdigest()==old[rel],rel
# Diagnostic hooks are compiled in this directory only. Production source is not edited.
s=(P/'trial/src/virtual_control/src/tracking_control.cpp').read_text()
s='extern int experiment_soft_mask;\n'+s
needle='      double start_inset = info.treatment.inset, end_inset = info.treatment.inset;'
assert s.count(needle)==1
s=s.replace(needle,'''      if ((experiment_soft_mask & (1 << k)) && info.treatment.centering_on) {
        info.treatment.centering_on = false;
        info.treatment.inset = std::min(preview_interval_boundary_margin_m_, 0.45 * raw_length);
      }
'''+needle)
(H/'tracking_diagnostic.cpp').write_text(s)
r=(P/'replay.cpp').read_text()
r='int experiment_soft_mask=0;\n'+r
needle='   if(spec.value("hold_wp5",false)'
assert r.count(needle)==1
r=r.replace(needle,'''   experiment_soft_mask=spec.value("soft_mask",0);
   if(spec.value("peer_B",false))experiment_force_B=c.value("peer_B",c["rec_B"].get<double>());
'''+needle)
needle='   row["input_cost"]='
assert r.count(needle)==1
r=r.replace(needle,'''   if(z.valid && spec.value("dense_output",false)) {
     Points published;for(const auto&q:Node::publishablePrediction(pc,z))published.push_back(position+R*q);
     std::vector<double> arc;auto dense=n.densifyPath(published,n.target_speed_mps_,arc);
     row["dense"]=J::array();for(auto&q:dense)row["dense"].push_back({q.x(),q.y()});row["dense_s"]=arc;
   }
'''+needle)
(H/'upper_replay.cpp').write_text(r)
def flags(target):
    ff=[]
    for line in (R/f'build/virtual_control/CMakeFiles/{target}.dir/flags.make').read_text().splitlines():
        if line.startswith(('CXX_DEFINES =','CXX_INCLUDES =','CXX_FLAGS =')):ff+=shlex.split(line.split('=',1)[1])
    link=shlex.split((R/f'build/virtual_control/CMakeFiles/{target}.dir/link.txt').read_text())
    return [f for f in ff if f!='-O3']+['-O1'],link[link.index(target)+1:]
ff,libs=flags('upper_planner_node')
with (H/'build_upper.log').open('w') as log:
    for src in ['tracking_diagnostic','upper_replay']:
        subprocess.run(['/usr/bin/c++',*ff,'-DVIRTUAL_CONTROL_UPPER_PLANNER_NO_MAIN','-c',str(H/(src+'.cpp')),'-o',str(H/(src+'.o'))],check=True,stdout=log,stderr=subprocess.STDOUT)
    subprocess.run(['/usr/bin/c++',str(H/'tracking_diagnostic.o'),str(H/'upper_replay.o'),'-o',str(H/'upper_replay'),*libs],check=True,stdout=log,stderr=subprocess.STDOUT)
print('upper built',flush=True)
