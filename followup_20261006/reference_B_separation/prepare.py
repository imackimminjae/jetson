from pathlib import Path
import json,hashlib,shutil,difflib
here=Path(__file__).resolve().parent; root=here.parents[1]; old=here.parent/'soft_cap12_trial'
assert not (here/'baseline').exists()
manifest=json.loads((old/'baseline/sha256.json').read_text())
for n,h in manifest.items():
 p=root/n; assert hashlib.sha256(p.read_bytes()).hexdigest()==h,n
 dest=here/'baseline'/n;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,dest)
(here/'baseline/sha256.json').write_text(json.dumps(manifest,indent=2))
shutil.copy2(old/'restore.py',here/'restore.py')
shutil.copy2(old/'build.py',here/'build.py')
orig=(root/'src/virtual_control/src/tracking_control.cpp').read_text()
s=(old/'tracking_control_trial.cpp').read_text()
s=s.replace('double experiment_soft_max_width_m = 0.0;', 'double experiment_force_B = -1.0; // Diagnostic only: replace updated B, before treatment.\ndouble experiment_soft_max_width_m = 0.0;')
a='  return info;\n}\n\nSdMapUpperPlannerNode::IntervalTreatment'; assert s.count(a)==1
s=s.replace(a,'  if (experiment_force_B > 0.0) {info.state.reference_length = experiment_force_B;}\n'+a)
(here/'tracking_control_trial.cpp').write_text(s)
(here/'diagnostic.patch').write_text(''.join(difflib.unified_diff(orig.splitlines(True),s.splitlines(True),fromfile='production',tofile='offline')))
s=(old/'replay.cpp').read_text().replace('extern double experiment_soft_max_width_m;','extern double experiment_soft_max_width_m;\nextern double experiment_force_B;')
s=s.replace('int legacy,double unused','int legacy,double unused,const J& spec')
s=s.replace('  Points route;', '''  if(spec.contains("r_dpsi"))n.upper_r_dpsi_=spec["r_dpsi"];
  if(spec.contains("soft_ratio"))n.preview_interval_soft_ratio_=spec["soft_ratio"];
  const bool fixed_ref=spec.value("fixed_ref",false), fixed_B=spec.value("fixed_B",false);
  Points route;''')
s=s.replace('if(legacy){wp=', 'if(legacy || spec.value("fixed_wp",false)){wp=')
# B_before is separately controlled; fixed waypoint must not reset B.
s=s.replace('pair[1]=c["rec_wp1"].get<int>();interval.reference_length=c["rec_B_before"].get<double>();}', 'pair[1]=c["rec_wp1"].get<int>();} if(legacy || fixed_B)interval.reference_length=c["rec_B_before"].get<double>();\n   experiment_force_B=fixed_B?c["rec_B"].get<double>():-1.0;')
s=s.replace('if(legacy)horizon=', 'if(legacy || fixed_ref)horizon=')
s=s.replace('(legacy || out.empty())','(legacy || fixed_ref || out.empty())')
s=s.replace('   row["B_before"]=', '   row["B_candidate"]=pc.interval_update.candidate_length;row["B_candidate_step"]=pc.interval_update.candidate_step;row["B_candidate_interval"]=pc.interval_update.candidate_interval;\n   row["B_before"]=')
s=s.replace('argc!=5','argc!=6').replace('OUT_JSON\\n','OUT_JSON SPECS_JSON\\n')
start=s.index(' std::vector<std::pair<int,double>> runs=')
end=s.index(' std::ofstream(argv[4])',start)
s=s[:start]+''' J specs;std::ifstream(argv[5])>>specs;
 for(const auto&spec:specs){
  rclcpp::NodeOptions opt;opt.arguments({"--ros-args","--params-file",argv[3],"--log-level","error"});
  auto node=std::make_shared<imac_ctrl::SdMapUpperPlannerNode>(opt);
  result[spec["name"].get<std::string>()]=imac_ctrl::LowerPathGenerationTestAccess::run(*node,in,grids,spec.value("legacy",0),spec.value("cap",0.0),spec);
 }
'''+s[end:]
(here/'replay.cpp').write_text(s)
# Reuse immutable exported input snapshots; export new bag to this folder only.
for p in old.glob('replay_*'):
 if p.suffix in ['.json','.bin']:shutil.copy2(p,here/p.name)
for p in old.glob('config_*.yaml'):shutil.copy2(p,here/p.name)
s=(old/'export_inputs.py').read_text().replace("['050536','050627','014143','014303','045314','044738']","['055105']")
(here/'export_new.py').write_text(s)
specs=[dict(name='legacy_carried'),dict(name='legacy_fixed',legacy=1)]
for cap in [0,12]:
 for ref in [False,True]:
  for B in [False,True]:specs.append(dict(name=f'cap{cap}_ref{int(ref)}_B{int(B)}',cap=cap,fixed_ref=ref,fixed_B=B,fixed_wp=True))
(here/'diagnostic_specs.json').write_text(json.dumps(specs,indent=2))
(here/'candidate_specs.json').write_text(json.dumps([dict(name='baseline'),dict(name='r3',r_dpsi=3.0),dict(name='soft07',soft_ratio=.7)],indent=2))
print('Prepared isolated sources, input snapshots, backup hashes and guarded restore.')
