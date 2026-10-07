from pathlib import Path
import json,hashlib,shutil,difflib
root=Path('/home/imac/ros2_ws'); here=root/'followup_20261006/soft_cap12_trial'
files=['src/virtual_control/src/tracking_control.cpp','src/virtual_control/include/virtual_control/tracking_control.hpp','src/virtual_control/config/tracking_control_split.yaml','src/virtual_control/src/lower_tracking_mpc_node.cpp','src/virtual_control/include/virtual_control/lower_tracking_mpc_node.hpp','src/virtual_control/include/virtual_control/egocentric_planner_geometry.hpp','src/virtual_control/include/virtual_control/lower_path_geometry.hpp']
assert not (here/'baseline/sha256.json').exists(), 'Existing experiment: do not overwrite'
manifest={}
for name in files:
 dest=here/'baseline'/name;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(root/name,dest);manifest[name]=hashlib.sha256(dest.read_bytes()).hexdigest()
(here/'baseline/sha256.json').write_text(json.dumps(manifest,indent=2)+'\n')
shutil.copy2(root/'followup_20261006/interval_consistency/restore.py',here/'restore.py')
def one(s,a,b):
 assert s.count(a)==1,a
 return s.replace(a,b)
original=(here/'baseline'/files[0]).read_text()
s=one(original,'namespace imac_ctrl\n{','double experiment_soft_max_width_m = 0.0;  // Offline only; zero disables the cap.\nnamespace imac_ctrl\n{')
s=one(s,'    result.threshold = settings.relative_length_factor * state.reference_length;', '''    result.threshold = settings.relative_length_factor * state.reference_length;
    if (experiment_soft_max_width_m > 0.0) {
      result.threshold = std::min(result.threshold, experiment_soft_max_width_m);
    }''')
(here/'tracking_control_trial.cpp').write_text(s)
(here/'trial.patch').write_text(''.join(difflib.unified_diff(original.splitlines(True),s.splitlines(True),fromfile='production/tracking_control.cpp',tofile='offline/tracking_control_trial.cpp')))
s=(root/'followup_20261006/interval_consistency/replay.cpp').read_text()
s=s.replace('extern int experiment_interval_mode;','extern double experiment_soft_max_width_m;').replace('experiment_interval_mode=legacy;','experiment_soft_max_width_m=unused;')
s=one(s,'std::vector<std::pair<int,double>> runs={{0,0},{1,0},{2,0},{3,0},{4,0}};','std::vector<std::pair<int,double>> runs={{0,0},{0,12}};')
s=one(s,'   row["used_previous"]=used;', '''   row["B_before"]=pc.interval_update.previous_reference_length;row["B"]=interval.reference_length;
   row["intervals"]=J::array();
   for(size_t k=0;k<pc.interval_info.size();++k)for(size_t i=0;i<pc.interval_info[k].size();++i){
    const auto&a=pc.interval_info[k][i];
    row["intervals"].push_back({{"k",k},{"i",i},{"raw",a.observation.length},{"processed",a.processed_length},{"soft",a.treatment.centering_on},{"threshold",a.treatment.threshold},{"fallback",a.observation.nominal_fallback}});
   }
   row["used_previous"]=used;''')
(here/'replay.cpp').write_text(s)
s=(root/'followup_20261006/interval_consistency/build.py').read_text().replace("['tracking_control_experiment','replay','sim']","['tracking_control_trial','replay']").replace('tracking_control_experiment','tracking_control_trial')
(here/'build.py').write_text(s)
print('Baseline backed up; isolated 12 m cap implementation and guarded restore prepared.')
