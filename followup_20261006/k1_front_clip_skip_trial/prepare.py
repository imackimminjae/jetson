from pathlib import Path
import hashlib,json,difflib
H=Path(__file__).resolve().parent
R=H.parents[1]
s=(H.parent/'deep_180803/tracking_control_trial.cpp').read_text()
s=s.replace('int experiment_force_turn_step = -1;', '''// Offline diagnostic only; set anew for every replay cycle.
std::vector<bool> experiment_skip_k1;
static bool skip_lane(int k, int candidate) {
  return k == 1 && candidate >= 0 &&
    static_cast<size_t>(candidate) < experiment_skip_k1.size() && experiment_skip_k1[candidate];
}
int experiment_force_turn_step = -1;''')
needle='''      for (int column = 0; column < input_count; ++column) {
        inequality[row][column] =
          axis.x() * G[2 * k][column] + axis.y() * G[2 * k + 1][column];'''
assert s.count(needle)==1
s=s.replace(needle,'''      if (skip_lane(k, candidate)) {
        // Redundant selector upper bounds preserve matrix dimensions.
        inequality[row][binary_index] = 1.0;
        inequality_bound[row++] = 1.0;
        inequality[row][binary_index] = 1.0;
        inequality_bound[row++] = 1.0;
        continue;
      }
'''+needle)
needle='''          const std::size_t candidate = static_cast<std::size_t>(selected[step]);
          const double heading'''
assert s.count(needle)==1
s=s.replace(needle,'''          const std::size_t candidate = static_cast<std::size_t>(selected[step]);
          if (skip_lane(k, static_cast<int>(candidate))) {
            direct_bound[direct_row++] = 1.0;
            direct_bound[direct_row++] = 1.0;
            continue;
          }
          const double heading''')
(H/'tracking_control_trial.cpp').write_text(s)
base=(H.parent/'deep_180803/tracking_control_trial.cpp').read_text()
(H/'trial.patch').write_text(''.join(difflib.unified_diff(base.splitlines(True),s.splitlines(True),fromfile='diagnostic_baseline.cpp',tofile='diagnostic_skip.cpp')))
s=(H.parent/'bev_registration/replay.cpp').read_text()
s=s.replace('using J=nlohmann::json;', 'extern std::vector<bool> experiment_skip_k1;\nusing J=nlohmann::json;')
needle='   n.applyTerminalPositionTargets(pc,'
assert s.count(needle)==1
s=s.replace(needle,'''   experiment_skip_k1.assign(pc.pmk.size()>1?pc.pmk[1].size():0,false);
   J clipped=J::array();
   for(size_t i=0;i<experiment_skip_k1.size();++i){
    const auto& info=pc.interval_info[1][i];
    const V a=pc.pmk[1][i],b=pc.pMk[1][i],u=(b-a).normalized();
    const V raw_a=a-info.treatment.inset*u,raw_b=b+info.treatment.inset*u;
    const bool front_a=!info.observation.start_boundary_observed && std::abs(raw_a.x()-g.origin_body.x())<0.11;
    const bool front_b=!info.observation.end_boundary_observed && std::abs(raw_b.x()-g.origin_body.x())<0.11;
    const bool front=!info.observation.nominal_fallback && (front_a||front_b);
    clipped.push_back(front);
    const int mode=spec.value("skip_k1_mode",0);
    experiment_skip_k1[i]=(mode==2 || (mode==1 && front));
   }
'''+needle)
s=s.replace('   row["position_targets"]=', '   row["k1_front_clipped"]=clipped;row["k1_skip_mask"]=experiment_skip_k1;\n   row["position_targets"]=')
(H/'replay.cpp').write_text(s)
(H/'build.py').write_text((H.parent/'deep_180803/build.py').read_text())
for origin in [0,2.95]:
 specs=[dict(name=('fixed' if legacy else 'carried')+f'_m{mode}',legacy=legacy,fixed_wp=True,grid_origin_x=origin,skip_k1_mode=mode) for legacy in [1,0] for mode in [0,1,2]]
 (H/f'specs_{origin}.json').write_text(json.dumps(specs,indent=2))
files=[R/'src/virtual_control/src/tracking_control.cpp',R/'src/virtual_control/config/tracking_control_split.yaml']
(H/'sources_before.json').write_text(json.dumps({str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in files},indent=2))
