from pathlib import Path
import shlex,subprocess
H=Path(__file__).resolve().parent;R=H.parents[1];T=H/'trial/src/virtual_control'
s=(H/'baseline/src/virtual_control/src/tracking_control.cpp').read_text()
s='#include <nlohmann/json.hpp>\nextern double experiment_force_B;\nextern int experiment_edge_mode;\nextern nlohmann::json experiment_intervals;\n'+s
s=s.replace('  const auto & next_state = output.interval_update.state;', '''  if (experiment_force_B >= 0.0) output.interval_update.state.reference_length = experiment_force_B;
  experiment_intervals = nlohmann::json::array();
  const auto & next_state = output.interval_update.state;''')
needle='''      const Eigen::Vector2d processed_start = p0 + info.treatment.inset * direction;
      const Eigen::Vector2d processed_end = p1 - info.treatment.inset * direction;'''
replacement='''      double start_inset = info.treatment.inset, end_inset = info.treatment.inset;
      const auto side_edge = [&](const Eigen::Vector2d & point, bool observed) {
          if (!grid || !grid->valid || observed || info.observation.nominal_fallback) return false;
          const Eigen::Vector2d q = point - grid->origin_body;
          const double local_y = -std::sin(grid->origin_yaw_rad) * q.x() + std::cos(grid->origin_yaw_rad) * q.y();
          return std::min(std::abs(local_y), std::abs(local_y - grid->height * grid->resolution)) <= preview_line_sample_m_ + 1e-6;
        };
      const bool start_clip = side_edge(p0, info.observation.start_boundary_observed);
      const bool end_clip = side_edge(p1, info.observation.end_boundary_observed);
      if (experiment_edge_mode && info.treatment.centering_on && info.observation.reference_inside && (start_clip || end_clip)) {
        if (experiment_edge_mode == 1) { // Candidate: retain observed-side margin, omit inset only at clipped side.
          if (start_clip) start_inset = 0.0;
          if (end_clip) end_inset = 0.0;
        } else if (experiment_edge_mode == 2) { // Diagnostic: symmetric fixed boundary margin instead of soft.
          start_inset = end_inset = std::min(preview_interval_boundary_margin_m_, 0.45 * raw_length);
        } else if (experiment_edge_mode == 3) { // Diagnostic upper bound: restore the raw clipped interval.
          start_inset = end_inset = 0.0;
        }
      }
      experiment_intervals.push_back({{"k",k},{"i",candidate},{"raw_body",{p0.x(),p0.y(),p1.x(),p1.y()}},
        {"inside",info.observation.reference_inside},{"observed",{info.observation.start_boundary_observed,info.observation.end_boundary_observed}},
        {"side_clip",{start_clip,end_clip}},{"insets",{start_inset,end_inset}}});
      const Eigen::Vector2d processed_start = p0 + start_inset * direction;
      const Eigen::Vector2d processed_end = p1 - end_inset * direction;'''
assert s.count(needle)==1;s=s.replace(needle,replacement);(T/'src/tracking_control.cpp').write_text(s)
r=(R/'followup_20261008/side_boundary_tail_trial/replay.cpp').read_text()
r=r.replace('using J=nlohmann::json;', 'int experiment_edge_mode=0;\nnlohmann::json experiment_intervals;\nusing J=nlohmann::json;')
r=r.replace('  n.planner_timer_->cancel();', '  experiment_edge_mode=spec.value("edge_mode",0);\n  n.planner_timer_->cancel();')
r=r.replace('spec.value("force_B",fixed_B?c["rec_B"].get<double>():-1.0)', 'spec.value("force_B",fixed_B?c["rec_B"].get<double>():spec.value("hold_update",false)?c["rec_B_before"].get<double>():-1.0)')
r=r.replace('if(legacy)ref.used_previous_solution', 'if(legacy || fixed_ref)ref.used_previous_solution')
r=r.replace('   row["B_candidate"]=', '   row["raw_detail"]=experiment_intervals;\n   row["B_candidate"]=')
# Fully fixed uses exact trace shape through existing generation. No change to production generation.
(H/'replay.cpp').write_text(r)
flags=['-I'+str(T/'include')]
for line in (R/'build/virtual_control/CMakeFiles/upper_planner_node.dir/flags.make').read_text().splitlines():
 if line.startswith(('CXX_DEFINES =','CXX_INCLUDES =','CXX_FLAGS =')):flags+=shlex.split(line.split('=',1)[1])
flags=[f for f in flags if f!='-O3']+['-O2','-DVIRTUAL_CONTROL_UPPER_PLANNER_NO_MAIN']
link=shlex.split((R/'build/virtual_control/CMakeFiles/upper_planner_node.dir/link.txt').read_text());libs=link[link.index('upper_planner_node')+1:]
with (H/'build.log').open('w') as log:
 for src,obj in [(T/'src/tracking_control.cpp',H/'tracking_control.o'),(H/'replay.cpp',H/'replay.o')]:
  subprocess.run(['/usr/bin/c++',*flags,'-c',str(src),'-o',str(obj)],check=True,stdout=log,stderr=subprocess.STDOUT)
 subprocess.run(['/usr/bin/c++',str(H/'replay.o'),str(H/'tracking_control.o'),'-o',str(H/'replay'),*libs],check=True,stdout=log,stderr=subprocess.STDOUT)
print('Built isolated replay')
