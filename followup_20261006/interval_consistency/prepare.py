"""Independent single-change tests of near-section contraction, using saved production."""
from pathlib import Path
import hashlib,json

here=Path(__file__).resolve().parent;base=here/'baseline'
for f,h in json.loads((base/'sha256.json').read_text()).items():
    assert hashlib.sha256((base/f).read_bytes()).hexdigest()==h,f
def read(p):return (base/p).read_text()
def one(s,a,b):
    assert s.count(a)==1,a
    return s.replace(a,b)
s='int experiment_interval_mode = 0;\n'+read('src/virtual_control/src/tracking_control.cpp')
needle='      const Eigen::Vector2d p0 = output.pmk[step][candidate];'
s=one(s,needle,'''      // Experimental near-section policy only. Never merge observed blocked runs.
      if (k >= 1 && k <= 2 && !info.observation.nominal_fallback) {
        const double soft_inset = (1.0 - preview_interval_soft_ratio_) * info.observation.length;
        const double margin_inset = std::min(preview_interval_boundary_margin_m_,
          0.45 * info.observation.length);
        if (experiment_interval_mode == 1) {
          // Continuous lower envelope of the two existing inset laws: no B switch.
          info.treatment.inset = std::min(soft_inset, margin_inset);
        } else if (experiment_interval_mode == 2) {
          // Fixed configured boundary margin, with the existing narrow-road cap.
          info.treatment.inset = margin_inset;
        } else if (experiment_interval_mode == 3) {
          // Diagnostic counterfactual only: separates raw-map change from contraction.
          info.treatment.inset = 0.0;
        } else if (experiment_interval_mode == 4 && reference.used_previous_solution &&
          hasIntervalReference(interval_state))
        {
          // One-cycle look-ahead transition: a new global B may tighten far sections now,
          // but near sections retain the less restrictive old-B treatment for this cycle.
          // Current raw road boundaries still apply immediately, including new obstacles.
          const auto prior = intervalTreatment(info.observation, interval_state,
            interval_centering_settings_, preview_interval_soft_ratio_,
            preview_interval_boundary_margin_m_, preview_interval_fallback_max_centering_length_m_);
          info.treatment.inset = std::min(info.treatment.inset, prior.inset);
        }
      }
'''+needle)
(here/'tracking_control_experiment.cpp').write_text(s)
r='extern int experiment_interval_mode;\n'+read('followup_20261006/verify/replay_prod.cpp')
r=one(r,'  n.planner_timer_->cancel(); if(legacy){n.upper_cumulative_increment_model_=false;n.branch_turn_window_at_split_=false;n.branch_turn_target_absolute_=false;n.upper_r_dpsi_=1.5;}',
      '  n.planner_timer_->cancel(); experiment_interval_mode=legacy;')
r=one(r,'std::vector<std::pair<int,double>> runs={{1,-1},{0,-1}};',
      'std::vector<std::pair<int,double>> runs={{0,0},{1,0},{2,0},{3,0},{4,0}};')
r=one(r,'   if(z.valid){Points world;',
      '   row["used_previous"]=used;row["reference"]=J::array(); for(const auto&q:pc.prk){V p=position+R*q;row["reference"].push_back({p.x(),p.y()});}\n   if(z.valid){Points world;')
(here/'replay.cpp').write_text(r)
s='extern int experiment_interval_mode;\n'+read('followup_20261006/verify/sim_prod.cpp')
s=one(s,'static void setup(Node&n,const J&c){',
      'static void setup(Node&n,const J&c){experiment_interval_mode=c.value("interval_mode",0);')

if not (here/'sim.cpp').exists() or (here/'sim.cpp').read_text()!=s:
    (here/'sim.cpp').write_text(s)
print('Prepared isolated interval variants; no production edits')
