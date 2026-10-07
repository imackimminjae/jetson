"""Create isolated experiments from immutable baseline files; never modify production."""
from pathlib import Path
import hashlib
import json

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
BASE = HERE / 'baseline'


def read(path):
    return (BASE / path).read_text()


def replace_once(text, old, new):
    assert text.count(old) == 1, old
    return text.replace(old, new)


manifest = json.loads((BASE / 'sha256.json').read_text())
for name, expected in manifest.items():
    assert hashlib.sha256((BASE / name).read_bytes()).hexdigest() == expected, name

planner = read('src/virtual_control/src/tracking_control.cpp')
planner = 'double experiment_near_weight = 0.0;\n' + planner
# Soft penalty only; retain every corridor and every feasibility constraint.
# prk[1:3] is the previous world path sampled at measured progress. Do not
# penalize seeded references or the two tail points that are newly extended.
needle = '  for (int row = 0; row < variable_count; ++row) {\n    for (int column = row + 1; column < variable_count; ++column) {'
addition = '''  if (experiment_near_weight > 0.0 && preview.used_previous_solution) {
    for (int k = 1; k <= std::min(2, horizon - 2); ++k) {
      const double weight = experiment_near_weight * (k == 1 ? 1.0 : 0.5);
      for (int r = 0; r < input_count; ++r) {
        for (int c = 0; c < input_count; ++c) {
          hessian[r][c] += weight * (
            G[2 * k][r] * G[2 * k][c] + G[2 * k + 1][r] * G[2 * k + 1][c]);
        }
      }
    }
  }

'''
planner = replace_once(planner, needle, addition + needle)
(HERE / 'tracking_control_experiment.cpp').write_text(planner)

replay = read('followup_20261006/verify/replay_prod.cpp')
replay = 'extern double experiment_near_weight;\n' + replay
replay = replace_once(replay, 'int legacy,double unused)', 'int legacy,double weight)')
replay = replace_once(replay, '  n.planner_timer_->cancel();', '  experiment_near_weight = weight;\n  n.planner_timer_->cancel();')
replay = replace_once(replay, 'std::vector<std::pair<int,double>> runs={{1,-1},{0,-1}};',
                     'std::vector<std::pair<int,double>> runs={{0,0},{0,0.03},{0,0.1},{0,0.3},{0,1.0},{0,3.0}};')
replay = replace_once(replay, '   if(z.valid){Points world;',
                     '   row["used_previous"] = used; row["reference"] = J::array();\n'
                     '   for(const auto &q:pc.prk){V p=position+R*q; row["reference"].push_back({p.x(),p.y()});}\n'
                     '   if(z.valid){Points world;')
(HERE / 'replay.cpp').write_text(replay)

sim = read('followup_20261006/verify/sim_prod.cpp')
sim = 'extern double experiment_near_weight;\n' + sim
sim = replace_once(sim, 'static void setup(Node&n,const J&c){',
                   'static void setup(Node&n,const J&c){experiment_near_weight=c.value("near_weight",0.0);')
(HERE / 'sim.cpp').write_text(sim)
print('Prepared isolated planner + replay + closed-loop simulator')
