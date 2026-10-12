# Builds an isolated harness from a patched COPY of production tracking_control.cpp. Production sources are only read.
from pathlib import Path
import shlex,subprocess,hashlib,json
H=Path(__file__).resolve().parent;R=Path('/home/imac/ros2_ws');SRC=R/'src/virtual_control'
src=(SRC/'src/tracking_control.cpp').read_text()
hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [SRC/'src/tracking_control.cpp',SRC/'src/lower_tracking_mpc_node.cpp',SRC/'include/virtual_control/tracking_control.hpp',SRC/'include/virtual_control/egocentric_planner_geometry.hpp',SRC/'include/virtual_control/lower_tracking_mpc_node.hpp']}
(H/'production_sha256.json').write_text(json.dumps(hashes,indent=2))
head='''#include <cmath>
// ---- experiment hooks (harness only) ----
int exp_b_mode=0;            // bit1 recover-after-N, bit2 confirm-decrease, bit4 ambiguous-split candidate, bit8 ratchet increase, bit16 split uses widest arm
int exp_recover_cycles=3;
double exp_split_min=3.0;
int exp_over_count=0;
double exp_prev_candidate=std::nan("");
void exp_reset(){exp_over_count=0;exp_prev_candidate=std::nan("");}
'''
old_loop='''    if (containing_count != 1U) {continue;}
    const auto & interval = intervals[containing_index];'''
new_loop='''    if (containing_count != 1U) {
      int split_best = -1; double split_len = 0.0;
      if ((exp_b_mode & 4) && containing_count == 0U && intervals.size() >= 2U) {
        // Reference lies between split arms: use the narrowest fully observed arm at this step.
        for (std::size_t i = 0; i < intervals.size(); ++i) {
          const auto & a = intervals[i];
          if (a.nominal_fallback || !a.start_boundary_observed || !a.end_boundary_observed ||
            !std::isfinite(a.length) || a.length < exp_split_min) {continue;}
          // bit16: per-step widest arm, then min across steps; default: narrowest arm.
          if (exp_b_mode & 16) {
            if (split_best < 0 || a.length > split_len) {split_best = static_cast<int>(i); split_len = a.length;}
            continue;
          }
          if ((exp_b_mode & 32) && !(settings.relative_length_factor * a.length < previous.reference_length)) {continue;}
          if (info.candidate_step < 0 || a.length < info.candidate_length) {
            info.candidate_length = a.length; info.candidate_step = static_cast<int>(k);
            info.candidate_interval = static_cast<int>(i);
          }
        }
        if (split_best >= 0 && (exp_b_mode & 32) && !(settings.relative_length_factor * split_len < previous.reference_length)) {split_best = -1;}
        if (split_best >= 0 && (info.candidate_step < 0 || split_len < info.candidate_length)) {
          info.candidate_length = split_len; info.candidate_step = static_cast<int>(k); info.candidate_interval = split_best;
        }
      }
      continue;
    }
    const auto & interval = intervals[containing_index];'''
old_acc='''  if (info.candidate_step >= 0 &&
    (!hasIntervalReference(previous) ||
    info.candidate_length <= settings.relative_length_factor * previous.reference_length))
  {
    info.state.reference_length = info.candidate_length;
  }
  return info;'''
new_acc='''  double effective = info.candidate_length;
  if (exp_b_mode & 2) {
    // A decrease must be seen in two consecutive previews.
    const double raw = info.candidate_length;
    if (info.candidate_step >= 0 && std::isfinite(exp_prev_candidate)) {effective = std::max(raw, exp_prev_candidate);}
    exp_prev_candidate = info.candidate_step >= 0 ? raw : std::nan("");
  }
  const double cap = settings.relative_length_factor * previous.reference_length;
  if (info.candidate_step >= 0 && (!hasIntervalReference(previous) || effective <= cap)) {
    info.state.reference_length = effective; exp_over_count = 0;
  } else if (info.candidate_step >= 0) {
    if (exp_b_mode & 8) {info.state.reference_length = cap;}
    if (exp_b_mode & 1) {
      if (++exp_over_count >= exp_recover_cycles) {info.state.reference_length = effective; exp_over_count = 0;}
    }
  } else {
    exp_over_count = 0;
  }
  return info;'''
assert src.count(old_loop)==1 and src.count(old_acc)==1
src=head+src.replace(old_loop,new_loop).replace(old_acc,new_acc)
(H/'tracking_control_exp.cpp').write_text(src)
flags=[]
for line in (R/'build/virtual_control/CMakeFiles/upper_planner_node.dir/flags.make').read_text().splitlines():
    if line.startswith(('CXX_DEFINES =','CXX_INCLUDES =','CXX_FLAGS =')):flags+=shlex.split(line.split('=',1)[1])
flags=[f for f in flags if f!='-O3']+['-O2','-DVIRTUAL_CONTROL_UPPER_PLANNER_NO_MAIN','-I'+str(SRC/'include')]
link=shlex.split((R/'build/virtual_control/CMakeFiles/upper_planner_node.dir/link.txt').read_text());libs=link[link.index('upper_planner_node')+1:]
with (H/'build.log').open('w') as log:
    subprocess.run(['nice','/usr/bin/c++',*flags,'-c',str(H/'tracking_control_exp.cpp'),'-o',str(H/'tc.o')],check=True,stdout=log,stderr=subprocess.STDOUT)
    lf=[]
    for line in (R/'build/virtual_control/CMakeFiles/lower_tracking_mpc_node.dir/flags.make').read_text().splitlines():
        if line.startswith(('CXX_DEFINES =','CXX_INCLUDES =','CXX_FLAGS =')):lf+=shlex.split(line.split('=',1)[1])
    lf=flags+[f for f in lf if f!='-O3']
    subprocess.run(['nice','/usr/bin/c++',*lf,'-c',str(H/'harness.cpp'),'-o',str(H/'harness.o')],check=True,stdout=log,stderr=subprocess.STDOUT)
    subprocess.run(['/usr/bin/c++',str(H/'harness.o'),str(H/'tc.o'),'-o',str(H/'harness'),*libs],check=True,stdout=log,stderr=subprocess.STDOUT)
print('built')
