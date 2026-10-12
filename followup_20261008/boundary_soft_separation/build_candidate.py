from pathlib import Path
import shlex,subprocess,difflib
H=Path(__file__).resolve().parent;R=H.parents[1];rel='src/virtual_control/src/tracking_control.cpp';base=(H/'baseline'/rel).read_text()
needle='''      const Eigen::Vector2d processed_start = p0 + info.treatment.inset * direction;
      const Eigen::Vector2d processed_end = p1 - info.treatment.inset * direction;'''
repl='''      double start_inset = info.treatment.inset;
      double end_inset = info.treatment.inset;
      // A lateral map edge is not an observed road boundary. Omit contraction
      // at that edge only when it would exclude the inherited reference.
      // Preserve the observed-side margin, raw corridor, B, and reference.
      if (k > 0 && grid && grid->valid && info.treatment.centering_on &&
        info.observation.reference_inside && !info.observation.nominal_fallback)
      {
        const auto excludes_at_side_edge = [&](const Eigen::Vector2d & point,
            bool observed, double reference_clearance) {
            if (observed || reference_clearance >= info.treatment.inset) return false;
            const Eigen::Vector2d q = point - grid->origin_body;
            const double local_y = -std::sin(grid->origin_yaw_rad) * q.x() +
              std::cos(grid->origin_yaw_rad) * q.y();
            return std::min(std::abs(local_y),
              std::abs(local_y - grid->height * grid->resolution)) <=
              preview_line_sample_m_ + 1e-6;
          };
        if (excludes_at_side_edge(p0, info.observation.start_boundary_observed,
          (output.prk[step] - p0).dot(direction))) start_inset = 0.0;
        if (excludes_at_side_edge(p1, info.observation.end_boundary_observed,
          (p1 - output.prk[step]).dot(direction))) end_inset = 0.0;
      }
      const Eigen::Vector2d processed_start = p0 + start_inset * direction;
      const Eigen::Vector2d processed_end = p1 - end_inset * direction;'''
assert base.count(needle)==1;s=base.replace(needle,repl);p=H/'candidate'/rel;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(s)
(H/'candidate.patch').write_text(''.join(difflib.unified_diff(base.splitlines(True),s.splitlines(True),fromfile='a/'+rel,tofile='b/'+rel)))
flags=[]
for l in (R/'build/virtual_control/CMakeFiles/upper_planner_node.dir/flags.make').read_text().splitlines():
 if l.startswith(('CXX_DEFINES =','CXX_INCLUDES =','CXX_FLAGS =')):flags+=shlex.split(l.split('=',1)[1])
flags=[f for f in flags if f!='-O3']+['-O2','-DVIRTUAL_CONTROL_UPPER_PLANNER_NO_MAIN']
link=shlex.split((R/'build/virtual_control/CMakeFiles/upper_planner_node.dir/link.txt').read_text());libs=link[link.index('upper_planner_node')+1:]
with (H/'candidate_build.log').open('w') as log:
 subprocess.run(['/usr/bin/c++',*flags,'-c',str(p),'-o',str(H/'candidate.o')],check=True,stdout=log,stderr=subprocess.STDOUT)
 subprocess.run(['/usr/bin/c++',str(H/'replay.o'),str(H/'candidate.o'),'-o',str(H/'candidate_replay'),*libs],check=True,stdout=log,stderr=subprocess.STDOUT)
print('Built candidate source (no diagnostic hooks)')
