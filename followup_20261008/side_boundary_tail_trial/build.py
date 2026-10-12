from pathlib import Path
import shlex,subprocess
H=Path(__file__).resolve().parent;R=H.parents[1];T=H/'trial/src/virtual_control'
flags=['-I'+str(T/'include')]
for line in (R/'build/virtual_control/CMakeFiles/upper_planner_node.dir/flags.make').read_text().splitlines():
 if line.startswith(('CXX_DEFINES =','CXX_INCLUDES =','CXX_FLAGS =')):flags+=shlex.split(line.split('=',1)[1])
flags=[f for f in flags if f!='-O3']+['-O2','-DVIRTUAL_CONTROL_UPPER_PLANNER_NO_MAIN']
link=shlex.split((R/'build/virtual_control/CMakeFiles/upper_planner_node.dir/link.txt').read_text());libs=link[link.index('upper_planner_node')+1:]
s=(R/'followup_20261006/k1_front_clip_skip_trial/production_replay.cpp').read_text();s=s.replace('rclcpp::Parameter("skip_front_clipped_k1_lane_constraints",spec.value("skip_k1_mode",0)==1)','rclcpp::Parameter("skip_front_clipped_k1_lane_constraints",false), rclcpp::Parameter("guard_side_boundary_terminal_loss",spec.value("guard",false))');s=s.replace('   row["k1_skip_mask"]=pc.k1_lane_constraint_excluded;', '''   row["k1_skip_mask"]=pc.k1_lane_constraint_excluded;
   row["side_boundary_tail_start"]=pc.side_boundary_tail_start;
   row["side_boundary_side"]=pc.side_boundary_side;
   row["published"]=J::array();for(const auto&q:Node::publishablePrediction(pc,z)){V p=position+R*q;row["published"].push_back({p.x(),p.y()});}''');s=s.replace('   auto pc=n.extractPreviewIntervals(ref,&g,interval);', '   if(legacy)ref.used_previous_solution=c.value("rec_used_previous",false);\n   auto pc=n.extractPreviewIntervals(ref,&g,interval);');(H/'replay.cpp').write_text(s)
with (H/'build.log').open('w') as log:
 for src,obj in [(T/'src/tracking_control.cpp',H/'tracking_control.o'),(H/'replay.cpp',H/'replay.o')]:
  if not obj.exists() or src.stat().st_mtime>obj.stat().st_mtime:
   subprocess.run(['/usr/bin/c++',*flags,'-c',str(src),'-o',str(obj)],check=True,stdout=log,stderr=subprocess.STDOUT)
 subprocess.run(['/usr/bin/c++',str(H/'replay.o'),str(H/'tracking_control.o'),'-o',str(H/'replay'),*libs],check=True,stdout=log,stderr=subprocess.STDOUT)
print('Built isolated replay')
