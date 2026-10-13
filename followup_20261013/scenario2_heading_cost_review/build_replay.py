from pathlib import Path
import re,json,hashlib,yaml
R=Path('/home/imac/ros2_ws'); O=Path(__file__).resolve().parent
src=(R/'src/virtual_control/src/tracking_control.cpp').read_text(); hdr=(R/'src/virtual_control/include/virtual_control/tracking_control.hpp').read_text()
solver=src[src.index('SdMapUpperPlannerNode::UpperSolveResult SdMapUpperPlannerNode::solveUpperMiqp('):src.index('\ndouble SdMapUpperPlannerNode::remainingPathLength(')]
terminal=src[src.index('void SdMapUpperPlannerNode::applyTerminalPositionTargets('):src.index('SdMapUpperPlannerNode::UpperSolveResult SdMapUpperPlannerNode::solveUpperMiqp(')]
result=hdr[hdr.index('  struct UpperSolveResult'):hdr.index('  void declareAndLoadParameters();')]
fields=sorted(set(re.findall(r'\b[a-z][a-z_0-9]+_\b',solver+terminal)))
config=yaml.safe_load((R/'drive_batches/20261013_041941_scenario2_173ca0/effective_config.yaml').read_text())
def find(d):
 if isinstance(d,dict):
  if 'lambda_relative_turn' in d:return d
  for v in d.values():
   a=find(v)
   if a:return a
cfg=find(config); cfg['wheelbase']=2.8
members=[]
for f in fields:
 key=f[:-1]
 if key not in cfg:raise ValueError(f)
 v=cfg[key]; typ='bool' if isinstance(v,bool) else 'int' if isinstance(v,int) else 'double'
 members.append(f'{typ} {f}={str(v).lower()};')
prefix='''#include <Eigen/Core>
#include <array>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <limits>
#include <sstream>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include "QuadraticProblem.h"
#include "matrix_utils.h"
#include "virtual_control/miqp_solution_validation.hpp"
using namespace imac_ctrl;
using nlohmann::json;
int forced_last=-1;
double clampd(double v,double a,double b){return std::max(a,std::min(v,b));}
double wrapToPi(double x){return atan2(sin(x),cos(x));}
bool finitePoint(const Eigen::Vector2d&p){return p.allFinite();}
struct Replay {
struct PreviewConstraintData {
std::vector<std::vector<Eigen::Vector2d>> pmk,pMk;
std::vector<int> Nck;std::vector<bool> k1_lane_constraint_excluded;
std::vector<Eigen::Vector2d> prk,position_targets;
std::vector<double> psirk;
std::array<double,2> road_segment_heading_rad;
double delta_psi_road_rad,stable_heading_rad,waypoint_bearing_rad,terminal_buffer_turn_rad;
int N_pred,side_boundary_tail_start=-1;
};
'''
methods='''UpperSolveResult solveUpperMiqp(const PreviewConstraintData&,double) const;
void applyTerminalPositionTargets(PreviewConstraintData&,const Eigen::Vector2d&,const Eigen::Vector2d&,double)const;
};
'''
main=r'''
int main(int argc,char**argv){
json records; std::ifstream(argv[1])>>records; json outputs=json::array();
for(auto &rec:records){
auto e=rec["event"];auto t=rec["upper_trace"].get<std::vector<double>>(); int N=e["horizon"]; Replay m; Replay::PreviewConstraintData p;
p.N_pred=N;p.Nck=e["candidate_counts"].get<std::vector<int>>();p.k1_lane_constraint_excluded=e["k1_lane_constraint_excluded"].get<std::vector<bool>>();p.side_boundary_tail_start=e["side_boundary_tail_start"];
double x=e["x"],y=e["y"],yaw=e["yaw_rad"];auto body=[&](double wx,double wy){return Eigen::Vector2d(cos(yaw)*(wx-x)+sin(yaw)*(wy-y),-sin(yaw)*(wx-x)+cos(yaw)*(wy-y));};
for(auto w:e["preview_world"])p.prk.push_back(body(w[0],w[1]));p.psirk.assign(t.begin()+18,t.begin()+18+N+1);
p.pmk.resize(N+1);p.pMk.resize(N+1);for(int k=0;k<=N;k++){p.pmk[k].resize(p.Nck[k],Eigen::Vector2d::Zero());p.pMk[k].resize(p.Nck[k],Eigen::Vector2d::Zero());}
for(auto w:e["intervals_world"]){int k=w[0],j=w[1];p.pmk[k][j]=body(w[2],w[3]);p.pMk[k][j]=body(w[4],w[5]);}
p.road_segment_heading_rad={t[11],t[12]};p.delta_psi_road_rad=t[13];
m.applyTerminalPositionTargets(p,body(-133,56),body(-76,67),3.0);
for(int force:{-1,0,1}){if(force>=0 && int(rec["attempt"])!=2)continue;forced_last=force;
for(double lam:{0.0,0.3,0.6,0.9,1.2,1.3,1.4,1.5,1.6,1.7,1.8,1.9,2.0,3.0,6.0,12.0,30.0,100.0}){
m.lambda_relative_turn_=lam;auto s=m.solveUpperMiqp(p,6.0);json pts=json::array();for(auto b:s.predicted_body)pts.push_back({x+cos(yaw)*b.x()-sin(yaw)*b.y(),y+sin(yaw)*b.x()+cos(yaw)*b.y()});
outputs.push_back({{"epoch",e["t_start"]},{"forced_last",force},{"attempt",rec["attempt"]},{"lambda",lam},{"valid",s.valid},{"position_cost",s.position_cost},{"input_cost",s.input_cost},{"turn_cost",s.turn_cost},{"objective",s.objective},{"branch_step",s.branch_step},{"turn_steps",s.turn_window_steps},{"selected",s.selected_corridors},{"predicted_world",pts},{"model_heading",s.psi_pred_model_rad},{"status",s.status},{"terminal_error",s.terminal_position_error}});
}
}
}
std::ofstream(argv[2])<<outputs.dump(2)<<"\n";
}
'''
code=prefix+result+'\n'.join(members)+'\n'+methods+(terminal+solver).replace('SdMapUpperPlannerNode::','Replay::').replace('Matrix<double> direct_inequality(0.0, direct_row_count, input_count);','if(forced_last>=0 && selected.back()!=forced_last)continue;\n        Matrix<double> direct_inequality(0.0, direct_row_count, input_count);')+main
(O/'replay.cpp').write_text(code)
paths=[R/'src/virtual_control/src/tracking_control.cpp',R/'src/virtual_control/include/virtual_control/tracking_control.hpp',R/'src/virtual_control/config/tracking_control_split.yaml',R/'src/virtual_control/data/knu_routes/scenario2.csv',R/'followup_20261013/scenario2_short_041941_review/selected_branch_cycles.json']
(O/'input_hashes.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},indent=2))
(O/'solver_verbatim.cpp.txt').write_text(solver)
print(fields)
