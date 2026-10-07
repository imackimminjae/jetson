// Open-loop replay: recorded BEV grid + recorded pose per cycle; plan memory carried by the variant itself. No ROS spin, no I/O.
#include "virtual_control/tracking_control.hpp"
#include "virtual_control/egocentric_planner_geometry.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
using J=nlohmann::json; using V=Eigen::Vector2d; using Points=std::vector<V>;
extern int g_branch_variant; extern int g_last_truncated_at; extern std::vector<std::vector<double>> g_combo_log;
static Eigen::Matrix2d rotation(double a){Eigen::Matrix2d r;r<<cos(a),-sin(a),sin(a),cos(a);return r;}
namespace imac_ctrl{
struct LowerPathGenerationTestAccess{
 using Node=SdMapUpperPlannerNode;
 static J run(Node&n,const J&in,const std::vector<int8_t>&grids,int variant,double rdpsi){
  n.planner_timer_->cancel(); g_branch_variant=variant; if(rdpsi>0)n.upper_r_dpsi_=rdpsi;
  Points route; for(const auto&q:in["route"])route.emplace_back(q[0].get<double>(),q[1].get<double>());
  Points previous; Node::IntervalCenteringState interval; int wp=-1; J out=J::array();
  for(const auto&c:in["cycles"]){
   V position(c["x"].get<double>(),c["y"].get<double>()); double yaw=c["yaw"], speed=c["v"];
   if(wp<0) wp=c["rec_wp0"].get<int>();
   Node::GridMapSnapshot g;g.valid=true;g.frame_id="Vehicle";g.width=g.height=128;g.resolution=.3125;g.origin_body=V(0,-20);
   size_t gi=c["grid_index"].get<size_t>(); g.data.assign(grids.begin()+gi*16384,grids.begin()+(gi+1)*16384);
   auto R=rotation(yaw); auto body=[&](const V&q)->V{return R.transpose()*(q-position);};
   auto pair=egocentric_planner_geometry::findWaypointPair(route,wp,position,std::max(.01,speed)*V(cos(yaw),sin(yaw)),n.waypoint_switch_eps_m_,n.waypoint_switch_distance_m_);wp=pair[0];
   int horizon=n.computeRequestedPreviewSteps(&g,n.target_speed_mps_);
   horizon=egocentric_planner_geometry::retainMatlabPreviewHorizon(horizon,previous.size(),n.preview_restore_full_horizon_);
   Points prefix;bool used=previous.size()==static_cast<size_t>(horizon+1)&&egocentric_planner_geometry::buildMeasuredFrameProgressPrefix(previous,position,yaw,n.target_speed_mps_*n.upper_prediction_dt_sec_,horizon,prefix);
   if(!used)previous.clear();
   auto ref=n.buildPreviewReference(wp>0?body(route[wp-1]):V::Zero(),body(route[wp]),body(route[pair[1]]),wp>0,n.target_speed_mps_,horizon,prefix);
   auto pc=n.extractPreviewIntervals(ref,&g,interval);interval=pc.interval_update.state;
   n.applyTerminalPositionTargets(pc,body(route[wp]),body(route[pair[1]]),n.target_speed_mps_*n.upper_prediction_dt_sec_);
   auto z=n.solveUpperMiqp(pc,n.target_speed_mps_);
   J row={{"t",c["t"]},{"valid",z.valid},{"status",z.status},{"branch",z.branch_step},{"wp",wp},{"counts",pc.Nck},{"selected",z.selected_corridors},{"truncated_at",g_last_truncated_at},{"combos",g_combo_log},{"turn_steps",z.turn_window_steps},{"turn_targets",z.turn_target_rad},{"turn_cost",z.turn_cost},{"position_cost",z.position_cost},{"road_dpsi",pc.delta_psi_road_rad},{"cands",[&]{J a=J::array();for(size_t k=0;k<pc.pmk.size();++k){J r=J::array();for(size_t i=0;i<pc.pmk[k].size();++i){V p0=position+R*pc.pmk[k][i],p1=position+R*pc.pMk[k][i];r.push_back({p0.x(),p0.y(),p1.x(),p1.y()});}a.push_back(r);}return a;}()}};
   if(z.valid){Points world;for(const auto&q:z.predicted_body)world.push_back(position+R*q);previous=world.size()>=3?world:Points{};J p=J::array();for(auto&q:world)p.push_back({q.x(),q.y()});row["planned"]=p;}
   out.push_back(row);
  }
  return out;
 }
};}
int main(int argc,char**argv){
 if(argc!=5){std::cerr<<"replay CYCLES_JSON GRIDS_BIN CONFIG_YAML OUT_JSON\n";return 2;}
 J in;std::ifstream(argv[1])>>in; std::ifstream gf(argv[2],std::ios::binary); std::vector<int8_t> grids((std::istreambuf_iterator<char>(gf)),{});
 rclcpp::init(0,nullptr); J result;
 std::vector<std::pair<int,double>> runs={{0,-1},{7,2.0},{7,3.0},{7,4.5}};
 for(auto [variant,lam]: runs){
  rclcpp::NodeOptions opt;opt.arguments({"--ros-args","--params-file",argv[3],"--log-level","error"});
  auto node=std::make_shared<imac_ctrl::SdMapUpperPlannerNode>(opt);
  result[std::to_string(variant)+"_"+std::to_string(lam)]=imac_ctrl::LowerPathGenerationTestAccess::run(*node,in,grids,variant,lam);
 }
 std::ofstream(argv[4])<<result.dump(); rclcpp::shutdown(); return 0;
}
