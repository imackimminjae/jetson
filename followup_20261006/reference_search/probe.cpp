// Frozen-pose counterfactuals only; never spin or publish vehicle commands.
extern int experiment_interval_mode;
#include "virtual_control/tracking_control.hpp"
#include "virtual_control/egocentric_planner_geometry.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
using J=nlohmann::json;
using V=Eigen::Vector2d;
using Points=std::vector<V>;
static Eigen::Matrix2d rotation(double a) {Eigen::Matrix2d r;r<<cos(a),-sin(a),sin(a),cos(a);return r;}
static J packed(const Points &p) {J a=J::array();for(const auto&q:p)a.push_back({q.x(),q.y()});return a;}

namespace imac_ctrl {
struct LowerPathGenerationTestAccess {
 using Node=SdMapUpperPlannerNode;
 // Independent route seed is a diagnostic, not a deployable curvature-smoothed planner.
 static void routeSeed(Node::PreviewReferenceData &ref,const Points &route,int wp,
   const V &position,const Eigen::Matrix2d &R,double ds) {
  int segment=std::max(0,wp-1);double best=1e100,progress=0;
  for(int i=std::max(0,wp-2);i<=std::min(wp,int(route.size())-2);++i) {
   V d=route[i+1]-route[i];double f=std::clamp((position-route[i]).dot(d)/d.squaredNorm(),0.,1.);
   double dist=(position-route[i]-f*d).squaredNorm();
   if(dist<best){best=dist;segment=i;progress=f*d.norm();}
  }
  auto sample=[&](double ahead)->V {
   int i=segment;double s=progress+ahead;
   while(i+1<int(route.size())-1 && s>(route[i+1]-route[i]).norm()) {
    s-=(route[i+1]-route[i]).norm();++i;
   }
   V p=route[i]+s*(route[i+1]-route[i]).normalized();return R.transpose()*(p-position);
  };
  ref.points_body[0]=V::Zero();
  for(size_t k=1;k<ref.points_body.size();++k)ref.points_body[k]=sample(k*ds);
  for(size_t k=0;k+1<ref.points_body.size();++k) {
   V d=ref.points_body[k+1]-ref.points_body[k];ref.heading_rad[k]=atan2(d.y(),d.x());
  }
  ref.heading_rad.back()=ref.heading_rad[ref.heading_rad.size()-2];
  ref.stable_heading_rad=ref.heading_rad.back();ref.used_previous_solution=false;
 }
 static J run(Node &n,const J &input,const std::vector<int8_t> &grids,int memory_mode) {
  n.planner_timer_->cancel();Points route;
  for(const auto&q:input["route"])route.emplace_back(q[0].get<double>(),q[1].get<double>());
  Points previous;Node::IntervalCenteringState interval;int wp=-1;J out=J::array();
  for(const auto&c:input["cycles"]) {
   V position(c["x"].get<double>(),c["y"].get<double>());double yaw=c["yaw"],speed=c["v"];
   if(wp<0)wp=c["rec_wp0"].get<int>();
   auto R=rotation(yaw);auto body=[&](const V&q)->V{return R.transpose()*(q-position);};
   auto pair=egocentric_planner_geometry::findWaypointPair(route,wp,position,
     std::max(.01,speed)*V(cos(yaw),sin(yaw)),n.waypoint_switch_eps_m_,n.waypoint_switch_distance_m_);wp=pair[0];
   Node::GridMapSnapshot g;g.valid=true;g.frame_id="Vehicle";g.width=g.height=128;
   g.resolution=.3125;g.origin_body=V(0,-20);size_t gi=c["grid_index"].get<size_t>();
   g.data.assign(grids.begin()+gi*16384,grids.begin()+(gi+1)*16384);
   int horizon=n.computeRequestedPreviewSteps(&g,n.target_speed_mps_);
   horizon=egocentric_planner_geometry::retainMatlabPreviewHorizon(horizon,previous.size(),n.preview_restore_full_horizon_);
   Points prefix;bool used=previous.size()==size_t(horizon+1)&&
    egocentric_planner_geometry::buildMeasuredFrameProgressPrefix(previous,position,yaw,
      n.target_speed_mps_*n.upper_prediction_dt_sec_,horizon,prefix);
   if(!used)previous.clear();
   J row={{"t",c["t"]},{"xy",{position.x(),position.y()}},{"wp",wp},
     {"B_before",interval.reference_length},{"probes",J::object()}};
   Points committed=previous;auto next_interval=interval;
   for(const std::string name:{"original","same_seed_original_inset","fresh_straight","route_polyline"}) {
    experiment_interval_mode=name=="original"?memory_mode:0;
    const bool fresh=name=="fresh_straight"||name=="route_polyline";
    auto ref=n.buildPreviewReference(wp>0?body(route[wp-1]):V::Zero(),body(route[wp]),
      body(route[pair[1]]),wp>0,n.target_speed_mps_,horizon,fresh?Points{}:prefix);
    if(name=="route_polyline")routeSeed(ref,route,wp,position,R,n.target_speed_mps_*n.upper_prediction_dt_sec_);
    auto pc=n.extractPreviewIntervals(ref,&g,interval);
    n.applyTerminalPositionTargets(pc,body(route[wp]),body(route[pair[1]]),n.target_speed_mps_*n.upper_prediction_dt_sec_);
    auto z=n.solveUpperMiqp(pc,n.target_speed_mps_);Points plan,reference;
    for(const auto&q:pc.prk)reference.push_back(position+R*q);
    if(z.valid)for(const auto&q:z.predicted_body)plan.push_back(position+R*q);
    J result={{"valid",z.valid},{"status",z.status},{"branch",z.branch_step},{"counts",pc.Nck},
      {"turn_steps",z.turn_window_steps},{"planned",packed(plan)},{"reference",packed(reference)},
      {"B_after",pc.interval_update.state.reference_length},{"turn_cost",z.turn_cost},{"position_cost",z.position_cost}};
    result["cands"]=J::array();
    for(size_t k=0;k<pc.pmk.size();++k){J a=J::array();for(size_t j=0;j<pc.pmk[k].size();++j){V a0=position+R*pc.pmk[k][j],a1=position+R*pc.pMk[k][j];a.push_back({a0.x(),a0.y(),a1.x(),a1.y()});}result["cands"].push_back(a);}
    row["probes"][name]=result;
    if(name=="original") {next_interval=pc.interval_update.state;if(z.valid)committed=plan.size()>=3?plan:Points{};}
   }
   previous=committed;interval=next_interval;out.push_back(row);
  }
  return out;
 }
};
}
int main(int argc,char **argv) {
 if(argc!=5)return 2;
 J input;std::ifstream(argv[1])>>input;std::ifstream f(argv[2],std::ios::binary);
 std::vector<int8_t> grids((std::istreambuf_iterator<char>(f)),{});rclcpp::init(0,nullptr);J result;
 for(int mode:{0,1}) {
  rclcpp::NodeOptions opt;opt.arguments({"--ros-args","--params-file",argv[3],"--log-level","error"});
  auto n=std::make_shared<imac_ctrl::SdMapUpperPlannerNode>(opt);
  result[std::to_string(mode)]=imac_ctrl::LowerPathGenerationTestAccess::run(*n,input,grids,mode);
 }
 std::ofstream(argv[4])<<result.dump();rclcpp::shutdown();return 0;
}
