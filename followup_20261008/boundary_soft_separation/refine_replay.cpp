double experiment_soft_max_width_m=0.0;
double experiment_force_B=-1.0;
int experiment_force_turn_step=-1;
// Open-loop replay: recorded BEV grid + recorded pose per cycle; plan memory carried by the variant itself. No ROS spin, no I/O.
#include "virtual_control/tracking_control.hpp"
#include "virtual_control/egocentric_planner_geometry.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
int experiment_edge_mode=0;
nlohmann::json experiment_intervals;
using J=nlohmann::json; using V=Eigen::Vector2d; using Points=std::vector<V>;

static Eigen::Matrix2d rotation(double a){Eigen::Matrix2d r;r<<cos(a),-sin(a),sin(a),cos(a);return r;}
namespace imac_ctrl{
struct LowerPathGenerationTestAccess{
 using Node=SdMapUpperPlannerNode;
 static J run(Node&n,const J&in,const std::vector<int8_t>&grids,int legacy,double unused,const J& spec){
  experiment_edge_mode=spec.value("edge_mode",0);
  n.planner_timer_->cancel(); experiment_soft_max_width_m=unused;
  if(spec.contains("r_dpsi"))n.upper_r_dpsi_=spec["r_dpsi"];
  if(spec.contains("soft_ratio"))n.preview_interval_soft_ratio_=spec["soft_ratio"];
  const bool fixed_ref=spec.value("fixed_ref",false), fixed_B=spec.value("fixed_B",false);
  experiment_force_turn_step=spec.value("force_turn_step",-1);
  if(spec.contains("lambda_turn"))n.lambda_relative_turn_=spec["lambda_turn"];
  Points route; for(const auto&q:in["route"])route.emplace_back(q[0].get<double>(),q[1].get<double>());
  Points previous; Node::IntervalCenteringState interval; interval.reference_length=in["cycles"][0]["rec_B_before"].get<double>(); int wp=-1; J out=J::array();
  for(const auto&c:in["cycles"]){
   V position(c["x"].get<double>(),c["y"].get<double>()); double yaw=c["yaw"], speed=c["v"];
   if(wp<0) wp=c["rec_wp0"].get<int>();
   Node::GridMapSnapshot g;g.valid=true;g.frame_id="Vehicle";g.width=g.height=128;g.resolution=.3125;g.origin_body=V(spec.value("grid_origin_x",0.0),-20);
   size_t gi=c["grid_index"].get<size_t>(); g.data.assign(grids.begin()+gi*16384,grids.begin()+(gi+1)*16384);
   auto R=rotation(yaw); auto body=[&](const V&q)->V{return R.transpose()*(q-position);};
   auto pair=egocentric_planner_geometry::findWaypointPair(route,wp,position,std::max(.01,speed)*V(cos(yaw),sin(yaw)),n.waypoint_switch_eps_m_,n.waypoint_switch_distance_m_);wp=pair[0]; if(legacy || spec.value("fixed_wp",false)){wp=c["rec_wp0"].get<int>();pair[0]=wp;pair[1]=c["rec_wp1"].get<int>();} if(legacy || fixed_B)interval.reference_length=c["rec_B_before"].get<double>();
   experiment_force_B=spec.value("force_B",fixed_B?c["rec_B"].get<double>():spec.value("hold_update",false)?c["rec_B_before"].get<double>():-1.0);
   if(spec.value("hold_wp5",false) && wp>=6 && position.x()<280) {wp=5;pair[0]=5;pair[1]=6;}
   int horizon=n.computeRequestedPreviewSteps(&g,n.target_speed_mps_);
   horizon=egocentric_planner_geometry::retainMatlabPreviewHorizon(horizon,previous.size(),n.preview_restore_full_horizon_);
   if(legacy || fixed_ref)horizon=static_cast<int>(c["rec_preview"].size())-1;
   Points prefix;bool used=previous.size()==static_cast<size_t>(horizon+1)&&egocentric_planner_geometry::buildMeasuredFrameProgressPrefix(previous,position,yaw,n.target_speed_mps_*n.upper_prediction_dt_sec_,horizon,prefix);
   if((legacy || fixed_ref || out.empty()) && c["rec_preview"].size()==static_cast<size_t>(horizon+1)){
    prefix.clear();for(const auto&q:c["rec_preview"])prefix.push_back(body(V(q[0].get<double>(),q[1].get<double>())));used=true;
   }
   if(!used)previous.clear();
   if(spec.value("fresh_ref",false)){prefix.clear();used=false;previous.clear();}
   auto ref=n.buildPreviewReference(wp>0?body(route[wp-1]):V::Zero(),body(route[wp]),body(route[pair[1]]),wp>0,n.target_speed_mps_,horizon,prefix);
   if(legacy || fixed_ref)ref.used_previous_solution=c.value("rec_used_previous",false);
   auto pc=n.extractPreviewIntervals(ref,&g,interval);interval=pc.interval_update.state;
   // Narrow follow-up candidates applied before the unchanged target/cost/QP calculation.
   if(spec.contains("edge_policy")){
    for(const auto&d:experiment_intervals){
     const size_t k=d["k"],i=d["i"]; if(k>=pc.pmk.size() || i>=pc.pmk[k].size())continue;
     auto& info=pc.interval_info[k][i];
     if(!info.treatment.centering_on || !info.observation.reference_inside)continue;
     V a(d["raw_body"][0].get<double>(),d["raw_body"][1].get<double>()),b(d["raw_body"][2].get<double>(),d["raw_body"][3].get<double>());
     V u=(b-a).normalized();double from_start=(pc.prk[k]-a).dot(u),from_end=(b-pc.prk[k]).dot(u),inset=info.treatment.inset;
     double si=inset,ei=inset;const bool skip=spec["edge_policy"]=="gated_skip";
     if(d["side_clip"][0].get<bool>() && from_start<inset)si=skip?0.0:std::max(0.0,from_start);
     if(d["side_clip"][1].get<bool>() && from_end<inset)ei=skip?0.0:std::max(0.0,from_end);
     pc.pmk[k][i]=a+si*u;pc.pMk[k][i]=b-ei*u;info.processed_length=(pc.pMk[k][i]-pc.pmk[k][i]).norm();pc.processed_lengths[k][i]=info.processed_length;
    }
   }
   if(spec.value("raw_corridors",false)){
    for(size_t k=0;k<pc.pmk.size();++k)for(size_t i=0;i<pc.pmk[k].size();++i){
     V a=pc.pmk[k][i],b=pc.pMk[k][i],u=(b-a).normalized(),mid=.5*(a+b);double raw=pc.raw_lengths[k][i];
     pc.pmk[k][i]=mid-.5*raw*u;pc.pMk[k][i]=mid+.5*raw*u;
    }
   }
   n.applyTerminalPositionTargets(pc,body(route[wp]),body(route[pair[1]]),n.target_speed_mps_*n.upper_prediction_dt_sec_);
   auto z=n.solveUpperMiqp(pc,n.target_speed_mps_);
   J row={{"t",c["t"]},{"valid",z.valid},{"status",z.status},{"branch",z.branch_step},{"wp",wp},{"counts",pc.Nck},{"selected",z.selected_corridors},{"turn_steps",z.turn_window_steps},{"turn_targets",z.turn_target_rad},{"turn_cost",z.turn_cost},{"position_cost",z.position_cost},{"road_dpsi",pc.delta_psi_road_rad},{"cands",[&]{J a=J::array();for(size_t k=0;k<pc.pmk.size();++k){J r=J::array();for(size_t i=0;i<pc.pmk[k].size();++i){V p0=position+R*pc.pmk[k][i],p1=position+R*pc.pMk[k][i];r.push_back({p0.x(),p0.y(),p1.x(),p1.y()});}a.push_back(r);}return a;}()}};
   row["raw_detail"]=experiment_intervals;
   row["B_candidate"]=pc.interval_update.candidate_length;row["B_candidate_step"]=pc.interval_update.candidate_step;row["B_candidate_interval"]=pc.interval_update.candidate_interval;
   row["B_before"]=pc.interval_update.previous_reference_length;row["B"]=interval.reference_length;
   row["intervals"]=J::array();
   for(size_t k=0;k<pc.interval_info.size();++k)for(size_t i=0;i<pc.interval_info[k].size();++i){
    const auto&a=pc.interval_info[k][i];
    row["intervals"].push_back({{"k",k},{"i",i},{"raw",a.observation.length},{"processed",a.processed_length},{"soft",a.treatment.centering_on},{"threshold",a.treatment.threshold},{"fallback",a.observation.nominal_fallback}});
   }
   row["k1_skip_mask"]=pc.k1_lane_constraint_excluded;
   row["side_boundary_tail_start"]=pc.side_boundary_tail_start;
   row["side_boundary_side"]=pc.side_boundary_side;
   row["published"]=J::array();for(const auto&q:Node::publishablePrediction(pc,z)){V p=position+R*q;row["published"].push_back({p.x(),p.y()});}
   row["position_targets"]=J::array();for(const auto&q:pc.position_targets){V p=position+R*q;row["position_targets"].push_back({p.x(),p.y()});}
   row["input_cost"]=z.input_cost;row["objective"]=z.objective;row["psirk"]=pc.psirk;
   row["used_previous"]=used;row["reference"]=J::array(); for(const auto&q:pc.prk){V p=position+R*q;row["reference"].push_back({p.x(),p.y()});}
   if(z.valid){Points world;for(const auto&q:z.predicted_body)world.push_back(position+R*q);previous=world.size()>=3?world:Points{};J p=J::array();for(auto&q:world)p.push_back({q.x(),q.y()});row["planned"]=p;}
   out.push_back(row);
  }
  return out;
 }
};}
int main(int argc,char**argv){
 if(argc!=6){std::cerr<<"replay CYCLES_JSON GRIDS_BIN CONFIG_YAML OUT_JSON SPECS_JSON\n";return 2;}
 J in;std::ifstream(argv[1])>>in; std::ifstream gf(argv[2],std::ios::binary); std::vector<int8_t> grids((std::istreambuf_iterator<char>(gf)),{});
 rclcpp::init(0,nullptr); J result;
 J specs;std::ifstream(argv[5])>>specs;
 for(const auto&spec:specs){
  rclcpp::NodeOptions opt;opt.arguments({"--ros-args","--params-file",argv[3],"--log-level","error"});
  opt.parameter_overrides({rclcpp::Parameter("skip_front_clipped_k1_lane_constraints",false), rclcpp::Parameter("guard_side_boundary_terminal_loss",spec.value("guard",false))});
  auto node=std::make_shared<imac_ctrl::SdMapUpperPlannerNode>(opt);
  result[spec["name"].get<std::string>()]=imac_ctrl::LowerPathGenerationTestAccess::run(*node,in,grids,spec.value("legacy",0),spec.value("cap",0.0),spec);
 }
 std::ofstream(argv[4])<<result.dump(); rclcpp::shutdown(); return 0;
}
