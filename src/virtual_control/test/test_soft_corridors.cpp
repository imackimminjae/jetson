#include "virtual_control/tracking_control.hpp"
#include "virtual_control/egocentric_planner_geometry.hpp"
#include <gtest/gtest.h>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <optional>
namespace imac_ctrl {
struct SoftCorridorTestAccess {
  using Preview=SdMapUpperPlannerNode::PreviewConstraintData;
  static void cancel(SdMapUpperPlannerNode & n) {n.planner_timer_->cancel();}
  static auto solve(SdMapUpperPlannerNode & n,const Preview & p) {return n.solveUpperMiqp(p,6.0);}
  static void targets(SdMapUpperPlannerNode & n,Preview & p,const Eigen::Vector2d & a,const Eigen::Vector2d & b) {n.applyTerminalPositionTargets(p,a,b,3.0);}
  static void feed(SdMapUpperPlannerNode & n,double delta,double cmd,double tau,double offset) {
    auto m=std::make_shared<std_msgs::msg::Float64MultiArray>();m->data.assign(32,0);
    m->data[0]=n.now().seconds();m->data[19]=delta;m->data[20]=cmd;m->data[21]=1;m->data[23]=tau;m->data[30]=offset;
    n.steeringStateCallback(m);
  }
  static Preview road(int horizon=2,double preferred_lower=1.5) {
    Preview p;p.N_pred=horizon;p.Nck.assign(horizon+1,1);p.psirk.assign(horizon+1,0);
    p.pmk.resize(horizon+1);p.pMk.resize(horizon+1);p.road_pmk.resize(horizon+1);p.road_pMk.resize(horizon+1);
    for(int k=0;k<=horizon;++k) {
      p.prk.push_back({3.0*k,0});p.position_targets.push_back({3.0*k,0});
      p.pmk[k].push_back({3.0*k,k==1?preferred_lower:-1});p.pMk[k].push_back({3.0*k,k==1?preferred_lower+.1:1});
      p.road_pmk[k].push_back({3.0*k,-2});p.road_pMk[k].push_back({3.0*k,2});
    }
    return p;
  }
  static bool runObservedRoadLoop(SdMapUpperPlannerNode & n,bool blocked) {
    n.pose_.valid=n.pose_.speed_valid=true;n.pose_.position={0,0};n.pose_.yaw_rad=0;
    n.pose_.speed_mps=6;n.pose_.frame_id="map";n.pose_.received=n.now();
    n.grid_.valid=true;n.grid_.width=n.grid_.height=128;n.grid_.resolution=.3125;
    n.grid_.origin_body={0,-20};n.grid_.origin_yaw_rad=0;n.grid_.frame_id="Vehicle";
    n.grid_.received=n.now();n.grid_.data.assign(128*128,0);
    if(blocked) {
      // No prediction station intersects this obstacle, but the published
      // dense reference crosses it on the way to the first station.
      for(int y=0;y<128;++y) {n.grid_.data[y*128+3]=100;}
    }
    n.grid_positive_is_drivable_=false;n.grid_value_threshold_=1;
    n.global_path_={{0,0},{30,0},{60,0}};n.global_path_frame_id_="map";
    n.plannerLoop();return n.previous_solution_valid_;
  }
};
}
using Access=imac_ctrl::SoftCorridorTestAccess;
class SoftCorridors:public ::testing::Test {
protected:
 static void SetUpTestSuite(){if(!rclcpp::ok())rclcpp::init(0,nullptr);}
 static void TearDownTestSuite(){rclcpp::shutdown();}
  static auto node(std::optional<bool> soft=std::nullopt,bool vehicle=false,double weight=10.0,double steering_change=0.0,
                   double minimum_candidate_width=0.0) {
  rclcpp::NodeOptions o;o.parameter_overrides({{"scale_mode","fullscale"},{"upper_prediction_dt_sec",.5},
    {"upper_cumulative_increment_model",true},{"target_speed_mps",6.0},
    {"max_steering_angle_rad",.3},
    {"upper_r_steering_delta",steering_change},
    {"road_candidate_min_width_m",minimum_candidate_width},
    {"upper_corridor_soft_quadratic_weight",weight},{"upper_use_vehicle_rollout",vehicle},
    {"upper_max_steering_rate_radps",vehicle?.75:0.0},{"upper_steering_state_timeout_sec",10.0},
    {"upper_vehicle_rollout_budget_ms",500.0}});
  // Ordinary cases exercise the node default; false explicitly tests legacy hard strips.
  if(soft.has_value()) {o.append_parameter_override("upper_soft_corridor_constraints",soft.value());}
  auto n=std::make_shared<imac_ctrl::SdMapUpperPlannerNode>(o);Access::cancel(*n);return n;
 }
};
TEST_F(SoftCorridors, NarrowCandidatesCannotBeSelectedAndOriginalIndicesArePreserved) {
 auto p=Access::road();p.raw_lengths={{4.0},{.1,.1,2.2},{4.0}};p.Nck[1]=3;
 p.pmk[1]=p.road_pmk[1]={{3,-.05},{3,-.25},{3,.25}};
 p.pMk[1]=p.road_pMk[1]={{3,.05},{3,-.15},{3,2.45}};
 const auto ordinary=Access::solve(*node(),p);ASSERT_TRUE(ordinary.valid)<<ordinary.status;
 EXPECT_EQ(ordinary.selected_corridors[1],0);
 const auto filtered=Access::solve(*node(true,false,10,0,2.0),p);
 ASSERT_TRUE(filtered.valid)<<filtered.status;
 EXPECT_EQ(filtered.selected_corridors[1],2);
 EXPECT_GE(filtered.predicted_body[1].y(),.25-.001);
}
TEST_F(SoftCorridors, WidthExclusionRejectsAnEmptySectionWithoutRestoringTinyCandidates) {
 auto n=node(true,false,10,0,2.0);auto p=Access::road();
 p.raw_lengths={{4.0},{.1},{4.0}};
 const auto r=Access::solve(*n,p);EXPECT_FALSE(r.valid);
 EXPECT_NE(r.status.find("no width-eligible road candidate at step=1"),std::string::npos);
}
TEST_F(SoftCorridors, WidthExclusionUsesOriginalWidthAndInclusiveThreshold) {
 auto n=node(true,false,10,0,2.0);auto p=Access::road(2,0);
 p.raw_lengths={{4.0},{2.0},{2.0}};
 const auto r=Access::solve(*n,p);ASSERT_TRUE(r.valid)<<r.status;
 // The preferred width at k1 is only0.1m; original width controls eligibility.
 EXPECT_EQ(r.selected_corridors[1],0);
 p.raw_lengths[1][0]=1.99;EXPECT_FALSE(Access::solve(*n,p).valid);
 EXPECT_THROW(node(true,false,10,0,-1),std::invalid_argument);
}
TEST_F(SoftCorridors, SteeringChangeWeightIsWiredAndRequiresPhysicalRollout) {
 EXPECT_THROW(node(true,false,10.0,1.0),std::invalid_argument);
 EXPECT_THROW(node(true,true,10.0,-1.0),std::invalid_argument);
 auto n=node(true,true,10.0,10.0);Access::feed(*n,0,0,.74,1.4);
 auto p=Access::road(2,.4);auto r=Access::solve(*n,p);ASSERT_TRUE(r.valid)<<r.status;
 ASSERT_TRUE(r.vehicle_rollout.valid);
 double energy=0;for(std::size_t i=1;i<r.vehicle_rollout.inputs.size();i+=2) {
  energy+=r.vehicle_rollout.inputs[i]*r.vehicle_rollout.inputs[i];
 }
 EXPECT_GT(r.vehicle_rollout.steering_change_cost,0);
 EXPECT_NEAR(r.vehicle_rollout.steering_change_cost,5.0*energy,1e-10);
 EXPECT_LE(r.max_hard_road_violation_m,.001);
}
TEST_F(SoftCorridors, SoftViolationIsAcceptedAndCharged) {
 auto n=node();auto p=Access::road();auto r=Access::solve(*n,p);
 ASSERT_TRUE(r.valid)<<r.status;EXPECT_GT(r.max_preferred_corridor_violation_m,.1);
 EXPECT_GT(r.preferred_corridor_cost,0);EXPECT_LE(r.max_hard_road_violation_m,.001);
 auto legacy=node(false);EXPECT_FALSE(Access::solve(*legacy,p).valid);
}
TEST_F(SoftCorridors, TurnGuidanceAnchorsAtK2AndMayIncludeK1InItsWindow) {
 auto n=node();auto p=Access::road(2,0.0);p.delta_psi_road_rad=.4;
 for(int k=1;k<=2;++k) {
  const double x=3.0*k;p.Nck[k]=2;
  // A narrow retained second interval must count without a 2 m width filter.
  p.pmk[k].push_back({x,.9});p.pMk[k].push_back({x,1.0});
  p.road_pmk[k].push_back({x,.9});p.road_pMk[k].push_back({x,1.0});
 }
 auto r=Access::solve(*n,p);ASSERT_TRUE(r.valid)<<r.status;
 EXPECT_EQ(r.branch_step,2);
 ASSERT_EQ(r.turn_window_steps.size(),2u);
 EXPECT_EQ(r.turn_window_steps[0],1);EXPECT_EQ(r.turn_window_steps[1],2);
 EXPECT_LE(r.max_hard_road_violation_m,.001);
}
TEST_F(SoftCorridors, PhysicalRefinementAlsoAcceptsPreferredViolation) {
 auto n=node(true,true);Access::feed(*n,0,0,.74,1.4);auto p=Access::road(2,.4);auto r=Access::solve(*n,p);
 ASSERT_TRUE(r.valid)<<r.status;ASSERT_TRUE(r.vehicle_rollout.valid);
 EXPECT_GT(r.max_preferred_corridor_violation_m,.1);EXPECT_GT(r.preferred_corridor_cost,0);
 EXPECT_NEAR(r.preferred_corridor_cost,r.vehicle_rollout.preferred_cost,1e-8);
 EXPECT_LE(r.max_hard_road_violation_m,.001);
}
TEST_F(SoftCorridors, HardRoadCannotBeBoughtWithPreferredSlack) {
 auto n=node();auto p=Access::road();p.road_pmk[1][0].y()=8;p.road_pMk[1][0].y()=9;
 p.pmk[1][0].y()=8.2;p.pMk[1][0].y()=8.8;
 EXPECT_FALSE(Access::solve(*n,p).valid);
}
TEST_F(SoftCorridors, SeparateRoadCandidatesNeverBridgeBlockedGap) {
 auto n=node();auto p=Access::road();p.Nck[1]=2;
 p.road_pmk[1]={{3,-.9},{3,.5}};p.road_pMk[1]={{3,-.5},{3,.9}};
 p.pmk[1]={{3,-.8},{3,.7}};p.pMk[1]={{3,-.7},{3,.8}};
 auto r=Access::solve(*n,p);ASSERT_TRUE(r.valid)<<r.status;
 EXPECT_GE(std::abs(r.predicted_body[1].y()),.5-.001);
 EXPECT_LE(std::abs(r.predicted_body[1].y()),.9+.001);
}
TEST_F(SoftCorridors, MissingRoadOrNominalFallbackCannotEstablishHardRoad) {
 auto n=node();auto p=Access::road();p.road_pmk.clear();EXPECT_FALSE(Access::solve(*n,p).valid);
 p=Access::road();p.source_modes={{0},{1},{0}};EXPECT_FALSE(Access::solve(*n,p).valid);
}
TEST_F(SoftCorridors, PreferenceMustBeNestedAndWeightsValid) {
 auto n=node();auto p=Access::road();p.pMk[1][0].y()=3;EXPECT_FALSE(Access::solve(*n,p).valid);
 EXPECT_THROW(node(true,false,-1),std::invalid_argument);
}
TEST_F(SoftCorridors, DenseReferenceCannotCrossObstacleBetweenStations) {
 auto clear=node();EXPECT_TRUE(Access::runObservedRoadLoop(*clear,false));
 auto blocked=node();EXPECT_FALSE(Access::runObservedRoadLoop(*blocked,true));
}
TEST_F(SoftCorridors, MixedIntegerFallbackKeepsSlackContinuous) {
 auto n=node();auto p=Access::road(9);
 for(int k=1;k<=9;++k) {
  p.Nck[k]=2;p.pmk[k].push_back({3.0*k,8.2});p.pMk[k].push_back({3.0*k,8.8});
  p.road_pmk[k].push_back({3.0*k,8});p.road_pMk[k].push_back({3.0*k,9});
 }
 auto r=Access::solve(*n,p);ASSERT_TRUE(r.valid)<<r.status;
 EXPECT_GT(r.max_preferred_corridor_violation_m,.1);EXPECT_LE(r.max_hard_road_violation_m,.001);
}
TEST_F(SoftCorridors, WidthExclusionAlsoConstrainsMixedIntegerFallback) {
 auto n=node(true,false,10,0,2.0);auto p=Access::road(9,0);
 p.raw_lengths.assign(10,{4.0});
 // Two eligible choices at nine stages force the mixed-integer solver (512 combinations).
 for(int k=1;k<=9;++k) {
  const double x=3.0*k;p.Nck[k]=3;p.raw_lengths[k]={.1,2.2,3.0};
  p.pmk[k]=p.road_pmk[k]={{x,-.05},{x,.25},{x,8.0}};
  p.pMk[k]=p.road_pMk[k]={{x,.05},{x,2.45},{x,11.0}};
 }
 const auto r=Access::solve(*n,p);ASSERT_TRUE(r.valid)<<r.status;
 for(int k=1;k<=9;++k) {
  ASSERT_GE(r.selected_corridors[k],1);ASSERT_LE(r.selected_corridors[k],2);
  EXPECT_GE(p.raw_lengths[k][r.selected_corridors[k]],2.0);
 }
 EXPECT_LE(r.max_hard_road_violation_m,.001);
}
TEST_F(SoftCorridors, RecordedFrozenInputs) {
 const char * file=std::getenv("SOFT_CORRIDOR_REPLAY_INPUT");const char * output=std::getenv("SOFT_CORRIDOR_REPLAY_OUTPUT");
 if(!file||!output){GTEST_SKIP()<<"Optional closed-log inputs";}
 const char * config=std::getenv("SOFT_CORRIDOR_REPLAY_CONFIG");ASSERT_NE(config,nullptr);
 std::ifstream in(file);ASSERT_TRUE(in.good());std::ofstream out(output);ASSERT_TRUE(out.good());
 out<<std::setprecision(17)<<"scenario,run,seq,profile,recorded_valid,valid,preferred_max_m,preferred_cost,hard_max_m,wall_ms,refinement_ms,iterations,status,path\n";
 std::vector<std::shared_ptr<imac_ctrl::SdMapUpperPlannerNode>> nodes;
 for(int profile=0;profile<3;++profile) {
  rclcpp::NodeOptions o;o.arguments({"--ros-args","--params-file",config});
  o.parameter_overrides({{"upper_soft_corridor_constraints",profile!=0},{"upper_steering_state_timeout_sec",10.0},
   {"upper_vehicle_rollout_budget_ms",profile==2?240.0:60.0},{"upper_vehicle_rollout_max_iterations",profile==2?40:12}});
  auto n=std::make_shared<imac_ctrl::SdMapUpperPlannerNode>(o);Access::cancel(*n);nodes.push_back(n);
 }
 int count;in>>count;ASSERT_GT(count,100);
 for(int f=0;f<count;++f) {
  int sc,run,seq,h,recorded,tail;double delta,cmd,tau,offset,incoming,outgoing,turn;Eigen::Vector2d a,b;
  in>>sc>>run>>seq>>h>>recorded>>tail>>delta>>cmd>>tau>>offset>>incoming>>outgoing>>turn>>a.x()>>a.y()>>b.x()>>b.y();
  Access::Preview p;p.N_pred=h;p.Nck.assign(h+1,1);p.prk.resize(h+1);p.psirk.resize(h+1);p.pmk.resize(h+1);p.pMk.resize(h+1);p.road_pmk.resize(h+1);p.road_pMk.resize(h+1);p.side_boundary_tail_start=tail;p.road_segment_heading_rad={incoming,outgoing};p.delta_psi_road_rad=turn;
  for(auto & xy:p.prk) {in>>xy.x()>>xy.y();}
  for(auto & angle:p.psirk) {in>>angle;}
  for(int k=1;k<=h;++k) {
   in>>p.Nck[k];for(int j=0;j<p.Nck[k];++j) {
    Eigen::Vector2d lo,hi,rawlo,rawhi;int skip;in>>lo.x()>>lo.y()>>hi.x()>>hi.y()>>rawlo.x()>>rawlo.y()>>rawhi.x()>>rawhi.y()>>skip;
    p.pmk[k].push_back(lo);p.pMk[k].push_back(hi);p.road_pmk[k].push_back(rawlo);p.road_pMk[k].push_back(rawhi);if(k==1)p.k1_lane_constraint_excluded.push_back(skip!=0);
   }
  }
  ASSERT_TRUE(in.good())<<f;Access::targets(*nodes[0],p,a,b);
  for(int profile=0;profile<3;++profile) {
   Access::feed(*nodes[profile],delta,cmd,tau,offset);auto r=Access::solve(*nodes[profile],p);
   if(r.valid&&profile) {
    EXPECT_LE(r.max_hard_road_violation_m,.001001)<<sc<<' '<<seq;
    EXPECT_LE(r.vehicle_rollout.trajectory.max_command_rate,.750001);
    EXPECT_LE(r.vehicle_rollout.trajectory.max_effective_rate,.750001);
   }
   out<<sc<<','<<run<<','<<seq<<','<<profile<<','<<recorded<<','<<r.valid<<','<<r.max_preferred_corridor_violation_m<<','<<r.preferred_corridor_cost<<','<<r.max_hard_road_violation_m<<','<<r.solve_time_ms<<','<<r.vehicle_rollout.elapsed_ms<<','<<r.vehicle_rollout.iterations<<','<<std::quoted(r.status)<<",\"";
   if(r.valid){for(auto & s:r.vehicle_rollout.trajectory.states)out<<s.x<<':'<<s.y<<':'<<s.yaw<<':'<<s.steering<<':'<<s.speed<<';';}
   out<<"\"\n";out.flush();
  }
 }
}
TEST_F(SoftCorridors, RecordedSteeringChangeSweep) {
 const char * input=std::getenv("STEERING_CHANGE_REPLAY_INPUT");
 const char * output=std::getenv("STEERING_CHANGE_REPLAY_OUTPUT");
 const char * config=std::getenv("STEERING_CHANGE_REPLAY_CONFIG");
 if(!input||!output||!config) {GTEST_SKIP()<<"Optional steering-change replay";}
 std::ifstream in(input);std::ofstream out(output);ASSERT_TRUE(in.good());ASSERT_TRUE(out.good());
 const std::vector<double> weights{0.0,3.0,10.0,30.0};
 std::vector<std::shared_ptr<imac_ctrl::SdMapUpperPlannerNode>> nodes;
 for(double weight:weights) {
  rclcpp::NodeOptions o;o.arguments({"--ros-args","--params-file",config});
  o.parameter_overrides({{"upper_r_steering_delta",weight},{"upper_steering_state_timeout_sec",10.0}});
  auto n=std::make_shared<imac_ctrl::SdMapUpperPlannerNode>(o);Access::cancel(*n);nodes.push_back(n);
 }
 out<<std::setprecision(17)<<"elapsed,seq,weight,valid,branch,delta_energy,command_tv_deg,max_delta_deg,command_05_deg,change_cost,preferred_max_m,hard_max_m,preferred_cost,position_cost,input_cost,turn_cost,wall_ms,status,path\n";
 int count;in>>count;ASSERT_GT(count,0);
 for(int f=0;f<count;++f) {
  double elapsed,delta,cmd,tau,offset;int seq,h,tail,branch;Eigen::Vector2d a,b;
  in>>elapsed>>seq>>h>>tail>>branch>>delta>>cmd>>tau>>offset>>a.x()>>a.y()>>b.x()>>b.y();
  Access::Preview p;p.N_pred=h;p.side_boundary_tail_start=tail;
  in>>p.road_segment_heading_rad[0]>>p.road_segment_heading_rad[1]>>p.delta_psi_road_rad;
  p.Nck.assign(h+1,1);p.prk.resize(h+1);p.psirk.resize(h+1);
  p.pmk.resize(h+1);p.pMk.resize(h+1);p.road_pmk.resize(h+1);p.road_pMk.resize(h+1);p.raw_lengths.resize(h+1);
  p.raw_lengths[0]={1.0};
  for(auto & xy:p.prk) {in>>xy.x()>>xy.y();}
  for(auto & angle:p.psirk) {in>>angle;}
  for(int k=1;k<=h;++k) {
   in>>p.Nck[k];for(int j=0;j<p.Nck[k];++j) {
    Eigen::Vector2d lo,hi,rawlo,rawhi;double length;int skip;
    in>>lo.x()>>lo.y()>>hi.x()>>hi.y()>>rawlo.x()>>rawlo.y()>>rawhi.x()>>rawhi.y()>>length>>skip;
    p.pmk[k].push_back(lo);p.pMk[k].push_back(hi);p.road_pmk[k].push_back(rawlo);p.road_pMk[k].push_back(rawhi);
    p.raw_lengths[k].push_back(length);if(k==1)p.k1_lane_constraint_excluded.push_back(skip!=0);
   }
  }
  ASSERT_TRUE(in.good())<<f;Access::targets(*nodes[0],p,a,b);
  for(std::size_t i=0;i<weights.size();++i) {
   Access::feed(*nodes[i],delta,cmd,tau,offset);const auto r=Access::solve(*nodes[i],p);
   // Recorded anchors may use the removed filters; current counts define the anchor.
   EXPECT_EQ(r.branch_step,imac_ctrl::egocentric_planner_geometry::firstBranchStep(p.Nck))<<f;
   double energy=0,tv=0,maximum=0,command05=NAN;
   if(r.valid) {
    const auto & v=r.vehicle_rollout;ASSERT_TRUE(v.valid);
    EXPECT_LE(r.max_hard_road_violation_m,.001001);
    EXPECT_LE(v.trajectory.max_command_rate,.750001);EXPECT_LE(v.trajectory.max_effective_rate,.750001);
    EXPECT_DOUBLE_EQ(v.trajectory.commands.front(),cmd);
    for(std::size_t k=1;k<v.inputs.size();k+=2) {energy+=v.inputs[k]*v.inputs[k];tv+=std::abs(v.inputs[k]);maximum=std::max(maximum,std::abs(v.inputs[k]));}
    command05=v.trajectory.commands[v.trajectory.coarse_indices[1]]*180/M_PI;
    EXPECT_NEAR(v.steering_change_cost,.5*weights[i]*energy,1e-9);
   }
   out<<elapsed<<','<<seq<<','<<weights[i]<<','<<r.valid<<','<<r.branch_step<<','<<energy<<','<<tv*180/M_PI<<','<<maximum*180/M_PI<<','<<command05<<','<<r.vehicle_rollout.steering_change_cost<<','<<r.max_preferred_corridor_violation_m<<','<<r.max_hard_road_violation_m<<','<<r.preferred_corridor_cost<<','<<r.position_cost<<','<<r.input_cost<<','<<r.turn_cost<<','<<r.solve_time_ms<<','<<std::quoted(r.status)<<",\"";
   if(r.valid) {
    const auto & trajectory=r.vehicle_rollout.trajectory;
    const int last_stage=tail>=2&&tail<=h?tail-1:h;
    const int last=trajectory.coarse_indices[last_stage];
    for(int k=0;k<=last;++k) {const auto & point=trajectory.states[k];out<<point.x<<':'<<point.y<<';';}
   }
   out<<"\"\n";out.flush();
  }
 }
}

TEST_F(SoftCorridors, RecordedBranchCostSweep) {
 const char * input=std::getenv("BRANCH_COST_REPLAY_INPUT");
 const char * output=std::getenv("BRANCH_COST_REPLAY_OUTPUT");
 const char * config=std::getenv("BRANCH_COST_REPLAY_CONFIG");
 const char * profiles=std::getenv("BRANCH_COST_REPLAY_PROFILES");
 if(!input||!output||!config||!profiles) {GTEST_SKIP()<<"Optional recorded branch-cost experiment";}
 std::ifstream in(input),profile_file(profiles);std::ofstream out(output);
 ASSERT_TRUE(in.good());ASSERT_TRUE(profile_file.good());ASSERT_TRUE(out.good());
 struct Profile {std::string name;double steering,turn,position,heading;};
 std::vector<Profile> variants;std::vector<std::shared_ptr<imac_ctrl::SdMapUpperPlannerNode>> nodes;
 Profile profile;
 while(profile_file>>profile.name>>profile.steering>>profile.turn>>profile.position>>profile.heading) {
  rclcpp::NodeOptions o;o.arguments({"--ros-args","--params-file",config});
  o.parameter_overrides({{"upper_r_steering_delta",profile.steering},{"lambda_relative_turn",profile.turn},
   {"lambda_position",profile.position},{"upper_r_dpsi",profile.heading},{"upper_steering_state_timeout_sec",10.0}});
  auto n=std::make_shared<imac_ctrl::SdMapUpperPlannerNode>(o);Access::cancel(*n);nodes.push_back(n);variants.push_back(profile);
 }
 ASSERT_FALSE(variants.empty());
 out<<std::setprecision(17)<<"frame,profile,valid,branch,objective,position_cost,input_cost,turn_cost,preferred_cost,change_cost,preferred_max_m,hard_max_m,command_tv_deg,terminal_error_m,wall_ms,status,selected,path,commands\n";
 int count;in>>count;ASSERT_GT(count,0);
 for(int f=0;f<count;++f) {
  double elapsed,delta,cmd,tau,offset;int seq,h,tail,branch;Eigen::Vector2d a,b;
  in>>elapsed>>seq>>h>>tail>>branch>>delta>>cmd>>tau>>offset>>a.x()>>a.y()>>b.x()>>b.y();
  Access::Preview p;p.N_pred=h;p.side_boundary_tail_start=tail;
  in>>p.road_segment_heading_rad[0]>>p.road_segment_heading_rad[1]>>p.delta_psi_road_rad;
  p.Nck.assign(h+1,1);p.prk.resize(h+1);p.psirk.resize(h+1);
  p.pmk.resize(h+1);p.pMk.resize(h+1);p.road_pmk.resize(h+1);p.road_pMk.resize(h+1);p.raw_lengths.resize(h+1);
  p.raw_lengths[0]={1.0};
  for(auto & xy:p.prk) {in>>xy.x()>>xy.y();}
  for(auto & angle:p.psirk) {in>>angle;}
  for(int k=1;k<=h;++k) {
   in>>p.Nck[k];for(int j=0;j<p.Nck[k];++j) {
    Eigen::Vector2d lo,hi,rawlo,rawhi;double length;int skip;
    in>>lo.x()>>lo.y()>>hi.x()>>hi.y()>>rawlo.x()>>rawlo.y()>>rawhi.x()>>rawhi.y()>>length>>skip;
    p.pmk[k].push_back(lo);p.pMk[k].push_back(hi);p.road_pmk[k].push_back(rawlo);p.road_pMk[k].push_back(rawhi);
    p.raw_lengths[k].push_back(length);if(k==1)p.k1_lane_constraint_excluded.push_back(skip!=0);
   }
  }
  ASSERT_TRUE(in.good())<<f;
  for(std::size_t i=0;i<nodes.size();++i) {
   auto experiment=p;
   if(variants[i].name=="force_goal") {
    // Diagnostic ablation: use the same first-multiple detector, but force
    // the farthest split onto the candidate whose midpoint
    // is closest to the outgoing waypoint. This is a diagnostic alternative,
    // not an automatic route correctness test.
    for(int k=h;k>=1;--k) {
     if(experiment.Nck[k]<2||(b-a).norm()<1e-9) {continue;}
     int best=-1;double distance=std::numeric_limits<double>::infinity();
     for(int j=0;j<experiment.Nck[k];++j) {
      if(experiment.raw_lengths[k][j]<2.0-1e-9) {continue;}
      const Eigen::Vector2d d=.5*(experiment.road_pmk[k][j]+experiment.road_pMk[k][j])-b;
      const double score=d.norm();
      if(score<distance) {distance=score;best=j;}
     }
     if(best<0) {continue;}
     experiment.pmk[k]={p.pmk[k][best]};experiment.pMk[k]={p.pMk[k][best]};
     experiment.road_pmk[k]={p.road_pmk[k][best]};experiment.road_pMk[k]={p.road_pMk[k][best]};experiment.Nck[k]=1;
     if(k==1) {experiment.k1_lane_constraint_excluded={p.k1_lane_constraint_excluded[best]};}
     break;
    }
   }
   Access::targets(*nodes[i],experiment,a,b);Access::feed(*nodes[i],delta,cmd,tau,offset);
   const auto r=Access::solve(*nodes[i],experiment);
   EXPECT_EQ(r.branch_step,imac_ctrl::egocentric_planner_geometry::firstBranchStep(experiment.Nck))<<seq;
   double tv=0;
   if(r.valid) {
    ASSERT_TRUE(r.vehicle_rollout.valid);EXPECT_LE(r.max_hard_road_violation_m,.001001);
    EXPECT_LE(r.vehicle_rollout.trajectory.max_command_rate,.750001);
    EXPECT_LE(r.vehicle_rollout.trajectory.max_effective_rate,.750001);
    for(std::size_t k=1;k<r.vehicle_rollout.inputs.size();k+=2)tv+=std::abs(r.vehicle_rollout.inputs[k]);
   }
   out<<seq<<','<<variants[i].name<<','<<r.valid<<','<<r.branch_step<<','<<r.objective<<','<<r.position_cost<<','<<r.input_cost<<','<<r.turn_cost<<','<<r.preferred_corridor_cost<<','<<r.vehicle_rollout.steering_change_cost<<','<<r.max_preferred_corridor_violation_m<<','<<r.max_hard_road_violation_m<<','<<tv*180/M_PI<<','<<r.terminal_position_error<<','<<r.solve_time_ms<<','<<std::quoted(r.status)<<",\"";
   for(int j:r.selected_corridors) {out<<j<<';';}
   out<<"\",\"";
   for(const auto & xy:r.predicted_body) {out<<xy.x()<<':'<<xy.y()<<';';}
   out<<"\",\"";
   if(r.vehicle_rollout.valid)for(int k:r.vehicle_rollout.trajectory.coarse_indices)out<<r.vehicle_rollout.trajectory.commands[k]<<';';
   out<<"\"\n";out.flush();
  }
 }
}
