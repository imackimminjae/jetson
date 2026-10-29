#include "virtual_control/tracking_control.hpp"
#include <gtest/gtest.h>
#include <cstdlib>
#include <fstream>
#include <iomanip>

namespace imac_ctrl {
struct SoftCorridorTestAccess {
  using Preview = SdMapUpperPlannerNode::PreviewConstraintData;
  static void cancel(SdMapUpperPlannerNode & n) {n.planner_timer_->cancel();}
  static auto solve(SdMapUpperPlannerNode & n, const Preview & p) {return n.solveUpperMiqp(p,6.0);}
  static void targets(SdMapUpperPlannerNode & n, Preview & p,
    const Eigen::Vector2d & a, const Eigen::Vector2d & b) {n.applyTerminalPositionTargets(p,a,b,3.0);}
  static void feed(SdMapUpperPlannerNode & n,double delta,double cmd,double tau,double offset) {
    auto m=std::make_shared<std_msgs::msg::Float64MultiArray>();m->data.assign(32,0);
    m->data[0]=n.now().seconds();m->data[19]=delta;m->data[20]=cmd;m->data[21]=1;
    m->data[23]=tau;m->data[30]=offset;n.steeringStateCallback(m);
  }
  static Preview road(int horizon, bool fork) {
    Preview p;p.N_pred=horizon;p.Nck.assign(horizon+1,1);
    p.psirk.assign(horizon+1,0);p.pmk.resize(horizon+1);p.pMk.resize(horizon+1);
    p.road_pmk.resize(horizon+1);p.road_pMk.resize(horizon+1);
    for(int k=0;k<=horizon;++k) {
      p.prk.push_back({3.0*k,0});p.position_targets.push_back({3.0*k,.3*k});
      p.pmk[k]=p.road_pmk[k]={{3.0*k,-2}};
      p.pMk[k]=p.road_pMk[k]={{3.0*k,2}};
      if(fork&&k>=1) {
        p.Nck[k]=2;p.pMk[k][0]=p.road_pMk[k][0]={3.0*k,-.15};
        p.pmk[k].push_back({3.0*k,.15});p.road_pmk[k].push_back({3.0*k,.15});
        p.pMk[k].push_back({3.0*k,2});p.road_pMk[k].push_back({3.0*k,2});
      }
    }
    return p;
  }
};
}
using Access=imac_ctrl::SoftCorridorTestAccess;
class MiqpSolverPolicy:public ::testing::Test {
protected:
 static void SetUpTestSuite(){if(!rclcpp::ok())rclcpp::init(0,nullptr);}
 static void TearDownTestSuite(){rclcpp::shutdown();}
 static auto node(bool bnb,const char * config=nullptr) {
   rclcpp::NodeOptions o;
   if(config)o.arguments({"--ros-args","--params-file",config});
   o.parameter_overrides({{"upper_miqp_force_bnb",bnb},{"scale_mode","fullscale"},
     {"upper_prediction_dt_sec",.5},{"upper_cumulative_increment_model",true},
     {"upper_steering_state_timeout_sec",10.0}});
   auto n=std::make_shared<imac_ctrl::SdMapUpperPlannerNode>(o);Access::cancel(*n);return n;
 }
};
TEST_F(MiqpSolverPolicy, OneCandidateAndForksUseBnbWithoutEnumeration) {
 auto bnb=node(true),hybrid=node(false);
 for(bool fork:{false,true})for(int horizon:{1,2,4,7}) {
   SCOPED_TRACE(::testing::Message()<<"fork="<<fork<<" horizon="<<horizon);
   const auto p=Access::road(horizon,fork);
   const auto a=Access::solve(*hybrid,p),b=Access::solve(*bnb,p);
   ASSERT_TRUE(a.valid)<<a.status;ASSERT_TRUE(b.valid)<<b.status;
   EXPECT_EQ(b.bnb_solve_calls,1);EXPECT_EQ(b.enumerated_corridor_combinations,0);
   EXPECT_EQ(a.bnb_solve_calls,0);EXPECT_GT(a.enumerated_corridor_combinations,0);
   EXPECT_NEAR(a.objective,b.objective,1e-3*(1+std::abs(a.objective)))
     <<" actual costs="<<a.position_cost+a.input_cost+a.turn_cost+a.preferred_corridor_cost
     <<","<<b.position_cost+b.input_cost+b.turn_cost+b.preferred_corridor_cost
     <<" terminal="<<a.terminal_position_error<<","<<b.terminal_position_error;
   EXPECT_LE(b.max_hard_road_violation_m,.001001);
 }
}
TEST_F(MiqpSolverPolicy, InfeasibleBnbDoesNotFallBackToEnumeration) {
 auto n=node(true);auto p=Access::road(2,false);
 p.pmk[1]=p.road_pmk[1]={{3,50}};p.pMk[1]=p.road_pMk[1]={{3,51}};
 const auto r=Access::solve(*n,p);
 EXPECT_FALSE(r.valid);EXPECT_EQ(r.bnb_solve_calls,1);EXPECT_EQ(r.enumerated_corridor_combinations,0);
}
TEST_F(MiqpSolverPolicy, RecordedMatchedInputs) {
 const char *input=std::getenv("C5_REPLAY_INPUT"),*output=std::getenv("C5_REPLAY_OUTPUT");
 const char *config=std::getenv("C5_REPLAY_CONFIG");
 if(!input||!output||!config){GTEST_SKIP()<<"Optional frozen-input solver comparison";}
 std::ifstream in(input);std::ofstream out(output);
 ASSERT_TRUE(in.good());ASSERT_TRUE(out.good());
 auto hybrid=node(false,config),bnb=node(true,config);
 out<<std::setprecision(17)<<"frame,mode,repeat,valid,objective,solver_ms,miqp_solver_ms,vehicle_solver_ms,wall_ms,bnb_calls,enumerated,polish_calls,hard_violation,preferred_violation,status,selected,path\n";
 int count;in>>count;ASSERT_GT(count,0);
 for(int frame=0;frame<count;++frame) {
   double elapsed,delta,cmd,tau,offset;int seq,h,tail,branch;Eigen::Vector2d a,b;
   in>>elapsed>>seq>>h>>tail>>branch>>delta>>cmd>>tau>>offset>>a.x()>>a.y()>>b.x()>>b.y();
   Access::Preview p;p.N_pred=h;p.side_boundary_tail_start=tail;
   in>>p.road_segment_heading_rad[0]>>p.road_segment_heading_rad[1]>>p.delta_psi_road_rad;
   p.Nck.assign(h+1,1);p.prk.resize(h+1);p.psirk.resize(h+1);
   p.pmk.resize(h+1);p.pMk.resize(h+1);p.road_pmk.resize(h+1);p.road_pMk.resize(h+1);p.raw_lengths.resize(h+1);p.raw_lengths[0]={1};
   for(auto &xy:p.prk){in>>xy.x()>>xy.y();}
   for(auto &angle:p.psirk){in>>angle;}
   for(int k=1;k<=h;++k){
     in>>p.Nck[k];for(int j=0;j<p.Nck[k];++j){
       Eigen::Vector2d lo,hi,rawlo,rawhi;double length;int skip;
       in>>lo.x()>>lo.y()>>hi.x()>>hi.y()>>rawlo.x()>>rawlo.y()>>rawhi.x()>>rawhi.y()>>length>>skip;
       p.pmk[k].push_back(lo);p.pMk[k].push_back(hi);p.road_pmk[k].push_back(rawlo);p.road_pMk[k].push_back(rawhi);
       p.raw_lengths[k].push_back(length);if(k==1)p.k1_lane_constraint_excluded.push_back(skip!=0);
     }
   }
   ASSERT_TRUE(in.good())<<frame;Access::targets(*hybrid,p,a,b);
   for(int repeat=0;repeat<2;++repeat)for(int index=0;index<2;++index){
     const bool use_bnb=(frame+repeat+index)%2;auto &n=use_bnb?bnb:hybrid;
     Access::feed(*n,delta,cmd,tau,offset);const auto r=Access::solve(*n,p);
     if(use_bnb){EXPECT_EQ(r.enumerated_corridor_combinations,0);}
     out<<frame<<','<<(use_bnb?"bnb":"hybrid")<<','<<repeat<<','<<r.valid<<','<<r.objective<<','<<r.solver_only_time_ms<<','<<r.solver_only_time_ms-r.vehicle_rollout.solver_ms<<','<<r.vehicle_rollout.solver_ms<<','<<r.solve_time_ms<<','<<r.bnb_solve_calls<<','<<r.enumerated_corridor_combinations<<','<<r.selected_corridor_polish_calls<<','<<r.max_hard_road_violation_m<<','<<r.max_preferred_corridor_violation_m<<','<<std::quoted(r.status)<<",\"";
     for(int selected:r.selected_corridors){out<<selected<<';';}
     out<<"\",\"";
     for(const auto &point:r.predicted_body){out<<point.x()<<':'<<point.y()<<';';}
     out<<"\"\n";out.flush();
   }
 }
}
