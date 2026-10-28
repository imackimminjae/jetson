#include "virtual_control/upper_vehicle_rollout.hpp"
#include <gtest/gtest.h>
#include <cmath>
namespace vr=imac_ctrl::upper_vehicle_rollout;
TEST(UpperVehicleMath, StraightRolloutMatchesAnalyticPosition) {
 vr::Settings s;auto r=vr::integrate(s,std::vector<double>(14,0));ASSERT_TRUE(r.valid);
 EXPECT_NEAR(r.states.back().x,21,1e-10);EXPECT_NEAR(r.states.back().y,0,1e-12);EXPECT_EQ(r.coarse_indices.size(),8u);
}
TEST(UpperVehicleMath, ConstantSteeringMatchesOffsetPointCircle) {
 vr::Settings s;s.initial_steering=s.initial_command=-.2;auto r=vr::integrate(s,{0,0});ASSERT_TRUE(r.valid);
 const double beta=std::atan(s.offset/s.wheelbase*std::tan(-.2));
 const double w=s.speed*std::cos(beta)*std::tan(-.2)/s.wheelbase,theta=w*s.dt,R=s.wheelbase/std::tan(-.2);
 EXPECT_NEAR(r.states.back().x,R*std::sin(theta)+s.offset*(std::cos(theta)-1),1e-9);
 EXPECT_NEAR(r.states.back().y,R*(1-std::cos(theta))+s.offset*std::sin(theta),1e-9);
 const double first=std::atan2(r.states[1].y,r.states[1].x);EXPECT_LT(std::abs(first-beta),.005);
 EXPECT_DOUBLE_EQ(r.states.front().steering,-.2);EXPECT_DOUBLE_EQ(r.states.front().yaw,0);
}
TEST(UpperVehicleMath, RampActuatorMatchesClosedFormAndRate) {
 vr::Settings s;s.initial_command=s.initial_steering=.1;auto r=vr::integrate(s,{0,.15});ASSERT_TRUE(r.valid);
 const double rate=.15/.5,T=.5,tau=.74;
 EXPECT_NEAR(r.states.back().steering,.1+rate*(T-tau*(1-std::exp(-T/tau))),1e-9);
 EXPECT_LE(r.max_effective_rate,.3+1e-9);EXPECT_NEAR(r.max_command_rate,.3,1e-12);
}
vr::Problem straightProblem() {
 vr::Problem p;p.settings.budget_ms=500;
 for(int k=0;k<=7;++k) {p.position_targets.push_back({3.0*k,0});p.position_weights.push_back(.01);p.seed_path.push_back({3.0*k,0});if(k) {p.corridors.push_back({{0,1},-2,2,true});p.reference_headings.push_back(0);}}
 return p;
}
TEST(UpperVehicleRefinement, ExactStraightPathAccepted) {
 auto p=straightProblem();auto r=vr::refine(p);ASSERT_TRUE(r.valid)<<r.status;EXPECT_LT(r.max_corridor_violation,1e-6);EXPECT_LT(r.objective,1e-10);
}
TEST(UpperVehicleRefinement, ImpossibleRoadNeverAcceptedByRestorationSlack) {
 auto p=straightProblem();p.corridors[0].lower=8;p.corridors[0].upper=9;
 auto r=vr::refine(p);EXPECT_FALSE(r.valid);
}
TEST(UpperVehicleRefinement, CourseStartsFromSteeringAndOffset) {
 auto p=straightProblem();p.settings.initial_steering=p.settings.initial_command=-.2;
 for(auto & c:p.corridors) {c.lower=-10;c.upper=10;}
 auto r=vr::refine(p);ASSERT_TRUE(r.valid)<<r.status;EXPECT_LT(std::abs(r.initial_course_error_rad),.005);
 EXPECT_LE(r.trajectory.max_command_rate,.500001);EXPECT_LE(r.trajectory.max_effective_rate,.500001);
}
TEST(UpperVehicleRefinement, UnreachablePreferenceIsAcceptedInsideHardRoad) {
 auto p=straightProblem();p.settings.iterations=40;
 p.preferred_corridors=p.corridors;
 p.preferred_corridors[0].lower=.4;p.preferred_corridors[0].upper=.6;
 auto r=vr::refine(p);ASSERT_TRUE(r.valid)<<r.status;
 EXPECT_GT(r.max_preferred_violation,.1);EXPECT_GT(r.preferred_cost,0);
 EXPECT_LE(r.max_corridor_violation,.001);
 EXPECT_LE(r.trajectory.max_command_rate,p.settings.max_rate+1e-6);
 EXPECT_LE(r.trajectory.max_effective_rate,p.settings.max_rate+1e-6);
 EXPECT_NEAR(r.objective,r.position_cost+r.input_cost+r.turn_cost+r.preferred_cost,1e-12);
}
TEST(UpperVehicleRefinement, PreferredSlackCannotRelaxImpossibleHardRoad) {
 auto p=straightProblem();p.preferred_corridors=p.corridors;
 p.corridors[0].lower=8;p.corridors[0].upper=9;
 EXPECT_FALSE(vr::refine(p).valid);
}
TEST(UpperVehicleRefinement, LargerPreferenceWeightChangesTradeoff) {
 auto p=straightProblem();p.settings.iterations=60;
 p.position_weights.assign(8,100.0);p.preferred_corridors=p.corridors;
 p.preferred_corridors[1].lower=.2;p.preferred_corridors[1].upper=.4;
 p.soft_linear_weight=0;p.soft_quadratic_weight=.1;
 auto low=vr::refine(p);ASSERT_TRUE(low.valid)<<low.status;
 p.soft_quadratic_weight=1000;
 auto high=vr::refine(p);ASSERT_TRUE(high.valid)<<high.status;
 EXPECT_LT(high.preferred_violations[1],low.preferred_violations[1]-.01);
 EXPECT_LE(high.max_corridor_violation,.001);
}
TEST(UpperVehicleRefinement, InvalidSoftWeightAndDimensionsRejected) {
 auto p=straightProblem();p.preferred_corridors.push_back(p.corridors[0]);
 EXPECT_FALSE(vr::refine(p).valid);
 p.preferred_corridors=p.corridors;p.soft_linear_weight=-1;
 EXPECT_FALSE(vr::refine(p).valid);
 p.soft_linear_weight=1;p.soft_quadratic_weight=NAN;
 EXPECT_FALSE(vr::refine(p).valid);
}
TEST(UpperVehicleRefinement, SteeringChangePenaltyReducesCommandVariation) {
 auto p=straightProblem();p.settings.iterations=60;p.settings.budget_ms=2000;
 p.position_weights.assign(8,10.0);
 for(int k=0;k<=7;++k) {p.position_targets[k][1]=k>=2?.6:0;}
 auto original=vr::refine(p);ASSERT_TRUE(original.valid)<<original.status;
 p.steering_change_weight=100;
 auto smooth=vr::refine(p);ASSERT_TRUE(smooth.valid)<<smooth.status;
 auto energy=[](const vr::Solution & r) {
   double sum=0;for(std::size_t k=1;k<r.inputs.size();k+=2) {sum+=r.inputs[k]*r.inputs[k];}return sum;
 };
 EXPECT_LT(energy(smooth),energy(original)*.8);
 EXPECT_DOUBLE_EQ(original.steering_change_cost,0);
 EXPECT_NEAR(smooth.steering_change_cost,.5*p.steering_change_weight*energy(smooth),1e-10);
 EXPECT_NEAR(smooth.objective,smooth.position_cost+smooth.input_cost+smooth.turn_cost+smooth.preferred_cost,1e-10);
 EXPECT_LE(smooth.max_corridor_violation,.001);
 EXPECT_LE(smooth.trajectory.max_command_rate,p.settings.max_rate+1e-6);
}
TEST(UpperVehicleRefinement, SteadyNonzeroSteeringHasNoChangePenalty) {
 auto p=straightProblem();p.settings.iterations=60;p.settings.budget_ms=2000;
 p.settings.initial_steering=p.settings.initial_command=.1;
 p.steering_change_weight=1000;
 auto target=vr::integrate(p.settings,std::vector<double>(14,0));ASSERT_TRUE(target.valid);
 for(int k=0;k<=7;++k) {
  const auto & x=target.states[target.coarse_indices[k]];
  p.position_targets[k]=p.seed_path[k]={x.x,x.y};
  if(k) {const auto & prev=target.states[target.coarse_indices[k-1]];
   p.reference_headings[k-1]=std::atan2(x.y-prev.y,x.x-prev.x);}
 }
 for(auto & c:p.corridors) {c.lower=-20;c.upper=20;}
 auto r=vr::refine(p);ASSERT_TRUE(r.valid)<<r.status;
 EXPECT_LT(r.steering_change_cost,1e-6);
 for(double command:r.trajectory.commands) {EXPECT_NEAR(command,.1,1e-4);}
}
TEST(UpperVehicleRefinement, SteeringChangeWeightMustBeFiniteAndNonnegative) {
 auto p=straightProblem();
 for(double w:{-1.0,double(NAN),double(INFINITY)}) {p.steering_change_weight=w;EXPECT_FALSE(vr::refine(p).valid);}
 p.steering_change_weight=100;p.corridors[0].lower=8;p.corridors[0].upper=9;
 EXPECT_FALSE(vr::refine(p).valid);
}
