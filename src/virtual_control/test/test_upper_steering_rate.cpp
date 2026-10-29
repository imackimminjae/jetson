#include "virtual_control/tracking_control.hpp"
#include <gtest/gtest.h>
#include <random>
#include <fstream>
#include <iomanip>
#include <cstdlib>
#include <thread>
#include <map>
#include <array>

namespace rate = imac_ctrl::upper_steering_rate;
namespace imac_ctrl {
struct UpperSteeringRateTestAccess {
  using Preview = SdMapUpperPlannerNode::PreviewConstraintData;
  static void cancel(SdMapUpperPlannerNode & n) {n.planner_timer_->cancel();}
  static void openRoadAndStep(SdMapUpperPlannerNode & n) {
    n.pose_.valid=n.pose_.speed_valid=true;n.pose_.position={0,0};n.pose_.yaw_rad=0;
    n.pose_.speed_mps=6;n.pose_.frame_id="map";n.pose_.received=n.now();
    n.grid_.valid=true;n.grid_.width=n.grid_.height=128;n.grid_.resolution=.3125;
    n.grid_.origin_body={0,-20};n.grid_.origin_yaw_rad=0;n.grid_.frame_id="Vehicle";
    n.grid_.received=n.now();n.grid_.data.assign(128*128,0);
    n.grid_positive_is_drivable_=false;n.grid_value_threshold_=1;
    n.global_path_={{0,0},{30,0},{60,0}};n.global_path_frame_id_="map";
    n.plannerLoop();
  }
  static auto feedback(SdMapUpperPlannerNode & n) {
    std::lock_guard<std::mutex> lock(n.steering_state_mtx_);return n.steering_state_;
  }
  static void feed(SdMapUpperPlannerNode & n, double angle, double age=0.0) {
    auto msg=std::make_shared<std_msgs::msg::Float64MultiArray>(); msg->data.assign(32,0.0);
    msg->data[0]=n.now().seconds()-age; msg->data[19]=angle;
    n.steeringStateCallback(msg);
  }
  static void feedVehicle(SdMapUpperPlannerNode & n,double angle,const std::array<double,5> & f) {
    auto msg=std::make_shared<std_msgs::msg::Float64MultiArray>();msg->data.assign(32,0.0);
    msg->data[0]=n.now().seconds();msg->data[19]=angle;msg->data[20]=f[0];msg->data[21]=f[1];
    msg->data[22]=f[2];msg->data[23]=f[3];msg->data[30]=f[4];n.steeringStateCallback(msg);
  }
  static auto solve(SdMapUpperPlannerNode & n, const Preview & p) {return n.solveUpperMiqp(p,6.0);}
  static void targets(SdMapUpperPlannerNode & n, Preview & p,
    const Eigen::Vector2d & w0, const Eigen::Vector2d & w1) {n.applyTerminalPositionTargets(p,w0,w1,3.0);}
  static auto road(SdMapUpperPlannerNode & n, double min_y=-20.0) {
    Preview p; p.N_pred=5; p.Nck.assign(6,1); p.psirk.assign(6,0.0);
    p.pmk.resize(6); p.pMk.resize(6); p.k1_lane_constraint_excluded={false};
    for(int k=0;k<=5;++k) {
      p.prk.push_back({3.0*k,0}); p.position_targets.push_back({3.0*k,1.0*k*k});
      p.pmk[k].push_back({3.0*k,k==1?min_y:-20.0});p.pMk[k].push_back({3.0*k,20.0});
    }
    return solve(n,p);
  }
};
}
using Access=imac_ctrl::UpperSteeringRateTestAccess;

class UpperSteeringRate : public ::testing::Test {
protected:
  static void SetUpTestSuite() {if(!rclcpp::ok()) {rclcpp::init(0,nullptr);}}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  static auto node(double limit=0.35,bool vehicle=false) {
    rclcpp::NodeOptions o;
    o.parameter_overrides({{"scale_mode","fullscale"},{"upper_prediction_dt_sec",0.5},
      {"upper_soft_corridor_constraints",false},  // This fixture supplies hard-strip geometry only.
      {"upper_cumulative_increment_model",true},{"upper_max_steering_rate_radps",limit},
      {"upper_steering_state_timeout_sec",10.0},{"upper_r_dpsi",3.0},
      {"upper_use_vehicle_rollout",vehicle},{"upper_vehicle_rollout_budget_ms",200.0},
      {"terminal_position_weight_multiplier",1000.0}});
    auto n=std::make_shared<imac_ctrl::SdMapUpperPlannerNode>(o);Access::cancel(*n);return n;
  }
};
TEST(SteeringRateMath, DisabledAddsNoRows) {
  rate::Settings s;EXPECT_TRUE(rate::build(7,s).valid);EXPECT_TRUE(rate::build(7,s).rows.empty());
}
TEST(SteeringRateMath, RejectsInvalidSettingsAndUnreachableAnchor) {
  rate::Settings s;s.rate_radps=-1;EXPECT_FALSE(rate::build(7,s).valid);
  s.rate_radps=std::numeric_limits<double>::infinity();EXPECT_FALSE(rate::build(7,s).valid);
  s.rate_radps=.1;s.initial_angle=.5;EXPECT_FALSE(rate::build(7,s).valid);
}
TEST(SteeringRateMath, FeedbackMustBeFreshFiniteAndUseEstimate) {
  std::vector<double> t(32,0);t[0]=100;t[19]=.17;t[20]=-.2;
  auto f=rate::parseFeedback(t,100.01);ASSERT_TRUE(f.valid);EXPECT_DOUBLE_EQ(f.angle,.17);
  EXPECT_TRUE(rate::fresh(f,100.02,.25));EXPECT_FALSE(rate::fresh(f,100.3,.25));
  EXPECT_FALSE(rate::fresh(f,99.99,.25));
  t[19]=std::numeric_limits<double>::quiet_NaN();EXPECT_FALSE(rate::parseFeedback(t,100.01).valid);
  t.resize(19);EXPECT_FALSE(rate::parseFeedback(t,100.01).valid);
}
TEST(SteeringRateMath, LinearRowsGuaranteeNonlinearRateWithChangingSpeed) {
  std::mt19937 gen(42);std::uniform_real_distribution<double> u(-1,1);
  int admitted=0;
  for(int attempt=0;attempt<12000;++attempt) {
    rate::Settings s;s.rate_radps=.35;s.initial_angle=.2*u(gen);
    auto rows=rate::build(7,s);ASSERT_TRUE(rows.valid);
    std::vector<double> x(14);double v=s.speed,delta=s.initial_angle;
    for(int k=0;k<7;++k) {
      x[2*k]=s.max_dv*u(gen);v+=x[2*k];
      delta=std::clamp(delta+.10*u(gen),-s.max_angle,s.max_angle);
      x[2*k+1]=s.dt*v/s.wheelbase*std::tan(delta);
    }
    bool feasible=true;
    for(const auto & row:rows.rows) {
      double lhs=0;for(std::size_t j=0;j<x.size();++j) {lhs+=row.coefficients[j]*x[j];}
      if(lhs>row.bound+1e-12) {feasible=false;break;}
    }
    if(feasible) {++admitted;EXPECT_TRUE(rate::evaluate(x,s).valid);}
  }
  EXPECT_GT(admitted,100);
}
TEST_F(UpperSteeringRate, MissingStaleAndFutureEstimatesAreRejected) {
  auto n=node();auto p=Access::road(*n);EXPECT_FALSE(p.valid);EXPECT_TRUE(p.steering_rate_limited);
  Access::feed(*n,0,11);EXPECT_FALSE(Access::road(*n).valid);
  Access::feed(*n,0,-1);EXPECT_FALSE(Access::road(*n).valid);
  Access::feed(*n,0);auto recovered=Access::road(*n);
  EXPECT_NE(recovered.status,"upper steering rate: missing, stale or future steering estimate");
}
TEST_F(UpperSteeringRate, DisabledDoesNotRequireFeedback) {
  auto n=node(0);auto r=Access::road(*n);EXPECT_TRUE(r.valid)<<r.status;
  EXPECT_FALSE(r.steering_rate_limited);
}
TEST_F(UpperSteeringRate, FirstStepAndFutureStepsRespectRate) {
  auto n=node();Access::feed(*n,0);auto r=Access::road(*n);
  ASSERT_TRUE(r.valid)<<r.status;ASSERT_EQ(r.model_steering_rad.size(),5u);
  EXPECT_LE(r.max_model_steering_rate_radps,.350002);
  EXPECT_LE(r.max_geometric_steering_rate_radps,.350002);
  EXPECT_LE(std::abs(r.model_steering_rad[0]),.175001);
}
TEST_F(UpperSteeringRate, OppositeCurrentSteeringAnchorsFirstStep) {
  auto n=node();Access::feed(*n,-.25);auto r=Access::road(*n);
  ASSERT_TRUE(r.valid)<<r.status;ASSERT_FALSE(r.model_steering_rad.empty());
  EXPECT_LE(r.model_steering_rad[0],-.075+1e-6);
}
TEST_F(UpperSteeringRate, TightRoadIsNotRelaxedToSatisfyRate) {
  auto n=node();Access::feed(*n,0);auto r=Access::road(*n,1.5);EXPECT_FALSE(r.valid);
}
TEST_F(UpperSteeringRate, InvalidRateParametersRejected) {
  EXPECT_THROW(node(-.1),std::invalid_argument);
}
TEST_F(UpperSteeringRate, FeedbackTransportOnIsolatedDomain) {
  if(!std::getenv("UPPER_STEERING_ROS_TEST")) {GTEST_SKIP()<<"Opt-in ROS transport test";}
  auto n=node();auto sender=std::make_shared<rclcpp::Node>("steering_test_sender");
  auto pub=sender->create_publisher<std_msgs::msg::Float64MultiArray>("/debug/lower_mpc_trace",10);
  rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(n);executor.add_node(sender);
  const auto end=std::chrono::steady_clock::now()+std::chrono::seconds(4);
  while(pub->get_subscription_count()==0 && std::chrono::steady_clock::now()<end) {
    executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_GT(pub->get_subscription_count(),0u);
  while(!Access::feedback(*n).valid && std::chrono::steady_clock::now()<end) {
    std_msgs::msg::Float64MultiArray msg;msg.data.assign(32,0.0);
    msg.data[0]=sender->now().seconds();msg.data[19]=.1;pub->publish(msg);
    executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(Access::feedback(*n).valid);EXPECT_DOUBLE_EQ(Access::feedback(*n).angle,.1);
  auto result=Access::road(*n);EXPECT_TRUE(result.valid)<<result.status;
}
TEST_F(UpperSteeringRate, PhysicalReferenceStartsAtCurrentCourse) {
  auto n=node(.5,true);Access::feedVehicle(*n,-.2,{-.2,1,0,.74,1.4});auto r=Access::road(*n);
  ASSERT_TRUE(r.valid)<<r.status;ASSERT_TRUE(r.vehicle_rollout.valid);
  EXPECT_LT(std::abs(r.vehicle_rollout.initial_course_error_rad),.005);
  EXPECT_GT(r.vehicle_rollout.trajectory.states.size(),r.predicted_body.size());
}
TEST_F(UpperSteeringRate, PhysicalFineCurvePublishedOnIsolatedDomain) {
  if(!std::getenv("UPPER_VEHICLE_ROS_TEST")) {GTEST_SKIP()<<"Opt-in physical reference publication test";}
  auto n=node(.5,true);auto sink=std::make_shared<rclcpp::Node>("vehicle_reference_test_sink");
  nav_msgs::msg::Path::SharedPtr received;
  auto sub=sink->create_subscription<nav_msgs::msg::Path>("/planner/lower_reference_path",
    rclcpp::QoS(1).reliable().transient_local(),[&](nav_msgs::msg::Path::SharedPtr p){received=p;});
  rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(n);executor.add_node(sink);
  Access::feedVehicle(*n,.1,{.1,1,0,.74,1.4});Access::openRoadAndStep(*n);
  const auto end=std::chrono::steady_clock::now()+std::chrono::seconds(3);
  while(!received && std::chrono::steady_clock::now()<end) {executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(10));}
  ASSERT_TRUE(received);ASSERT_GT(received->poses.size(),50u);
  const auto & a=received->poses[0].pose.position;const auto & b=received->poses[1].pose.position;
  EXPECT_NEAR(a.x,0,1e-10);EXPECT_NEAR(a.y,0,1e-10);
  const double expected=std::atan(.5*std::tan(.1));
  EXPECT_NEAR(std::atan2(b.y-a.y,b.x-a.x),expected,.005);
}
TEST_F(UpperSteeringRate, RecordedFrozenReplay) {
  const char * fixture=std::getenv("UPPER_STEERING_REPLAY_INPUT");
  const char * output=std::getenv("UPPER_STEERING_REPLAY_OUTPUT");
  if(!fixture||!output) {GTEST_SKIP()<<"Optional closed-log replay fixture";}
  std::ifstream in(fixture);ASSERT_TRUE(in.good());
  std::ofstream out(output);ASSERT_TRUE(out.good());
  out<<std::setprecision(17)<<"sc,seq,rate,recorded_valid,valid,first_heading_deg,model_rate,geometry_rate,baseline_error,status,path,initial_course_error,command_rate,effective_rate,rollout_ms,iterations,fine_path\n";
  int version,count;in>>version>>count;ASSERT_EQ(version,1);
  // Use the original experiment config for all cost, lane and model settings.
  const char * config=std::getenv("UPPER_STEERING_REPLAY_CONFIG");ASSERT_NE(config,nullptr);
  std::vector<std::shared_ptr<imac_ctrl::SdMapUpperPlannerNode>> nodes;
  const bool vehicle=std::getenv("UPPER_VEHICLE_REPLAY")!=nullptr;
  std::map<std::pair<int,int>,std::array<double,5>> feedback;
  if(vehicle) {
    const char * path=std::getenv("UPPER_VEHICLE_REPLAY_FEEDBACK");ASSERT_NE(path,nullptr);
    std::ifstream f(path);int sc,seq;std::array<double,5> x;
    while(f>>sc>>seq>>x[0]>>x[1]>>x[2]>>x[3]>>x[4]) {feedback[{sc,seq}]=x;}
    ASSERT_FALSE(feedback.empty());
  }
  const std::vector<double> limits=vehicle?std::vector<double>{0.0,.5}:std::vector<double>{0.0,.35,.5,.7};
  for(double limit:limits) {
    rclcpp::NodeOptions o;o.arguments({"--ros-args","--params-file",config});
    // Historical replay inputs contain hard-strip bounds, not raw/soft road pairs.
    o.parameter_overrides({{"upper_soft_corridor_constraints",false},
      {"upper_max_steering_rate_radps",limit},{"upper_steering_state_timeout_sec",10.0},
      {"upper_use_vehicle_rollout",vehicle && limit>0},
      {"upper_vehicle_rollout_budget_ms",std::getenv("UPPER_VEHICLE_REPLAY_BUDGET_MS")?std::stod(std::getenv("UPPER_VEHICLE_REPLAY_BUDGET_MS")):60.0}});
    auto n=std::make_shared<imac_ctrl::SdMapUpperPlannerNode>(o);Access::cancel(*n);nodes.push_back(n);
  }
  int matched=0;
  for(int c=0;c<count;++c) {
    int sc,seq,n,recorded,tail;double anchor,incoming,outgoing,turn;Eigen::Vector2d w0,w1;
    in>>sc>>seq>>n>>recorded>>anchor>>tail>>incoming>>outgoing>>turn>>w0.x()>>w0.y()>>w1.x()>>w1.y();
    Access::Preview p;p.N_pred=n;p.side_boundary_tail_start=tail;
    p.road_segment_heading_rad={incoming,outgoing};p.delta_psi_road_rad=turn;
    p.prk.resize(n+1);p.psirk.resize(n+1);p.Nck.assign(n+1,1);p.pmk.resize(n+1);p.pMk.resize(n+1);
    for(auto & xy:p.prk) {in>>xy.x()>>xy.y();}for(auto & h:p.psirk) {in>>h;}
    for(int k=1;k<=n;++k) {
      in>>p.Nck[k];
      for(int j=0;j<p.Nck[k];++j) {
        Eigen::Vector2d a,b;int skip;in>>a.x()>>a.y()>>b.x()>>b.y()>>skip;
        p.pmk[k].push_back(a);p.pMk[k].push_back(b);if(k==1) {p.k1_lane_constraint_excluded.push_back(skip!=0);}
      }
    }
    std::vector<Eigen::Vector2d> expected(n+1);for(auto & xy:expected) {in>>xy.x()>>xy.y();}
    ASSERT_TRUE(in.good())<<c;Access::targets(*nodes[0],p,w0,w1);
    for(std::size_t j=0;j<nodes.size();++j) {
      if(j) {if(vehicle) {ASSERT_TRUE(feedback.count({sc,seq}));Access::feedVehicle(*nodes[j],anchor,feedback.at({sc,seq}));}else {Access::feed(*nodes[j],anchor);}}
      auto r=Access::solve(*nodes[j],p);double error=0,head=NAN;
      if(r.valid) {
        auto d=r.predicted_body[1]-r.predicted_body[0];head=std::atan2(d.y(),d.x())*180/3.141592653589793;
        if(j==0&&recorded) {
          for(int k=0;k<=n;++k) {error=std::max(error,(r.predicted_body[k]-expected[k]).norm());}
          EXPECT_LT(error,.002)<<"sc="<<sc<<" seq="<<seq;++matched;
        }
        if(j && vehicle) {
          EXPECT_TRUE(r.vehicle_rollout.valid);
          EXPECT_LT(std::abs(r.vehicle_rollout.initial_course_error_rad),.01);
          EXPECT_LE(r.vehicle_rollout.trajectory.max_command_rate,limits[j]+2e-6);
          EXPECT_LE(r.vehicle_rollout.trajectory.max_effective_rate,limits[j]+2e-6);
          EXPECT_LE(r.vehicle_rollout.max_corridor_violation,.001001);
        } else if(j) {
          EXPECT_LE(r.max_model_steering_rate_radps,limits[j]+2e-6);
          EXPECT_LE(r.max_geometric_steering_rate_radps,limits[j]+2e-6);
        }
      } else if(j==0&&recorded) {ADD_FAILURE()<<"baseline failed sc="<<sc<<" seq="<<seq<<": "<<r.status;}
      out<<sc<<','<<seq<<','<<limits[j]<<','<<recorded<<','<<r.valid<<','<<head<<','<<r.max_model_steering_rate_radps<<','<<r.max_geometric_steering_rate_radps<<','<<error<<','<<std::quoted(r.status)<<",\"";
      if(r.valid) {for(auto & xy:r.predicted_body) {out<<xy.x()<<':'<<xy.y()<<';';}}
      out<<"\","<<r.vehicle_rollout.initial_course_error_rad<<','<<r.vehicle_rollout.trajectory.max_command_rate
        <<','<<r.vehicle_rollout.trajectory.max_effective_rate<<','<<r.vehicle_rollout.elapsed_ms<<','<<r.vehicle_rollout.iterations<<",\"";
      if(r.valid && r.vehicle_rollout.valid) {for(auto & x:r.vehicle_rollout.trajectory.states) {out<<x.x<<':'<<x.y<<':'<<x.yaw<<':'<<x.steering<<':'<<x.speed<<';';}}
      out<<"\"\n";
    }
  }
  EXPECT_GT(matched,100);std::cout<<"REPLAY cases="<<count<<" baseline matched="<<matched<<std::endl;
}
