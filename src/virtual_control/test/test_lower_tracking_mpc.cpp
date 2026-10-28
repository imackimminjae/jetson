// Compile the real controller in this test target to exercise private prediction,
// solver failure, selection, filtering, and published outputs together.
#include "../src/lower_tracking_mpc_node.cpp"
#include <gtest/gtest.h>
#include <rcl/time.h>
#include <thread>

namespace imac_ctrl
{
struct LowerTrackingMpcTestAccess
{
  using Impl = LowerTrackingMpcNode::Impl;
  static Impl & impl(LowerTrackingMpcNode & node) {return *node.impl_;}
};
}  // namespace imac_ctrl

using imac_ctrl::prepareLowerPath;
using imac_ctrl::LowerPathGeometry;
using Points = std::vector<Eigen::Vector2d>;
constexpr double pi = 3.14159265358979323846;

TEST(LowerPath, StraightAndDuplicateCoordinates)
{
  const Points points{{0, 0}, {1, 0}, {1, 0}, {2, 0}};
  const std::vector<double> s{0, 1, 1, 2};
  const auto path = prepareLowerPath(points, &s);
  ASSERT_TRUE(path.valid);
  EXPECT_EQ(path.s, (std::vector<double>{0, 1, 2}));
  for (double k : path.kappa) {EXPECT_DOUBLE_EQ(k, 0.0);}
}

TEST(LowerPath, SignedCircularCurvatureAndUnwrap)
{
  for (double sign : {-1.0, 1.0}) {
    Points points;
    std::vector<double> s;
    const double radius = 4.0;
    for (int i = 0; i < 50; ++i) {
      const double theta = sign * (1.0 + i * 0.03);
      points.emplace_back(radius * std::cos(theta), radius * std::sin(theta));
      s.push_back(radius * i * 0.03);
    }
    const auto path = prepareLowerPath(points, &s);
    ASSERT_TRUE(path.valid);
    for (std::size_t i = 0; i + 2 < points.size(); ++i) {
      EXPECT_NEAR(path.kappa[i], sign / radius, 1e-12);
    }
    // Repeating the last segment heading gives half curvature at the
    // penultimate point and zero at the endpoint, as specified.
    EXPECT_NEAR(path.kappa[48], sign / (2 * radius), 1e-12);
    EXPECT_DOUBLE_EQ(path.kappa.back(), 0.0);
    for (std::size_t i = 1; i < points.size(); ++i) {
      EXPECT_LT(std::abs(path.psi[i] - path.psi[i - 1]), 0.031);
    }
  }
}

TEST(LowerPath, OriginalInterpolationCoordinatesAtCornerAndShortEnd)
{
  const Points points{{0, 0}, {0.6, 0}, {1, 0.2}, {1, 0.25}};
  const std::vector<double> s{0, 0.6, 1.2, 1.25};
  const auto original = prepareLowerPath(points, &s);
  const auto chord = prepareLowerPath(points);
  ASSERT_TRUE(original.valid);
  ASSERT_TRUE(chord.valid);
  EXPECT_NEAR(original.kappa[0], std::atan2(0.2, 0.4) / 0.6, 1e-12);
  EXPECT_NEAR(original.kappa[1], (pi / 2) / 1.2, 1e-12);
  EXPECT_NEAR(original.kappa[2], (pi / 2 - std::atan2(0.2, 0.4)) / 0.65, 1e-12);
  EXPECT_DOUBLE_EQ(original.kappa[3], 0.0);
  EXPECT_GT(std::abs(original.kappa[2] - chord.kappa[2]), 0.1);
  EXPECT_TRUE(original.original_s);
  EXPECT_FALSE(chord.original_s);
}

TEST(LowerPath, InvalidInputsAreRejected)
{
  EXPECT_FALSE(prepareLowerPath({}).valid);
  EXPECT_FALSE(prepareLowerPath({{1, 1}, {1, 1}}).valid);
  const Points points{{0, 0}, {1, 1}, {2, 2}};
  for (const auto & s : std::vector<std::vector<double>>{
      {0, 1}, {0, 1, 1}, {0, 2, 1}, {0, NAN, 2}, {0, 1, INFINITY}})
  {
    EXPECT_FALSE(prepareLowerPath(points, &s).valid);
  }
  EXPECT_FALSE(prepareLowerPath({{0, 0}, {NAN, 1}, {2, 2}}).valid);
  EXPECT_FALSE(prepareLowerPath({{0, 0}, {1, INFINITY}}).valid);
}

class LowerMpc : public testing::Test
{
protected:
  using Impl = imac_ctrl::LowerTrackingMpcTestAccess::Impl;
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("publish_pwm", true),
      rclcpp::Parameter("mavlink_enable", false)});
    node = std::make_shared<imac_ctrl::LowerTrackingMpcNode>(options);
    impl = &imac_ctrl::LowerTrackingMpcTestAccess::impl(*node);
    impl->controller_timer_->cancel();
    pose.valid = pose.speed_valid = true;
    pose.frame_id = "map";
    pose.received = node->now();
    pose.speed_mps = 1.2;
    pose.position = Eigen::Vector2d(0.02, 0.05);
    pose.yaw_rad = 0.04;
    const std::vector<double> s{0, 0.4, 0.8, 1.2, 1.4};
    path = prepareLowerPath({{0, 0}, {0.4, 0}, {0.78, 0.12}, {1.1, 0.36}, {1.24, 0.50}}, &s);
  }
  void installInputs()
  {
    impl->pose_ = pose;
    impl->path_ = path;
    impl->path_frame_id_ = "map";
    impl->path_received_ = node->now();
  }
  std::vector<double> runAndReadTrace()
  {
    std::vector<double> trace;
    auto sub = node->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/debug/lower_mpc_trace", 10,
      [&](const std_msgs::msg::Float64MultiArray & msg) {trace = msg.data;});
    // Local discovery is synchronous for the intra-process participant;
    // allow middleware delivery without running controller timers.
    impl->controllerLoop();
    const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (trace.empty() && std::chrono::steady_clock::now() < until) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return trace;
  }
  std::shared_ptr<imac_ctrl::LowerTrackingMpcNode> node;
  Impl * impl;
  Impl::PoseSnapshot pose;
  LowerPathGeometry path;
};

TEST_F(LowerMpc, NearestDensePointAndEndRepeat)
{
  pose.position = Eigen::Vector2d(0.39, 0.02);
  const auto result = impl->solveTrackingMpc(pose, path);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.nearest_point, 1);
  const double psi = std::atan2(0.12, 0.38);
  EXPECT_NEAR(result.lateral_error_m, -std::sin(psi) * -0.01 + std::cos(psi) * 0.02, 1e-12);
  EXPECT_NEAR(result.heading_error_rad, pose.yaw_rad - psi, 1e-12);
  for (int k = 0; k < impl->lower_prediction_steps_; ++k) {
    EXPECT_DOUBLE_EQ(result.curvature_reference[k], path.kappa[std::min(k + 1, 4)]);
  }
}

TEST_F(LowerMpc, CondensedPredictionEqualsIndependentStateIteration)
{
  for (bool actuator_enabled : {false, true}) {
    impl->lower_enable_steering_actuator_model_ = actuator_enabled;
    impl->lower_steering_actuator_delay_sec_ = 0.2;
    impl->resetSteeringActuatorModelState();
    impl->estimated_effective_steering_angle_rad_ = 0.07;
    impl->delayed_steering_command_history_rad_ = {0.03, -0.04};
    const auto result = impl->solveTrackingMpc(pose, path);
    ASSERT_TRUE(result.valid);
    Eigen::VectorXd commands(impl->lower_prediction_steps_);
    for (int k = 0; k < commands.size(); ++k) {commands[k] = 0.08 * std::sin(k + 0.3);}
    const Eigen::VectorXd prediction = result.affine_prediction + result.condensed_prediction * commands;
    double ey = result.lateral_error_m, epsi = result.heading_error_rad, effective = 0.07;
    const double dt = impl->lower_prediction_dt_sec_, v = pose.speed_mps, L = impl->wheelbase_;
    const double tau = impl->lower_steering_actuator_time_constant_sec_;
    for (int k = 0; k < commands.size(); ++k) {
      ey += v * dt * epsi;
      double steering_integral = dt * commands[k];
      if (actuator_enabled) {
        const double delayed = k == 0 ? 0.03 : (k == 1 ? -0.04 : commands[k - 2]);
        const double decay = std::exp(-dt / tau);
        steering_integral = tau * (1 - decay) * effective + (dt - tau * (1 - decay)) * delayed;
        effective = decay * effective + (1 - decay) * delayed;
      }
      epsi += v / L * steering_integral;
      EXPECT_NEAR(prediction[2 * k], ey, 1e-12);
      EXPECT_NEAR(prediction[2 * k + 1], epsi, 1e-12);
    }
  }
}

TEST_F(LowerMpc, DelayedNonlinearVehicleRecoversAtRequestedSpeeds)
{
  impl->wheelbase_ = 2.8;
  impl->lower_prediction_dt_sec_ = 0.1;
  impl->lower_prediction_steps_ = 30;
  impl->lower_q_lateral_ = 1.0;
  impl->lower_q_heading_ = 0.5;
  impl->lower_r_steering_ = 0.30;
  impl->lower_rd_steering_rate_ = 8.0;
  impl->lower_max_steering_angle_rad_ = 0.30;
  impl->lower_max_steering_rate_radps_ = 1.0471975512;
  impl->lower_enable_steering_actuator_model_ = true;
  impl->lower_steering_actuator_delay_sec_ = 0.30;
  impl->lower_steering_actuator_time_constant_sec_ = 1.10;

  // A nonlinear bicycle plant runs at 200 Hz, independently of the MPC's
  // 10 Hz linear prediction. Include response mismatch around the fitted model.
  for (double speed : {1.5, 3.0, 6.0}) {
    for (const auto & response : std::vector<std::pair<double, double>>{
        {0.30, 1.10}, {0.20, 0.85}, {0.40, 1.30}})
    {
      SCOPED_TRACE(testing::Message() << "speed=" << speed <<
        " delay=" << response.first << " tau=" << response.second);
      Points points;
      for (int i = 0; i < 500; ++i) {points.emplace_back(i * speed * 0.1, 0.0);}
      const auto straight = prepareLowerPath(points);
      ASSERT_TRUE(straight.valid);
      impl->resetSteeringActuatorModelState();
      pose.speed_mps = speed;
      pose.position = Eigen::Vector2d(0.0, 0.5);
      pose.yaw_rad = 0.05;
      constexpr double plant_dt = 0.005;
      std::deque<double> pending(
        static_cast<std::size_t>(std::lround(response.first / plant_dt)), 0.0);
      double effective = 0.0;
      double previous_command = 0.0;
      double peak_error = std::abs(pose.position.y());
      for (int cycle = 0; cycle < 200; ++cycle) {
        const auto result = impl->solveTrackingMpc(pose, straight);
        ASSERT_TRUE(result.valid) << result.status;
        const double command = result.steering_sequence_rad.front();
        EXPECT_LE(std::abs(command), 0.30 + 1e-7);
        EXPECT_LE(std::abs(command - previous_command),
          impl->lower_max_steering_rate_radps_ * 0.1 + 1e-7);
        previous_command = command;
        for (int substep = 0; substep < 20; ++substep) {
          const double delayed = pending.front();
          pending.pop_front();
          pending.push_back(command);
          pose.position.x() += plant_dt * speed * std::cos(pose.yaw_rad);
          pose.position.y() += plant_dt * speed * std::sin(pose.yaw_rad);
          pose.yaw_rad += plant_dt * speed / 2.8 * std::tan(effective);
          effective += plant_dt * (delayed - effective) / response.second;
        }
        impl->advanceSteeringActuatorModel(command);
        peak_error = std::max(peak_error, std::abs(pose.position.y()));
      }
      EXPECT_LT(peak_error, 0.9);
      EXPECT_LT(std::abs(pose.position.y()), 0.05);
      EXPECT_LT(std::abs(pose.yaw_rad), 0.02);
    }
  }
}

TEST_F(LowerMpc, DaqpCostScalingMatchesIndependentUnconstrainedSolution)
{
  // A one-step interior optimum detects a wrong H/f relative scale or a
  // incorrect heading-error contribution to the linear term.
  impl->lower_prediction_steps_ = 1;
  impl->lower_max_steering_rate_radps_ = 100.0;
  impl->last_applied_steering_angle_rad_ = 0.06;
  const auto result = impl->solveTrackingMpc(pose, path);
  ASSERT_TRUE(result.valid);
  const double b = pose.speed_mps * impl->lower_prediction_dt_sec_ / impl->wheelbase_;
  const double h = result.heading_error_rad - result.tracking_target[1];
  const double expected = (impl->lower_rd_steering_rate_ * 0.06 - impl->lower_q_heading_ * b * h) /
    (impl->lower_q_heading_ * b * b + impl->lower_r_steering_ + impl->lower_rd_steering_rate_);
  EXPECT_NEAR(result.steering_sequence_rad[0], expected, 1e-7);
}

TEST_F(LowerMpc, StationaryQpRespectsFirstAndAllSteeringLimits)
{
  pose.speed_mps = 0;
  impl->last_applied_steering_angle_rad_ = 0.30;
  impl->lower_max_steering_rate_radps_ = 0.05;
  const auto result = impl->solveTrackingMpc(pose, path);
  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(std::isfinite(result.solver_only_time_ms));
  double previous = 0.30;
  for (double delta : result.steering_sequence_rad) {
    EXPECT_LE(std::abs(delta), impl->lower_max_steering_angle_rad_ + 1e-7);
    EXPECT_LE(std::abs(delta - previous), 0.005 + 1e-7);
    previous = delta;
  }
  EXPECT_GT(result.steering_sequence_rad.front(), 0.29);
}

TEST_F(LowerMpc, InfeasibleQpFallbackReachesFilteredOutputAndNextCycleHistory)
{
  installInputs();
  impl->force_qp_failure_for_test_ = true;
  impl->last_applied_steering_angle_rad_ = -0.10;
  impl->previous_filtered_steer_norm_ = -0.10 / impl->lower_max_steering_angle_rad_;
  impl->have_filtered_command_ = true;
  impl->lower_max_steering_rate_radps_ = 0.2;
  // Seed conflicting old commands: neither may replace the new fallback.
  impl->previous_lower_steering_sequence_ = {-0.3, -0.3};
  impl->last_valid_steering_angle_rad_ = -0.3;
  impl->last_valid_steering_time_ = node->now();
  const auto failed = impl->solveTrackingMpc(pose, path);
  ASSERT_FALSE(failed.valid);
  ASSERT_TRUE(failed.fallback_valid);
  const double formula = -0.35 * failed.lateral_error_m - failed.heading_error_rad;
  const double expected = std::clamp(std::clamp(formula, -0.35, 0.35), -0.12, -0.08);
  EXPECT_NEAR(failed.fallback_steering_rad, expected, 1e-12);
  const auto trace = runAndReadTrace();
  ASSERT_EQ(trace.size(), 32u);
  EXPECT_EQ(trace[10], 0);  // QP failed
  EXPECT_EQ(trace[12], 5);  // Error-feedback fallback selected
  EXPECT_NEAR(trace[25], expected, 1e-12);
  EXPECT_EQ(trace[27], 1);
  EXPECT_NEAR(trace[29], -0.10, 1e-12);
  EXPECT_NEAR(trace[26], impl->last_applied_steering_angle_rad_, 1e-8);
  EXPECT_GT(std::abs(trace[25] - trace[26]), 1e-4);  // filter remains active
  const double applied = impl->last_applied_steering_angle_rad_;
  const auto next = impl->solveTrackingMpc(pose, path);
  EXPECT_DOUBLE_EQ(next.delta_prev_rad, applied);
  EXPECT_LE(std::abs(next.fallback_steering_rad - applied), 0.02 + 1e-12);
}

TEST_F(LowerMpc, InvalidAndStopInputsNeverUseErrorFeedbackFallback)
{
  impl->force_qp_failure_for_test_ = true;
  for (int condition = 0; condition < 7; ++condition) {
    installInputs();
    impl->goal_reached_ = false;
    impl->batch_supervision_ = false;
    if (condition == 0) {impl->path_.valid = false;}
    if (condition == 1) {impl->pose_.valid = false;}
    if (condition == 2) {impl->pose_.received = node->now() - rclcpp::Duration::from_seconds(10);}
    if (condition == 3) {impl->path_received_ = node->now() - rclcpp::Duration::from_seconds(10);}
    if (condition == 4) {impl->path_frame_id_ = "other_frame";}
    if (condition == 5) {impl->goal_reached_ = true;}
    if (condition == 6) {impl->batch_supervision_ = true; impl->batch_run_allowed_ = false;}
    const auto trace = runAndReadTrace();
    ASSERT_EQ(trace.size(), 32u);
    EXPECT_NE(trace[12], 5);
    EXPECT_EQ(trace[27], 0);
    EXPECT_EQ(trace[10], 0);
    EXPECT_EQ(trace[14], 0);  // no throttle
  }
}

TEST_F(LowerMpc, RealGoalImmediatelyZerosKeepaliveAndLatchesUntilStationaryReset)
{
  impl->lower_latch_stop_until_reset_ = true;
  installInputs();
  impl->rememberMavlinkActuator(0.1, 0.2, true);
  auto goal = std::make_shared<std_msgs::msg::Bool>();
  goal->data = true;
  impl->goalCallback(goal);  // No controller timer needed for the cached zero output.
  EXPECT_TRUE(impl->stop_latched_);
  EXPECT_EQ(impl->stop_latch_reason_, "goal_reached");
  EXPECT_DOUBLE_EQ(impl->mavlink_last_throttle_norm_, 0.0);
  EXPECT_DOUBLE_EQ(impl->mavlink_last_steering_norm_, 0.0);
  goal->data = false;
  impl->goalCallback(goal);
  installInputs();
  const auto trace = runAndReadTrace();
  ASSERT_EQ(trace.size(), 32u);
  EXPECT_EQ(trace[12], 4);
  EXPECT_EQ(trace[14], 0);
  auto reset = std::make_shared<std_msgs::msg::Bool>();
  reset->data = true;
  impl->resetCallback(reset);  // Moving Motive cannot release the latch.
  EXPECT_TRUE(impl->stop_latched_);
  impl->pose_.speed_mps = 0;
  impl->pose_.received = node->now() - rclcpp::Duration::from_seconds(10);
  impl->resetCallback(reset);
  EXPECT_TRUE(impl->stop_latched_);
  impl->pose_.received = node->now();
  impl->resetCallback(reset);
  EXPECT_FALSE(impl->stop_latched_);
  EXPECT_TRUE(impl->stop_latch_reason_.empty());
}

TEST_F(LowerMpc, RealInputLossCannotRestartFromRecoveredPoseAndNewPath)
{
  impl->lower_latch_stop_until_reset_ = true;
  installInputs();
  const auto moving = runAndReadTrace();
  ASSERT_EQ(moving.size(), 32u);
  ASSERT_TRUE(impl->have_driven_);
  impl->pose_.received = node->now() - rclcpp::Duration::from_seconds(10);
  const auto lost = runAndReadTrace();
  ASSERT_EQ(lost.size(), 32u);
  EXPECT_TRUE(impl->stop_latched_);
  EXPECT_EQ(impl->stop_latch_reason_, "stale pose");
  EXPECT_EQ(lost[14], 0);
  installInputs();  // Valid inputs recover, but no explicit reset occurred.
  const auto recovered = runAndReadTrace();
  ASSERT_EQ(recovered.size(), 32u);
  EXPECT_EQ(recovered[12], 4);
  EXPECT_EQ(recovered[14], 0);
  EXPECT_EQ(impl->stop_latch_reason_, "stale pose");
}

TEST_F(LowerMpc, HeaderStampedPoseIgnoresCallbackBurstAndReorderedSamples)
{
  impl->pose_use_header_stamp_for_speed_ = true;
  impl->pose_speed_lpf_alpha_ = 1.;
  auto clock = node->get_clock()->get_clock_handle();
  ASSERT_EQ(rcl_enable_ros_time_override(clock), RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(clock, 100000000000LL), RCL_RET_OK);
  auto msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
  msg->header.frame_id = "map";
  msg->header.stamp.sec = 10;
  impl->poseStampedCallback(msg);
  EXPECT_FALSE(impl->pose_.speed_valid);
  msg->header.stamp.nanosec = 20000000;
  msg->pose.position.x = .013;
  impl->poseStampedCallback(msg);  // Receipt clock unchanged; measurement interval=20ms.
  ASSERT_TRUE(impl->pose_.speed_valid);
  EXPECT_NEAR(impl->pose_.speed_mps, .65, 1e-12);
  const auto received = impl->pose_.received;
  ASSERT_EQ(rcl_set_ros_time_override(clock, 100300000000LL), RCL_RET_OK);
  msg->pose.position.x = 100.;
  impl->poseStampedCallback(msg);  // Duplicate stamp must not refresh freshness.
  EXPECT_EQ(impl->pose_.received, received);
  EXPECT_DOUBLE_EQ(impl->pose_.position.x(), .013);
  msg->header.stamp.nanosec = 10000000;
  impl->poseStampedCallback(msg);  // Reordered stamp also ignored.
  EXPECT_EQ(impl->pose_.received, received);
  msg->header.stamp.nanosec = 800000000;
  msg->pose.position.x = .52;
  impl->poseStampedCallback(msg);  // Measurement gap still invalidates velocity.
  EXPECT_FALSE(impl->pose_.speed_valid);
  msg->header.stamp.nanosec = 820000000;
  msg->pose.position.x = .533;
  impl->poseStampedCallback(msg);
  ASSERT_TRUE(impl->pose_.speed_valid);
  EXPECT_NEAR(impl->pose_.speed_mps, .65, 1e-12);
  msg->header.stamp.sec = msg->header.stamp.nanosec = 0;
  impl->poseStampedCallback(msg);  // Clock-source change reseeds, never mixes time bases.
  EXPECT_FALSE(impl->pose_.speed_valid);
  ASSERT_EQ(rcl_set_ros_time_override(clock, 100320000000LL), RCL_RET_OK);
  msg->pose.position.x = .546;
  impl->poseStampedCallback(msg);
  ASSERT_TRUE(impl->pose_.speed_valid);
  EXPECT_NEAR(impl->pose_.speed_mps, .65, 1e-9);
}

TEST_F(LowerMpc, LegacyPoseTimingRemainsDefaultAndStopCausesStayDistinct)
{
  EXPECT_FALSE(impl->pose_use_header_stamp_for_speed_);
  auto clock = node->get_clock()->get_clock_handle();
  ASSERT_EQ(rcl_enable_ros_time_override(clock), RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(clock, 100000000000LL), RCL_RET_OK);
  auto msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
  msg->header.stamp.sec = 10;
  impl->poseStampedCallback(msg);
  msg->header.stamp.nanosec = 20000000;
  msg->pose.position.x = .013;
  impl->poseStampedCallback(msg);
  EXPECT_FALSE(impl->pose_.speed_valid);  // Legacy callback dt=0 is still rejected.
  EXPECT_EQ(impl->selectFailureCommand("", false).failure_reason,
    "waiting for lower reference path");
  impl->lower_latch_stop_until_reset_ = true;
  auto stop = std::make_shared<std_msgs::msg::Bool>();
  stop->data = true;
  impl->applyStopSignal(stop, "manual_stop");
  EXPECT_TRUE(impl->stop_latched_);
  EXPECT_EQ(impl->stop_latch_reason_, "manual_stop");
  EXPECT_DOUBLE_EQ(impl->mavlink_last_throttle_norm_, 0.);
  impl->goalCallback(stop);
  EXPECT_EQ(impl->stop_latch_reason_, "manual_stop");  // Preserve the first cause.
}

TEST_F(LowerMpc, AtomicPathValidationAndExternalPolylineMode)
{
  nav_msgs::msg::Path msg;
  msg.header.frame_id = "map";
  for (const auto & point : path.points) {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.pose.position.x = point.x();
    pose_msg.pose.position.y = point.y();
    msg.poses.push_back(pose_msg);
  }
  impl->acceptPath(msg, &path.s);
  EXPECT_TRUE(impl->path_.valid);
  EXPECT_EQ(impl->path_.s, path.s);
  std::vector<double> wrong_s{0};
  impl->acceptPath(msg, &wrong_s);
  EXPECT_FALSE(impl->path_.valid);
  impl->acceptPath(msg, nullptr);
  EXPECT_TRUE(impl->path_.valid);
  EXPECT_FALSE(impl->path_.original_s);
  msg.poses[1].pose.position.x = NAN;
  impl->acceptPath(msg, nullptr);
  EXPECT_FALSE(impl->path_.valid);
}

TEST_F(LowerMpc, ValidReplanPreservesSteeringAnchorUntilExplicitReset)
{
  impl->reset_on_path_change_ = false;
  impl->lower_enable_steering_actuator_model_ = true;
  impl->lower_prediction_dt_sec_ = 0.05;
  impl->lower_steering_actuator_time_constant_sec_ = 0.74;
  installInputs();
  for (auto & point : impl->path_.points) {point.y() += 2.0;}
  ASSERT_TRUE(impl->pathMateriallyDifferent(impl->path_.points, path.points));
  for (int i = 0; i < 20; ++i) {impl->advanceSteeringActuatorModel(0.24);}
  const double effective_before = impl->estimated_effective_steering_angle_rad_;
  ASSERT_GT(effective_before, 0.1);
  impl->reset_pending_ = false;

  nav_msgs::msg::Path msg;
  msg.header.frame_id = "map";
  for (const auto & point : path.points) {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.pose.position.x = point.x();
    pose_msg.pose.position.y = point.y();
    msg.poses.push_back(pose_msg);
  }
  impl->acceptPath(msg, &path.s);
  ASSERT_TRUE(impl->path_.valid);
  EXPECT_FALSE(impl->reset_pending_);
  EXPECT_DOUBLE_EQ(impl->last_applied_steering_angle_rad_, 0.24);
  EXPECT_DOUBLE_EQ(impl->estimated_effective_steering_angle_rad_, effective_before);
  const auto trace = runAndReadTrace();
  ASSERT_EQ(trace.size(), 32u);
  ASSERT_EQ(trace[12], 0);  // Successful QP after a material path change.
  EXPECT_DOUBLE_EQ(trace[29], 0.24);  // Rate limit uses the real preceding command.
  EXPECT_DOUBLE_EQ(trace[19], effective_before);
  EXPECT_LE(std::abs(trace[26] - 0.24),
    impl->lower_max_steering_rate_radps_ * impl->lower_prediction_dt_sec_ + 1e-6);

  auto reset = std::make_shared<std_msgs::msg::Bool>();
  reset->data = true;
  impl->resetCallback(reset);
  ASSERT_TRUE(impl->reset_pending_);
  const auto reset_trace = runAndReadTrace();
  ASSERT_EQ(reset_trace.size(), 32u);
  EXPECT_DOUBLE_EQ(reset_trace[29], 0.0);
  EXPECT_DOUBLE_EQ(reset_trace[19], 0.0);
}

TEST_F(LowerMpc, ReplanResetDisabledStillResetsOnInvalidPathAndPoseJump)
{
  impl->reset_on_path_change_ = false;
  installInputs();
  impl->reset_pending_ = false;
  nav_msgs::msg::Path empty;
  empty.header.frame_id = "map";
  impl->acceptPath(empty, nullptr);
  EXPECT_FALSE(impl->path_.valid);
  EXPECT_TRUE(impl->reset_pending_);

  installInputs();
  impl->reset_pending_ = false;
  auto odom = std::make_shared<nav_msgs::msg::Odometry>();
  odom->header.frame_id = "map";
  odom->pose.pose.position.x = pose.position.x() + impl->pose_jump_reset_threshold_m_ + 0.1;
  odom->pose.pose.position.y = pose.position.y();
  impl->odomCallback(odom);
  EXPECT_TRUE(impl->reset_pending_);
}

TEST_F(LowerMpc, CurvaturePreviewUsesMeasuredTravelOnOriginalCoordinates)
{
  impl->lower_reference_follow_measured_speed_ = true;
  impl->lower_prediction_dt_sec_ = 0.1;
  impl->lower_prediction_steps_ = 5;
  pose.position = Eigen::Vector2d(0.0, 0.0);
  pose.speed_mps = 3.0;
  const std::vector<double> coordinates{0, 0.6, 1.2, 1.25};
  const auto reference = prepareLowerPath({{0, 0}, {0.6, 0}, {1, 0.2}, {1, 0.25}}, &coordinates);
  auto result = impl->solveTrackingMpc(pose, reference);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.reference_horizon[1].x(), 0.3, 1e-12);
  EXPECT_NEAR(result.curvature_reference[1],
    (reference.kappa[0] + reference.kappa[1]) * 0.5, 1e-12);
  EXPECT_NEAR((result.reference_horizon[4] - reference.points[2]).norm(), 0, 1e-12);
  EXPECT_NEAR((result.reference_horizon[5] - reference.points.back()).norm(), 0, 1e-12);
  pose.speed_mps = 0.0;
  result = impl->solveTrackingMpc(pose, reference);
  ASSERT_TRUE(result.valid);
  for (const auto & point : result.reference_horizon) {
    EXPECT_NEAR(point.norm(), 0, 1e-12);
  }
}

TEST_F(LowerMpc, DelayedCurvedVehicleWithSpeedMismatch)
{
  impl->wheelbase_ = 2.8;
  impl->lower_prediction_dt_sec_ = 0.1;
  impl->lower_q_lateral_ = 1.0;
  impl->lower_q_heading_ = 0.5;
  impl->lower_r_steering_ = 0.30;
  impl->lower_rd_steering_rate_ = 8.0;
  impl->lower_max_steering_angle_rad_ = 0.30;
  impl->lower_max_steering_rate_radps_ = 1.0471975512;
  impl->lower_steering_actuator_delay_sec_ = 0.30;
  impl->lower_steering_actuator_time_constant_sec_ = 1.00;
  Points curve;
  std::vector<double> coordinates;
  double length = 0;
  for (int i = 0; i < 600; ++i) {
    const double x = i * 0.6;
    Eigen::Vector2d point(x, 2.0 * std::sin(0.12 * x));
    if (!curve.empty()) {length += (point - curve.back()).norm();}
    curve.push_back(point);
    coordinates.push_back(length);
  }
  const auto reference = prepareLowerPath(curve, &coordinates);
  ASSERT_TRUE(reference.valid);
  for (double cruise_speed : {3.0, 6.0}) {
    for (double plant_tau : {0.85, 1.30}) {
      double rms[2]{};
      double peak[2]{};
      for (int candidate = 0; candidate < 2; ++candidate) {
        impl->lower_prediction_steps_ = candidate ? 30 : 20;
        impl->lower_enable_steering_actuator_model_ = candidate;
        impl->lower_reference_follow_measured_speed_ = candidate;
        impl->resetSteeringActuatorModelState();
        Eigen::Vector2d actual(0.0, 0.5);
        double actual_yaw = std::atan(0.24) + 0.03;
        double yaw_rate = 0.0;
        constexpr double plant_dt = 0.005;
        const double plant_delay = plant_tau > 1 ? 0.4 : 0.2;
        std::deque<double> pending(
          static_cast<std::size_t>(std::lround(plant_delay / plant_dt)), 0.0);
        double squared_error = 0.0;
        for (int cycle = 0; cycle < 350; ++cycle) {
          const double elapsed = cycle * 0.1;
          const double speed = std::min(cruise_speed, 0.5 + 0.4 * elapsed);
          pose.position = actual;
          // Independent measurement error; do not feed the prediction its own state.
          pose.yaw_rad = actual_yaw + 0.017 * std::sin(1.3 * elapsed);
          pose.speed_mps = speed;
          const auto result = impl->solveTrackingMpc(pose, reference);
          ASSERT_TRUE(result.valid) << result.status;
          const double command = result.steering_sequence_rad.front();
          EXPECT_LE(std::abs(command), 0.30 + 1e-7);
          for (int substep = 0; substep < 20; ++substep) {
            const double delayed = pending.front();
            pending.pop_front();
            pending.push_back(command);
            actual.x() += plant_dt * speed * std::cos(actual_yaw);
            actual.y() += plant_dt * speed * std::sin(actual_yaw);
            actual_yaw += plant_dt * yaw_rate;
            // SIH-fit response is yaw rate, not the MPC steering-state equation.
            const double gain = plant_tau > 1 ? 1.1 : 0.9;
            yaw_rate += plant_dt / plant_tau *
              (speed / 2.8 * std::tan(gain * delayed) - yaw_rate);
          }
          impl->advanceSteeringActuatorModel(command);
          const double error = actual.y() - 2.0 * std::sin(0.12 * actual.x());
          peak[candidate] = std::max(peak[candidate], std::abs(error));
          squared_error += error * error;
        }
        rms[candidate] = std::sqrt(squared_error / 350);
      }
      std::cout << "SWAY speed=" << cruise_speed << " tau=" << plant_tau <<
        " baseline_rms=" << rms[0] << " revised_rms=" << rms[1] <<
        " baseline_peak=" << peak[0] << " revised_peak=" << peak[1] << std::endl;
      EXPECT_LT(rms[1], rms[0]);
      EXPECT_LT(peak[1], 1.0);
    }
  }
}

TEST_F(LowerMpc, CenterPredictionMatchesContinuousMotionIntegration)
{
  impl->wheelbase_ = 2.8;
  impl->lower_reference_point_to_rear_axle_m_ = 1.4;
  impl->lower_prediction_steps_ = 12;
  impl->lower_prediction_dt_sec_ = 0.1;
  impl->lower_steering_actuator_time_constant_sec_ = 0.05;
  pose.speed_mps = 6.0;
  for (bool actuator : {false, true}) {
    for (double delay : {0.0, 0.2}) {
      impl->lower_enable_steering_actuator_model_ = actuator;
      impl->lower_steering_actuator_delay_sec_ = delay;
      impl->resetSteeringActuatorModelState();
      impl->estimated_effective_steering_angle_rad_ = 0.07;
      for (double & queued : impl->delayed_steering_command_history_rad_) {queued = -0.03;}
      const auto result = impl->solveTrackingMpc(pose, path);
      ASSERT_TRUE(result.valid);
      Eigen::VectorXd commands(12);
      for (int k = 0; k < 12; ++k) {commands[k] = 0.1 * std::sin(0.6 * k);}
      const Eigen::VectorXd prediction =
        result.affine_prediction + result.condensed_prediction * commands;
      Eigen::Vector3d state(result.lateral_error_m, result.heading_error_rad, 0.07);
      for (int k = 0; k < 12; ++k) {
        const int input_index = k - (actuator ? static_cast<int>(std::lround(delay / 0.1)) : 0);
        const double command = input_index < 0 ? -0.03 : commands[input_index];
        if (!actuator) {state.z() = command;}
        // Independent RK4 integration of continuous lateral, yaw and actuator ODEs.
        const auto derivative = [&](const Eigen::Vector3d & x) -> Eigen::Vector3d {
            return {6.0 * (x.y() + 0.5 * x.z()), 6.0 / 2.8 * x.z(),
              actuator ? (command - x.z()) / 0.05 : 0.0};
          };
        constexpr double dt = 0.001;
        for (int j = 0; j < 100; ++j) {
          const Eigen::Vector3d a = derivative(state);
          const Eigen::Vector3d b = derivative(state + dt * a / 2);
          const Eigen::Vector3d c = derivative(state + dt * b / 2);
          const Eigen::Vector3d d = derivative(state + dt * c);
          state += dt / 6 * (a + 2 * b + 2 * c + d);
        }
        EXPECT_NEAR(prediction[2 * k], state.x(), 1e-8);
        EXPECT_NEAR(prediction[2 * k + 1], state.y() + 0.5 * state.z(), 1e-8);
      }
    }
  }
}

TEST_F(LowerMpc, PhysicalSteeringCalibrationIsSeparateFromPlanningLimit)
{
  impl->lower_max_steering_angle_rad_ = 0.30;
  impl->lower_steering_command_scale_rad_ = 0.35;
  impl->enable_output_filter_ = false;
  EXPECT_NEAR(impl->filterSteering(0.30, node->now()), 0.30 / 0.35, 1e-12);
  EXPECT_NEAR(impl->filterSteering(-0.30, node->now()), -0.30 / 0.35, 1e-12);
  installInputs();
  const auto trace = runAndReadTrace();
  ASSERT_EQ(trace.size(), 32u);
  ASSERT_EQ(trace[10], 1);
  EXPECT_NEAR(trace[9] * 0.35, trace[8], 1e-10);
  EXPECT_NEAR(trace[26], trace[8], 1e-8);
  EXPECT_NEAR(impl->last_applied_steering_angle_rad_, trace[8], 1e-10);
  EXPECT_DOUBLE_EQ(trace[31], 0.35);
}

TEST_F(LowerMpc, FastCenterBicycleTracksBothCurveDirections)
{
  impl->wheelbase_ = 2.8;
  impl->lower_reference_point_to_rear_axle_m_ = 1.4;
  impl->lower_steering_command_scale_rad_ = 0.35;
  impl->lower_prediction_steps_ = 30;
  impl->lower_prediction_dt_sec_ = 0.1;
  impl->lower_q_lateral_ = 1.0;
  impl->lower_q_heading_ = 0.5;
  impl->lower_r_steering_ = 0.30;
  impl->lower_rd_steering_rate_ = 40.0;
  impl->lower_max_steering_angle_rad_ = 0.30;
  impl->lower_max_steering_rate_radps_ = 1.0471975512;
  impl->lower_enable_steering_actuator_model_ = true;
  impl->lower_reference_follow_measured_speed_ = true;
  impl->lower_steering_actuator_delay_sec_ = 0.0;
  impl->lower_steering_actuator_time_constant_sec_ = 0.05;
  impl->enable_output_filter_ = false;
  for (double speed : {3.0, 6.0}) {
    for (double sign : {-1.0, 1.0}) {
      for (double plant_tau : {0.03, 0.08}) {
        Points curve;
        for (int i = 0; i < 800; ++i) {
          const double x = i * 0.3;
          curve.emplace_back(x, sign * 2.0 * std::sin(0.12 * x));
        }
        const auto reference = prepareLowerPath(curve);
        ASSERT_TRUE(reference.valid);
        impl->resetSteeringActuatorModelState();
        Eigen::Vector2d actual(0, sign * 0.5);
        double yaw = sign * (std::atan(0.24) + 0.03);
        double delta = 0;
        std::deque<double> pending(20, 0.0);  // Unmodeled 20 ms transport delay.
        double squared_error = 0, peak_error = 0, late_peak = 0;
        for (int cycle = 0; cycle < 350; ++cycle) {
          pose.position = actual;
          pose.yaw_rad = yaw + 0.017 * std::sin(1.3 * cycle * 0.1);
          pose.speed_mps = speed;
          const auto result = impl->solveTrackingMpc(pose, reference);
          ASSERT_TRUE(result.valid) << result.status;
          const double physical_command = result.steering_sequence_rad.front();
          EXPECT_LE(std::abs(physical_command), 0.30 + 1e-7);
          const double normalized = impl->filterSteering(physical_command, node->now());
          for (int j = 0; j < 100; ++j) {
            const double delayed = pending.front();
            pending.pop_front();
            pending.push_back(0.35 * normalized);
            delta += 0.001 / plant_tau * (delayed - delta);
            const double beta = std::atan(0.5 * std::tan(delta));
            actual.x() += 0.001 * speed * std::cos(yaw + beta);
            actual.y() += 0.001 * speed * std::sin(yaw + beta);
            yaw += 0.001 * speed / 2.8 * std::cos(beta) * std::tan(delta);
          }
          impl->advanceSteeringActuatorModel(normalized * 0.35);
          const double error = actual.y() - sign * 2.0 * std::sin(0.12 * actual.x());
          peak_error = std::max(peak_error, std::abs(error));
          if (cycle > 100) {late_peak = std::max(late_peak, std::abs(error));}
          squared_error += error * error;
        }
        const double rms = std::sqrt(squared_error / 350);
        std::cout << "CENTER speed=" << speed << " sign=" << sign << " tau=" << plant_tau <<
          " rms=" << rms << " peak=" << peak_error << " late_peak=" << late_peak << std::endl;
        EXPECT_LT(peak_error, 0.75);
        EXPECT_LT(rms, 0.20);
        EXPECT_LT(late_peak, 0.30);
      }
    }
  }
}

TEST_F(LowerMpc, SteeringDampingRejectsSmallRepeatedReferenceChanges)
{
  impl->wheelbase_ = 2.8;
  impl->lower_reference_point_to_rear_axle_m_ = 1.4;
  impl->lower_prediction_steps_ = 30;
  impl->lower_prediction_dt_sec_ = .1;
  impl->lower_q_lateral_ = 1;
  impl->lower_q_heading_ = .5;
  impl->lower_r_steering_ = .3;
  impl->lower_max_steering_angle_rad_ = .3;
  impl->lower_max_steering_rate_radps_ = 1.0471975512;
  impl->lower_enable_steering_actuator_model_ = true;
  impl->lower_reference_follow_measured_speed_ = true;
  impl->lower_steering_actuator_delay_sec_ = 0;
  impl->lower_steering_actuator_time_constant_sec_ = .05;
  for (double speed : {3., 6.}) {
    double variation[2]{};
    double lateral_rms[2]{};
    for (int candidate = 0; candidate < 2; ++candidate) {
      impl->lower_rd_steering_rate_ = candidate ? 40. : 8.;
      impl->resetSteeringActuatorModelState();
      Eigen::Vector2d actual(0, 0);
      double yaw = 0, delta = 0, previous = 0;
      LowerPathGeometry reference;
      for (int cycle = 0; cycle < 300; ++cycle) {
        if (cycle % 5 == 0) {
          // A 2 Hz planner moves an otherwise straight reference by 6 cm.
          const double y = (cycle / 5) % 2 ? -.03 : .03;
          reference = prepareLowerPath({{0, y}, {100, y}, {200, y}, {300, y}});
        }
        pose.position = actual;
        pose.yaw_rad = yaw + .0087 * std::sin(2 * pi * .7 * cycle * .1);
        pose.speed_mps = speed;
        const auto result = impl->solveTrackingMpc(pose, reference);
        ASSERT_TRUE(result.valid);
        const double command = result.steering_sequence_rad.front();
        for (int j = 0; j < 100; ++j) {
          delta += .001 / .05 * (command - delta);
          const double beta = std::atan(.5 * std::tan(delta));
          actual.x() += .001 * speed * std::cos(yaw + beta);
          actual.y() += .001 * speed * std::sin(yaw + beta);
          yaw += .001 * speed / 2.8 * std::cos(beta) * std::tan(delta);
        }
        impl->advanceSteeringActuatorModel(command);
        if (cycle >= 50) {
          variation[candidate] += std::abs(command - previous);
          lateral_rms[candidate] += actual.y() * actual.y();
        }
        previous = command;
      }
      lateral_rms[candidate] = std::sqrt(lateral_rms[candidate] / 250);
    }
    std::cout << "DAMPING speed=" << speed << " variation_before=" << variation[0] <<
      " variation_after=" << variation[1] << " lateral_rms_before=" << lateral_rms[0] <<
      " lateral_rms_after=" << lateral_rms[1] << std::endl;
    EXPECT_LT(variation[1], .8 * variation[0]);
    EXPECT_LT(lateral_rms[1], .08);
  }
}

TEST_F(LowerMpc, CurvatureCannotAffectStatePredictionOrFallback)
{
  for (bool actuator : {false, true}) {
    impl->lower_enable_steering_actuator_model_ = actuator;
    impl->resetSteeringActuatorModelState();
    const auto original = impl->solveTrackingMpc(pose, path);
    auto changed = path;
    for (double & curvature : changed.kappa) {curvature = 100.0;}
    const auto diagnostic_only = impl->solveTrackingMpc(pose, changed);
    ASSERT_TRUE(original.valid);
    ASSERT_TRUE(diagnostic_only.valid);
    EXPECT_NE(original.curvature_reference, diagnostic_only.curvature_reference);
    EXPECT_NEAR((original.affine_prediction - diagnostic_only.affine_prediction).norm(), 0, 1e-12);
    EXPECT_NEAR((original.condensed_prediction - diagnostic_only.condensed_prediction).norm(), 0, 1e-12);
    EXPECT_NEAR(original.fallback_steering_rad, diagnostic_only.fallback_steering_rad, 1e-12);
    for (std::size_t k = 0; k < original.steering_sequence_rad.size(); ++k) {
      EXPECT_NEAR(original.steering_sequence_rad[k], diagnostic_only.steering_sequence_rad[k], 1e-10);
    }
  }
}

TEST_F(LowerMpc, FutureGeometryRemainsATrackingTargetWithoutCurvatureInput)
{
  impl->lower_enable_steering_actuator_model_ = true;
  impl->lower_reference_follow_measured_speed_ = true;
  impl->lower_prediction_steps_ = 30;
  impl->resetSteeringActuatorModelState();
  pose.position = Eigen::Vector2d::Zero();
  pose.yaw_rad = 0;
  pose.speed_mps = 1.0;
  const auto straight = prepareLowerPath({{0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}});
  auto bend = prepareLowerPath({{0, 0}, {1, 0}, {2, .2}, {3, .6}, {4, 1.2}});
  // Erase all curvature samples: only the positions and headings may steer.
  for (double & curvature : bend.kappa) {curvature = 0;}
  const auto a = impl->solveTrackingMpc(pose, straight);
  const auto b = impl->solveTrackingMpc(pose, bend);
  ASSERT_TRUE(a.valid);
  ASSERT_TRUE(b.valid);
  EXPECT_NEAR((a.affine_prediction - b.affine_prediction).norm(), 0, 1e-12);
  EXPECT_NEAR(a.tracking_target.norm(), 0, 1e-12);
  EXPECT_GT(b.tracking_target.norm(), .1);
  EXPECT_GT(b.steering_sequence_rad.front(), a.steering_sequence_rad.front() + 1e-3);
}
