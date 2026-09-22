// Compile the real controller in this test target to exercise private prediction,
// solver failure, selection, filtering, and published outputs together.
#include "../src/lower_tracking_mpc_node.cpp"
#include <gtest/gtest.h>
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
      epsi += v / L * steering_integral - v * dt * result.curvature_reference[k];
      EXPECT_NEAR(prediction[2 * k], ey, 1e-12);
      EXPECT_NEAR(prediction[2 * k + 1], epsi, 1e-12);
    }
  }
}

TEST_F(LowerMpc, DaqpCostScalingMatchesIndependentUnconstrainedSolution)
{
  // A one-step interior optimum detects a wrong H/f relative scale or a
  // missing affine curvature contribution to the linear term.
  impl->lower_prediction_steps_ = 1;
  impl->lower_max_steering_rate_radps_ = 100.0;
  impl->last_applied_steering_angle_rad_ = 0.06;
  const auto result = impl->solveTrackingMpc(pose, path);
  ASSERT_TRUE(result.valid);
  const double b = pose.speed_mps * impl->lower_prediction_dt_sec_ / impl->wheelbase_;
  const double h = result.heading_error_rad - pose.speed_mps * impl->lower_prediction_dt_sec_ * path.kappa[0];
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
  const double formula = std::atan(impl->wheelbase_ * path.kappa[0]) -
    0.35 * failed.lateral_error_m - failed.heading_error_rad;
  const double expected = std::clamp(std::clamp(formula, -0.35, 0.35), -0.12, -0.08);
  EXPECT_NEAR(failed.fallback_steering_rad, expected, 1e-12);
  const auto trace = runAndReadTrace();
  ASSERT_EQ(trace.size(), 30u);
  EXPECT_EQ(trace[10], 0);  // QP failed
  EXPECT_EQ(trace[12], 5);  // Curvature fallback selected
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

TEST_F(LowerMpc, InvalidAndStopInputsNeverUseCurvatureFallback)
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
    ASSERT_EQ(trace.size(), 30u);
    EXPECT_NE(trace[12], 5);
    EXPECT_EQ(trace[27], 0);
    EXPECT_EQ(trace[10], 0);
    EXPECT_EQ(trace[14], 0);  // no throttle
  }
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
