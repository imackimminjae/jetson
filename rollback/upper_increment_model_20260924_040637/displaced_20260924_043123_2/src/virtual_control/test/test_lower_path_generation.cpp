#include "virtual_control/tracking_control.hpp"
#include "virtual_control/lower_path_geometry.hpp"
#include "virtual_control/planner_stage_timer.hpp"
#include <gtest/gtest.h>
#include <thread>

namespace imac_ctrl
{
struct LowerPathGenerationTestAccess
{
  static void cancel(SdMapUpperPlannerNode & node) {node.planner_timer_->cancel();}
  static void step(SdMapUpperPlannerNode & node) {node.plannerLoop();}
  static auto solveIncrementFixture(
    SdMapUpperPlannerNode & node, bool curved_reference, double turn_sign = 1.0)
  {
    node.upper_consistent_increment_model_ = true;
    node.upper_prediction_dt_sec_ = 1.0;
    node.wheelbase_ = 1.0;
    node.max_steering_angle_rad_ = 0.3;
    node.a_max_mps2_ = 1.0;
    node.turn_preview_steps_ = 2;
    SdMapUpperPlannerNode::PreviewConstraintData preview;
    preview.N_pred = 3;
    preview.prk = {Eigen::Vector2d::Zero()};
    preview.psirk = {0, 0, 0, 0};
    preview.Nck = {1, 1, 1, curved_reference ? 2 : 1};
    preview.pmk.resize(4);
    preview.pMk.resize(4);
    for (int k = 0; k < 3; ++k) {
      const double heading = curved_reference ? turn_sign * (0.2 + 0.05 * k) : 0.0;
      const double distance = curved_reference ? 1.0 + 0.1 * k : 1.0;
      preview.psirk[k] = heading;
      preview.prk.push_back(preview.prk.back() + distance *
        Eigen::Vector2d(std::cos(heading), std::sin(heading)));
    }
    preview.psirk[3] = preview.psirk[2];
    preview.position_targets = preview.prk;
    preview.road_segment_heading_rad = {{turn_sign * 0.15, turn_sign * 0.30}};
    preview.delta_psi_road_rad = turn_sign * 0.15;
    for (int k = 1; k <= 3; ++k) {
      const Eigen::Vector2d normal(-std::sin(preview.psirk[k]), std::cos(preview.psirk[k]));
      const double y = curved_reference ? 0.0 : turn_sign * 0.01 * k * (k + 1);
      for (int c = 0; c < preview.Nck[k]; ++c) {
        preview.pmk[k].push_back(preview.prk[k] + (y - 1e-6) * normal);
        preview.pMk[k].push_back(preview.prk[k] + (y + 1e-6) * normal);
      }
    }
    return node.solveUpperMiqp(preview, 1.0);
  }
  static auto solveRoad(SdMapUpperPlannerNode & node, double minimum_y)
  {
    SdMapUpperPlannerNode::PreviewConstraintData preview;
    preview.N_pred = 2;
    preview.prk = {{0, 0}, {1, 0}, {2, 0}};
    preview.psirk = {0, 0, 0};
    preview.position_targets = {{0, 0}, {1, 1}, {2, 2}};
    preview.Nck = {1, 1, 1};
    preview.pmk = {{{0, -5}}, {{1, minimum_y}}, {{2, -5}}};
    preview.pMk = {{{0, 5}}, {{1, minimum_y + 1.0}}, {{2, 5}}};
    return node.solveUpperMiqp(preview, 1.0);
  }

  static auto densify(SdMapUpperPlannerNode & node, std::vector<double> & s)
  {
    // dt=0.1 and speed=6 => spacing=0.6, crossing the corner at s=1.0.
    return node.densifyPath({{0, 0}, {1, 0}, {1, 0.25}}, 6.0, s);
  }
  static void publish(SdMapUpperPlannerNode & node,
    const std::vector<Eigen::Vector2d> & points, const std::vector<double> & s)
  {
    node.publishPath(node.dense_path_pub_, points, "map", &s);
  }
};
}  // namespace imac_ctrl

TEST(UpperIncrementModel, RepeatedTurnInputsAccumulateInPublishedGeometry)
{
  rclcpp::init(0, nullptr);
  {
    auto node = std::make_shared<imac_ctrl::SdMapUpperPlannerNode>();
    using Access = imac_ctrl::LowerPathGenerationTestAccess;
    Access::cancel(*node);
    for (const double sign : {-1.0, 1.0}) {
      const auto result = Access::solveIncrementFixture(*node, false, sign);
      ASSERT_TRUE(result.valid) << result.status;
      double speed = 1.0, yaw = 0.0;
      Eigen::Vector2d nonlinear_position = Eigen::Vector2d::Zero();
      for (int k = 0; k < 3; ++k) {
        EXPECT_NEAR(result.actual_input[2 * k + 1], sign * 0.02, 1e-5);
        speed += result.actual_input[2 * k];
        yaw += result.actual_input[2 * k + 1];
        nonlinear_position += speed * Eigen::Vector2d(std::cos(yaw), std::sin(yaw));
        const Eigen::Vector2d segment = result.predicted_body[k + 1] - result.predicted_body[k];
        EXPECT_NEAR(std::atan2(segment.y(), segment.x()), yaw, 1e-4);
        EXPECT_LT((result.predicted_body[k + 1] - nonlinear_position).norm(), 0.003);
        EXPECT_LE(std::abs(result.actual_input[2 * k + 1]), std::tan(0.3) * speed + 1e-6);
      }
    }
  }
  rclcpp::shutdown();
}

TEST(UpperIncrementModel, NominalCurvedReferenceAlreadyMeetsRoadTurnTarget)
{
  rclcpp::init(0, nullptr);
  {
    auto node = std::make_shared<imac_ctrl::SdMapUpperPlannerNode>();
    using Access = imac_ctrl::LowerPathGenerationTestAccess;
    Access::cancel(*node);
    for (const double sign : {-1.0, 1.0}) {
      const auto result = Access::solveIncrementFixture(*node, true, sign);
      ASSERT_TRUE(result.valid) << result.status;
      ASSERT_EQ(result.turn_window_steps.size(), 3u);
      EXPECT_LT(result.turn_cost, 1e-9);
      for (int k = 0; k < 3; ++k) {
        EXPECT_NEAR(result.correction_input[2 * k], 0.0, 1e-5);
        EXPECT_NEAR(result.correction_input[2 * k + 1], 0.0, 1e-5);
        EXPECT_NEAR(result.nominal_input[2 * k], k == 0 ? 0.0 : 0.1, 1e-9);
        EXPECT_NEAR(result.psi_pred_model_rad[k], sign * (0.2 + 0.05 * k), 1e-5);
        EXPECT_NEAR(result.psi_pred_cost_rad[k], result.turn_target_rad[k], 1e-5);
      }
    }
  }
  rclcpp::shutdown();
}

TEST(LowerPathGeneration, DensifierPublishesOriginalCoordinatesWithSamePathSnapshot)
{
  rclcpp::init(0, nullptr);
  {
    auto node = std::make_shared<imac_ctrl::SdMapUpperPlannerNode>();
    using Access = imac_ctrl::LowerPathGenerationTestAccess;
    Access::cancel(*node);
    std::vector<double> s;
    const auto points = Access::densify(*node, s);
    ASSERT_EQ(s.size(), 4u);
    EXPECT_NEAR(s[2], 1.2, 1e-12);
    EXPECT_NEAR(s.back(), 1.25, 1e-12);
    EXPECT_NEAR(points[2].x(), 1.0, 1e-12);
    EXPECT_NEAR(points[2].y(), 0.2, 1e-12);
    nav_msgs::msg::Path::SharedPtr plain;
    imac_interfaces::msg::PathWithArcLength::SharedPtr bundled;
    auto qos = rclcpp::QoS(1).reliable().transient_local();
    auto plain_sub = node->create_subscription<nav_msgs::msg::Path>(
      "/planner/lower_reference_path", qos,
      [&](nav_msgs::msg::Path::SharedPtr msg) {plain = msg;});
    auto bundle_sub = node->create_subscription<imac_interfaces::msg::PathWithArcLength>(
      "/planner/lower_reference_path/with_arclength", qos,
      [&](imac_interfaces::msg::PathWithArcLength::SharedPtr msg) {bundled = msg;});
    Access::publish(*node, points, s);
    const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while ((!plain || !bundled) && std::chrono::steady_clock::now() < until) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    ASSERT_TRUE(plain);
    ASSERT_TRUE(bundled);
    EXPECT_EQ(*plain, bundled->path);
    EXPECT_EQ(bundled->s, s);
    EXPECT_TRUE(imac_ctrl::prepareLowerPath(points, &bundled->s).valid);
  }
  rclcpp::shutdown();
}


TEST(PlannerStageTimer, BlockingTimeIsNotChargedAsCpuWork)
{
  imac_ctrl::PlannerStageTimer timer;
  timer.mark("setup");
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  timer.mark("injected_wait");
  ASSERT_EQ(timer.stages().size(), 2u);
  const auto & blocked = timer.stages().back();
  EXPECT_GE(blocked.wall_ms, 20.0);
  ASSERT_TRUE(std::isfinite(blocked.cpu_ms));
  EXPECT_GE(blocked.cpu_ms, 0.0);
  EXPECT_LT(blocked.cpu_ms, blocked.wall_ms * 0.5);
  double wall_sum = 0.0, cpu_sum = 0.0;
  for (const auto & stage : timer.stages()) {
    wall_sum += stage.wall_ms;
    cpu_sum += stage.cpu_ms;
  }
  EXPECT_NEAR(wall_sum, timer.wallMs(), 1e-6);
  EXPECT_NEAR(cpu_sum, timer.cpuMs(), 1e-6);
}

TEST(LowerPathGeneration, StageReportSurvivesMissingInputEarlyReturn)
{
  rclcpp::init(0, nullptr);
  {
    auto node = std::make_shared<imac_ctrl::SdMapUpperPlannerNode>();
    using Access = imac_ctrl::LowerPathGenerationTestAccess;
    Access::cancel(*node);
    std::string report;
    auto sub = node->create_subscription<std_msgs::msg::String>(
      "/debug/upper_stage_timing", rclcpp::QoS(10).best_effort(),
      [&](const std_msgs::msg::String & msg) {report = msg.data;});
    Access::step(*node);
    const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (report.empty() && std::chrono::steady_clock::now() < until) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    ASSERT_FALSE(report.empty());
    EXPECT_NE(report.find("\"outcome\":\"early_return\""), std::string::npos);
    EXPECT_NE(report.find("pose_snapshot"), std::string::npos);
    EXPECT_NE(report.find("cycle_cleanup"), std::string::npos);
    EXPECT_NE(report.find("\"cpu_ms\":"), std::string::npos);
    EXPECT_EQ(report.find("nan"), std::string::npos);
  }
  rclcpp::shutdown();
}

TEST(LowerPathGeneration, RoadCanRequireTurnBeyondRemovedInitialHeadingLimit)
{
  rclcpp::init(0, nullptr);
  {
    auto node = std::make_shared<imac_ctrl::SdMapUpperPlannerNode>();
    using Access = imac_ctrl::LowerPathGenerationTestAccess;
    Access::cancel(*node);
    const auto feasible = Access::solveRoad(*node, 0.8);
    ASSERT_TRUE(feasible.valid) << feasible.status;
    const auto chord = feasible.predicted_body[1] - feasible.predicted_body[0];
    EXPECT_GT(std::abs(std::atan2(chord.y(), chord.x())), 0.0872664626);
    EXPECT_GE(feasible.predicted_body[1].y(), 0.8 - 1e-6);
    EXPECT_LE(feasible.predicted_body[1].y(), 1.8 + 1e-6);
    const auto blocked = Access::solveRoad(*node, 100.0);
    EXPECT_FALSE(blocked.valid);  // Original road and vehicle input constraints still apply.
  }
  rclcpp::shutdown();
}
