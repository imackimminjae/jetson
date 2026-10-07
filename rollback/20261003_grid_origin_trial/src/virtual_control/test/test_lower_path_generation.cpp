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
