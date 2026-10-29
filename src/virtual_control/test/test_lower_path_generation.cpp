#include "virtual_control/tracking_control.hpp"
#include "virtual_control/lower_path_geometry.hpp"
#include "virtual_control/planner_stage_timer.hpp"
#include <gtest/gtest.h>
#include <rcl/time.h>
#include <thread>
#include <cstdlib>
#include <fstream>
#include <iomanip>

namespace imac_ctrl
{
struct LowerPathGenerationTestAccess
{
  static void cancel(SdMapUpperPlannerNode & node) {node.planner_timer_->cancel();}
  static void step(SdMapUpperPlannerNode & node) {node.plannerLoop();}
  static void receivePose(SdMapUpperPlannerNode & node,
    const geometry_msgs::msg::PoseStamped::SharedPtr & msg) {node.poseStampedCallback(msg);}
  static auto poseState(const SdMapUpperPlannerNode & node) {return node.pose_;}
  static void receivePath(SdMapUpperPlannerNode & node, const std::string & frame)
  {
    auto message = std::make_shared<nav_msgs::msg::Path>();
    message->header.frame_id = frame;
    geometry_msgs::msg::PoseStamped point;
    point.pose.position.x = 1.25;
    point.pose.position.y = -2.5;
    message->poses.push_back(point);
    point.pose.position.x = 3.75;
    message->poses.push_back(point);
    node.globalPathCallback(message);
  }
  static auto pathState(const SdMapUpperPlannerNode & node)
  {
    return std::make_pair(node.global_path_frame_id_, node.global_path_);
  }
  static void receiveGrid(
    SdMapUpperPlannerNode & node, const nav_msgs::msg::OccupancyGrid::SharedPtr & msg)
  {
    node.gridMapCallback(msg);
  }
  static auto gridOrigin(const SdMapUpperPlannerNode & node) {return node.grid_.origin_body;}
  static bool sampleGrid(
    const SdMapUpperPlannerNode & node, const Eigen::Vector2d & point, int8_t & value)
  {
    return node.sampleGridValue(point, node.grid_, value);
  }
  static auto solveRoad(
    SdMapUpperPlannerNode & node, double minimum_y, bool skip_k1 = false,
    double k2_minimum_y = -5.0, bool skip_tail = false)
  {
    SdMapUpperPlannerNode::PreviewConstraintData preview;
    preview.N_pred = 2;
    preview.prk = {{0, 0}, {1, 0}, {2, 0}};
    preview.psirk = {0, 0, 0};
    preview.position_targets = {{0, 0}, {1, 1}, {2, 2}};
    preview.Nck = {1, 1, 1};
    preview.pmk = {{{0, -5}}, {{1, minimum_y}}, {{2, k2_minimum_y}}};
    preview.pMk = {{{0, 5}}, {{1, minimum_y + 1.0}}, {{2, k2_minimum_y + 10.0}}};
    // This fixture's observed road and preferred strip coincide.
    preview.road_pmk = preview.pmk;
    preview.road_pMk = preview.pMk;
    preview.k1_lane_constraint_excluded = {skip_k1};
    preview.side_boundary_tail_start = skip_tail ? 2 : -1;
    return node.solveUpperMiqp(preview, 1.0);
  }

  static auto clippedRoad(SdMapUpperPlannerNode & node, double heading, bool unknown)
  {
    SdMapUpperPlannerNode::GridMapSnapshot grid;
    grid.valid = true;
    grid.width = grid.height = 128;
    grid.resolution = 0.3125;
    grid.origin_body = {2.95, -20.0};
    grid.data.resize(128 * 128);
    for (int y = 0; y < 128; ++y) {
      for (int x = 0; x < 128; ++x) {
        const double lateral = -20.0 + (y + 0.5) * grid.resolution;
        grid.data[y * 128 + x] = unknown ? -1 : (std::abs(lateral) < 4.0 ? 0 : 100);
      }
    }
    SdMapUpperPlannerNode::PreviewReferenceData reference;
    reference.valid = true;
    reference.points_body = {{0.0, 0.0}, {4.5, 0.0}, {5.5, 0.0}};
    reference.heading_rad = {heading, heading, heading};
    return node.extractPreviewIntervals(
      reference, &grid, SdMapUpperPlannerNode::IntervalCenteringState{});
  }

  static auto sideLoss(SdMapUpperPlannerNode & node, int variant, int side, double angle)
  {
    SdMapUpperPlannerNode::GridMapSnapshot grid;
    grid.valid = true; grid.width = grid.height = 128; grid.resolution = 0.3125;
    grid.origin_yaw_rad = angle;
    const double c = std::cos(angle), sn = std::sin(angle);
    Eigen::Matrix2d rotation; rotation << c, -sn, sn, c;
    grid.origin_body = rotation * Eigen::Vector2d(0, -20);
    grid.data.assign(128 * 128, 100);
    for (int y = 0; y < 128; ++y) {
      for (int x = 0; x < 128; ++x) {
        const double forward = (x + 0.5) * grid.resolution;
        const double lateral = side * (-20 + (y + 0.5) * grid.resolution);
        const bool near = forward > 14 && forward < 18;
        const bool far = variant == 1 && forward > 18;
        if (std::abs(lateral) < 3 || ((near || far) && lateral > 15)) {
          grid.data[y * 128 + x] = 0;
        }
        if (variant == 2 && near && lateral > 18.5) {
          grid.data[y * 128 + x] = -1;  // Interior unknown is not the map edge.
        }
      }
    }
    SdMapUpperPlannerNode::PreviewReferenceData ref;
    ref.valid = true; ref.used_previous_solution = variant != 3;
    for (const auto & point : std::vector<Eigen::Vector2d>{{0,0},{8,0},{16,18.0*side},{20,21.0*side}}) {
      ref.points_body.push_back(rotation * point);
      ref.heading_rad.push_back(angle);
    }
    return node.extractPreviewIntervals(ref, &grid, {});
  }
  using DenseGrid = SdMapUpperPlannerNode::GridMapSnapshot;
  using DenseReference = SdMapUpperPlannerNode::PreviewReferenceData;
  static auto denseExtract(SdMapUpperPlannerNode & node, const DenseGrid & grid,
    const DenseReference & reference, double previous_b = 0.0)
  {
    return node.extractPreviewIntervals(reference, &grid, {previous_b});
  }
  static void k1Freedom(SdMapUpperPlannerNode & node, bool enabled)
  {
    node.preview_interval_k1_disable_extra_centering_ = enabled;
  }
  static auto uniformCenteringRoad(SdMapUpperPlannerNode & node, double half_width)
  {
    DenseGrid grid;
    grid.valid = true; grid.width = 100; grid.height = 200;
    grid.resolution = .1; grid.origin_body = {0., -10.};
    grid.data.assign(20000, 100);
    for (int y = 0; y < 200; ++y) {
      for (int x = 0; x < 100; ++x) {
        if (std::abs(-10. + (y + .5) * .1) < half_width) {grid.data[y * 100 + x] = 0;}
      }
    }
    DenseReference ref; ref.valid = true;
    ref.points_body = {{0,0},{3,0},{6,0}}; ref.heading_rad = {0,0,0};
    return denseExtract(node, grid, ref, half_width * 2);
  }
  static auto denseSynthetic(SdMapUpperPlannerNode & node, bool unknown_edge)
  {
    DenseGrid grid;
    grid.valid = true; grid.width = 100; grid.height = 200;
    grid.resolution = 0.1; grid.origin_body = {0.0, -10.0};
    grid.data.resize(20000);
    for (int gy = 0; gy < 200; ++gy) {
      for (int gx = 0; gx < 100; ++gx) {
        const double x = (gx + 0.5) * 0.1;
        const double y = -10.0 + (gy + 0.5) * 0.1;
        const bool near = x >= 0.5 && x < 2.8;
        const double half_width = near ? 2.4 : (x < 4.5 ? 11.0 : 3.5);
        grid.data[gy * 100 + gx] =
          (unknown_edge && near && y >= half_width) ? -1 :
          (std::abs(y) < half_width ? 0 : 100);
      }
    }
    DenseReference reference;
    reference.valid = true;
    reference.points_body = {{0,0},{3,0},{6,0}};
    reference.heading_rad = {0,0,0};
    return denseExtract(node, grid, reference);
  }

  static auto prefixOnly(const SdMapUpperPlannerNode::PreviewConstraintData & p)
  {
    SdMapUpperPlannerNode::UpperSolveResult r;
    r.valid = true; r.predicted_body = p.prk;
    auto published = SdMapUpperPlannerNode::publishablePrediction(p, r);
    return std::make_pair(published, r.predicted_body);
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

TEST(LowerPathGeneration, ExplicitFrameAliasOnlyRenamesConfirmedGlobalPathFrame)
{
  rclcpp::init(0, nullptr);
  {
    for (bool enable_alias : {false, true}) {
      rclcpp::NodeOptions options;
      options.parameter_overrides({
        rclcpp::Parameter("path_frame_id", "motive_world"),
        rclcpp::Parameter("global_path_frame_alias", enable_alias ? "map" : "")});
      imac_ctrl::SdMapUpperPlannerNode node(options);
      using Access = imac_ctrl::LowerPathGenerationTestAccess;
      Access::cancel(node);
      for (const std::string frame : {"map", "/map", "motive_world", "odom"}) {
        Access::receivePath(node, frame);
        const auto state = Access::pathState(node);
        const bool is_alias = frame == "map" || frame == "/map";
        // Equivalent /map heartbeats preserve the previously stored frame spelling.
        const std::string expected_frame = enable_alias && is_alias ? "motive_world" :
          (frame == "/map" ? "map" : frame);
        EXPECT_EQ(state.first, expected_frame);
        ASSERT_EQ(state.second.size(), 2u);
        EXPECT_DOUBLE_EQ(state.second[0].x(), 1.25);
        EXPECT_DOUBLE_EQ(state.second[0].y(), -2.5);
        EXPECT_DOUBLE_EQ(state.second[1].x(), 3.75);
      }
    }
  }
  rclcpp::shutdown();
}

TEST(LowerPathGeneration, HeaderPoseTimingHandlesBurstsWithoutRefreshingDuplicateFreshness)
{
  rclcpp::init(0, nullptr);
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("pose_use_header_stamp_for_speed", true),
      rclcpp::Parameter("pose_speed_lpf_alpha", 1.)});
    imac_ctrl::SdMapUpperPlannerNode node(options);
    using Access = imac_ctrl::LowerPathGenerationTestAccess;
    Access::cancel(node);
    auto clock = node.get_clock()->get_clock_handle();
    ASSERT_EQ(rcl_enable_ros_time_override(clock), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(clock, 100000000000LL), RCL_RET_OK);
    auto msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
    msg->header.stamp.sec = 10;
    Access::receivePose(node, msg);
    EXPECT_FALSE(Access::poseState(node).speed_valid);
    msg->header.stamp.nanosec = 20000000;
    msg->pose.position.x = .013;
    Access::receivePose(node, msg);
    ASSERT_TRUE(Access::poseState(node).speed_valid);
    EXPECT_NEAR(Access::poseState(node).speed_mps, .65, 1e-12);
    const auto received = Access::poseState(node).received;
    ASSERT_EQ(rcl_set_ros_time_override(clock, 100300000000LL), RCL_RET_OK);
    msg->pose.position.x = 100.;
    Access::receivePose(node, msg);
    EXPECT_EQ(Access::poseState(node).received, received);
    msg->header.stamp.nanosec = 10000000;
    Access::receivePose(node, msg);
    EXPECT_EQ(Access::poseState(node).received, received);
    EXPECT_DOUBLE_EQ(Access::poseState(node).position.x(), .013);
    msg->header.stamp.nanosec = 800000000;
    msg->pose.position.x = .52;
    Access::receivePose(node, msg);
    EXPECT_FALSE(Access::poseState(node).speed_valid);
    msg->header.stamp.nanosec = 820000000;
    msg->pose.position.x = .533;
    Access::receivePose(node, msg);
    ASSERT_TRUE(Access::poseState(node).speed_valid);
    EXPECT_NEAR(Access::poseState(node).speed_mps, .65, 1e-12);
    msg->header.stamp.sec = msg->header.stamp.nanosec = 0;
    Access::receivePose(node, msg);
    EXPECT_FALSE(Access::poseState(node).speed_valid);
    ASSERT_EQ(rcl_set_ros_time_override(clock, 100320000000LL), RCL_RET_OK);
    msg->pose.position.x = .546;
    Access::receivePose(node, msg);
    EXPECT_NEAR(Access::poseState(node).speed_mps, .65, 1e-9);
    imac_ctrl::SdMapUpperPlannerNode legacy;
    Access::cancel(legacy);
    EXPECT_FALSE(legacy.get_parameter("pose_use_header_stamp_for_speed").as_bool());
  }
  rclcpp::shutdown();
}

TEST(LowerPathGeneration, K1FreedomChangesOnlyFirstProcessedStripAndPreservesB)
{
  rclcpp::init(0, nullptr);
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("grid_positive_is_drivable", false),
      rclcpp::Parameter("grid_value_threshold", 1),
      rclcpp::Parameter("preview_interval_boundary_margin_m", 2.0),
      rclcpp::Parameter("preview_interval_soft_ratio", .7)});
    imac_ctrl::SdMapUpperPlannerNode node(options);
    imac_ctrl::LowerPathGenerationTestAccess::cancel(node);
    for (double half_width : {2.45, 5.0}) {
      imac_ctrl::LowerPathGenerationTestAccess::k1Freedom(node, false);
      const auto before = imac_ctrl::LowerPathGenerationTestAccess::uniformCenteringRoad(node, half_width);
      imac_ctrl::LowerPathGenerationTestAccess::k1Freedom(node, true);
      const auto after = imac_ctrl::LowerPathGenerationTestAccess::uniformCenteringRoad(node, half_width);
      ASSERT_EQ(before.N_pred, 2); ASSERT_EQ(after.N_pred, 2);
      EXPECT_DOUBLE_EQ(before.interval_update.state.reference_length, after.interval_update.state.reference_length);
      for (int k = 0; k <= 2; ++k) {
        ASSERT_EQ(before.Nck[k], after.Nck[k]); ASSERT_EQ(after.Nck[k], 1);
        EXPECT_DOUBLE_EQ(before.raw_lengths[k][0], after.raw_lengths[k][0]);
        if (k != 1 || half_width < 3.) {
          EXPECT_NEAR((before.pmk[k][0]-after.pmk[k][0]).norm(), 0., 1e-12);
          EXPECT_NEAR((before.pMk[k][0]-after.pMk[k][0]).norm(), 0., 1e-12);
        } else {
          EXPECT_GT(after.processed_lengths[k][0], before.processed_lengths[k][0]);
          EXPECT_NEAR(after.processed_lengths[k][0], after.raw_lengths[k][0]-4., 1e-12);
        }
      }
    }
  }
  rclcpp::shutdown();
}

TEST(LowerPathGeneration, SideLossRequiresReferenceConnectedMapEdgeAndMissingTail)
{
  rclcpp::init(0, nullptr);
  for (bool enabled : {false, true}) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("guard_side_boundary_terminal_loss", enabled),
      rclcpp::Parameter("grid_positive_is_drivable", false),
      rclcpp::Parameter("grid_value_threshold", 0)});
    auto node = std::make_shared<imac_ctrl::SdMapUpperPlannerNode>(options);
    using Access = imac_ctrl::LowerPathGenerationTestAccess;
    Access::cancel(*node);
    for (int side : {-1, 1}) {
      for (double angle : {0.0, 0.7}) {
        for (int variant : {0, 1, 2, 3}) {
          const auto p = Access::sideLoss(*node, variant, side, angle);
          ASSERT_EQ(p.N_pred, 3);
          const bool expected = enabled && variant == 0;
          EXPECT_EQ(p.side_boundary_tail_start, expected ? 3 : -1);
          EXPECT_EQ(p.side_boundary_side, expected ? side : 0);
          EXPECT_EQ(p.prk.size(), 4u);  // Reference/model size never changes.
          const auto result = Access::prefixOnly(p);
          EXPECT_EQ(result.first.size(), expected ? 3u : 4u);
          EXPECT_EQ(result.second.size(), 4u);  // Full solution still available for carry.
          for (std::size_t j = 0; j < result.first.size(); ++j) {
            EXPECT_LT((result.first[j] - p.prk[j]).norm(), 1e-12);
          }
        }
      }
    }
  }
  rclcpp::shutdown();
}

TEST(LowerPathGeneration, SideTailOmissionKeepsEarlierRoadAndAllVehicleLimits)
{
  rclcpp::init(0, nullptr);
  for (bool enabled : {false, true}) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("upper_soft_corridor_constraints", false),
      rclcpp::Parameter("guard_side_boundary_terminal_loss", enabled)});
    auto node = std::make_shared<imac_ctrl::SdMapUpperPlannerNode>(options);
    using Access = imac_ctrl::LowerPathGenerationTestAccess;
    Access::cancel(*node);
    const auto r = Access::solveRoad(*node, -0.5, false, 100.0, true);
    ASSERT_EQ(r.valid, enabled) << r.status;
    EXPECT_FALSE(Access::solveRoad(*node, 100.0, false, 100.0, true).valid);
    if (r.valid) {
      EXPECT_GE(r.predicted_body[1].y(), -0.5 - 1e-6);
      EXPECT_LE(r.predicted_body[1].y(), 0.5 + 1e-6);
      const double dt = node->get_parameter("upper_prediction_dt_sec").as_double();
      const double phi = std::tan(node->get_parameter("max_steering_angle_rad").as_double()) *
        dt / node->get_parameter("wheelbase").as_double();
      double speed = 1.0;
      for (std::size_t j = 0; j < r.actual_input.size(); j += 2) {
        EXPECT_LE(std::abs(r.actual_input[j]), node->get_parameter("a_max_mps2").as_double() * dt + 1e-6);
        speed += r.actual_input[j];
        EXPECT_LE(std::abs(r.actual_input[j + 1]), phi * speed + 1e-6);
      }
    }
  }
  rclcpp::shutdown();
}

TEST(LowerPathGeneration, K1TrialOnlyMarksObservedFrontClippedCandidates)
{
  rclcpp::init(0, nullptr);
  for (bool enabled : {false, true}) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("skip_front_clipped_k1_lane_constraints", enabled),
      rclcpp::Parameter("grid_positive_is_drivable", false),
      rclcpp::Parameter("grid_value_threshold", 1)});
    auto node = std::make_shared<imac_ctrl::SdMapUpperPlannerNode>(options);
    using Access = imac_ctrl::LowerPathGenerationTestAccess;
    Access::cancel(*node);
    const auto clipped = Access::clippedRoad(*node, 0.7853981633974483, false);
    ASSERT_EQ(clipped.N_pred, 2);
    ASSERT_EQ(clipped.k1_lane_constraint_excluded.size(), 1u);
    EXPECT_EQ(clipped.k1_lane_constraint_excluded[0], enabled);
    EXPECT_EQ(clipped.Nck[2], 1);
    const auto full = Access::clippedRoad(*node, 0.0, false);
    ASSERT_EQ(full.k1_lane_constraint_excluded.size(), 1u);
    EXPECT_FALSE(full.k1_lane_constraint_excluded[0]);
    const auto unknown = Access::clippedRoad(*node, 0.0, true);
    for (bool excluded : unknown.k1_lane_constraint_excluded) {EXPECT_FALSE(excluded);}
  }
  rclcpp::shutdown();
}

TEST(LowerPathGeneration, K1TrialRetainsLaterRoadAndVehicleInputConstraints)
{
  rclcpp::init(0, nullptr);
  for (bool enabled : {false, true}) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("upper_soft_corridor_constraints", false),
      rclcpp::Parameter("skip_front_clipped_k1_lane_constraints", enabled)});
    auto node = std::make_shared<imac_ctrl::SdMapUpperPlannerNode>(options);
    using Access = imac_ctrl::LowerPathGenerationTestAccess;
    Access::cancel(*node);
    const auto result = Access::solveRoad(*node, 100.0, true);
    ASSERT_EQ(result.valid, enabled) << result.status;
    EXPECT_FALSE(Access::solveRoad(*node, 100.0, true, 100.0).valid);
    if (result.valid) {
      const double dt = node->get_parameter("upper_prediction_dt_sec").as_double();
      const double phi = std::tan(node->get_parameter("max_steering_angle_rad").as_double()) *
        dt / node->get_parameter("wheelbase").as_double();
      const double max_dv = node->get_parameter("a_max_mps2").as_double() * dt;
      double speed = 1.0;
      for (std::size_t j = 0; j < result.actual_input.size(); j += 2) {
        EXPECT_LE(std::abs(result.actual_input[j]), max_dv + 1e-6);
        speed += result.actual_input[j];
        EXPECT_LE(std::abs(result.actual_input[j + 1]), phi * speed + 1e-6);
      }
    }
  }
  rclcpp::shutdown();
}

TEST(LowerPathGeneration, GridOriginTrialChangesCellCoordinatesOnlyWhenEnabled)
{
  rclcpp::init(0, nullptr);
  for (const bool enabled : {false, true}) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("grid_origin_x_override_enabled", enabled),
      rclcpp::Parameter("grid_origin_x_override_m", 0.0)});
    auto node = std::make_shared<imac_ctrl::SdMapUpperPlannerNode>(options);
    using Access = imac_ctrl::LowerPathGenerationTestAccess;
    Access::cancel(*node);
    auto grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    grid->header.frame_id = "base_link";
    grid->info.resolution = 1.0;
    grid->info.width = 4;
    grid->info.height = 4;
    grid->info.origin.position.x = 2.95;
    grid->info.origin.position.y = -2.0;
    grid->info.origin.orientation.w = 1.0;
    for (int i = 0; i < 16; ++i) {grid->data.push_back(static_cast<int8_t>(i));}
    Access::receiveGrid(*node, grid);
    EXPECT_DOUBLE_EQ(Access::gridOrigin(*node).x(), enabled ? 0.0 : 2.95);
    EXPECT_DOUBLE_EQ(Access::gridOrigin(*node).y(), -2.0);
    int8_t value = -1;
    EXPECT_EQ(Access::sampleGrid(*node, {0.5, -1.5}, value), enabled);
    EXPECT_TRUE(Access::sampleGrid(*node, {enabled ? 0.5 : 3.45, -1.5}, value));
    EXPECT_EQ(value, 0);
    EXPECT_FALSE(Access::sampleGrid(*node, {enabled ? 4.1 : 7.05, -1.5}, value));
    EXPECT_DOUBLE_EQ(grid->info.origin.position.x, 2.95);
    EXPECT_EQ(grid->data[15], 15);
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

TEST(LowerPathGeneration, DenseBOnlyFindsNearbyWidthWithoutAddingSolverSteps)
{
  rclcpp::init(0, nullptr);
  for (bool unknown_edge : {false, true}) {
    rclcpp::NodeOptions coarse_options;
    coarse_options.parameter_overrides({
      rclcpp::Parameter("grid_positive_is_drivable", false),
      rclcpp::Parameter("grid_value_threshold", 0)});
    rclcpp::NodeOptions dense_options;
    dense_options.parameter_overrides({
      rclcpp::Parameter("grid_positive_is_drivable", false),
      rclcpp::Parameter("grid_value_threshold", 0),
      rclcpp::Parameter("preview_interval_b_sample_spacing_m", 0.6)});
    auto coarse = std::make_shared<imac_ctrl::SdMapUpperPlannerNode>(coarse_options);
    auto dense = std::make_shared<imac_ctrl::SdMapUpperPlannerNode>(dense_options);
    using Access = imac_ctrl::LowerPathGenerationTestAccess;
    Access::cancel(*coarse); Access::cancel(*dense);
    const auto a = Access::denseSynthetic(*coarse, unknown_edge);
    const auto b = Access::denseSynthetic(*dense, unknown_edge);
    EXPECT_EQ(a.N_pred, 2);
    EXPECT_EQ(b.N_pred, a.N_pred);
    EXPECT_EQ(b.Nck, a.Nck);
    EXPECT_EQ(b.raw_lengths, a.raw_lengths);
    EXPECT_EQ(b.prk, a.prk);
    EXPECT_EQ(b.psirk, a.psirk);
    EXPECT_EQ(b.source_modes, a.source_modes);
    EXPECT_EQ(b.fallback_reason_codes, a.fallback_reason_codes);
    EXPECT_TRUE(a.b_probes.empty());
    ASSERT_EQ(b.b_probes.size(), 8u);
    EXPECT_NEAR(a.interval_update.candidate_length, 6.9, 0.11);
    if (unknown_edge) {
      EXPECT_NEAR(b.interval_update.candidate_length, a.interval_update.candidate_length, 1e-12);
      EXPECT_EQ(b.interval_update.candidate_step, 2);
    } else {
      EXPECT_NEAR(b.interval_update.candidate_length, 4.7, 0.11);
      EXPECT_GE(b.interval_update.candidate_step, b.b_original_sample_count);
    }
  }
  rclcpp::shutdown();
}

TEST(LowerPathGeneration, DenseBSpacingRejectsNonfiniteAndOversampling)
{
  rclcpp::init(0, nullptr);
  for (double spacing : {-0.1, 0.01, std::numeric_limits<double>::quiet_NaN()}) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("preview_interval_b_sample_spacing_m", spacing)});
    EXPECT_THROW(std::make_shared<imac_ctrl::SdMapUpperPlannerNode>(options), std::invalid_argument);
  }
  rclcpp::shutdown();
}

// Optional saved-input audit: this test publishes nothing and cancels node timers.
// File has multiple recorded grids/references. Its output is analysis data only.
TEST(LowerPathGeneration, DenseBRecordedFrozenInputs)
{
  const char * fixture = std::getenv("DENSE_B_REPLAY_INPUT");
  if (!fixture) {GTEST_SKIP() << "No optional recorded-input file";}
  const char * destination = std::getenv("DENSE_B_REPLAY_OUTPUT");
  ASSERT_NE(destination, nullptr);
  std::ifstream input(fixture);
  ASSERT_TRUE(input.good());
  std::ofstream output(destination);
  ASSERT_TRUE(output.good());
  output << std::setprecision(17);
  output << "sc,seq,coarse_candidate,coarse_B,dense_candidate,dense_B,dense_source_s,extra_probes,coarse_ms,dense_ms,raw_equal,horizon_equal,coarse_source,dense_source\n";
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions a_options, b_options;
  const std::vector<rclcpp::Parameter> common{
    rclcpp::Parameter("scale_mode", "fullscale"),
    rclcpp::Parameter("grid_positive_is_drivable", false),
    rclcpp::Parameter("grid_value_threshold", 1),
    rclcpp::Parameter("preview_interval_transition_relative_band", 0.25),
    rclcpp::Parameter("guard_side_boundary_terminal_loss", true)};
  a_options.parameter_overrides(common);
  auto dense_parameters = common;
  dense_parameters.emplace_back("preview_interval_b_sample_spacing_m", 0.6);
  b_options.parameter_overrides(dense_parameters);
  auto a_node = std::make_shared<imac_ctrl::SdMapUpperPlannerNode>(a_options);
  auto b_node = std::make_shared<imac_ctrl::SdMapUpperPlannerNode>(b_options);
  using Access = imac_ctrl::LowerPathGenerationTestAccess;
  Access::cancel(*a_node); Access::cancel(*b_node);
  int cases = 0; input >> cases;
  ASSERT_GT(cases, 0);
  for (int i = 0; i < cases; ++i) {
    int sc, seq, n;
    double previous_b, origin_yaw;
    Access::DenseGrid grid; Access::DenseReference reference;
    input >> sc >> seq >> n >> previous_b;
    input >> grid.width >> grid.height >> grid.resolution >> grid.origin_body.x() >>
      grid.origin_body.y() >> origin_yaw;
    grid.origin_yaw_rad = origin_yaw; grid.valid = true;
    grid.data.resize(static_cast<std::size_t>(grid.width * grid.height));
    for (auto & v : grid.data) {int value; input >> value; v = static_cast<int8_t>(value);}
    reference.valid = true; reference.used_previous_solution = true;
    for (int k = 0; k <= n; ++k) {
      double x,y,h; input >> x >> y >> h;
      reference.points_body.emplace_back(x,y); reference.heading_rad.push_back(h);
    }
    ASSERT_TRUE(input.good());
    const auto started = std::chrono::steady_clock::now();
    const auto a = Access::denseExtract(*a_node, grid, reference, previous_b);
    const auto middle = std::chrono::steady_clock::now();
    const auto b = Access::denseExtract(*b_node, grid, reference, previous_b);
    const auto ended = std::chrono::steady_clock::now();
    const bool raw_equal = a.raw_lengths == b.raw_lengths && a.Nck == b.Nck &&
      a.source_modes == b.source_modes && a.fallback_reason_codes == b.fallback_reason_codes;
    EXPECT_TRUE(raw_equal) << "sc=" << sc << " seq=" << seq;
    EXPECT_EQ(a.N_pred, b.N_pred) << "sc=" << sc << " seq=" << seq;
    EXPECT_EQ(a.prk, b.prk); EXPECT_EQ(a.psirk, b.psirk);
    double source_s = -1.0;
    if (b.interval_update.candidate_step >= b.b_original_sample_count) {
      source_s = b.b_probes.at(static_cast<std::size_t>(
        b.interval_update.candidate_step - b.b_original_sample_count)).arc_length_m;
    }
    output << sc << ',' << seq << ',' << a.interval_update.candidate_length << ',' <<
      a.interval_update.state.reference_length << ',' << b.interval_update.candidate_length << ',' <<
      b.interval_update.state.reference_length << ',' << source_s << ',' << b.b_probes.size() << ',' <<
      std::chrono::duration<double,std::milli>(middle - started).count() << ',' <<
      std::chrono::duration<double,std::milli>(ended - middle).count() << ',' <<
      raw_equal << ',' << (a.N_pred == b.N_pred) << ',' << a.interval_update.candidate_step << ',' <<
      b.interval_update.candidate_step << '\n';
  }
  rclcpp::shutdown();
}
