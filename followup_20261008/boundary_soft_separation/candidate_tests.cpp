#include "virtual_control/tracking_control.hpp"
#include "virtual_control/lower_path_geometry.hpp"
#include "virtual_control/planner_stage_timer.hpp"
#include <gtest/gtest.h>
#include <thread>

namespace imac_ctrl
{
struct LowerPathGenerationTestAccess
{
  static auto edgeInsetScene(SdMapUpperPlannerNode & node, int side, double angle, int variant) {
    SdMapUpperPlannerNode::GridMapSnapshot grid;
    grid.valid=true;grid.width=grid.height=128;grid.resolution=.3125;grid.origin_yaw_rad=angle;
    Eigen::Matrix2d R;R<<cos(angle),-sin(angle),sin(angle),cos(angle);
    grid.origin_body=R*Eigen::Vector2d(0,-20);grid.data.assign(128*128,100);
    for(int y=0;y<128;++y)for(int x=0;x<128;++x){
      const double yy=side*(-20+(y+.5)*.3125);
      if(yy>10)grid.data[y*128+x]=0;
      if(variant==2 && yy>19)grid.data[y*128+x]=-1;
      if(variant==3 && yy>19)grid.data[y*128+x]=100;
    }
    SdMapUpperPlannerNode::PreviewReferenceData ref;ref.valid=true;ref.used_previous_solution=true;
    const double yy=variant==1?14:variant==2||variant==3?18:variant==4?9:19;
    ref.points_body={Eigen::Vector2d::Zero(),R*Eigen::Vector2d(8,side*yy),R*Eigen::Vector2d(12,side*yy)};
    ref.heading_rad={angle,angle,angle};
    auto p=node.extractPreviewIntervals(ref,&grid,SdMapUpperPlannerNode::IntervalCenteringState{10});
    // Transform endpoints back to map axes for a symmetric left/right assertion.
    for(size_t k=0;k<p.pmk.size();++k)for(size_t i=0;i<p.pmk[k].size();++i){p.pmk[k][i]=R.transpose()*p.pmk[k][i];p.pMk[k][i]=R.transpose()*p.pMk[k][i];}
    return p;
  }
  static void cancel(SdMapUpperPlannerNode & node) {node.planner_timer_->cancel();}
  static void step(SdMapUpperPlannerNode & node) {node.plannerLoop();}
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
    options.parameter_overrides({rclcpp::Parameter("guard_side_boundary_terminal_loss", enabled)});
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

TEST(LowerPathGeneration, CandidateOnlyRelaxesReferenceExcludingLateralMapEdge) {
  rclcpp::init(0,nullptr);
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("grid_positive_is_drivable",false),rclcpp::Parameter("grid_value_threshold",0),rclcpp::Parameter("preview_interval_soft_ratio",.7)});
  auto node=std::make_shared<imac_ctrl::SdMapUpperPlannerNode>(options);
  using A=imac_ctrl::LowerPathGenerationTestAccess;A::cancel(*node);
  for(int side:{-1,1})for(double angle:{0.,.7})for(int variant:{0,1,2,3,4}){
    auto p=A::edgeInsetScene(*node,side,angle,variant);
    ASSERT_EQ(p.N_pred,2);ASSERT_EQ(p.Nck[1],1);
    const auto& obs=p.interval_info[1][0].observation;
    const double W=obs.length,processed=p.processed_lengths[1][0];
    ASSERT_TRUE(p.interval_info[1][0].treatment.centering_on);
    if(variant==0){
      EXPECT_NEAR(processed,.7*W,1e-8);
      EXPECT_NEAR(std::max(side*p.pmk[1][0].y(),side*p.pMk[1][0].y()),20.,.10001);
      EXPECT_NEAR(p.interval_update.state.reference_length,10.,1e-8);
    }else{
      EXPECT_NEAR(processed,.4*W,1e-8); // Inside/unknown/observed/nonmember controls retain symmetric treatment.
    }
  }
  node.reset();rclcpp::shutdown();
}
