#define VIRTUAL_CONTROL_COMPARISON_NO_MAIN
#include "../src/comparison_planner_node.cpp"
#include "virtual_control/lower_path_geometry.hpp"
#include <gtest/gtest.h>
#include <thread>

using namespace imac_ctrl::comparison;  // NOLINT
TEST(ComparisonAdapter, AtomicSnapshotFramesTimestampAndFailureInvalidation)
{
  if (!rclcpp::ok()) {int argc=0; rclcpp::init(argc,nullptr);}
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"planner","mpcc"},{"reference_source","sparse"},{"upper_planner_rate_hz",10.0},
    {"odom_topic","/comparison_test/odom"},{"grid_map_topic","/comparison_test/grid"},
    {"global_path_topic","/comparison_test/route"},
    {"lower_reference_path_topic","/comparison_test/result"},
    {"target_speed_mps",2.0},{"extra_road_margin_m",0.1},
    {"wheelbase",0.3},{"vehicle_length_m",0.42},{"vehicle_width_m",0.2},
    {"reference_point_to_rear_axle_m",0.15},{"upper_preview_steps",3}});
  auto planner=std::make_shared<ComparisonPlannerNode>(options);
  auto peer=std::make_shared<rclcpp::Node>("comparison_test_peer");
  auto latched=rclcpp::QoS(1).reliable().transient_local();
  auto odom=peer->create_publisher<nav_msgs::msg::Odometry>("/comparison_test/odom",10);
  auto grid=peer->create_publisher<nav_msgs::msg::OccupancyGrid>("/comparison_test/grid",10);
  auto route=peer->create_publisher<nav_msgs::msg::Path>("/comparison_test/route",latched);
  imac_interfaces::msg::PathWithArcLength received;
  bool have_valid=false,have_empty_after_valid=false;
  auto sub=peer->create_subscription<imac_interfaces::msg::PathWithArcLength>(
    "/comparison_test/result/with_arclength",latched,
    [&](imac_interfaces::msg::PathWithArcLength::ConstSharedPtr msg) {
      if (!msg->path.poses.empty()) {received=*msg; have_valid=true;}
      else if (have_valid) {have_empty_after_valid=true;}
    });
  rclcpp::executors::SingleThreadedExecutor exec; exec.add_node(peer); exec.add_node(planner);
  nav_msgs::msg::Path path; path.header.frame_id="map";
  for (double x : {0.0,10.0,20.0}) {
    geometry_msgs::msg::PoseStamped p; p.header=path.header; p.pose.position.x=x;
    p.pose.orientation.w=1; path.poses.push_back(p);
  }
  nav_msgs::msg::OccupancyGrid bev; bev.header.frame_id="Vehicle";
  bev.info.width=120; bev.info.height=80; bev.info.resolution=0.25;
  bev.info.origin.position.x=-5; bev.info.origin.position.y=-10;
  bev.info.origin.orientation.w=1; bev.data.assign(120*80,0);
  nav_msgs::msg::Odometry state; state.header.frame_id="map";
  state.twist.twist.linear.x=2;
  double grid_stamp_offset=0;
  auto drive=[&](double seconds) {
      const auto end=std::chrono::steady_clock::now()+std::chrono::duration<double>(seconds);
      while (std::chrono::steady_clock::now()<end) {
        state.header.stamp=peer->now();
        bev.header.stamp=peer->now()+rclcpp::Duration::from_seconds(grid_stamp_offset);
        odom->publish(state); grid->publish(bev); route->publish(path);
        exec.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    };
  drive(1.0);
  ASSERT_TRUE(have_valid);
  EXPECT_EQ(received.path.header.frame_id,"map");
  EXPECT_GT(rclcpp::Time(received.path.header.stamp).nanoseconds(),0);
  ASSERT_EQ(received.path.poses.size(),received.s.size());
  std::vector<Point> points;
  for (const auto & p : received.path.poses) {
    EXPECT_EQ(p.header.frame_id,"map"); EXPECT_EQ(p.header.stamp,received.path.header.stamp);
    points.emplace_back(p.pose.position.x,p.pose.position.y);
  }
  EXPECT_TRUE(imac_ctrl::prepareLowerPath(points,&received.s).valid);
  EXPECT_FALSE(have_empty_after_valid); // Identical route heartbeats preserve the mission.
  bev.header.frame_id="wrong_frame"; drive(0.3); EXPECT_TRUE(have_empty_after_valid);
  bev.header.frame_id="Vehicle"; drive(0.3); have_empty_after_valid=false;
  grid_stamp_offset=-1; drive(0.3); EXPECT_TRUE(have_empty_after_valid);
  grid_stamp_offset=0;
  exec.remove_node(planner); planner.reset();
  // An exact-mode MPCC must not silently consume the latched sparse route.
  options.append_parameter_override("reference_source","exact");
  options.append_parameter_override("exact_reference_topic","/comparison_test/exact");
  planner=std::make_shared<ComparisonPlannerNode>(options); exec.add_node(planner);
  have_valid=false; drive(0.4); EXPECT_FALSE(have_valid);
  auto exact=peer->create_publisher<nav_msgs::msg::Path>("/comparison_test/exact",latched);
  exact->publish(path); drive(0.6); EXPECT_TRUE(have_valid);
  exec.remove_node(planner); exec.remove_node(peer);
}
