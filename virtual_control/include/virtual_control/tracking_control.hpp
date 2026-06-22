#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <Eigen/Dense>
#include <array>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace imac_ctrl
{

class TrackingControllerNode : public rclcpp::Node
{
public:
  TrackingControllerNode();

private:
  struct GridMapSnapshot
  {
    double resolution{0.0};
    int width{0};
    int height{0};
    double origin_x{0.0};
    double origin_y{0.0};
    double origin_yaw{0.0};
    std::string frame_id;
    std::vector<int8_t> data;
    bool valid{false};
  };

  struct PreviewConstraintData
  {
    std::vector<std::vector<Eigen::Vector2d>> pmk;
    std::vector<std::vector<Eigen::Vector2d>> pMk;
    std::vector<int> Nck;
    std::vector<double> mk;
    std::vector<double> nk;
    std::vector<double> ck;
    std::vector<Eigen::Vector2d> prk;
    std::vector<double> psirk;
    int N_pred{0};
  };

  struct SolverControlStep
  {
    double dv_mps{0.0};
    double dpsi_rad{0.0};
    double steer_norm{0.0};
  };

  struct PreviewFallbackDirection
  {
    Eigen::Vector2d dir_before_clamp;
    double yaw_before_clamp{0.0};
    double yaw_after_clamp{0.0};
    bool wp0_usable{false};
  };

  void publishcontrolCmd(double dv_mps, double dpsi_rad, double steer_norm, bool valid_cmd);
  void publishZeroDebugCmd(const std::string & reason);
  bool publishPreviousSolverCmd(const std::string & reason);
  void publishGoalStopCmd(const std::string & reason);

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void poseStampedCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void upperGuidesCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void gridMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  PreviewConstraintData makePreviewConstraints2(
    const std::vector<Eigen::Vector2d>& preview_pts_body,
    const std::vector<double>& preview_psi_rad,
    int N_pred,
    const Eigen::Vector2d& body_origin_world,
    const Eigen::Matrix2d& R_wb,
    const std::vector<double>& lane_offsets,
    double lane_width,
    const GridMapSnapshot* grid_map);

  void controlLoop();

  std::array<int, 2> findWaypointPair(
    const std::vector<Eigen::Vector2d>& waypoints,
    int wp0_idx,
    const Eigen::Vector2d& position,
    const Eigen::Vector2d& velocity,
    double eps) const;

  PreviewFallbackDirection computePreviewFallbackDirection(
    const Eigen::Vector2d& wp0_body,
    const Eigen::Vector2d& wp1_body) const;

  void buildPreviewFromPredictionSeed(
    const Eigen::Vector2d& wp0_body,
    const Eigen::Vector2d& wp1_body,
    int horizon_steps,
    double delta_s,
    const std::vector<Eigen::Vector2d>& p_seed_body,
    std::vector<Eigen::Vector2d>& preview_pts_body,
    std::vector<double>& preview_psi_rad) const;

  void publishPath(const std::vector<Eigen::Vector2d> & pts, const std::string & frame_id);

private:
  double loop_rate_hz_ctrl_{10.0};
  double max_steer_left_{0.52};
  double max_steer_right_{-0.52};
  double v_max_{12.5};
  double a_max_{5.0};
  double wheelbase_{4.7};
  double lane_width_{5.0};
  std::vector<double> lane_offsets_;
  double miqp_big_m_{100.0};
  double w_segment_preference_{1e-3};
  double rd_jerk_{30.0};
  double dpsi_delta_max_rad_{0.05};
  double wp_switch_eps_{5.0};
  std::string grid_map_topic_{"/bev/occupancy_grid"};
  int grid_value_threshold_{0};
  bool grid_positive_is_drivable_{false};
  double preview_line_sample_m_{0.03};
  int preview_min_segment_samples_{1};
  bool require_grid_map_{true};
  bool reset_waypoint_on_path_update_{false};
  double input_timeout_sec_{0.60};
  bool publish_zero_on_failure_{true};
  bool use_upper_guides_{true};
  double upper_guides_timeout_sec_{0.80};
  double upper_guides_change_reset_tol_m_{0.25};
  double goal_stop_distance_m_{0.50};
  bool preview_use_path_tangent_when_seed_empty_{true};
  double preview_max_yaw_rad_{0.35};
  double preview_min_forward_x_m_{0.20};
  double preview_max_lateral_y_m_{0.30};
  double preview_min_wp_norm_m_{0.10};
  bool preview_reset_on_bad_seed_{true};
  double preview_seed_max_lateral_y_m_{0.50};
  double preview_seed_min_forward_x_m_{-0.10};
  std::string state_input_type_{"odom"};
  std::string pose_stamped_topic_{"/motive/vehicle/pose"};
  double pose_x_offset_{0.0};
  double pose_y_offset_{0.0};
  double pose_z_offset_{0.0};
  double pose_yaw_offset_rad_{0.0};
  double pose_position_scale_{1.0};
  bool pose_swap_xy_{false};
  bool pose_invert_x_{false};
  bool pose_invert_y_{false};
  double pose_speed_lpf_alpha_{0.4};
  double pose_max_dt_for_speed_{0.5};

  bool have_pose_{false};
  bool have_grid_map_{false};
  bool have_upper_guides_{false};

  double cur_x_{0.0};
  double cur_y_{0.0};
  double cur_yaw_{0.0};
  double cur_speed_{0.0};
  bool have_prev_pose_stamped_{false};
  Eigen::Vector2d prev_pose_stamped_xy_{Eigen::Vector2d::Zero()};
  rclcpp::Time prev_pose_stamped_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Time last_pose_rx_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_grid_rx_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_upper_guides_rx_time_{0, 0, RCL_ROS_TIME};

  std::vector<Eigen::Vector2d> ref_path_;
  GridMapSnapshot grid_map_;
  std::vector<Eigen::Vector2d> preview_seed_body_;
  std::vector<Eigen::Vector2d> upper_guides_world_;
  std::vector<SolverControlStep> previous_solver_cmds_;
  std::size_t previous_solver_cmd_index_{0};
  double prev_first_dpsi_{0.0};
  bool have_prev_first_dpsi_{false};

  int wp0_index_{0};
  int wp1_index_{0};
  bool waypoint_seeded_{false};
  bool goal_reached_{false};

  mutable std::mutex map_mtx_;
  mutable std::mutex grid_mtx_;
  mutable std::mutex upper_guides_mtx_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_nav_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr upper_guides_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_map_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_debug_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr applied_cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_curve_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mpc_trace_pub_;

  rclcpp::TimerBase::SharedPtr loop_timer_;
};

}  // namespace imac_ctrl
