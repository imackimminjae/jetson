#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <imac_interfaces/msg/virtual_control_command.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <Eigen/Dense>
#include <array>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <netinet/in.h>
#include <string>
#include <vector>

extern "C" {
#include <mavlink/v2.0/common/mavlink.h>
// If this include layout is not available in your ROS MAVLink package, use:
// #include <mavlink/common/mavlink.h>
}

namespace imac_ctrl
{

class TrackingControllerNode : public rclcpp::Node
{
public:
  TrackingControllerNode();
  ~TrackingControllerNode() override;

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

  struct VirtualDriveCmd
  {
    double steer{0.0};
    double throttle{0.0};
    double brake{0.0};
  };

  void publishcontrolCmd(double dv_mps, double dpsi_rad, double steer_norm, bool valid_cmd);
  void publishZeroDebugCmd(const std::string & reason);
  bool publishPreviousSolverCmd(const std::string & reason);
  void publishGoalStopCmd(const std::string & reason);

  void updateTargetSpeedProfile(double dt);
  VirtualDriveCmd computeVirtualDriveCmd(double steer_norm, bool valid_cmd, double dt);
  void publishVirtualControlCommand(double steer_norm, bool valid_cmd);
  void publishVirtualStopCommand();

  void mavlinkInitUdp();
  void mavlinkClose();
  void mavlinkPoll();
  bool mavlinkSendMessage(const mavlink_message_t & msg);
  void mavlinkSendManualNeutral();
  void mavlinkSendSetManualMode();
  void mavlinkSendArmCommand(bool arm, bool force);
  void mavlinkManageModeAndArm();
  void mavlinkSendActuator(double throttle_norm, double steering_norm);
  void mavlinkSendHeldActuator();
  double computeThrottleNorm(double dv_mps, bool valid_cmd) const;
  void publishMavlinkCmd(double dv_mps, double steer_norm, bool valid_cmd);

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
  double goal_stop_distance_m_{0.30};
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

  std::string virtual_cmd_topic_{"/cmd_control"};
  double target_speed_mps_{1.0};
  double target_accel_mps2_{0.0};
  double speed_kp_{0.45};
  double speed_ki_{0.05};
  double throttle_ff_{0.08};
  double max_virtual_throttle_{0.35};
  double max_virtual_brake_{0.70};
  double speed_deadband_mps_{0.03};
  double speed_integral_limit_{2.0};
  double target_speed_profile_mps_{0.0};
  double speed_integral_{0.0};
  bool target_speed_profile_initialized_{false};

  bool mavlink_enable_{false};
  std::string mavlink_bind_ip_{"0.0.0.0"};
  int mavlink_bind_port_{14540};
  int mavlink_source_system_{245};
  int mavlink_source_component_{190};
  int mavlink_target_system_{1};
  int mavlink_target_component_{1};
  bool mavlink_send_neutral_on_invalid_{true};
  bool mavlink_auto_manual_mode_{true};
  bool mavlink_auto_arm_{false};
  bool mavlink_force_arm_{false};
  bool mavlink_disarm_on_shutdown_{true};
  double mavlink_throttle_max_norm_{0.2};
  double mavlink_steer_sign_{1.0};
  double last_valid_mavlink_steer_norm_{0.0};
  rclcpp::Time last_valid_mavlink_cmd_time_;
  double mavlink_hold_last_valid_sec_{0.30};
  int mavlink_socket_{-1};
  bool mavlink_peer_known_{false};
  sockaddr_in mavlink_peer_addr_{};
  mavlink_status_t mavlink_rx_status_{};
  bool mavlink_px4_manual_{false};
  bool mavlink_px4_armed_{false};
  rclcpp::Time mavlink_last_mode_request_time_;
  rclcpp::Time mavlink_last_arm_request_time_;
  std::mutex mavlink_mtx_;
  double mavlink_last_throttle_norm_{0.0};
  double mavlink_last_steering_norm_{0.0};
  bool mavlink_have_actuator_cmd_{false};
  std::mutex mavlink_cmd_mtx_;

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
  rclcpp::Publisher<imac_interfaces::msg::VirtualControlCommand>::SharedPtr virtual_cmd_pub_;

  rclcpp::TimerBase::SharedPtr loop_timer_;
  rclcpp::TimerBase::SharedPtr mavlink_keepalive_timer_;
};

}  // namespace imac_ctrl
