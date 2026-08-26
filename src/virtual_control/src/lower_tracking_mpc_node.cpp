#include "virtual_control/lower_tracking_mpc_node.hpp"

#include "QuadraticProblem.h"
#include "matrix_utils.h"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <imac_interfaces/msg/virtual_control_command.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

extern "C" {
#include <mavlink/v2.0/common/mavlink.h>
}

#ifndef MAV_CMD_DO_SET_ACTUATOR
#define MAV_CMD_DO_SET_ACTUATOR 187
#endif

#ifndef MAV_CMD_COMPONENT_ARM_DISARM
#define MAV_CMD_COMPONENT_ARM_DISARM 400
#endif

namespace
{

double clampd(double value, double lower, double upper)
{
  return std::max(lower, std::min(value, upper));
}

double wrapToPi(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

bool finitePoint(const Eigen::Vector2d & point)
{
  return std::isfinite(point.x()) && std::isfinite(point.y());
}

std::string normalizedFrameId(const std::string & frame_id)
{
  std::string normalized = frame_id;
  while (!normalized.empty() && normalized.front() == '/') {
    normalized.erase(normalized.begin());
  }
  return normalized;
}

bool frameIdsEquivalent(const std::string & lhs, const std::string & rhs)
{
  return normalizedFrameId(lhs) == normalizedFrameId(rhs);
}

double quaternionYaw(const geometry_msgs::msg::Quaternion & orientation)
{
  tf2::Quaternion quaternion(
    orientation.x, orientation.y, orientation.z, orientation.w);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
  return yaw;
}

constexpr uint8_t kPx4CustomMainModeManual = 1;

uint32_t px4CustomMode(uint8_t main_mode, uint8_t sub_mode = 0)
{
  return (static_cast<uint32_t>(sub_mode) << 24) |
         (static_cast<uint32_t>(main_mode) << 16);
}

}  // namespace

namespace imac_ctrl
{

struct LowerTrackingMpcNode::Impl
{
  struct PoseSnapshot
  {
    Eigen::Vector2d position{Eigen::Vector2d::Zero()};
    double yaw_rad{0.0};
    double speed_mps{0.0};
    std::string frame_id;
    rclcpp::Time received{0, 0, RCL_ROS_TIME};
    bool speed_valid{false};
    bool valid{false};
  };

  struct PathProjection
  {
    bool valid{false};
    int segment_index{-1};
    double segment_ratio{0.0};
    double path_s{0.0};
    Eigen::Vector2d point{Eigen::Vector2d::Zero()};
    Eigen::Vector2d tangent{Eigen::Vector2d::UnitX()};
    double path_yaw_rad{0.0};
    double lateral_error_m{0.0};
  };

  struct MpcResult
  {
    bool valid{false};
    std::vector<double> steering_sequence_rad;
    std::vector<Eigen::Vector2d> reference_horizon;
    double objective{0.0};
    double solve_time_ms{0.0};
    double lateral_error_m{0.0};
    double heading_error_rad{0.0};
    int nearest_segment{-1};
    std::string status;
  };

  struct SelectedCommand
  {
    double steering_angle_rad{0.0};
    bool drive_valid{false};
    int fallback_mode{3};  // 0=solver, 1=previous sequence, 2=hold, 3=neutral, 4=goal stop
    std::string reason;
  };

  struct VirtualDriveCommand
  {
    double steer{0.0};
    double throttle{0.0};
    double brake{0.0};
  };

  explicit Impl(LowerTrackingMpcNode & node)
  : node_(node)
  {
    declareAndLoadParameters();
    validateParameters();
    createInterfaces();
    if (mavlinkActive()) {
      mavlinkInitUdp();
      const auto keepalive_period = std::chrono::duration<double>(
        1.0 / std::max(1e-6, mavlink_keepalive_rate_hz_));
      mavlink_keepalive_timer_ = node_.create_wall_timer(
        keepalive_period, [this]() {
          mavlinkPoll();
          mavlinkSendManualNeutral();
          mavlinkManageModeAndArm();
          mavlinkSendHeldActuator();
        });
    }
    const auto period = std::chrono::duration<double>(1.0 / lower_controller_rate_hz_);
    controller_timer_ = node_.create_wall_timer(period, std::bind(&Impl::controllerLoop, this));
    RCLCPP_INFO(
      node_.get_logger(),
      "LowerTrackingMpcNode ready: rate=%.2fHz dt=%.3fs N=%d path=%s "
      "scale=%s wheelbase=%.2fm length=%.2fm bev=%.2fm width=%.2fm margin=%.2fm "
      "backend=%s virtual=%s",
      lower_controller_rate_hz_, lower_prediction_dt_sec_, lower_prediction_steps_,
      lower_reference_path_topic_.c_str(), scale_mode_.c_str(), wheelbase_,
      vehicle_length_m_, bev_forward_m_, preview_interval_nominal_road_width_m_,
      preview_interval_boundary_margin_m_, pixhawk_output_backend_.c_str(),
      virtual_cmd_topic_.c_str());
  }

  ~Impl()
  {
    controller_timer_.reset();
    mavlink_keepalive_timer_.reset();
    rememberMavlinkActuator(0.0, 0.0, false);
    if (mavlinkActive()) {
      mavlinkSendActuator(0.0, 0.0);
      mavlinkSendManualNeutral();
      if (mavlink_disarm_on_shutdown_) {
        mavlinkSendArmCommand(false, false);
      }
    }
    mavlinkClose();
  }

  void declareAndLoadParameters()
  {
    node_.declare_parameter<std::string>("scale_mode", scale_mode_);
    node_.get_parameter("scale_mode", scale_mode_);
    if (scale_mode_ == "fullscale") {
      wheelbase_ = 2.80;
      vehicle_length_m_ = 4.70;
      bev_forward_m_ = 15.0;
      preview_interval_nominal_road_width_m_ = 4.50;
      preview_interval_boundary_margin_m_ = 1.50;
    } else {
      wheelbase_ = 0.30;
      vehicle_length_m_ = 0.42;
      bev_forward_m_ = 1.50;
      preview_interval_nominal_road_width_m_ = 0.45;
      preview_interval_boundary_margin_m_ = 0.15;
    }

    node_.declare_parameter<double>("lower_controller_rate_hz", lower_controller_rate_hz_);
    node_.declare_parameter<double>("lower_prediction_dt_sec", lower_prediction_dt_sec_);
    node_.declare_parameter<int>("lower_prediction_steps", lower_prediction_steps_);
    node_.declare_parameter<double>("lower_q_lateral", lower_q_lateral_);
    node_.declare_parameter<double>("lower_q_heading", lower_q_heading_);
    node_.declare_parameter<double>("lower_r_steering", lower_r_steering_);
    node_.declare_parameter<double>("lower_rd_steering_rate", lower_rd_steering_rate_);
    node_.declare_parameter<double>(
      "lower_max_steering_angle_rad", lower_max_steering_angle_rad_);
    node_.declare_parameter<double>(
      "lower_max_steering_rate_radps", lower_max_steering_rate_radps_);
    node_.declare_parameter<double>("lower_min_path_spacing_m", lower_min_path_spacing_m_);
    node_.declare_parameter<double>(
      "lower_min_effective_speed_mps",
      lower_min_effective_speed_mps_);
    node_.declare_parameter<double>("wheelbase", wheelbase_);
    node_.declare_parameter<double>("vehicle_length_m", vehicle_length_m_);
    node_.declare_parameter<double>("bev_forward_m", bev_forward_m_);
    node_.declare_parameter<double>(
      "preview_interval_nominal_road_width_m",
      preview_interval_nominal_road_width_m_);
    node_.declare_parameter<double>(
      "preview_interval_boundary_margin_m",
      preview_interval_boundary_margin_m_);
    node_.declare_parameter<double>("input_timeout_sec", input_timeout_sec_);
    node_.declare_parameter<double>("lower_path_timeout_sec", lower_path_timeout_sec_);
    node_.declare_parameter<double>("solver_hold_last_valid_sec", solver_hold_last_valid_sec_);
    node_.declare_parameter<double>(
      "path_change_reset_threshold_m",
      path_change_reset_threshold_m_);
    node_.declare_parameter<double>("pose_jump_reset_threshold_m", pose_jump_reset_threshold_m_);
    node_.declare_parameter<bool>("publish_zero_on_failure", publish_zero_on_failure_);
    node_.declare_parameter<bool>(
      "publish_applied_cmd_when_invalid", publish_applied_cmd_when_invalid_);
    node_.declare_parameter<std::string>("state_input_type", state_input_type_);
    node_.declare_parameter<std::string>("odom_topic", odom_topic_);
    node_.declare_parameter<std::string>("pose_stamped_topic", pose_stamped_topic_);
    node_.declare_parameter<bool>(
      "pose_stamped_yaw_is_orientation_z", pose_stamped_yaw_is_orientation_z_);
    node_.declare_parameter<double>("pose_x_offset", pose_x_offset_);
    node_.declare_parameter<double>("pose_y_offset", pose_y_offset_);
    node_.declare_parameter<double>("pose_z_offset", pose_z_offset_);
    node_.declare_parameter<double>("pose_yaw_offset_rad", pose_yaw_offset_rad_);
    node_.declare_parameter<double>("pose_position_scale", pose_position_scale_);
    node_.declare_parameter<bool>("pose_swap_xy", pose_swap_xy_);
    node_.declare_parameter<bool>("pose_invert_x", pose_invert_x_);
    node_.declare_parameter<bool>("pose_invert_y", pose_invert_y_);
    node_.declare_parameter<double>("pose_speed_lpf_alpha", pose_speed_lpf_alpha_);
    node_.declare_parameter<double>("pose_max_dt_for_speed", pose_max_dt_for_speed_);
    node_.declare_parameter<std::string>(
      "lower_reference_path_topic", lower_reference_path_topic_);
    node_.declare_parameter<std::string>("goal_reached_topic", goal_reached_topic_);
    node_.declare_parameter<std::string>("reset_topic", reset_topic_);

    node_.declare_parameter<bool>("enable_output_filter", enable_output_filter_);
    node_.declare_parameter<double>("steer_norm_max", steer_norm_max_);
    node_.declare_parameter<double>("steer_norm_rate_max", steer_norm_rate_max_);
    node_.declare_parameter<double>("steer_norm_lpf_alpha", steer_norm_lpf_alpha_);
    node_.declare_parameter<double>("steer_sign", steer_sign_);

    node_.declare_parameter<std::string>("virtual_cmd_topic", virtual_cmd_topic_);
    node_.declare_parameter<double>("target_speed_mps", target_speed_mps_);
    node_.declare_parameter<double>("target_accel_mps2", target_accel_mps2_);
    node_.declare_parameter<double>("speed_kp", speed_kp_);
    node_.declare_parameter<double>("speed_ki", speed_ki_);
    node_.declare_parameter<double>("throttle_ff", throttle_ff_);
    node_.declare_parameter<double>("max_virtual_throttle", max_virtual_throttle_);
    node_.declare_parameter<double>("max_virtual_brake", max_virtual_brake_);
    node_.declare_parameter<double>("speed_deadband_mps", speed_deadband_mps_);
    node_.declare_parameter<double>("speed_integral_limit", speed_integral_limit_);

    node_.declare_parameter<std::string>("duty_topic", duty_topic_);
    node_.declare_parameter<std::string>("pwm_topic", pwm_topic_);
    node_.declare_parameter<bool>("publish_pwm", publish_pwm_);
    node_.declare_parameter<int>("pwm_min", pwm_min_);
    node_.declare_parameter<int>("pwm_center", pwm_center_);
    node_.declare_parameter<int>("pwm_max", pwm_max_);
    node_.declare_parameter<int>("throttle_pwm_min", throttle_pwm_min_);
    node_.declare_parameter<int>("throttle_pwm_max", throttle_pwm_max_);
    node_.declare_parameter<int>("steering_pwm_min", steering_pwm_min_);
    node_.declare_parameter<int>("steering_pwm_max", steering_pwm_max_);

    node_.declare_parameter<std::string>(
      "pixhawk_output_backend", pixhawk_output_backend_);
    node_.declare_parameter<bool>("mavlink_enable", mavlink_enable_);
    node_.declare_parameter<std::string>("mavlink_bind_ip", mavlink_bind_ip_);
    node_.declare_parameter<int>("mavlink_bind_port", mavlink_bind_port_);
    node_.declare_parameter<int>("mavlink_source_system", mavlink_source_system_);
    node_.declare_parameter<int>("mavlink_source_component", mavlink_source_component_);
    node_.declare_parameter<int>("mavlink_target_system", mavlink_target_system_);
    node_.declare_parameter<int>("mavlink_target_component", mavlink_target_component_);
    node_.declare_parameter<bool>(
      "mavlink_send_neutral_on_invalid", mavlink_send_neutral_on_invalid_);
    node_.declare_parameter<double>(
      "mavlink_hold_last_valid_sec", mavlink_hold_last_valid_sec_);
    node_.declare_parameter<bool>("mavlink_auto_manual_mode", mavlink_auto_manual_mode_);
    node_.declare_parameter<bool>("mavlink_auto_arm", mavlink_auto_arm_);
    node_.declare_parameter<bool>("mavlink_force_arm", mavlink_force_arm_);
    node_.declare_parameter<bool>("mavlink_disarm_on_shutdown", mavlink_disarm_on_shutdown_);
    node_.declare_parameter<double>("mavlink_throttle_max_norm", mavlink_throttle_max_norm_);
    node_.declare_parameter<double>("mavlink_steer_sign", mavlink_steer_sign_);
    node_.declare_parameter<double>("mavlink_keepalive_rate_hz", mavlink_keepalive_rate_hz_);

    node_.get_parameter("lower_controller_rate_hz", lower_controller_rate_hz_);
    node_.get_parameter("lower_prediction_dt_sec", lower_prediction_dt_sec_);
    node_.get_parameter("lower_prediction_steps", lower_prediction_steps_);
    node_.get_parameter("lower_q_lateral", lower_q_lateral_);
    node_.get_parameter("lower_q_heading", lower_q_heading_);
    node_.get_parameter("lower_r_steering", lower_r_steering_);
    node_.get_parameter("lower_rd_steering_rate", lower_rd_steering_rate_);
    node_.get_parameter("lower_max_steering_angle_rad", lower_max_steering_angle_rad_);
    node_.get_parameter("lower_max_steering_rate_radps", lower_max_steering_rate_radps_);
    node_.get_parameter("lower_min_path_spacing_m", lower_min_path_spacing_m_);
    node_.get_parameter("lower_min_effective_speed_mps", lower_min_effective_speed_mps_);
    node_.get_parameter("wheelbase", wheelbase_);
    node_.get_parameter("vehicle_length_m", vehicle_length_m_);
    node_.get_parameter("bev_forward_m", bev_forward_m_);
    node_.get_parameter(
      "preview_interval_nominal_road_width_m",
      preview_interval_nominal_road_width_m_);
    node_.get_parameter(
      "preview_interval_boundary_margin_m",
      preview_interval_boundary_margin_m_);
    node_.get_parameter("input_timeout_sec", input_timeout_sec_);
    node_.get_parameter("lower_path_timeout_sec", lower_path_timeout_sec_);
    node_.get_parameter("solver_hold_last_valid_sec", solver_hold_last_valid_sec_);
    node_.get_parameter("path_change_reset_threshold_m", path_change_reset_threshold_m_);
    node_.get_parameter("pose_jump_reset_threshold_m", pose_jump_reset_threshold_m_);
    node_.get_parameter("publish_zero_on_failure", publish_zero_on_failure_);
    node_.get_parameter(
      "publish_applied_cmd_when_invalid", publish_applied_cmd_when_invalid_);
    node_.get_parameter("state_input_type", state_input_type_);
    node_.get_parameter("odom_topic", odom_topic_);
    node_.get_parameter("pose_stamped_topic", pose_stamped_topic_);
    node_.get_parameter("pose_stamped_yaw_is_orientation_z", pose_stamped_yaw_is_orientation_z_);
    node_.get_parameter("pose_x_offset", pose_x_offset_);
    node_.get_parameter("pose_y_offset", pose_y_offset_);
    node_.get_parameter("pose_z_offset", pose_z_offset_);
    node_.get_parameter("pose_yaw_offset_rad", pose_yaw_offset_rad_);
    node_.get_parameter("pose_position_scale", pose_position_scale_);
    node_.get_parameter("pose_swap_xy", pose_swap_xy_);
    node_.get_parameter("pose_invert_x", pose_invert_x_);
    node_.get_parameter("pose_invert_y", pose_invert_y_);
    node_.get_parameter("pose_speed_lpf_alpha", pose_speed_lpf_alpha_);
    node_.get_parameter("pose_max_dt_for_speed", pose_max_dt_for_speed_);
    node_.get_parameter("lower_reference_path_topic", lower_reference_path_topic_);
    node_.get_parameter("goal_reached_topic", goal_reached_topic_);
    node_.get_parameter("reset_topic", reset_topic_);
    node_.get_parameter("enable_output_filter", enable_output_filter_);
    node_.get_parameter("steer_norm_max", steer_norm_max_);
    node_.get_parameter("steer_norm_rate_max", steer_norm_rate_max_);
    node_.get_parameter("steer_norm_lpf_alpha", steer_norm_lpf_alpha_);
    node_.get_parameter("steer_sign", steer_sign_);
    node_.get_parameter("virtual_cmd_topic", virtual_cmd_topic_);
    node_.get_parameter("target_speed_mps", target_speed_mps_);
    node_.get_parameter("target_accel_mps2", target_accel_mps2_);
    node_.get_parameter("speed_kp", speed_kp_);
    node_.get_parameter("speed_ki", speed_ki_);
    node_.get_parameter("throttle_ff", throttle_ff_);
    node_.get_parameter("max_virtual_throttle", max_virtual_throttle_);
    node_.get_parameter("max_virtual_brake", max_virtual_brake_);
    node_.get_parameter("speed_deadband_mps", speed_deadband_mps_);
    node_.get_parameter("speed_integral_limit", speed_integral_limit_);
    node_.get_parameter("duty_topic", duty_topic_);
    node_.get_parameter("pwm_topic", pwm_topic_);
    node_.get_parameter("publish_pwm", publish_pwm_);
    node_.get_parameter("pwm_min", pwm_min_);
    node_.get_parameter("pwm_center", pwm_center_);
    node_.get_parameter("pwm_max", pwm_max_);
    node_.get_parameter("throttle_pwm_min", throttle_pwm_min_);
    node_.get_parameter("throttle_pwm_max", throttle_pwm_max_);
    node_.get_parameter("steering_pwm_min", steering_pwm_min_);
    node_.get_parameter("steering_pwm_max", steering_pwm_max_);
    node_.get_parameter("pixhawk_output_backend", pixhawk_output_backend_);
    node_.get_parameter("mavlink_enable", mavlink_enable_);
    node_.get_parameter("mavlink_bind_ip", mavlink_bind_ip_);
    node_.get_parameter("mavlink_bind_port", mavlink_bind_port_);
    node_.get_parameter("mavlink_source_system", mavlink_source_system_);
    node_.get_parameter("mavlink_source_component", mavlink_source_component_);
    node_.get_parameter("mavlink_target_system", mavlink_target_system_);
    node_.get_parameter("mavlink_target_component", mavlink_target_component_);
    node_.get_parameter("mavlink_send_neutral_on_invalid", mavlink_send_neutral_on_invalid_);
    node_.get_parameter("mavlink_hold_last_valid_sec", mavlink_hold_last_valid_sec_);
    node_.get_parameter("mavlink_auto_manual_mode", mavlink_auto_manual_mode_);
    node_.get_parameter("mavlink_auto_arm", mavlink_auto_arm_);
    node_.get_parameter("mavlink_force_arm", mavlink_force_arm_);
    node_.get_parameter("mavlink_disarm_on_shutdown", mavlink_disarm_on_shutdown_);
    node_.get_parameter("mavlink_throttle_max_norm", mavlink_throttle_max_norm_);
    node_.get_parameter("mavlink_steer_sign", mavlink_steer_sign_);
    node_.get_parameter("mavlink_keepalive_rate_hz", mavlink_keepalive_rate_hz_);
    target_speed_profile_mps_ = std::max(0.0, target_speed_mps_);
  }

  void validateParameters() const
  {
    auto fail = [this](const std::string & message) {
        RCLCPP_FATAL(node_.get_logger(), "invalid lower MPC parameter: %s", message.c_str());
        throw std::invalid_argument(message);
      };
    if (lower_controller_rate_hz_ <= 0.0) {fail("lower_controller_rate_hz must be > 0");}
    if (lower_prediction_dt_sec_ <= 0.0) {fail("lower_prediction_dt_sec must be > 0");}
    if (lower_prediction_steps_ <= 0) {fail("lower_prediction_steps must be > 0");}
    if (scale_mode_ != "model" && scale_mode_ != "fullscale") {
      fail("scale_mode must be 'model' or 'fullscale'");
    }
    if (wheelbase_ <= 0.0) {fail("wheelbase must be > 0");}
    if (vehicle_length_m_ <= 0.0) {fail("vehicle_length_m must be > 0");}
    if (bev_forward_m_ <= 0.0) {fail("bev_forward_m must be > 0");}
    if (preview_interval_nominal_road_width_m_ <= 0.0) {
      fail("preview_interval_nominal_road_width_m must be > 0");
    }
    if (preview_interval_boundary_margin_m_ < 0.0) {
      fail("preview_interval_boundary_margin_m must be >= 0");
    }
    if (lower_max_steering_angle_rad_ <= 0.0) {
      fail("lower_max_steering_angle_rad must be > 0");
    }
    if (lower_max_steering_rate_radps_ < 0.0) {
      fail("lower_max_steering_rate_radps must be >= 0");
    }
    if (lower_min_path_spacing_m_ <= 0.0 || lower_min_effective_speed_mps_ <= 0.0) {
      fail("lower path spacing and minimum effective speed must be > 0");
    }
    if (input_timeout_sec_ < 0.0 || lower_path_timeout_sec_ < 0.0 ||
      solver_hold_last_valid_sec_ < 0.0 || mavlink_hold_last_valid_sec_ < 0.0)
    {
      fail("timeouts must be >= 0");
    }
    if (steer_norm_max_ <= 0.0 || steer_norm_rate_max_ < 0.0) {
      fail("normalized steering limit must be > 0 and rate limit >= 0");
    }
    if (steer_norm_lpf_alpha_ < 0.0 || steer_norm_lpf_alpha_ > 1.0) {
      fail("steer_norm_lpf_alpha must be in [0,1]");
    }
    if (state_input_type_ != "pose_stamped" && state_input_type_ != "odom") {
      fail("state_input_type must be 'pose_stamped' or 'odom'");
    }
    if (!std::isfinite(pose_position_scale_) || std::abs(pose_position_scale_) < 1e-12) {
      fail("pose_position_scale must be finite and nonzero");
    }
    if (pose_speed_lpf_alpha_ < 0.0 || pose_speed_lpf_alpha_ > 1.0) {
      fail("pose_speed_lpf_alpha must be in [0,1]");
    }
    if (pose_max_dt_for_speed_ <= 0.0) {
      fail("pose_max_dt_for_speed must be > 0");
    }
    if (pixhawk_output_backend_ != "disabled" &&
      pixhawk_output_backend_ != "mavlink_udp")
    {
      fail(
        "pixhawk_output_backend supports only 'disabled' and 'mavlink_udp'; no MAVROS implementation exists in this repository");
    }
    if (mavlink_keepalive_rate_hz_ <= 0.0) {
      fail("mavlink_keepalive_rate_hz must be > 0");
    }
  }

  void createInterfaces()
  {
    auto path_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    path_sub_ = node_.create_subscription<nav_msgs::msg::Path>(
      lower_reference_path_topic_, path_qos,
      std::bind(&Impl::pathCallback, this, std::placeholders::_1));
    goal_sub_ = node_.create_subscription<std_msgs::msg::Bool>(
      goal_reached_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&Impl::goalCallback, this, std::placeholders::_1));
    reset_sub_ = node_.create_subscription<std_msgs::msg::Bool>(
      reset_topic_, 10, std::bind(&Impl::resetCallback, this, std::placeholders::_1));
    if (state_input_type_ == "odom") {
      odom_sub_ = node_.create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10, std::bind(&Impl::odomCallback, this, std::placeholders::_1));
    } else {
      pose_sub_ = node_.create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_stamped_topic_, 10,
        std::bind(&Impl::poseStampedCallback, this, std::placeholders::_1));
    }

    raw_cmd_pub_ = node_.create_publisher<geometry_msgs::msg::Twist>(
      "/debug/control_cmd", 10);
    applied_cmd_pub_ = node_.create_publisher<geometry_msgs::msg::Twist>(
      "/control/applied_cmd", 10);
    local_curve_pub_ = node_.create_publisher<nav_msgs::msg::Path>(
      "/controller/local_curve", 10);
    lower_trace_pub_ = node_.create_publisher<std_msgs::msg::Float64MultiArray>(
      "/debug/lower_mpc_trace", 10);
    virtual_cmd_pub_ = node_.create_publisher<imac_interfaces::msg::VirtualControlCommand>(
      virtual_cmd_topic_, 10);
    duty_pub_ = node_.create_publisher<std_msgs::msg::Float64MultiArray>(duty_topic_, 10);
    if (publish_pwm_) {
      pwm_pub_ = node_.create_publisher<std_msgs::msg::Int32MultiArray>(pwm_topic_, 10);
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    PoseSnapshot next;
    next.position = Eigen::Vector2d(
      message->pose.pose.position.x, message->pose.pose.position.y);
    next.yaw_rad = quaternionYaw(message->pose.pose.orientation);
    next.speed_mps = std::hypot(
      message->twist.twist.linear.x, message->twist.twist.linear.y);
    next.frame_id = message->header.frame_id;
    next.received = node_.now();
    next.speed_valid = std::isfinite(next.speed_mps);
    next.valid = finitePoint(next.position) && std::isfinite(next.yaw_rad) && next.speed_valid;
    std::lock_guard<std::mutex> lock(data_mtx_);
    if (pose_.valid && (next.position - pose_.position).norm() > pose_jump_reset_threshold_m_) {
      reset_pending_ = true;
    }
    pose_ = next;
  }

  void poseStampedCallback(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    double x = message->pose.position.x;
    double y = message->pose.position.y;
    if (pose_swap_xy_) {std::swap(x, y);}
    if (pose_invert_x_) {x = -x;}
    if (pose_invert_y_) {y = -y;}
    x = pose_position_scale_ * x + pose_x_offset_;
    y = pose_position_scale_ * y + pose_y_offset_;
    const double transformed_z =
      pose_position_scale_ * message->pose.position.z + pose_z_offset_;
    (void)transformed_z;

    PoseSnapshot next;
    next.position = Eigen::Vector2d(x, y);
    const double raw_yaw = pose_stamped_yaw_is_orientation_z_ ?
      message->pose.orientation.z : quaternionYaw(message->pose.orientation);
    next.yaw_rad = wrapToPi(raw_yaw + pose_yaw_offset_rad_);
    next.frame_id = message->header.frame_id;
    next.received = node_.now();
    next.valid = finitePoint(next.position) && std::isfinite(next.yaw_rad);
    std::lock_guard<std::mutex> lock(data_mtx_);
    if (!next.valid) {
      next.speed_mps = 0.0;
      next.speed_valid = false;
      have_previous_pose_measurement_ = false;
    } else if (have_previous_pose_measurement_) {
      const double dt = (next.received - previous_pose_time_).seconds();
      if (dt > 1e-4 && dt <= pose_max_dt_for_speed_) {
        const double measured = (next.position - previous_pose_measurement_).norm() / dt;
        if (std::isfinite(measured)) {
          const double alpha = clampd(pose_speed_lpf_alpha_, 0.0, 1.0);
          const double previous_speed = pose_.speed_valid && std::isfinite(pose_.speed_mps) ?
            pose_.speed_mps : measured;
          next.speed_mps = alpha * measured + (1.0 - alpha) * previous_speed;
          next.speed_valid = std::isfinite(next.speed_mps);
        }
      } else {
        next.speed_mps = 0.0;
        next.speed_valid = false;
      }
    } else {
      next.speed_mps = 0.0;
      next.speed_valid = false;
    }
    if (next.valid && pose_.valid &&
      (next.position - pose_.position).norm() > pose_jump_reset_threshold_m_)
    {
      reset_pending_ = true;
    }
    if (next.valid) {
      previous_pose_measurement_ = next.position;
      previous_pose_time_ = next.received;
      have_previous_pose_measurement_ = true;
    }
    pose_ = next;
  }

  double directedMeanPathDistance(
    const std::vector<Eigen::Vector2d> & source,
    const std::vector<Eigen::Vector2d> & target) const
  {
    if (source.empty() || target.empty()) {
      return std::numeric_limits<double>::infinity();
    }
    const std::size_t samples = std::min<std::size_t>(20, source.size());
    double sum = 0.0;
    for (std::size_t sample = 0; sample < samples; ++sample) {
      const std::size_t index = samples == 1 ? 0 :
        sample * (source.size() - 1) / (samples - 1);
      double nearest = std::numeric_limits<double>::infinity();
      for (const auto & point : target) {
        nearest = std::min(nearest, (source[index] - point).norm());
      }
      sum += nearest;
    }
    return sum / static_cast<double>(samples);
  }

  bool pathMateriallyDifferent(
    const std::vector<Eigen::Vector2d> & old_path,
    const std::vector<Eigen::Vector2d> & new_path) const
  {
    if (old_path.empty() || new_path.empty()) {return true;}
    const double difference = std::max(
      directedMeanPathDistance(old_path, new_path),
      directedMeanPathDistance(new_path, old_path));
    return difference > path_change_reset_threshold_m_;
  }

  void pathCallback(const nav_msgs::msg::Path::SharedPtr message)
  {
    std::vector<Eigen::Vector2d> next;
    next.reserve(message->poses.size());
    for (const auto & pose : message->poses) {
      const Eigen::Vector2d point(pose.pose.position.x, pose.pose.position.y);
      if (finitePoint(point) &&
        (next.empty() || (point - next.back()).norm() >= lower_min_path_spacing_m_ * 0.1))
      {
        next.push_back(point);
      }
    }
    std::lock_guard<std::mutex> lock(data_mtx_);
    const bool hard_change = pathMateriallyDifferent(path_, next);
    path_ = std::move(next);
    path_frame_id_ = message->header.frame_id.empty() ? "map" : message->header.frame_id;
    path_received_ = node_.now();
    if (path_.size() < 2 || hard_change) {
      reset_pending_ = true;
    }
  }

  void goalCallback(const std_msgs::msg::Bool::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(data_mtx_);
    if (goal_reached_ != message->data) {
      reset_pending_ = true;
    }
    goal_reached_ = message->data;
  }

  void resetCallback(const std_msgs::msg::Bool::SharedPtr message)
  {
    if (!message->data) {return;}
    std::lock_guard<std::mutex> lock(data_mtx_);
    reset_pending_ = true;
  }

  PathProjection projectToPath(
    const std::vector<Eigen::Vector2d> & path,
    const Eigen::Vector2d & vehicle_position) const
  {
    PathProjection projection;
    if (path.size() < 2) {return projection;}
    double accumulated_s = 0.0;
    double best_distance_squared = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i + 1 < path.size(); ++i) {
      const Eigen::Vector2d segment = path[i + 1] - path[i];
      const double length = segment.norm();
      if (length <= 1e-9) {continue;}
      const Eigen::Vector2d tangent = segment / length;
      const double ratio = clampd(
        (vehicle_position - path[i]).dot(segment) / segment.squaredNorm(), 0.0, 1.0);
      const Eigen::Vector2d point = path[i] + ratio * segment;
      const Eigen::Vector2d error = vehicle_position - point;
      const double distance_squared = error.squaredNorm();
      if (distance_squared < best_distance_squared) {
        best_distance_squared = distance_squared;
        projection.valid = true;
        projection.segment_index = static_cast<int>(i);
        projection.segment_ratio = ratio;
        projection.path_s = accumulated_s + ratio * length;
        projection.point = point;
        projection.tangent = tangent;
        projection.path_yaw_rad = std::atan2(tangent.y(), tangent.x());
        // Sign convention: a vehicle on the left side of the directed path has e_y > 0.
        projection.lateral_error_m = tangent.x() * error.y() - tangent.y() * error.x();
      }
      accumulated_s += length;
    }
    return projection;
  }

  std::vector<double> buildArcLength(const std::vector<Eigen::Vector2d> & path) const
  {
    std::vector<double> arc(path.size(), 0.0);
    for (std::size_t i = 1; i < path.size(); ++i) {
      arc[i] = arc[i - 1] + (path[i] - path[i - 1]).norm();
    }
    return arc;
  }

  std::pair<Eigen::Vector2d, double> samplePath(
    const std::vector<Eigen::Vector2d> & path,
    const std::vector<double> & arc,
    double query_s) const
  {
    if (path.size() < 2) {return {Eigen::Vector2d::Zero(), 0.0};}
    query_s = clampd(query_s, 0.0, arc.back());
    auto upper = std::upper_bound(arc.begin(), arc.end(), query_s);
    std::size_t segment = upper == arc.begin() ? 0 :
      static_cast<std::size_t>(std::distance(arc.begin(), upper) - 1);
    segment = std::min(segment, path.size() - 2);
    while (segment + 1 < path.size() - 1 &&
      (path[segment + 1] - path[segment]).norm() <= 1e-9)
    {
      ++segment;
    }
    const Eigen::Vector2d delta = path[segment + 1] - path[segment];
    const double length = delta.norm();
    const double ratio = length > 1e-9 ?
      clampd((query_s - arc[segment]) / length, 0.0, 1.0) : 0.0;
    const double yaw = length > 1e-9 ? std::atan2(delta.y(), delta.x()) : 0.0;
    return {path[segment] + ratio * delta, yaw};
  }

  MpcResult solveTrackingMpc(
    const PoseSnapshot & pose,
    const std::vector<Eigen::Vector2d> & path) const
  {
    MpcResult result;
    const auto started = std::chrono::steady_clock::now();
    if (pose.speed_mps < 1e-3) {
      result.steering_sequence_rad.assign(
        static_cast<std::size_t>(lower_prediction_steps_), 0.0);
      result.solve_time_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
      result.status = "stationary steering reset";
      result.valid = true;
      return result;
    }
    const PathProjection projection = projectToPath(path, pose.position);
    if (!projection.valid) {
      result.status = "no valid path segment";
      return result;
    }
    result.nearest_segment = projection.segment_index;
    result.lateral_error_m = projection.lateral_error_m;
    result.heading_error_rad = wrapToPi(pose.yaw_rad - projection.path_yaw_rad);

    const int horizon = lower_prediction_steps_;
    const double speed = std::max(lower_min_effective_speed_mps_, pose.speed_mps);
    const double sample_spacing = std::max(
      lower_min_path_spacing_m_, speed * lower_prediction_dt_sec_);
    const auto arc = buildArcLength(path);
    std::vector<double> reference_yaw(static_cast<std::size_t>(horizon + 1), 0.0);
    result.reference_horizon.reserve(static_cast<std::size_t>(horizon + 1));
    for (int k = 0; k <= horizon; ++k) {
      const auto sampled = samplePath(path, arc, projection.path_s + k * sample_spacing);
      result.reference_horizon.push_back(sampled.first);
      double yaw = sampled.second;
      if (k > 0) {
        yaw = reference_yaw[static_cast<std::size_t>(k - 1)] +
          wrapToPi(yaw - reference_yaw[static_cast<std::size_t>(k - 1)]);
      }
      reference_yaw[static_cast<std::size_t>(k)] = yaw;
    }

    Eigen::Matrix2d system;
    system << 1.0, speed * lower_prediction_dt_sec_, 0.0, 1.0;
    Eigen::Vector2d input(
      0.0, speed / wheelbase_ * lower_prediction_dt_sec_);
    Eigen::Vector2d affine_state(
      result.lateral_error_m, result.heading_error_rad);
    Eigen::MatrixXd input_map = Eigen::MatrixXd::Zero(2, horizon);
    Eigen::VectorXd affine = Eigen::VectorXd::Zero(2 * horizon);
    Eigen::MatrixXd condensed = Eigen::MatrixXd::Zero(2 * horizon, horizon);
    for (int k = 0; k < horizon; ++k) {
      const double path_yaw_increment = wrapToPi(
        reference_yaw[static_cast<std::size_t>(k + 1)] -
        reference_yaw[static_cast<std::size_t>(k)]);
      affine_state = system * affine_state + Eigen::Vector2d(0.0, -path_yaw_increment);
      input_map = system * input_map;
      input_map.col(k) += input;
      affine.segment<2>(2 * k) = affine_state;
      condensed.block(2 * k, 0, 2, horizon) = input_map;
    }

    Eigen::MatrixXd state_weight = Eigen::MatrixXd::Zero(2 * horizon, 2 * horizon);
    for (int k = 0; k < horizon; ++k) {
      state_weight(2 * k, 2 * k) = lower_q_lateral_;
      state_weight(2 * k + 1, 2 * k + 1) = lower_q_heading_;
    }
    Eigen::MatrixXd difference = Eigen::MatrixXd::Zero(horizon, horizon);
    difference(0, 0) = 1.0;
    for (int k = 1; k < horizon; ++k) {
      difference(k, k) = 1.0;
      difference(k, k - 1) = -1.0;
    }
    Eigen::MatrixXd hessian_eigen = 2.0 * (
      condensed.transpose() * state_weight * condensed +
      lower_r_steering_ * Eigen::MatrixXd::Identity(horizon, horizon) +
      lower_rd_steering_rate_ * difference.transpose() * difference);
    Eigen::VectorXd linear_eigen =
      2.0 * condensed.transpose() * state_weight * affine;
    const double previous = clampd(
      last_mpc_steering_angle_rad_,
      -lower_max_steering_angle_rad_, lower_max_steering_angle_rad_);
    linear_eigen(0) -= 2.0 * lower_rd_steering_rate_ * previous;
    hessian_eigen.diagonal().array() += 1e-9;

    Matrix<double> hessian(0.0, horizon, horizon);
    Vector<double> linear(0.0, horizon);
    for (int r = 0; r < horizon; ++r) {
      linear[r] = linear_eigen(r);
      for (int c = 0; c < horizon; ++c) {
        hessian[r][c] = hessian_eigen(r, c);
      }
    }
    Matrix<double> inequality(0.0, 4 * horizon, horizon);
    Vector<double> bound(0.0, 4 * horizon);
    int row = 0;
    for (int k = 0; k < horizon; ++k) {
      inequality[row][k] = 1.0;
      bound[row++] = lower_max_steering_angle_rad_;
      inequality[row][k] = -1.0;
      bound[row++] = lower_max_steering_angle_rad_;
    }
    const double max_step = lower_max_steering_rate_radps_ * lower_prediction_dt_sec_;
    for (int k = 0; k < horizon; ++k) {
      if (k == 0) {
        inequality[row][0] = 1.0;
        bound[row++] = max_step + previous;
        inequality[row][0] = -1.0;
        bound[row++] = max_step - previous;
      } else {
        inequality[row][k] = 1.0;
        inequality[row][k - 1] = -1.0;
        bound[row++] = max_step;
        inequality[row][k] = -1.0;
        inequality[row][k - 1] = 1.0;
        bound[row++] = max_step;
      }
    }

    QuadraticProblem solver(false);
    Variable * variable = solver.vector_variable(horizon, "lower_steering");
    if (!solver.add_variable(variable)) {
      result.status = "DAQP add_variable failed";
      return result;
    }
    const Var steering = solver.get_variable(variable);
    Constraint constraint(solver.get_main_variable());
    constraint.set_constraint_variable(steering, inequality);
    constraint.set_known_term(bound);
    solver.add_leq_constraint(constraint);
    solver.set_Q_matrix(hessian);
    solver.set_q0_vector(linear);
    Vector<double> argument;
    const double objective = solver.solve_problem(argument);
    result.solve_time_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    if (!std::isfinite(objective) || argument.size() < static_cast<unsigned int>(horizon)) {
      result.status = "DAQP lower QP failed";
      return result;
    }
    result.steering_sequence_rad.resize(static_cast<std::size_t>(horizon));
    for (int k = 0; k < horizon; ++k) {
      if (!std::isfinite(argument[k])) {
        result.steering_sequence_rad.clear();
        result.status = "DAQP lower QP returned a non-finite command";
        return result;
      }
      result.steering_sequence_rad[static_cast<std::size_t>(k)] = clampd(
        argument[k], -lower_max_steering_angle_rad_, lower_max_steering_angle_rad_);
    }
    result.objective = objective;
    result.valid = !result.steering_sequence_rad.empty();
    result.status = result.valid ? "solved" : "empty solution";
    return result;
  }

  void resetControllerState(const std::string & reason)
  {
    previous_lower_steering_sequence_.clear();
    previous_lower_steering_index_ = 0;
    last_mpc_steering_angle_rad_ = 0.0;
    last_valid_steering_angle_rad_ = 0.0;
    last_valid_steering_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    have_filtered_command_ = false;
    previous_filtered_steer_norm_ = 0.0;
    last_filter_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    speed_integral_ = 0.0;
    target_speed_profile_mps_ = std::max(0.0, target_speed_mps_);
    RCLCPP_WARN(node_.get_logger(), "lower MPC state reset: %s", reason.c_str());
  }

  SelectedCommand selectFailureCommand(const std::string & reason, bool allow_sequence)
  {
    SelectedCommand selected;
    selected.reason = reason;
    if (allow_sequence &&
      previous_lower_steering_index_ < previous_lower_steering_sequence_.size())
    {
      selected.steering_angle_rad =
        previous_lower_steering_sequence_[previous_lower_steering_index_++];
      selected.drive_valid = true;
      selected.fallback_mode = 1;
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 1000,
        "lower QP failure: consuming previous lower sequence (%zu/%zu): %s",
        previous_lower_steering_index_, previous_lower_steering_sequence_.size(),
        reason.c_str());
      return selected;
    }
    const rclcpp::Time current_time = node_.now();
    const bool can_hold = last_valid_steering_time_.nanoseconds() > 0 &&
      (current_time - last_valid_steering_time_).seconds() <= solver_hold_last_valid_sec_;
    if (can_hold) {
      selected.steering_angle_rad = last_valid_steering_angle_rad_;
      selected.fallback_mode = 2;
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 1000,
        "lower control invalid: holding steering with neutral throttle: %s", reason.c_str());
    } else {
      selected.steering_angle_rad = 0.0;
      selected.fallback_mode = 3;
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 1000,
        "lower control invalid: neutral steering/throttle: %s", reason.c_str());
    }
    return selected;
  }

  double filterSteering(double steering_angle_rad, const rclcpp::Time & current_time)
  {
    double normalized = steering_angle_rad / lower_max_steering_angle_rad_;
    normalized = clampd(normalized, -steer_norm_max_, steer_norm_max_);
    if (!enable_output_filter_) {
      previous_filtered_steer_norm_ = normalized;
      last_filter_time_ = current_time;
      have_filtered_command_ = true;
      return normalized;
    }
    double dt = 1.0 / lower_controller_rate_hz_;
    if (have_filtered_command_ && last_filter_time_.nanoseconds() > 0) {
      const double measured = (current_time - last_filter_time_).seconds();
      if (measured > 1e-4 && measured < 1.0) {dt = measured;}
    }
    const double previous = have_filtered_command_ ? previous_filtered_steer_norm_ : normalized;
    const double maximum_change = steer_norm_rate_max_ * dt;
    const double rate_limited = clampd(
      normalized, previous - maximum_change, previous + maximum_change);
    const double filtered = clampd(
      steer_norm_lpf_alpha_ * rate_limited +
      (1.0 - steer_norm_lpf_alpha_) * previous,
      -steer_norm_max_, steer_norm_max_);
    previous_filtered_steer_norm_ = filtered;
    last_filter_time_ = current_time;
    have_filtered_command_ = true;
    return filtered;
  }

  void updateTargetSpeedProfile(double dt)
  {
    if (std::abs(target_accel_mps2_) > 1e-9) {
      target_speed_profile_mps_ += target_accel_mps2_ * dt;
    } else {
      target_speed_profile_mps_ = target_speed_mps_;
    }
    target_speed_profile_mps_ = std::max(0.0, target_speed_profile_mps_);
  }

  VirtualDriveCommand computeVirtualDrive(
    double steering_norm,
    bool drive_valid,
    bool hold_steering,
    double current_speed)
  {
    VirtualDriveCommand command;
    command.steer = hold_steering || drive_valid ?
      clampd(steering_norm, -1.0, 1.0) : 0.0;
    if (!drive_valid) {
      speed_integral_ = 0.0;
      return command;
    }
    const double dt = 1.0 / lower_controller_rate_hz_;
    updateTargetSpeedProfile(dt);
    const double error = target_speed_profile_mps_ - std::max(0.0, current_speed);
    if (std::abs(error) <= speed_deadband_mps_) {
      command.throttle = clampd(throttle_ff_, 0.0, max_virtual_throttle_);
      return command;
    }
    speed_integral_ = clampd(
      speed_integral_ + error * dt,
      -std::abs(speed_integral_limit_), std::abs(speed_integral_limit_));
    const double control = speed_kp_ * error + speed_ki_ * speed_integral_;
    if (control >= 0.0) {
      command.throttle = clampd(throttle_ff_ + control, 0.0, max_virtual_throttle_);
    } else {
      command.brake = clampd(-control, 0.0, max_virtual_brake_);
    }
    return command;
  }

  int normalizedToPwm(double normalized, int minimum, int maximum) const
  {
    minimum = std::max(pwm_min_, std::min(minimum, maximum));
    maximum = std::min(pwm_max_, std::max(minimum, maximum));
    const double value = clampd(normalized, -1.0, 1.0);
    const double pwm = value >= 0.0 ?
      pwm_center_ + value * (maximum - pwm_center_) :
      pwm_center_ + value * (pwm_center_ - minimum);
    return std::max(minimum, std::min(maximum, static_cast<int>(std::lround(pwm))));
  }

  void publishLocalReference(
    const std::vector<Eigen::Vector2d> & points,
    const std::string & frame_id)
  {
    nav_msgs::msg::Path message;
    message.header.stamp = node_.now();
    message.header.frame_id = frame_id;
    message.poses.resize(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
      message.poses[i].header = message.header;
      message.poses[i].pose.position.x = points[i].x();
      message.poses[i].pose.position.y = points[i].y();
      message.poses[i].pose.orientation.w = 1.0;
    }
    local_curve_pub_->publish(message);
  }

  void publishGoalStop(const PoseSnapshot & pose, const std::string & frame_id)
  {
    (void)frame_id;
    previous_lower_steering_sequence_.clear();
    previous_lower_steering_index_ = 0;
    last_mpc_steering_angle_rad_ = 0.0;
    last_valid_steering_angle_rad_ = 0.0;
    last_valid_steering_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    speed_integral_ = 0.0;
    target_speed_profile_mps_ = 0.0;
    previous_filtered_steer_norm_ = 0.0;
    have_filtered_command_ = true;
    last_filter_time_ = node_.now();

    geometry_msgs::msg::Twist raw;
    raw.linear.z = 0.0;
    raw_cmd_pub_->publish(raw);
    if (publish_applied_cmd_when_invalid_) {applied_cmd_pub_->publish(raw);}
    imac_interfaces::msg::VirtualControlCommand virtual_command;
    virtual_command.steer = 0.0f;
    virtual_command.throttle = 0.0f;
    virtual_command.brake = static_cast<float>(clampd(max_virtual_brake_, 0.0, 1.0));
    virtual_cmd_pub_->publish(virtual_command);
    publishDutyAndPwm(0.0, 0.0, false);
    rememberMavlinkActuator(0.0, 0.0, false);
    if (mavlinkActive()) {mavlinkSendActuator(0.0, 0.0);}
    publishTrace(
      pose, MpcResult(), SelectedCommand{0.0, false, 4, "goal reached"}, 0.0,
      virtual_command, 0.0, 0.0);
    RCLCPP_INFO_THROTTLE(
      node_.get_logger(), *node_.get_clock(), 1000,
      "goal stop applied: steer=0 throttle=0 brake=%.3f MAVLink neutral",
      static_cast<double>(virtual_command.brake));
  }

  void publishDutyAndPwm(double throttle, double steering, bool valid)
  {
    std_msgs::msg::Float64MultiArray duty;
    duty.data = {throttle, steering, valid ? 1.0 : 0.0};
    duty_pub_->publish(duty);
    if (pwm_pub_) {
      std_msgs::msg::Int32MultiArray pwm;
      pwm.data = {
        normalizedToPwm(steering, steering_pwm_min_, steering_pwm_max_),
        normalizedToPwm(throttle, throttle_pwm_min_, throttle_pwm_max_)
      };
      pwm_pub_->publish(pwm);
    }
  }

  void publishTrace(
    const PoseSnapshot & pose,
    const MpcResult & mpc,
    const SelectedCommand & selected,
    double applied_steering_norm,
    const imac_interfaces::msg::VirtualControlCommand & virtual_command,
    double mavlink_throttle,
    double mavlink_steering)
  {
    std_msgs::msg::Float64MultiArray trace;
    trace.data = {
      node_.now().seconds(), pose.position.x(), pose.position.y(), pose.yaw_rad,
      pose.speed_mps, static_cast<double>(mpc.nearest_segment),
      mpc.lateral_error_m, mpc.heading_error_rad,
      mpc.steering_sequence_rad.empty() ? 0.0 : mpc.steering_sequence_rad.front(),
      applied_steering_norm, mpc.valid ? 1.0 : 0.0, mpc.solve_time_ms,
      static_cast<double>(selected.fallback_mode),
      static_cast<double>(virtual_command.steer),
      static_cast<double>(virtual_command.throttle),
      static_cast<double>(virtual_command.brake),
      mavlink_throttle, mavlink_steering, mpc.objective
    };
    lower_trace_pub_->publish(trace);
  }

  void publishSelectedCommand(
    const PoseSnapshot & pose,
    const MpcResult & mpc,
    const SelectedCommand & selected)
  {
    SelectedCommand command = selected;
    if (!std::isfinite(command.steering_angle_rad)) {
      command.steering_angle_rad = 0.0;
      command.drive_valid = false;
      command.fallback_mode = 3;
      command.reason = "non-finite steering command";
    }
    const rclcpp::Time current_time = node_.now();
    last_mpc_steering_angle_rad_ = command.steering_angle_rad;
    if (command.drive_valid) {
      last_valid_steering_angle_rad_ = command.steering_angle_rad;
      last_valid_steering_time_ = current_time;
    }
    const double raw_normalized = clampd(
      command.steering_angle_rad / lower_max_steering_angle_rad_, -1.0, 1.0);
    const double applied_normalized = filterSteering(
      command.steering_angle_rad, current_time);

    geometry_msgs::msg::Twist raw;
    raw.linear.x = command.steering_angle_rad;
    raw.linear.y = raw_normalized;
    raw.linear.z = command.drive_valid ? 1.0 : 0.0;
    raw.angular.z = mpc.heading_error_rad;
    raw_cmd_pub_->publish(raw);
    geometry_msgs::msg::Twist applied = raw;
    applied.linear.y = applied_normalized;
    if (command.drive_valid || publish_applied_cmd_when_invalid_) {
      applied_cmd_pub_->publish(applied);
    }

    const bool hold_steering = command.fallback_mode == 2;
    const VirtualDriveCommand drive = computeVirtualDrive(
      applied_normalized, command.drive_valid, hold_steering, pose.speed_mps);
    imac_interfaces::msg::VirtualControlCommand virtual_message;
    virtual_message.steer = static_cast<float>(drive.steer);
    virtual_message.throttle = static_cast<float>(drive.throttle);
    virtual_message.brake = static_cast<float>(drive.brake);
    virtual_cmd_pub_->publish(virtual_message);

    const double platform_steering =
      -clampd(steer_sign_ * applied_normalized, -1.0, 1.0);
    const double platform_throttle = command.drive_valid ?
      clampd(mavlink_throttle_max_norm_, 0.0, 1.0) : 0.0;
    publishDutyAndPwm(platform_throttle, platform_steering, command.drive_valid);

    double mavlink_throttle = 0.0;
    double mavlink_steering = 0.0;
    publishMavlinkCommand(
      command.drive_valid, applied_normalized, mavlink_throttle, mavlink_steering);
    publishTrace(
      pose, mpc, command, applied_normalized, virtual_message,
      mavlink_throttle, mavlink_steering);
  }

  void controllerLoop()
  {
    PoseSnapshot pose;
    std::vector<Eigen::Vector2d> path;
    std::string frame_id;
    rclcpp::Time path_received(0, 0, RCL_ROS_TIME);
    bool goal_reached = false;
    bool reset_pending = false;
    {
      std::lock_guard<std::mutex> lock(data_mtx_);
      pose = pose_;
      path = path_;
      frame_id = path_frame_id_;
      path_received = path_received_;
      goal_reached = goal_reached_;
      reset_pending = reset_pending_;
      reset_pending_ = false;
    }
    if (reset_pending) {resetControllerState("path/mission/pose/reset event");}
    if (goal_reached) {
      publishGoalStop(pose, frame_id);
      return;
    }

    const rclcpp::Time current_time = node_.now();
    if (!pose.valid) {
      const MpcResult failed;
      publishSelectedCommand(
        pose, failed, selectFailureCommand("waiting for pose", false));
      return;
    }
    if (!pose.speed_valid) {
      const MpcResult failed;
      publishSelectedCommand(
        pose, failed, selectFailureCommand("invalid finite-difference speed", false));
      return;
    }
    if (input_timeout_sec_ > 0.0 &&
      (current_time - pose.received).seconds() > input_timeout_sec_)
    {
      const MpcResult failed;
      publishSelectedCommand(pose, failed, selectFailureCommand("stale pose", false));
      return;
    }
    if (path.size() < 2) {
      const MpcResult failed;
      publishSelectedCommand(pose, failed, selectFailureCommand("empty lower path", false));
      return;
    }
    if (!normalizedFrameId(pose.frame_id).empty() &&
      !normalizedFrameId(frame_id).empty() &&
      !frameIdsEquivalent(pose.frame_id, frame_id))
    {
      const MpcResult failed;
      publishSelectedCommand(
        pose, failed, selectFailureCommand("pose/path frame mismatch", false));
      return;
    }
    if (lower_path_timeout_sec_ > 0.0 && path_received.nanoseconds() > 0 &&
      (current_time - path_received).seconds() > lower_path_timeout_sec_)
    {
      const MpcResult failed;
      publishSelectedCommand(pose, failed, selectFailureCommand("stale lower path", false));
      return;
    }

    MpcResult mpc = solveTrackingMpc(pose, path);
    if (!mpc.valid) {
      publishSelectedCommand(
        pose, mpc, selectFailureCommand(mpc.status, true));
      return;
    }
    SelectedCommand selected;
    selected.steering_angle_rad = mpc.steering_sequence_rad.front();
    selected.drive_valid = true;
    selected.fallback_mode = 0;
    selected.reason = "lower QP solved";
    previous_lower_steering_sequence_.assign(
      mpc.steering_sequence_rad.begin() + 1, mpc.steering_sequence_rad.end());
    previous_lower_steering_index_ = 0;
    last_mpc_steering_angle_rad_ = selected.steering_angle_rad;
    last_valid_steering_angle_rad_ = selected.steering_angle_rad;
    last_valid_steering_time_ = current_time;
    publishLocalReference(mpc.reference_horizon, frame_id);
    publishSelectedCommand(pose, mpc, selected);
  }

  bool mavlinkActive() const
  {
    return mavlink_enable_ && pixhawk_output_backend_ == "mavlink_udp";
  }

  void rememberMavlinkActuator(double throttle, double steering, bool valid)
  {
    std::lock_guard<std::mutex> lock(mavlink_cmd_mtx_);
    mavlink_last_throttle_norm_ = clampd(throttle, -1.0, 1.0);
    mavlink_last_steering_norm_ = clampd(steering, -1.0, 1.0);
    mavlink_have_actuator_cmd_ = true;
    if (valid) {
      last_valid_mavlink_steering_norm_ = mavlink_last_steering_norm_;
      last_valid_mavlink_cmd_time_ = node_.now();
    }
  }

  void publishMavlinkCommand(
    bool valid,
    double applied_steering_norm,
    double & sent_throttle,
    double & sent_steering)
  {
    if (!mavlinkActive()) {return;}
    const rclcpp::Time current_time = node_.now();
    if (valid) {
      sent_throttle = clampd(mavlink_throttle_max_norm_, 0.0, 1.0);
      sent_steering = -clampd(
        mavlink_steer_sign_ * applied_steering_norm, -1.0, 1.0);
      rememberMavlinkActuator(sent_throttle, sent_steering, true);
      mavlinkSendActuator(sent_throttle, sent_steering);
      return;
    }
    const bool hold = last_valid_mavlink_cmd_time_.nanoseconds() > 0 &&
      (current_time - last_valid_mavlink_cmd_time_).seconds() < mavlink_hold_last_valid_sec_;
    if (hold) {
      sent_steering = last_valid_mavlink_steering_norm_;
      rememberMavlinkActuator(0.0, sent_steering, false);
      mavlinkSendActuator(0.0, sent_steering);
    } else if (mavlink_send_neutral_on_invalid_) {
      rememberMavlinkActuator(0.0, 0.0, false);
      mavlinkSendActuator(0.0, 0.0);
    }
  }

  void mavlinkInitUdp()
  {
    std::lock_guard<std::mutex> lock(mavlink_mtx_);
    if (mavlink_socket_ >= 0) {return;}
    mavlink_socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (mavlink_socket_ < 0) {
      RCLCPP_ERROR(
        node_.get_logger(), "MAVLink UDP socket creation failed: %s", std::strerror(errno));
      return;
    }
    int reuse = 1;
    ::setsockopt(mavlink_socket_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in local_address{};
    local_address.sin_family = AF_INET;
    local_address.sin_port = htons(static_cast<uint16_t>(mavlink_bind_port_));
    if (mavlink_bind_ip_ == "0.0.0.0") {
      local_address.sin_addr.s_addr = htonl(INADDR_ANY);
    } else if (::inet_pton(
        AF_INET, mavlink_bind_ip_.c_str(), &local_address.sin_addr) != 1)
    {
      RCLCPP_ERROR(node_.get_logger(), "invalid mavlink_bind_ip: %s", mavlink_bind_ip_.c_str());
      ::close(mavlink_socket_);
      mavlink_socket_ = -1;
      return;
    }
    if (::bind(
        mavlink_socket_, reinterpret_cast<sockaddr *>(&local_address),
        sizeof(local_address)) < 0)
    {
      RCLCPP_ERROR(
        node_.get_logger(), "MAVLink UDP bind failed on %s:%d: %s",
        mavlink_bind_ip_.c_str(), mavlink_bind_port_, std::strerror(errno));
      ::close(mavlink_socket_);
      mavlink_socket_ = -1;
      return;
    }
    const int flags = ::fcntl(mavlink_socket_, F_GETFL, 0);
    if (flags >= 0) {::fcntl(mavlink_socket_, F_SETFL, flags | O_NONBLOCK);}
    mavlink_peer_known_ = false;
    std::memset(&mavlink_peer_addr_, 0, sizeof(mavlink_peer_addr_));
    std::memset(&mavlink_rx_status_, 0, sizeof(mavlink_rx_status_));
    mavlink_px4_manual_ = false;
    mavlink_px4_armed_ = false;
    mavlink_last_mode_request_time_ = node_.now();
    mavlink_last_arm_request_time_ = node_.now();
    RCLCPP_INFO(
      node_.get_logger(),
      "MAVLink UDP bound at %s:%d; waiting for MAVProxy --out peer",
      mavlink_bind_ip_.c_str(), mavlink_bind_port_);
  }

  void mavlinkClose()
  {
    std::lock_guard<std::mutex> lock(mavlink_mtx_);
    if (mavlink_socket_ >= 0) {
      ::close(mavlink_socket_);
      mavlink_socket_ = -1;
    }
    mavlink_peer_known_ = false;
  }

  void mavlinkPoll()
  {
    std::lock_guard<std::mutex> lock(mavlink_mtx_);
    if (!mavlinkActive() || mavlink_socket_ < 0) {return;}
    std::array<uint8_t, 2048> buffer{};
    while (true) {
      sockaddr_in source{};
      socklen_t source_length = sizeof(source);
      const ssize_t received = ::recvfrom(
        mavlink_socket_, buffer.data(), buffer.size(), 0,
        reinterpret_cast<sockaddr *>(&source), &source_length);
      if (received < 0) {
        if (errno == EWOULDBLOCK || errno == EAGAIN) {break;}
        RCLCPP_WARN_THROTTLE(
          node_.get_logger(), *node_.get_clock(), 2000,
          "MAVLink recvfrom failed: %s", std::strerror(errno));
        break;
      }
      for (ssize_t index = 0; index < received; ++index) {
        mavlink_message_t message{};
        if (mavlink_parse_char(
            MAVLINK_COMM_0, buffer[static_cast<std::size_t>(index)],
            &message, &mavlink_rx_status_))
        {
          handleMavlinkMessage(message, source);
        }
      }
    }
  }

  void handleMavlinkMessage(
    const mavlink_message_t & message,
    const sockaddr_in & source)
  {
    if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
      if (!mavlink_peer_known_) {
        mavlink_peer_addr_ = source;
        mavlink_peer_known_ = true;
        char address[INET_ADDRSTRLEN] = {};
        ::inet_ntop(AF_INET, &source.sin_addr, address, sizeof(address));
        RCLCPP_INFO(
          node_.get_logger(), "MAVLink peer detected: %s:%d",
          address, ntohs(source.sin_port));
      }
      mavlink_heartbeat_t heartbeat{};
      mavlink_msg_heartbeat_decode(&message, &heartbeat);
      mavlink_target_system_ = message.sysid;
      mavlink_target_component_ = 1;
      const uint8_t main_mode = static_cast<uint8_t>((heartbeat.custom_mode >> 16) & 0xffU);
      mavlink_px4_manual_ = main_mode == kPx4CustomMainModeManual;
      mavlink_px4_armed_ =
        (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
    } else if (message.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
      mavlink_command_ack_t acknowledgement{};
      mavlink_msg_command_ack_decode(&message, &acknowledgement);
      RCLCPP_INFO_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 1000,
        "MAVLink COMMAND_ACK command=%u result=%u",
        static_cast<unsigned>(acknowledgement.command),
        static_cast<unsigned>(acknowledgement.result));
    } else if (message.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
      mavlink_statustext_t status{};
      mavlink_msg_statustext_decode(&message, &status);
      char text[51] = {};
      std::memcpy(text, status.text, 50);
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 1000,
        "PX4 STATUSTEXT severity=%u text=%s",
        static_cast<unsigned>(status.severity), text);
    }
  }

  bool mavlinkSendMessage(const mavlink_message_t & message)
  {
    std::lock_guard<std::mutex> lock(mavlink_mtx_);
    if (!mavlinkActive() || mavlink_socket_ < 0) {return false;}
    if (!mavlink_peer_known_) {
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 2000,
        "MAVLink peer unknown; check MAVProxy --out=127.0.0.1:%d", mavlink_bind_port_);
      return false;
    }
    std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};
    const uint16_t length = mavlink_msg_to_send_buffer(buffer.data(), &message);
    const ssize_t sent = ::sendto(
      mavlink_socket_, buffer.data(), length, 0,
      reinterpret_cast<sockaddr *>(&mavlink_peer_addr_), sizeof(mavlink_peer_addr_));
    if (sent != static_cast<ssize_t>(length)) {
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 2000,
        "MAVLink sendto failed: sent=%zd expected=%u error=%s",
        sent, static_cast<unsigned>(length), std::strerror(errno));
      return false;
    }
    return true;
  }

  void mavlinkSendManualNeutral()
  {
    if (!mavlinkActive()) {return;}
    mavlink_message_t message{};
    mavlink_msg_manual_control_pack(
      static_cast<uint8_t>(mavlink_source_system_),
      static_cast<uint8_t>(mavlink_source_component_), &message,
      static_cast<uint8_t>(mavlink_target_system_),
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    mavlinkSendMessage(message);
  }

  void mavlinkSendSetManualMode()
  {
    if (!mavlinkActive()) {return;}
    mavlink_message_t message{};
    const uint32_t custom_mode = px4CustomMode(kPx4CustomMainModeManual);
    mavlink_msg_set_mode_pack(
      static_cast<uint8_t>(mavlink_source_system_),
      static_cast<uint8_t>(mavlink_source_component_), &message,
      static_cast<uint8_t>(mavlink_target_system_),
      MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, custom_mode);
    const bool sent = mavlinkSendMessage(message);
    RCLCPP_INFO_THROTTLE(
      node_.get_logger(), *node_.get_clock(), 1000,
      "MAVLink SET_MODE MANUAL sent=%d", sent ? 1 : 0);
  }

  void mavlinkSendArmCommand(bool arm, bool force)
  {
    if (!mavlinkActive()) {return;}
    mavlink_message_t message{};
    mavlink_msg_command_long_pack(
      static_cast<uint8_t>(mavlink_source_system_),
      static_cast<uint8_t>(mavlink_source_component_), &message,
      static_cast<uint8_t>(mavlink_target_system_),
      static_cast<uint8_t>(mavlink_target_component_),
      MAV_CMD_COMPONENT_ARM_DISARM, 0,
      arm ? 1.0f : 0.0f, force ? 21196.0f : 0.0f,
      0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    const bool sent = mavlinkSendMessage(message);
    RCLCPP_WARN_THROTTLE(
      node_.get_logger(), *node_.get_clock(), 1000,
      "MAVLink %s force=%d sent=%d", arm ? "ARM" : "DISARM",
      force ? 1 : 0, sent ? 1 : 0);
  }

  void mavlinkManageModeAndArm()
  {
    if (!mavlinkActive()) {return;}
    bool request_mode = false;
    bool request_arm = false;
    const rclcpp::Time current_time = node_.now();
    {
      std::lock_guard<std::mutex> lock(mavlink_mtx_);
      if (!mavlink_peer_known_) {return;}
      if (mavlink_auto_manual_mode_ && !mavlink_px4_manual_ &&
        (current_time - mavlink_last_mode_request_time_).seconds() > 1.0)
      {
        request_mode = true;
        mavlink_last_mode_request_time_ = current_time;
      }
      if (mavlink_auto_arm_ && mavlink_px4_manual_ && !mavlink_px4_armed_ &&
        (current_time - mavlink_last_arm_request_time_).seconds() > 1.0)
      {
        request_arm = true;
        mavlink_last_arm_request_time_ = current_time;
      }
    }
    if (request_mode) {mavlinkSendSetManualMode();}
    if (request_arm) {mavlinkSendArmCommand(true, mavlink_force_arm_);}
  }

  void mavlinkSendActuator(double throttle, double steering)
  {
    if (!mavlinkActive()) {return;}
    const float not_a_number = std::numeric_limits<float>::quiet_NaN();
    mavlink_message_t message{};
    mavlink_msg_command_long_pack(
      static_cast<uint8_t>(mavlink_source_system_),
      static_cast<uint8_t>(mavlink_source_component_), &message,
      static_cast<uint8_t>(mavlink_target_system_),
      static_cast<uint8_t>(mavlink_target_component_),
      MAV_CMD_DO_SET_ACTUATOR, 0,
      static_cast<float>(clampd(throttle, -1.0, 1.0)),
      static_cast<float>(clampd(steering, -1.0, 1.0)),
      not_a_number, not_a_number, not_a_number, not_a_number, 0.0f);
    mavlinkSendMessage(message);
  }

  void mavlinkSendHeldActuator()
  {
    if (!mavlinkActive()) {return;}
    double throttle = 0.0;
    double steering = 0.0;
    bool have_command = false;
    {
      std::lock_guard<std::mutex> lock(mavlink_cmd_mtx_);
      throttle = mavlink_last_throttle_norm_;
      steering = mavlink_last_steering_norm_;
      have_command = mavlink_have_actuator_cmd_;
    }
    if (have_command) {mavlinkSendActuator(throttle, steering);}
  }

  LowerTrackingMpcNode & node_;
  double lower_controller_rate_hz_{10.0};
  double lower_prediction_dt_sec_{0.1};
  int lower_prediction_steps_{10};
  double lower_q_lateral_{1.0};
  double lower_q_heading_{0.5};
  double lower_r_steering_{0.30};
  double lower_rd_steering_rate_{8.0};
  double lower_max_steering_angle_rad_{0.35};
  double lower_max_steering_rate_radps_{2.09439510239};
  double lower_min_path_spacing_m_{0.05};
  double lower_min_effective_speed_mps_{0.10};
  std::string scale_mode_{"model"};
  double wheelbase_{0.3};
  double vehicle_length_m_{0.42};
  double bev_forward_m_{1.5};
  double preview_interval_nominal_road_width_m_{0.45};
  double preview_interval_boundary_margin_m_{0.15};
  double input_timeout_sec_{0.60};
  double lower_path_timeout_sec_{1.50};
  double solver_hold_last_valid_sec_{0.30};
  double path_change_reset_threshold_m_{0.75};
  double pose_jump_reset_threshold_m_{1.0};
  bool publish_zero_on_failure_{true};
  bool publish_applied_cmd_when_invalid_{true};
  std::string state_input_type_{"odom"};
  std::string odom_topic_{"/px4/sih/odom_map"};
  std::string pose_stamped_topic_{"/motive/vehicle/pose"};
  bool pose_stamped_yaw_is_orientation_z_{true};
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
  std::string lower_reference_path_topic_{"/planner/lower_reference_path"};
  std::string goal_reached_topic_{"/planner/goal_reached"};
  std::string reset_topic_{"/controller/reset"};

  bool enable_output_filter_{true};
  double steer_norm_max_{1.0};
  double steer_norm_rate_max_{2.0};
  double steer_norm_lpf_alpha_{0.5};
  double steer_sign_{1.0};
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
  std::string duty_topic_{"/control/duty_cmd"};
  std::string pwm_topic_{"/cmd_pwm"};
  bool publish_pwm_{true};
  int pwm_min_{1000};
  int pwm_center_{1500};
  int pwm_max_{2000};
  int throttle_pwm_min_{1500};
  int throttle_pwm_max_{2000};
  int steering_pwm_min_{1000};
  int steering_pwm_max_{2000};

  std::string pixhawk_output_backend_{"disabled"};
  bool mavlink_enable_{false};
  std::string mavlink_bind_ip_{"0.0.0.0"};
  int mavlink_bind_port_{14540};
  int mavlink_source_system_{245};
  int mavlink_source_component_{190};
  int mavlink_target_system_{1};
  int mavlink_target_component_{1};
  bool mavlink_send_neutral_on_invalid_{true};
  double mavlink_hold_last_valid_sec_{0.30};
  bool mavlink_auto_manual_mode_{true};
  bool mavlink_auto_arm_{false};
  bool mavlink_force_arm_{false};
  bool mavlink_disarm_on_shutdown_{true};
  double mavlink_throttle_max_norm_{0.17};
  double mavlink_steer_sign_{1.0};
  double mavlink_keepalive_rate_hz_{20.0};

  mutable std::mutex data_mtx_;
  PoseSnapshot pose_;
  bool have_previous_pose_measurement_{false};
  Eigen::Vector2d previous_pose_measurement_{Eigen::Vector2d::Zero()};
  rclcpp::Time previous_pose_time_{0, 0, RCL_ROS_TIME};
  std::vector<Eigen::Vector2d> path_;
  std::string path_frame_id_{"map"};
  rclcpp::Time path_received_{0, 0, RCL_ROS_TIME};
  bool goal_reached_{false};
  bool reset_pending_{false};

  std::vector<double> previous_lower_steering_sequence_;
  std::size_t previous_lower_steering_index_{0};
  double last_mpc_steering_angle_rad_{0.0};
  double last_valid_steering_angle_rad_{0.0};
  rclcpp::Time last_valid_steering_time_{0, 0, RCL_ROS_TIME};
  bool have_filtered_command_{false};
  double previous_filtered_steer_norm_{0.0};
  rclcpp::Time last_filter_time_{0, 0, RCL_ROS_TIME};

  int mavlink_socket_{-1};
  bool mavlink_peer_known_{false};
  sockaddr_in mavlink_peer_addr_{};
  mavlink_status_t mavlink_rx_status_{};
  bool mavlink_px4_manual_{false};
  bool mavlink_px4_armed_{false};
  rclcpp::Time mavlink_last_mode_request_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time mavlink_last_arm_request_time_{0, 0, RCL_ROS_TIME};
  std::mutex mavlink_mtx_;
  double mavlink_last_throttle_norm_{0.0};
  double mavlink_last_steering_norm_{0.0};
  bool mavlink_have_actuator_cmd_{false};
  double last_valid_mavlink_steering_norm_{0.0};
  rclcpp::Time last_valid_mavlink_cmd_time_{0, 0, RCL_ROS_TIME};
  std::mutex mavlink_cmd_mtx_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr raw_cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr applied_cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_curve_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr lower_trace_pub_;
  rclcpp::Publisher<imac_interfaces::msg::VirtualControlCommand>::SharedPtr virtual_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr duty_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pwm_pub_;
  rclcpp::TimerBase::SharedPtr controller_timer_;
  rclcpp::TimerBase::SharedPtr mavlink_keepalive_timer_;
};

LowerTrackingMpcNode::LowerTrackingMpcNode(const rclcpp::NodeOptions & options)
: Node("lower_tracking_mpc_node", options),
  impl_(std::make_unique<Impl>(*this))
{
}

LowerTrackingMpcNode::~LowerTrackingMpcNode() = default;

}  // namespace imac_ctrl

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<imac_ctrl::LowerTrackingMpcNode>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
