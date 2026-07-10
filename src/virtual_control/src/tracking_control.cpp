#include "virtual_control/tracking_control.hpp"
#include "QuadraticProblem.h"
#include "matrix_utils.h"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <limits>
#include <mutex>
#include <utility>
#include <vector>
#include <sstream>
#include <array>
#include <cerrno>
#include <cstring>
#include <string>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#ifdef solve
#undef solve
#endif

using std::placeholders::_1;
using Eigen::Vector2d;

namespace imac_ctrl
{

static inline int clampi(int v, int lo, int hi)
{
  return std::max(lo, std::min(hi, v));
}

static inline bool pathsApproximatelyEqual(
  const std::vector<Eigen::Vector2d>& a,
  const std::vector<Eigen::Vector2d>& b,
  double tol = 5e-2)
{
  if (a.size() != b.size()) {
    return false;
  }
  for (size_t i = 0; i < a.size(); ++i) {
    if ((a[i] - b[i]).norm() > tol) {
      return false;
    }
  }
  return true;
}

static inline double clampd(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

static double remainingPathLengthFromClosestProjection(
  const std::vector<Eigen::Vector2d>& path,
  const Eigen::Vector2d& position)
{
  if (path.size() < 2) {
    return 0.0;
  }

  double total_length = 0.0;
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    total_length += (path[i + 1] - path[i]).norm();
  }

  double best_dist_sq = std::numeric_limits<double>::infinity();
  double best_remaining = total_length;
  double accumulated = 0.0;

  for (size_t i = 0; i + 1 < path.size(); ++i) {
    const Eigen::Vector2d seg = path[i + 1] - path[i];
    const double seg_len = seg.norm();
    if (seg_len < 1e-9) {
      continue;
    }

    const double t = clampd(
      (position - path[i]).dot(seg) / (seg_len * seg_len),
      0.0,
      1.0);
    const Eigen::Vector2d projection = path[i] + t * seg;
    const double dist_sq = (position - projection).squaredNorm();
    const double remaining = std::max(0.0, total_length - (accumulated + t * seg_len));

    if (dist_sq < best_dist_sq ||
      (std::abs(dist_sq - best_dist_sq) < 1e-9 && remaining < best_remaining))
    {
      best_dist_sq = dist_sq;
      best_remaining = remaining;
    }

    accumulated += seg_len;
  }

  return best_remaining;
}

#ifndef MAV_CMD_DO_SET_ACTUATOR
#define MAV_CMD_DO_SET_ACTUATOR 187
#endif

#ifndef MAV_CMD_COMPONENT_ARM_DISARM
#define MAV_CMD_COMPONENT_ARM_DISARM 400
#endif

static constexpr uint8_t PX4_CUSTOM_MAIN_MODE_MANUAL = 1;

static inline uint32_t px4CustomMode(uint8_t main_mode, uint8_t sub_mode = 0)
{
  return (static_cast<uint32_t>(sub_mode) << 24) |
         (static_cast<uint32_t>(main_mode) << 16);
}

TrackingControllerNode::~TrackingControllerNode()
{
  mavlink_keepalive_timer_.reset();

  if (mavlink_enable_ && mavlink_disarm_on_shutdown_) {
    mavlinkSendActuator(0.0, 0.0);
    mavlinkSendArmCommand(false, false);
  }
  mavlinkClose();
}

void TrackingControllerNode::mavlinkInitUdp()
{
  std::lock_guard<std::mutex> lk(mavlink_mtx_);

  if (!mavlink_enable_) {
    return;
  }
  if (mavlink_socket_ >= 0) {
    return;
  }

  mavlink_socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (mavlink_socket_ < 0) {
    RCLCPP_ERROR(get_logger(), "MAVLink UDP socket creation failed: %s", std::strerror(errno));
    return;
  }

  int reuse = 1;
  ::setsockopt(mavlink_socket_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

  sockaddr_in local_addr{};
  local_addr.sin_family = AF_INET;
  local_addr.sin_port = htons(static_cast<uint16_t>(mavlink_bind_port_));

  if (mavlink_bind_ip_ == "0.0.0.0") {
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  } else if (::inet_pton(AF_INET, mavlink_bind_ip_.c_str(), &local_addr.sin_addr) != 1) {
    RCLCPP_ERROR(get_logger(), "Invalid mavlink_bind_ip: %s", mavlink_bind_ip_.c_str());
    ::close(mavlink_socket_);
    mavlink_socket_ = -1;
    return;
  }

  if (::bind(mavlink_socket_, reinterpret_cast<sockaddr*>(&local_addr), sizeof(local_addr)) < 0) {
    RCLCPP_ERROR(
      get_logger(),
      "MAVLink UDP bind failed on %s:%d: %s",
      mavlink_bind_ip_.c_str(), mavlink_bind_port_, std::strerror(errno));
    ::close(mavlink_socket_);
    mavlink_socket_ = -1;
    return;
  }

  const int flags = ::fcntl(mavlink_socket_, F_GETFL, 0);
  if (flags >= 0) {
    ::fcntl(mavlink_socket_, F_SETFL, flags | O_NONBLOCK);
  }

  mavlink_peer_known_ = false;
  std::memset(&mavlink_peer_addr_, 0, sizeof(mavlink_peer_addr_));
  std::memset(&mavlink_rx_status_, 0, sizeof(mavlink_rx_status_));
  mavlink_px4_manual_ = false;
  mavlink_px4_armed_ = false;
  mavlink_last_mode_request_time_ = this->now();
  mavlink_last_arm_request_time_ = this->now();

  RCLCPP_INFO(
    get_logger(),
    "MAVLink UDP bound at %s:%d. Waiting for MAVProxy peer. Use MAVProxy --out=127.0.0.1:%d",
    mavlink_bind_ip_.c_str(), mavlink_bind_port_, mavlink_bind_port_);
}

void TrackingControllerNode::mavlinkClose()
{
  std::lock_guard<std::mutex> lk(mavlink_mtx_);
  if (mavlink_socket_ >= 0) {
    ::close(mavlink_socket_);
    mavlink_socket_ = -1;
  }
  mavlink_peer_known_ = false;
}

void TrackingControllerNode::mavlinkPoll()
{
  std::lock_guard<std::mutex> lk(mavlink_mtx_);

  if (!mavlink_enable_ || mavlink_socket_ < 0) {
    return;
  }

  std::array<uint8_t, 2048> buffer{};

  while (true) {
    sockaddr_in src_addr{};
    socklen_t src_len = sizeof(src_addr);

    const ssize_t n = ::recvfrom(
      mavlink_socket_,
      buffer.data(),
      buffer.size(),
      0,
      reinterpret_cast<sockaddr*>(&src_addr),
      &src_len);

    if (n < 0) {
      if (errno == EWOULDBLOCK || errno == EAGAIN) {
        break;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "MAVLink recvfrom failed: %s", std::strerror(errno));
      break;
    }

    for (ssize_t i = 0; i < n; ++i) {
      mavlink_message_t msg{};

      if (mavlink_parse_char(
          MAVLINK_COMM_0,
          buffer[static_cast<size_t>(i)],
          &msg,
          &mavlink_rx_status_))
      {
        if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
          if (!mavlink_peer_known_) {
            mavlink_peer_addr_ = src_addr;
            mavlink_peer_known_ = true;

            char ip_str[INET_ADDRSTRLEN] = {};
            ::inet_ntop(AF_INET, &src_addr.sin_addr, ip_str, sizeof(ip_str));

            RCLCPP_INFO(
              get_logger(),
              "MAVLink peer detected from HEARTBEAT: %s:%d",
              ip_str,
              ntohs(src_addr.sin_port));
          }

          mavlink_heartbeat_t hb{};
          mavlink_msg_heartbeat_decode(&msg, &hb);

          mavlink_target_system_ = msg.sysid;
          mavlink_target_component_ = 1;

          const uint8_t px4_main_mode =
            static_cast<uint8_t>((hb.custom_mode >> 16) & 0xFF);

          mavlink_px4_manual_ = px4_main_mode == PX4_CUSTOM_MAIN_MODE_MANUAL;
          mavlink_px4_armed_ = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
        } else if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
          mavlink_command_ack_t ack{};
          mavlink_msg_command_ack_decode(&msg, &ack);

          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "MAVLink COMMAND_ACK: command=%u result=%u",
            static_cast<unsigned>(ack.command),
            static_cast<unsigned>(ack.result));
        } else if (msg.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
          mavlink_statustext_t st{};
          mavlink_msg_statustext_decode(&msg, &st);

          char text[51] = {};
          std::memcpy(text, st.text, 50);

          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "PX4 STATUSTEXT: severity=%u text=%s",
            static_cast<unsigned>(st.severity),
            text);
        }
      }
    }
  }
}

bool TrackingControllerNode::mavlinkSendMessage(const mavlink_message_t & msg)
{
  std::lock_guard<std::mutex> lk(mavlink_mtx_);

  if (!mavlink_enable_ || mavlink_socket_ < 0) {
    return false;
  }

  if (!mavlink_peer_known_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "MAVLink peer is not known yet. Check MAVProxy --out=127.0.0.1:%d",
      mavlink_bind_port_);
    return false;
  }

  std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> txbuf{};
  const uint16_t len = mavlink_msg_to_send_buffer(txbuf.data(), &msg);

  const ssize_t sent = ::sendto(
    mavlink_socket_,
    txbuf.data(),
    len,
    0,
    reinterpret_cast<sockaddr*>(&mavlink_peer_addr_),
    sizeof(mavlink_peer_addr_));

  if (sent != static_cast<ssize_t>(len)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "MAVLink sendto failed: sent=%zd expected=%u errno=%s",
      sent, static_cast<unsigned>(len), std::strerror(errno));
    return false;
  }

  return true;
}

void TrackingControllerNode::mavlinkSendManualNeutral()
{
  if (!mavlink_enable_) {
    return;
  }

  mavlink_message_t msg{};
  mavlink_msg_manual_control_pack(
    static_cast<uint8_t>(mavlink_source_system_),
    static_cast<uint8_t>(mavlink_source_component_),
    &msg,
    static_cast<uint8_t>(mavlink_target_system_),
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0);

  mavlinkSendMessage(msg);
}

void TrackingControllerNode::mavlinkSendSetManualMode()
{
  if (!mavlink_enable_) {
    return;
  }

  const uint32_t custom_mode = px4CustomMode(PX4_CUSTOM_MAIN_MODE_MANUAL);

  mavlink_message_t msg{};
  mavlink_msg_set_mode_pack(
    static_cast<uint8_t>(mavlink_source_system_),
    static_cast<uint8_t>(mavlink_source_component_),
    &msg,
    static_cast<uint8_t>(mavlink_target_system_),
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    custom_mode);

  const bool ok = mavlinkSendMessage(msg);

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "MAVLink SET_MODE MANUAL sent: custom_mode=%u ok=%d",
    custom_mode, ok ? 1 : 0);
}

void TrackingControllerNode::mavlinkSendArmCommand(bool arm, bool force)
{
  if (!mavlink_enable_) {
    return;
  }

  const float force_code = force ? 21196.0f : 0.0f;

  mavlink_message_t msg{};
  mavlink_msg_command_long_pack(
    static_cast<uint8_t>(mavlink_source_system_),
    static_cast<uint8_t>(mavlink_source_component_),
    &msg,
    static_cast<uint8_t>(mavlink_target_system_),
    static_cast<uint8_t>(mavlink_target_component_),
    MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    arm ? 1.0f : 0.0f,
    force_code,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f);

  const bool ok = mavlinkSendMessage(msg);

  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "MAVLink %s sent: force=%d ok=%d",
    arm ? "ARM" : "DISARM",
    force ? 1 : 0,
    ok ? 1 : 0);
}

void TrackingControllerNode::mavlinkManageModeAndArm()
{
  if (!mavlink_enable_) {
    return;
  }

  const rclcpp::Time now = this->now();
  bool send_manual = false;
  bool send_arm = false;

  {
    std::lock_guard<std::mutex> lk(mavlink_mtx_);
    if (!mavlink_peer_known_) {
      return;
    }

    if (mavlink_auto_manual_mode_ && !mavlink_px4_manual_) {
      if (mavlink_last_mode_request_time_.nanoseconds() == 0 ||
        (now - mavlink_last_mode_request_time_).seconds() > 1.0)
      {
        send_manual = true;
        mavlink_last_mode_request_time_ = now;
      }
    }

    if (mavlink_auto_arm_ && mavlink_px4_manual_ && !mavlink_px4_armed_) {
      if (mavlink_last_arm_request_time_.nanoseconds() == 0 ||
        (now - mavlink_last_arm_request_time_).seconds() > 1.0)
      {
        send_arm = true;
        mavlink_last_arm_request_time_ = now;
      }
    }
  }

  if (send_manual) {
    mavlinkSendSetManualMode();
  }
  if (send_arm) {
    mavlinkSendArmCommand(true, mavlink_force_arm_);
  }
}

void TrackingControllerNode::mavlinkSendActuator(double throttle_norm, double steering_norm)
{
  if (!mavlink_enable_) {
    return;
  }

  mavlinkPoll();

  throttle_norm = clampd(throttle_norm, -1.0, 1.0);
  steering_norm = clampd(steering_norm, -1.0, 1.0);

  const float nan_f = std::numeric_limits<float>::quiet_NaN();

  mavlink_message_t msg{};
  mavlink_msg_command_long_pack(
    static_cast<uint8_t>(mavlink_source_system_),
    static_cast<uint8_t>(mavlink_source_component_),
    &msg,
    static_cast<uint8_t>(mavlink_target_system_),
    static_cast<uint8_t>(mavlink_target_component_),
    MAV_CMD_DO_SET_ACTUATOR,
    0,
    static_cast<float>(throttle_norm),
    static_cast<float>(steering_norm),
    nan_f,
    nan_f,
    nan_f,
    nan_f,
    0.0f);

  const bool ok = mavlinkSendMessage(msg);

  RCLCPP_DEBUG_THROTTLE(
    get_logger(), *get_clock(), 500,
    "MAVLink actuator tx: throttle=%.3f steering=%.3f ok=%d manual=%d armed=%d",
    throttle_norm,
    steering_norm,
    ok ? 1 : 0,
    mavlink_px4_manual_ ? 1 : 0,
    mavlink_px4_armed_ ? 1 : 0);
}

void TrackingControllerNode::mavlinkSendHeldActuator()
{
  if (!mavlink_enable_) {
    return;
  }

  double throttle_norm = 0.0;
  double steering_norm = 0.0;
  bool have_cmd = false;
  {
    std::lock_guard<std::mutex> lk(mavlink_cmd_mtx_);
    throttle_norm = mavlink_last_throttle_norm_;
    steering_norm = mavlink_last_steering_norm_;
    have_cmd = mavlink_have_actuator_cmd_;
  }

  if (!have_cmd) {
    throttle_norm = 0.0;
    steering_norm = 0.0;
  }

  mavlinkSendActuator(throttle_norm, steering_norm);
}

double TrackingControllerNode::computeThrottleNorm(double dv_mps, bool valid_cmd) const
{
  (void)dv_mps;

  if (!valid_cmd) {
    return 0.0;
  }

  // PX4 normalized actuator output: 0.0 -> 1500 us, 0.17 -> about 1585 us.
  // For the current safety test, every valid control command gets the fixed
  // forward throttle. Invalid commands still send neutral.
  const double throttle_max_norm =
    (mavlink_throttle_max_norm_ > 0.0) ? mavlink_throttle_max_norm_ : 0.17;
  return clampd(throttle_max_norm, 0.0, 1.0);
}

void TrackingControllerNode::publishMavlinkCmd(
  double dv_mps,
  double steer_norm,
  bool valid_cmd)
{
  if (!mavlink_enable_) {
    return;
  }

  const rclcpp::Time now = this->now();

  const auto remember_mavlink_actuator =
    [this](double throttle_norm, double steering_norm) {
      std::lock_guard<std::mutex> lk(mavlink_cmd_mtx_);
      mavlink_last_throttle_norm_ = throttle_norm;
      mavlink_last_steering_norm_ = steering_norm;
      mavlink_have_actuator_cmd_ = true;
    };

  if (valid_cmd) {
    const double throttle_norm = computeThrottleNorm(dv_mps, true);
    const double steering_norm = -clampd(
      mavlink_steer_sign_ * steer_norm,
      -1.0,
      1.0);

    last_valid_mavlink_steer_norm_ = steering_norm;
    last_valid_mavlink_cmd_time_ = now;

    remember_mavlink_actuator(throttle_norm, steering_norm);
    mavlinkSendActuator(throttle_norm, steering_norm);
    return;
  }

  const bool have_recent_valid =
    last_valid_mavlink_cmd_time_.nanoseconds() > 0 &&
    (now - last_valid_mavlink_cmd_time_).seconds() < mavlink_hold_last_valid_sec_;

  if (have_recent_valid) {
    // Safety policy:
    // On invalid command, throttle must always go neutral.
    // Steering is held briefly to avoid 1500us chatter due to one-frame solver/input failures.
    remember_mavlink_actuator(0.0, last_valid_mavlink_steer_norm_);
    mavlinkSendActuator(0.0, last_valid_mavlink_steer_norm_);

    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "MAVLink invalid cmd: holding last steering %.3f for %.2f sec",
      last_valid_mavlink_steer_norm_,
      mavlink_hold_last_valid_sec_);

    return;
  }

  {
    std::lock_guard<std::mutex> lk(mavlink_cmd_mtx_);
    mavlink_last_throttle_norm_ = 0.0;
    mavlink_last_steering_norm_ = 0.0;
    mavlink_have_actuator_cmd_ = true;
  }

  if (mavlink_send_neutral_on_invalid_) {
    mavlinkSendActuator(0.0, 0.0);

    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "MAVLink invalid cmd: sending neutral throttle/steering");
  }
}

TrackingControllerNode::TrackingControllerNode()
: Node("tracking_controller_node")
{
  // ---- Declare parameters ----
  this->declare_parameter<double>("loop_rate_hz_ctrl", 20.0);
  this->declare_parameter<double>("max_steer_left", 0.35);
  this->declare_parameter<double>("max_steer_right", -0.35);
  this->declare_parameter<double>("v_max", 1.0);
  this->declare_parameter<double>("a_max_mps2", 1.0);
  this->declare_parameter<double>("wheelbase", 0.3);                 // [m]
  this->declare_parameter<double>("lane_width", 0.5);
  this->declare_parameter<std::vector<double>>("lane_offsets_m", {0.0});
  this->declare_parameter<double>("miqp_big_m", 100.0);
  this->declare_parameter<double>("w_segment_preference", 1e-3);
  this->declare_parameter<double>("dpsi_delta_max_rad", 0.05);
  this->declare_parameter<double>("wp_switch_eps", 0.5);
  // Body-grid OccupancyGrid input from rgb_to_occupancy:
  //   rgb_to_occupancy:
  //     -p publish_occupancy_grid:=true
  //     -p bev_occupancy_grid_topic:=/bev/occupancy_grid
  //     -p occupancy_grid_rviz_standard_values:=true
  //     -p occupancy_rotate_cw_90:=false
  //     -p occupancy_use_crop:=true
  //     -p occupancy_grid_width:=64
  //     -p occupancy_grid_height:=64
  //   tracking_controller_node:
  //     -p grid_map_topic:=/bev/occupancy_grid
  //     -p grid_positive_is_drivable:=false
  //     -p grid_value_threshold:=0
  //     -p require_grid_map:=true
  //
  // occupancy_rotate_cw_90 must stay false for control: it rotates only the
  // data array, not the physical origin/orientation used for lookup.
  this->declare_parameter<std::string>("grid_map_topic", "/bev/occupancy_grid");
  this->declare_parameter<int>("grid_value_threshold", 0);
  this->declare_parameter<bool>("grid_positive_is_drivable", false);
  this->declare_parameter<double>("preview_line_sample_m", 0.03);
  this->declare_parameter<int>("preview_min_segment_samples", 1);
  this->declare_parameter<bool>("require_grid_map", true);
  this->declare_parameter<bool>("reset_waypoint_on_path_update", false);
  this->declare_parameter<double>("input_timeout_sec", 0.60);
  this->declare_parameter<bool>("publish_zero_on_failure", true);
  this->declare_parameter<bool>("use_upper_guides", true);
  this->declare_parameter<double>("upper_guides_timeout_sec", 0.80);
  this->declare_parameter<double>("upper_guides_change_reset_tol_m", 0.25);
  this->declare_parameter<double>("goal_stop_distance_m", 0.50);
  this->declare_parameter<bool>("preview_use_path_tangent_when_seed_empty", true);
  this->declare_parameter<double>("preview_max_yaw_rad", 0.35);
  this->declare_parameter<double>("preview_min_forward_x_m", 0.20);
  this->declare_parameter<double>("preview_max_lateral_y_m", 0.30);
  this->declare_parameter<double>("preview_min_wp_norm_m", 0.10);
  this->declare_parameter<bool>("preview_reset_on_bad_seed", true);
  this->declare_parameter<double>("preview_seed_max_lateral_y_m", 0.50);
  this->declare_parameter<double>("preview_seed_min_forward_x_m", -0.10);
  // State input examples:
  //   Motive PoseStamped:
  //     ros2 run virtual_control tracking_control --ros-args
  //       -p state_input_type:=pose_stamped
  //       -p pose_stamped_topic:=/motive/vehicle/pose
  //   MATLAB/ROS odom:
  //     ros2 run virtual_control tracking_control --ros-args -p state_input_type:=odom
  // Motive coordinates must be aligned to /debug/global_path's map frame with
  // the pose_* correction parameters below. If multiple rigid bodies are used,
  // re-enable target_rigid_body_id filtering in motive_pose_publisher.
  this->declare_parameter<std::string>("state_input_type", "odom");
  this->declare_parameter<std::string>("pose_stamped_topic", "/motive/vehicle/pose");
  this->declare_parameter<double>("pose_x_offset", 0.0);
  this->declare_parameter<double>("pose_y_offset", 0.0);
  this->declare_parameter<double>("pose_z_offset", 0.0);
  this->declare_parameter<double>("pose_yaw_offset_rad", 0.0);
  this->declare_parameter<double>("pose_position_scale", 1.0);
  this->declare_parameter<bool>("pose_swap_xy", false);
  this->declare_parameter<bool>("pose_invert_x", false);
  this->declare_parameter<bool>("pose_invert_y", false);
  this->declare_parameter<double>("pose_speed_lpf_alpha", 0.4);
  this->declare_parameter<double>("pose_max_dt_for_speed", 0.5);

  // ---- Output steering filter parameters ----
  this->declare_parameter<bool>("enable_output_filter", true);
  this->declare_parameter<double>("steer_norm_max", 1.0);
  this->declare_parameter<double>("steer_norm_rate_max", 2.0);
  this->declare_parameter<double>("steer_norm_lpf_alpha", 0.5);
  this->declare_parameter<bool>("publish_applied_cmd_when_invalid", true);

  // ---- Isaac Sim virtual vehicle command parameters ----
  this->declare_parameter<std::string>("virtual_cmd_topic", "/cmd_control");
  this->declare_parameter<double>("target_speed_mps", 1.0);
  this->declare_parameter<double>("target_accel_mps2", 0.0);
  this->declare_parameter<double>("speed_kp", 0.45);
  this->declare_parameter<double>("speed_ki", 0.05);
  this->declare_parameter<double>("throttle_ff", 0.08);
  this->declare_parameter<double>("max_virtual_throttle", 0.35);
  this->declare_parameter<double>("max_virtual_brake", 0.70);
  this->declare_parameter<double>("speed_deadband_mps", 0.03);
  this->declare_parameter<double>("speed_integral_limit", 2.0);

  // ---- MAVLink bridge parameters ----
  this->declare_parameter<bool>("mavlink_enable", false);
  this->declare_parameter<std::string>("mavlink_bind_ip", "0.0.0.0");
  this->declare_parameter<int>("mavlink_bind_port", 14540);
  this->declare_parameter<int>("mavlink_source_system", 245);
  this->declare_parameter<int>("mavlink_source_component", 190);
  this->declare_parameter<int>("mavlink_target_system", 1);
  this->declare_parameter<int>("mavlink_target_component", 1);
  this->declare_parameter<bool>("mavlink_send_neutral_on_invalid", true);
  this->declare_parameter<double>("mavlink_hold_last_valid_sec", 0.30);
  this->declare_parameter<bool>("mavlink_auto_manual_mode", true);
  this->declare_parameter<bool>("mavlink_auto_arm", false);
  this->declare_parameter<bool>("mavlink_force_arm", false);
  this->declare_parameter<bool>("mavlink_disarm_on_shutdown", true);
  this->declare_parameter<double>("mavlink_throttle_max_norm", 0.17);
  this->declare_parameter<double>("mavlink_steer_sign", 1.0);

  // ---- Get parameters ----
  this->get_parameter("loop_rate_hz_ctrl", loop_rate_hz_ctrl_);
  this->get_parameter("max_steer_left", max_steer_left_);
  this->get_parameter("max_steer_right", max_steer_right_);
  this->get_parameter("v_max", v_max_);
  this->get_parameter("a_max_mps2", a_max_);
  this->get_parameter("wheelbase", wheelbase_);
  this->get_parameter("lane_width", lane_width_);
  lane_offsets_ = this->get_parameter("lane_offsets_m").as_double_array();
  this->get_parameter("miqp_big_m", miqp_big_m_);
  this->get_parameter("w_segment_preference", w_segment_preference_);
  this->get_parameter("dpsi_delta_max_rad", dpsi_delta_max_rad_);
  this->get_parameter("wp_switch_eps", wp_switch_eps_);
  this->get_parameter("grid_map_topic", grid_map_topic_);
  this->get_parameter("grid_value_threshold", grid_value_threshold_);
  this->get_parameter("grid_positive_is_drivable", grid_positive_is_drivable_);
  this->get_parameter("preview_line_sample_m", preview_line_sample_m_);
  this->get_parameter("preview_min_segment_samples", preview_min_segment_samples_);
  this->get_parameter("require_grid_map", require_grid_map_);
  this->get_parameter("reset_waypoint_on_path_update", reset_waypoint_on_path_update_);
  this->get_parameter("input_timeout_sec", input_timeout_sec_);
  this->get_parameter("publish_zero_on_failure", publish_zero_on_failure_);
  this->get_parameter("use_upper_guides", use_upper_guides_);
  this->get_parameter("upper_guides_timeout_sec", upper_guides_timeout_sec_);
  this->get_parameter("upper_guides_change_reset_tol_m", upper_guides_change_reset_tol_m_);
  this->get_parameter("goal_stop_distance_m", goal_stop_distance_m_);
  this->get_parameter(
    "preview_use_path_tangent_when_seed_empty", preview_use_path_tangent_when_seed_empty_);
  this->get_parameter("preview_max_yaw_rad", preview_max_yaw_rad_);
  this->get_parameter("preview_min_forward_x_m", preview_min_forward_x_m_);
  this->get_parameter("preview_max_lateral_y_m", preview_max_lateral_y_m_);
  this->get_parameter("preview_min_wp_norm_m", preview_min_wp_norm_m_);
  this->get_parameter("preview_reset_on_bad_seed", preview_reset_on_bad_seed_);
  this->get_parameter("preview_seed_max_lateral_y_m", preview_seed_max_lateral_y_m_);
  this->get_parameter("preview_seed_min_forward_x_m", preview_seed_min_forward_x_m_);
  this->get_parameter("state_input_type", state_input_type_);
  this->get_parameter("pose_stamped_topic", pose_stamped_topic_);
  this->get_parameter("pose_x_offset", pose_x_offset_);
  this->get_parameter("pose_y_offset", pose_y_offset_);
  this->get_parameter("pose_z_offset", pose_z_offset_);
  this->get_parameter("pose_yaw_offset_rad", pose_yaw_offset_rad_);
  this->get_parameter("pose_position_scale", pose_position_scale_);
  this->get_parameter("pose_swap_xy", pose_swap_xy_);
  this->get_parameter("pose_invert_x", pose_invert_x_);
  this->get_parameter("pose_invert_y", pose_invert_y_);
  this->get_parameter("pose_speed_lpf_alpha", pose_speed_lpf_alpha_);
  this->get_parameter("pose_max_dt_for_speed", pose_max_dt_for_speed_);

  this->get_parameter("enable_output_filter", enable_output_filter_);
  this->get_parameter("steer_norm_max", steer_norm_max_);
  this->get_parameter("steer_norm_rate_max", steer_norm_rate_max_);
  this->get_parameter("steer_norm_lpf_alpha", steer_norm_lpf_alpha_);
  this->get_parameter("publish_applied_cmd_when_invalid", publish_applied_cmd_when_invalid_);

  this->get_parameter("virtual_cmd_topic", virtual_cmd_topic_);
  this->get_parameter("target_speed_mps", target_speed_mps_);
  this->get_parameter("target_accel_mps2", target_accel_mps2_);
  this->get_parameter("speed_kp", speed_kp_);
  this->get_parameter("speed_ki", speed_ki_);
  this->get_parameter("throttle_ff", throttle_ff_);
  this->get_parameter("max_virtual_throttle", max_virtual_throttle_);
  this->get_parameter("max_virtual_brake", max_virtual_brake_);
  this->get_parameter("speed_deadband_mps", speed_deadband_mps_);
  this->get_parameter("speed_integral_limit", speed_integral_limit_);
  target_speed_profile_mps_ = clampd(target_speed_mps_, 0.0, v_max_);
  target_speed_profile_initialized_ = true;

  this->get_parameter("mavlink_enable", mavlink_enable_);
  this->get_parameter("mavlink_bind_ip", mavlink_bind_ip_);
  this->get_parameter("mavlink_bind_port", mavlink_bind_port_);
  this->get_parameter("mavlink_source_system", mavlink_source_system_);
  this->get_parameter("mavlink_source_component", mavlink_source_component_);
  this->get_parameter("mavlink_target_system", mavlink_target_system_);
  this->get_parameter("mavlink_target_component", mavlink_target_component_);
  this->get_parameter("mavlink_send_neutral_on_invalid", mavlink_send_neutral_on_invalid_);
  this->get_parameter("mavlink_hold_last_valid_sec", mavlink_hold_last_valid_sec_);
  this->get_parameter("mavlink_auto_manual_mode", mavlink_auto_manual_mode_);
  this->get_parameter("mavlink_auto_arm", mavlink_auto_arm_);
  this->get_parameter("mavlink_force_arm", mavlink_force_arm_);
  this->get_parameter("mavlink_disarm_on_shutdown", mavlink_disarm_on_shutdown_);
  this->get_parameter("mavlink_throttle_max_norm", mavlink_throttle_max_norm_);
  this->get_parameter("mavlink_steer_sign", mavlink_steer_sign_);

  RCLCPP_INFO(
    get_logger(),
    "TrackingControllerNode @ ctrl=%.1f Hz, v_max=%.2f",
    loop_rate_hz_ctrl_, v_max_);

  RCLCPP_INFO(
    get_logger(),
    "TrackingControllerNode stabilization: preview_tangent=%d preview_max_yaw=%.3f min_forward=%.3f max_lateral=%.3f min_wp_norm=%.3f reset_bad_seed=%d output_filter=%d steer_max=%.3f steer_rate=%.3f steer_lpf_alpha=%.3f goal_stop=%.3f",
    preview_use_path_tangent_when_seed_empty_ ? 1 : 0,
    preview_max_yaw_rad_,
    preview_min_forward_x_m_,
    preview_max_lateral_y_m_,
    preview_min_wp_norm_m_,
    preview_reset_on_bad_seed_ ? 1 : 0,
    enable_output_filter_ ? 1 : 0,
    steer_norm_max_,
    steer_norm_rate_max_,
    steer_norm_lpf_alpha_,
    goal_stop_distance_m_);

  // ---- Subscribers ----
  path_nav_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/debug/global_path", 10, std::bind(&TrackingControllerNode::pathCallback, this, _1));

  upper_guides_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/planner/upper_guides", 10, std::bind(&TrackingControllerNode::upperGuidesCallback, this, _1));

  if (state_input_type_ == "pose_stamped") {
    pose_stamped_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_stamped_topic_, 10,
      std::bind(&TrackingControllerNode::poseStampedCallback, this, _1));
    RCLCPP_INFO(
      this->get_logger(),
      "state input: PoseStamped topic=%s. Motive rigid-body ID filtering belongs in motive_pose_publisher if multiple rigid bodies are active.",
      pose_stamped_topic_.c_str());
  } else {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/debug/ego_odom", 10, std::bind(&TrackingControllerNode::odomCallback, this, _1));
    RCLCPP_INFO(this->get_logger(), "state input: Odometry topic=/debug/ego_odom");
  }

  grid_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    grid_map_topic_, 10, std::bind(&TrackingControllerNode::gridMapCallback, this, _1));

  // ---- Publishers ----
  control_debug_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>("/debug/control_cmd", 10);

  applied_cmd_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>("/control/applied_cmd", 10);

  local_curve_pub_ =
    this->create_publisher<nav_msgs::msg::Path>("/controller/local_curve", 10);

  mpc_trace_pub_ =
    this->create_publisher<std_msgs::msg::Float64MultiArray>("/debug/mpc_trace", 10);

  virtual_cmd_pub_ =
    this->create_publisher<imac_interfaces::msg::VirtualControlCommand>(
      virtual_cmd_topic_, 10);

  // ---- Control loop timer ----
  auto period = std::chrono::duration<double>(1.0 / loop_rate_hz_ctrl_);
  loop_timer_ = this->create_wall_timer(
    period, std::bind(&TrackingControllerNode::controlLoop, this));

  RCLCPP_INFO(this->get_logger(), "TrackingControllerNode controlLoop at %.1f Hz (/debug/control_cmd uses dv,dpsi semantics; linear.z is valid flag)", loop_rate_hz_ctrl_);

  if (mavlink_enable_) {
    mavlinkInitUdp();

    const double keepalive_hz = 20.0;
    auto mavlink_period = std::chrono::duration<double>(1.0 / keepalive_hz);

    mavlink_keepalive_timer_ = this->create_wall_timer(
      mavlink_period,
      [this]() {
        mavlinkPoll();
        mavlinkSendManualNeutral();
        mavlinkManageModeAndArm();
        mavlinkSendHeldActuator();
      });

    RCLCPP_INFO(
      this->get_logger(),
      "MAVLink bridge enabled: bind=%s:%d, source=%d:%d, target=%d:%d, auto_manual=%d, auto_arm=%d, force_arm=%d, throttle_max_norm=%.3f",
      mavlink_bind_ip_.c_str(),
      mavlink_bind_port_,
      mavlink_source_system_,
      mavlink_source_component_,
      mavlink_target_system_,
      mavlink_target_component_,
      mavlink_auto_manual_mode_ ? 1 : 0,
      mavlink_auto_arm_ ? 1 : 0,
      mavlink_force_arm_ ? 1 : 0,
      mavlink_throttle_max_norm_);
  }
}

void TrackingControllerNode::updateTargetSpeedProfile(double dt)
{
  if (!target_speed_profile_initialized_) {
    target_speed_profile_mps_ = clampd(target_speed_mps_, 0.0, v_max_);
    target_speed_profile_initialized_ = true;
  }

  if (std::abs(target_accel_mps2_) > 1e-9) {
    target_speed_profile_mps_ += target_accel_mps2_ * dt;
  } else {
    target_speed_profile_mps_ = target_speed_mps_;
  }

  target_speed_profile_mps_ = clampd(target_speed_profile_mps_, 0.0, v_max_);
}

double TrackingControllerNode::computeFilteredSteerNorm(
  double steer_norm,
  const rclcpp::Time & now) const
{
  const double steer_max = std::max(0.0, std::abs(steer_norm_max_));
  const double raw_steer = clampd(steer_norm, -steer_max, steer_max);

  if (!enable_output_filter_) {
    return raw_steer;
  }

  double dt = 1.0 / std::max(1e-6, loop_rate_hz_ctrl_);
  if (have_prev_applied_cmd_ && last_applied_cmd_time_.nanoseconds() > 0) {
    const double measured_dt = (now - last_applied_cmd_time_).seconds();
    if (measured_dt > 1e-4 && measured_dt < 1.0) {
      dt = measured_dt;
    }
  }

  const double prev = have_prev_applied_cmd_ ? prev_applied_steer_norm_ : raw_steer;
  const double max_step = std::max(0.0, steer_norm_rate_max_) * dt;
  const double rate_limited = clampd(raw_steer, prev - max_step, prev + max_step);
  const double alpha = clampd(steer_norm_lpf_alpha_, 0.0, 1.0);
  return clampd(alpha * rate_limited + (1.0 - alpha) * prev, -steer_max, steer_max);
}

TrackingControllerNode::VirtualDriveCmd TrackingControllerNode::computeVirtualDriveCmd(
  double steer_norm,
  bool valid_cmd,
  double dt)
{
  VirtualDriveCmd out;

  if (!valid_cmd) {
    speed_integral_ = 0.0;
    out.steer = 0.0;
    out.throttle = 0.0;
    out.brake = 0.0;
    return out;
  }

  out.steer = clampd(steer_norm, -1.0, 1.0);

  const double v_target = clampd(target_speed_profile_mps_, 0.0, v_max_);
  const double v_now = std::max(0.0, cur_speed_);
  const double speed_error = v_target - v_now;

  if (std::abs(speed_error) <= speed_deadband_mps_) {
    out.throttle = clampd(throttle_ff_, 0.0, max_virtual_throttle_);
    out.brake = 0.0;
    return out;
  }

  speed_integral_ += speed_error * dt;
  speed_integral_ = clampd(
    speed_integral_,
    -std::abs(speed_integral_limit_),
    std::abs(speed_integral_limit_));

  const double u = speed_kp_ * speed_error + speed_ki_ * speed_integral_;

  if (u >= 0.0) {
    out.throttle = clampd(throttle_ff_ + u, 0.0, max_virtual_throttle_);
    out.brake = 0.0;
  } else {
    out.throttle = 0.0;
    out.brake = clampd(-u, 0.0, max_virtual_brake_);
  }

  return out;
}

void TrackingControllerNode::publishVirtualControlCommand(
  double steer_norm,
  bool valid_cmd)
{
  if (!virtual_cmd_pub_) {
    return;
  }

  const double dt = 1.0 / std::max(1e-6, loop_rate_hz_ctrl_);
  if (valid_cmd) {
    updateTargetSpeedProfile(dt);
  }
  const VirtualDriveCmd cmd = computeVirtualDriveCmd(steer_norm, valid_cmd, dt);

  imac_interfaces::msg::VirtualControlCommand msg;
  msg.steer = static_cast<float>(cmd.steer);
  msg.throttle = static_cast<float>(cmd.throttle);
  msg.brake = static_cast<float>(cmd.brake);
  virtual_cmd_pub_->publish(msg);

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 500,
    "VirtualControlCommand: topic=%s v_ref=%.3f v_now=%.3f steer=%.3f throttle=%.3f brake=%.3f valid=%d",
    virtual_cmd_topic_.c_str(),
    target_speed_profile_mps_,
    cur_speed_,
    cmd.steer,
    cmd.throttle,
    cmd.brake,
    valid_cmd ? 1 : 0);
}

void TrackingControllerNode::publishVirtualStopCommand()
{
  if (!virtual_cmd_pub_) {
    return;
  }

  speed_integral_ = 0.0;
  target_speed_profile_mps_ = 0.0;

  imac_interfaces::msg::VirtualControlCommand msg;
  msg.steer = 0.0f;
  msg.throttle = 0.0f;
  msg.brake = static_cast<float>(clampd(max_virtual_brake_, 0.0, 1.0));
  virtual_cmd_pub_->publish(msg);

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 500,
    "VirtualControlCommand STOP: topic=%s v_now=%.3f steer=0.000 throttle=0.000 brake=%.3f",
    virtual_cmd_topic_.c_str(),
    cur_speed_,
    static_cast<double>(msg.brake));
}

void TrackingControllerNode::publishcontrolCmd(
  double dv_mps,
  double dpsi_rad,
  double steer_norm,
  bool valid_cmd)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x = dv_mps;
  msg.linear.y = steer_norm;
  msg.linear.z = valid_cmd ? 1.0 : 0.0;  // raw debug validity flag
  msg.angular.z = dpsi_rad;
  if (control_debug_pub_) {
    control_debug_pub_->publish(msg);
  }

  const rclcpp::Time now = this->now();
  double applied_dv = dv_mps;
  double applied_dpsi = dpsi_rad;
  double applied_steer = 0.0;

  if (valid_cmd) {
    applied_steer = computeFilteredSteerNorm(steer_norm, now);
  } else {
    applied_dv = 0.0;
    applied_dpsi = 0.0;
    applied_steer = 0.0;
  }

  prev_applied_steer_norm_ = applied_steer;
  last_applied_cmd_time_ = now;
  have_prev_applied_cmd_ = true;

  geometry_msgs::msg::Twist applied_msg;
  applied_msg.linear.x = applied_dv;
  applied_msg.linear.y = applied_steer;
  applied_msg.linear.z = valid_cmd ? 1.0 : 0.0;
  applied_msg.angular.z = applied_dpsi;

  if (applied_cmd_pub_ && (valid_cmd || publish_applied_cmd_when_invalid_)) {
    applied_cmd_pub_->publish(applied_msg);
  }

  publishVirtualControlCommand(applied_steer, valid_cmd);

  publishMavlinkCmd(applied_dv, applied_steer, valid_cmd);
}

void TrackingControllerNode::publishZeroDebugCmd(const std::string & reason)
{
  if (publishPreviousSolverCmd(reason)) {
    return;
  }

  if (publish_zero_on_failure_) {
    publishcontrolCmd(0.0, 0.0, 0.0, false);
  }
  RCLCPP_WARN_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000,
    "%s", reason.c_str());
}

bool TrackingControllerNode::publishPreviousSolverCmd(const std::string & reason)
{
  if (previous_solver_cmds_.empty()) {
    return false;
  }

  const std::size_t idx = std::min(
    previous_solver_cmd_index_,
    previous_solver_cmds_.size() - 1);
  const SolverControlStep cmd = previous_solver_cmds_[idx];
  if (previous_solver_cmd_index_ + 1 < previous_solver_cmds_.size()) {
    ++previous_solver_cmd_index_;
  }

  publishcontrolCmd(cmd.dv_mps, cmd.dpsi_rad, cmd.steer_norm, true);
  prev_first_dpsi_ = cmd.dpsi_rad;
  have_prev_first_dpsi_ = true;

  RCLCPP_WARN_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000,
    "%s; using previous solver command step %zu/%zu: dv=%.3f dpsi=%.3f steer=%.3f",
    reason.c_str(),
    idx + 1,
    previous_solver_cmds_.size(),
    cmd.dv_mps,
    cmd.dpsi_rad,
    cmd.steer_norm);

  return true;
}

void TrackingControllerNode::publishGoalStopCmd(const std::string & reason)
{
  previous_solver_cmds_.clear();
  previous_solver_cmd_index_ = 0;
  preview_seed_body_.clear();
  have_prev_first_dpsi_ = false;
  prev_first_dpsi_ = 0.0;
  geometry_msgs::msg::Twist msg;
  msg.linear.x = -std::max(0.0, cur_speed_);
  msg.linear.y = 0.0;
  msg.linear.z = 1.0;
  msg.angular.z = 0.0;

  if (control_debug_pub_) {
    control_debug_pub_->publish(msg);
  }
  if (applied_cmd_pub_) {
    applied_cmd_pub_->publish(msg);
  }

  publishVirtualStopCommand();

  {
    std::lock_guard<std::mutex> lk(mavlink_cmd_mtx_);
    mavlink_last_throttle_norm_ = 0.0;
    mavlink_last_steering_norm_ = 0.0;
    mavlink_have_actuator_cmd_ = true;
  }
  mavlinkSendActuator(0.0, 0.0);

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000,
    "%s cur_speed=%.3f goal stop command: dv=%.3f dpsi=0.000",
    reason.c_str(),
    cur_speed_,
    msg.linear.x);
}

void TrackingControllerNode::odomCallback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  cur_x_ = msg->pose.pose.position.x;
  cur_y_ = msg->pose.pose.position.y;

  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  (void)roll;
  (void)pitch;

  cur_yaw_ = yaw;
  cur_speed_ = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
  have_pose_ = true;
  last_pose_rx_time_ = this->now();
}

void TrackingControllerNode::poseStampedCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  double x = msg->pose.position.x;
  double y = msg->pose.position.y;
  const double raw_x = x;
  const double raw_y = y;

  if (pose_swap_xy_) {
    std::swap(x, y);
  }
  if (pose_invert_x_) {
    x = -x;
  }
  if (pose_invert_y_) {
    y = -y;
  }

  x = pose_position_scale_ * x + pose_x_offset_;
  y = pose_position_scale_ * y + pose_y_offset_;
  const double z = pose_position_scale_ * msg->pose.position.z + pose_z_offset_;

  double yaw = msg->pose.orientation.z + pose_yaw_offset_rad_;
  yaw = std::atan2(std::sin(yaw), std::cos(yaw));

  const rclcpp::Time now = this->now();
  const Eigen::Vector2d xy(x, y);

  if (have_prev_pose_stamped_) {
    const double dt = (now - prev_pose_stamped_time_).seconds();
    if (dt > 1e-4 && dt <= pose_max_dt_for_speed_) {
      const double v_meas = (xy - prev_pose_stamped_xy_).norm() / dt;
      const double alpha = std::clamp(pose_speed_lpf_alpha_, 0.0, 1.0);
      cur_speed_ = alpha * v_meas + (1.0 - alpha) * cur_speed_;
      cur_speed_ = clampd(cur_speed_, 0.0, 2.0);
    }
  } else {
    cur_speed_ = 0.0;
    have_prev_pose_stamped_ = true;
  }

  prev_pose_stamped_xy_ = xy;
  prev_pose_stamped_time_ = now;

  cur_x_ = x;
  cur_y_ = y;
  cur_yaw_ = yaw;
  have_pose_ = true;
  last_pose_rx_time_ = now;

  RCLCPP_DEBUG_THROTTLE(
    this->get_logger(), *this->get_clock(), 500,
    "PoseStamped state: frame=%s raw=(%.3f, %.3f, %.3f) map=(%.3f, %.3f, %.3f) yaw=%.3f speed=%.3f",
    msg->header.frame_id.c_str(),
    raw_x,
    raw_y,
    msg->pose.position.z,
    cur_x_,
    cur_y_,
    z,
    cur_yaw_,
    cur_speed_);
}

void TrackingControllerNode::pathCallback(
  const nav_msgs::msg::Path::SharedPtr msg)
{
  std::vector<Eigen::Vector2d> g;
  g.reserve(msg->poses.size());
  for (const auto & pose_stamped : msg->poses) {
    g.emplace_back(
      pose_stamped.pose.position.x,
      pose_stamped.pose.position.y);
  }

  {
    std::lock_guard<std::mutex> lk(map_mtx_);

    const bool same_path = !ref_path_.empty() && pathsApproximatelyEqual(ref_path_, g);
    if (same_path && !reset_waypoint_on_path_update_) {
      return;
    }

    ref_path_ = g;
    goal_reached_ = false;
    previous_solver_cmds_.clear();
    previous_solver_cmd_index_ = 0;
    have_prev_first_dpsi_ = false;
    prev_first_dpsi_ = 0.0;

    if (ref_path_.size() >= 2) {
      wp0_index_ = 0;
      wp1_index_ = 1;
      waypoint_seeded_ = true;
      preview_seed_body_.clear();
    } else {
      wp0_index_ = 0;
      wp1_index_ = 0;
      waypoint_seeded_ = false;
      preview_seed_body_.clear();
    }
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Received debug/global_path: %zu points; goal_reached reset",
    msg->poses.size());
}

void TrackingControllerNode::upperGuidesCallback(
  const nav_msgs::msg::Path::SharedPtr msg)
{
  if (!msg || msg->poses.size() < 2) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "upper guides path is invalid or has fewer than 2 poses");
    return;
  }

  const Eigen::Vector2d wp0_world(
    msg->poses[0].pose.position.x,
    msg->poses[0].pose.position.y);
  const Eigen::Vector2d wp1_world(
    msg->poses[1].pose.position.x,
    msg->poses[1].pose.position.y);

  {
    std::lock_guard<std::mutex> lk(upper_guides_mtx_);
    const bool changed = !have_upper_guides_ ||
      upper_guides_world_.size() != 2 ||
      (upper_guides_world_[0] - wp0_world).norm() > upper_guides_change_reset_tol_m_ ||
      (upper_guides_world_[1] - wp1_world).norm() > upper_guides_change_reset_tol_m_;
    upper_guides_world_.clear();
    upper_guides_world_.push_back(wp0_world);
    upper_guides_world_.push_back(wp1_world);
    have_upper_guides_ = true;
    last_upper_guides_rx_time_ = this->now();
    if (changed) {
      preview_seed_body_.clear();
    }
  }
}


void TrackingControllerNode::gridMapCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  GridMapSnapshot snapshot;
  snapshot.resolution = msg->info.resolution;
  snapshot.width = static_cast<int>(msg->info.width);
  snapshot.height = static_cast<int>(msg->info.height);
  snapshot.origin_x = msg->info.origin.position.x;
  snapshot.origin_y = msg->info.origin.position.y;
  snapshot.frame_id = msg->header.frame_id;
  snapshot.data.assign(msg->data.begin(), msg->data.end());

  tf2::Quaternion q(
    msg->info.origin.orientation.x,
    msg->info.origin.orientation.y,
    msg->info.origin.orientation.z,
    msg->info.origin.orientation.w);
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  snapshot.origin_yaw = yaw;
  (void)roll;
  (void)pitch;

  const bool dims_ok = snapshot.width > 0 && snapshot.height > 0 && snapshot.resolution > 0.0;
  const bool data_ok =
    static_cast<int>(snapshot.data.size()) == snapshot.width * snapshot.height;
  snapshot.valid = dims_ok && data_ok;

  {
    std::lock_guard<std::mutex> lk(grid_mtx_);
    grid_map_ = snapshot;
    have_grid_map_ = snapshot.valid;
    if (snapshot.valid) {
      last_grid_rx_time_ = this->now();
    }
  }

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 2000,
    "Local body-grid accepted: topic=%s frame='%s' size=%dx%d res=%.4f origin=(%.3f, %.3f, yaw=%.3f) value_rule=%s threshold=%d valid=%d",
    grid_map_topic_.c_str(),
    snapshot.frame_id.c_str(),
    snapshot.width,
    snapshot.height,
    snapshot.resolution,
    snapshot.origin_x,
    snapshot.origin_y,
    snapshot.origin_yaw,
    grid_positive_is_drivable_ ? "high_value_is_drivable" : "low_value_is_drivable",
    grid_value_threshold_,
    snapshot.valid ? 1 : 0);
}


TrackingControllerNode::PreviewConstraintData TrackingControllerNode::makePreviewConstraints2(
  const std::vector<Eigen::Vector2d>& preview_pts_body,
  const std::vector<double>& preview_psi_rad,
  int N_pred,
  const Eigen::Vector2d& body_origin_world,
  const Eigen::Matrix2d& R_wb,
  const std::vector<double>& lane_offsets,
  double lane_width,
  const GridMapSnapshot* grid_map)
{
  (void)body_origin_world;
  (void)R_wb;

  PreviewConstraintData out;
  if (preview_pts_body.size() < 2 || N_pred <= 0) {
    return out;
  }

  const int n_pred = std::min(N_pred, static_cast<int>(preview_pts_body.size()) - 1);
  if (n_pred <= 0) {
    return out;
  }
  out.N_pred = n_pred;

  std::vector<double> lanes = lane_offsets;
  if (lanes.empty()) {
    lanes.push_back(0.0);
  }
  const int L = static_cast<int>(lanes.size());
  const double half_w = 0.5 * lane_width;
  const bool use_grid_map = (grid_map != nullptr) && grid_map->valid;
  const double sample_ds = std::max(0.02, preview_line_sample_m_);
  const int min_segment_samples = std::max(1, preview_min_segment_samples_);
  const double soft_ratio = 0.8;

  out.pmk.assign(n_pred + 1, std::vector<Eigen::Vector2d>());
  out.pMk.assign(n_pred + 1, std::vector<Eigen::Vector2d>());
  out.Nck.assign(n_pred + 1, 0);
  out.mk.assign(n_pred + 1, 0.0);
  out.nk.assign(n_pred + 1, 0.0);
  out.ck.assign(n_pred + 1, 0.0);
  out.prk.assign(n_pred + 1, Eigen::Vector2d::Zero());
  out.psirk.assign(n_pred + 1, 0.0);

  auto body_to_grid = [&](const Eigen::Vector2d& p_body, int& gx, int& gy) -> bool {
    if (!use_grid_map) {
      return false;
    }
    const double dx = p_body.x() - grid_map->origin_x;
    const double dy = p_body.y() - grid_map->origin_y;
    const double c = std::cos(grid_map->origin_yaw);
    const double s = std::sin(grid_map->origin_yaw);
    const double lx = c * dx + s * dy;
    const double ly = -s * dx + c * dy;
    gx = static_cast<int>(std::floor(lx / grid_map->resolution));
    gy = static_cast<int>(std::floor(ly / grid_map->resolution));
    return gx >= 0 && gx < grid_map->width && gy >= 0 && gy < grid_map->height;
  };

  auto is_drivable = [&](int8_t val) -> bool {
    if (val < 0) {
      return false;
    }
    if (grid_positive_is_drivable_) {
      return static_cast<int>(val) >= grid_value_threshold_;
    }
    return static_cast<int>(val) <= grid_value_threshold_;
  };

  const double map_span = use_grid_map ?
    0.5 * std::sqrt(
      std::pow(grid_map->width * grid_map->resolution, 2.0) +
      std::pow(grid_map->height * grid_map->resolution, 2.0)) : 0.0;

  for (int k = 0; k <= n_pred; ++k) {
    const int p_idx = std::min(k, static_cast<int>(preview_pts_body.size()) - 1);
    const int psi_idx = preview_psi_rad.empty() ? 0 :
      std::min(k, static_cast<int>(preview_psi_rad.size()) - 1);
    const double psi_k = preview_psi_rad.empty() ? 0.0 : preview_psi_rad[psi_idx];

    out.prk[k] = preview_pts_body[p_idx];
    out.psirk[k] = psi_k;
    out.mk[k] = std::cos(psi_k);
    out.nk[k] = std::sin(psi_k);
    out.ck[k] = -(out.mk[k] * out.prk[k].x() + out.nk[k] * out.prk[k].y());

    const Eigen::Vector2d n_vec(-std::sin(psi_k), std::cos(psi_k));
    auto add_lane_fallback_corridor = [&]() {
      out.Nck[k] = L;
      out.pmk[k].reserve(L);
      out.pMk[k].reserve(L);
      for (int l = 0; l < L; ++l) {
        out.pmk[k].push_back(out.prk[k] + (lanes[l] - half_w) * n_vec);
        out.pMk[k].push_back(out.prk[k] + (lanes[l] + half_w) * n_vec);
      }
    };

    if (!use_grid_map) {
      add_lane_fallback_corridor();
      continue;
    }

    const double visible_x_min = grid_map->origin_x + 0.5 * grid_map->resolution;
    if (out.prk[k].x() < visible_x_min) {
      add_lane_fallback_corridor();

      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "preview k=%d is before visible grid x range: prk_x=%.3f visible_x_min=%.3f; using lane fallback corridor",
        k, out.prk[k].x(), visible_x_min);

      continue;
    }

    const Eigen::Vector2d line_dir(-out.nk[k], out.mk[k]);
    bool in_run = false;
    std::vector<Eigen::Vector2d> run_points;
    run_points.reserve(64);
    int sample_count = 0;
    int in_bounds_count = 0;
    int drivable_count = 0;
    Eigen::Vector2d first_body_sample = Eigen::Vector2d::Zero();
    bool first_body_sample_set = false;
    Eigen::Vector2d prk_body = out.prk[k];
    int prk_gx = -1;
    int prk_gy = -1;
    bool prk_in_bounds = false;
    int prk_grid_value = -999;
    if (use_grid_map) {
      prk_in_bounds = body_to_grid(prk_body, prk_gx, prk_gy);
      if (prk_in_bounds) {
        const int prk_idx = prk_gy * grid_map->width + prk_gx;
        if (prk_idx >= 0 && prk_idx < static_cast<int>(grid_map->data.size())) {
          prk_grid_value = static_cast<int>(grid_map->data[prk_idx]);
        }
      }
    }

    auto flush_run = [&]() {
      if (static_cast<int>(run_points.size()) < min_segment_samples) {
        run_points.clear();
        return;
      }
      const Eigen::Vector2d pmin = run_points.front();
      const Eigen::Vector2d pmax = run_points.back();
      out.pmk[k].push_back(soft_ratio * pmin + (1.0 - soft_ratio) * pmax);
      out.pMk[k].push_back(soft_ratio * pmax + (1.0 - soft_ratio) * pmin);
      run_points.clear();
    };

    for (double s = -map_span; s <= map_span + 1e-9; s += sample_ds) {
      const Eigen::Vector2d ptest_body = out.prk[k] + s * line_dir;
      if (!first_body_sample_set) {
        first_body_sample = ptest_body;
        first_body_sample_set = true;
      }
      ++sample_count;
      int gx = -1;
      int gy = -1;
      bool drivable = false;
      if (body_to_grid(ptest_body, gx, gy)) {
        ++in_bounds_count;
        const int idx = gy * grid_map->width + gx;
        if (idx >= 0 && idx < static_cast<int>(grid_map->data.size())) {
          drivable = is_drivable(grid_map->data[idx]);
        }
      }
      if (drivable) {
        ++drivable_count;
      }

      if (drivable) {
        if (!in_run) {
          in_run = true;
          run_points.clear();
        }
        run_points.push_back(ptest_body);
      } else if (in_run) {
        flush_run();
        in_run = false;
      }
    }
    if (in_run) {
      flush_run();
    }

    out.Nck[k] = static_cast<int>(out.pmk[k].size());
    if (out.Nck[k] == 0) {
      std::ostringstream lane_offsets_stream;
      lane_offsets_stream << "[";
      for (size_t i = 0; i < lane_offsets.size(); ++i) {
        if (i > 0) {
          lane_offsets_stream << ", ";
        }
        lane_offsets_stream << lane_offsets[i];
      }
      lane_offsets_stream << "]";

      RCLCPP_WARN(
        this->get_logger(),
        "preview corridor empty at k=%d: requested_N_pred=%d resolved_N_pred=%d psi=%.3f samples=%d in_bounds=%d drivable=%d prk_body=(%.3f,%.3f) prk_grid=(%d,%d) prk_val=%d prk_in_bounds=%d first_body=(%.3f,%.3f) map_span=%.3f lane_width=%.3f lane_offsets=%s",
        k, N_pred, n_pred, psi_k, sample_count, in_bounds_count, drivable_count,
        prk_body.x(), prk_body.y(),
        prk_gx, prk_gy, prk_grid_value, prk_in_bounds ? 1 : 0,
        first_body_sample.x(), first_body_sample.y(), map_span, lane_width,
        lane_offsets_stream.str().c_str());
      out.N_pred = k - 1;
      out.pmk.resize(out.N_pred + 1);
      out.pMk.resize(out.N_pred + 1);
      out.Nck.resize(out.N_pred + 1);
      out.mk.resize(out.N_pred + 1);
      out.nk.resize(out.N_pred + 1);
      out.ck.resize(out.N_pred + 1);
      out.prk.resize(out.N_pred + 1);
      out.psirk.resize(out.N_pred + 1);
      return out;
    }
  }

  return out;
}

// control loop
void TrackingControllerNode::controlLoop()
{
  if (!have_pose_) {
    publishZeroDebugCmd("waiting for pose input");
    return;
  }

  const rclcpp::Time now = this->now();
  if (input_timeout_sec_ > 0.0 && last_pose_rx_time_.nanoseconds() > 0) {
    const double pose_age = (now - last_pose_rx_time_).seconds();
    if (pose_age > input_timeout_sec_) {
      bool goal_reached = false;
      {
        std::lock_guard<std::mutex> lk(map_mtx_);
        goal_reached = goal_reached_;
      }
      if (goal_reached) {
        publishGoalStopCmd("path goal reached; stale pose input -> keep stop command");
      } else {
        publishZeroDebugCmd("stale pose input");
      }
      return;
    }
  }

  std::vector<Eigen::Vector2d> ref;
  int wp0_idx = 0;
  int wp1_idx = 1;
  bool waypoint_seeded = false;
  bool goal_reached = false;
  {
    std::lock_guard<std::mutex> lk(map_mtx_);
    if (ref_path_.empty()) {
      publishZeroDebugCmd("waiting for global path input");
      return;
    }
    if (ref_path_.size() < 2) {
      publishZeroDebugCmd("reference path is invalid");
      return;
    }
    ref  = ref_path_;
    wp0_idx = wp0_index_;
    wp1_idx = wp1_index_;
    waypoint_seeded = waypoint_seeded_;
    goal_reached = goal_reached_;
  }

  const Eigen::Vector2d pos_cur(cur_x_, cur_y_);
  const double goal_remaining_m =
    remainingPathLengthFromClosestProjection(ref, pos_cur);
  const double goal_distance_m = (ref.back() - pos_cur).norm();
  const bool goal_stop_enabled = goal_stop_distance_m_ >= 0.0;
  const double goal_stop_distance_m = std::max(0.0, goal_stop_distance_m_);

  if (goal_reached ||
    (goal_stop_enabled &&
    (goal_remaining_m <= goal_stop_distance_m || goal_distance_m <= goal_stop_distance_m)))
  {
    {
      std::lock_guard<std::mutex> lk(map_mtx_);
      goal_reached_ = true;
      wp0_index_ = std::max(0, static_cast<int>(ref.size()) - 2);
      wp1_index_ = std::max(0, static_cast<int>(ref.size()) - 1);
      waypoint_seeded_ = true;
    }

    std::ostringstream reason;
    reason << "path goal reached"
           << " remaining=" << goal_remaining_m
           << " goal_dist=" << goal_distance_m
           << " stop_dist=" << goal_stop_distance_m;
    publishGoalStopCmd(reason.str());
    return;
  }

  const double v = std::max(0.05, cur_speed_);
  const Eigen::Vector2d vel_world(v * std::cos(cur_yaw_), v * std::sin(cur_yaw_));

  const double dt = 1.0 / loop_rate_hz_ctrl_;
  const double v_ref = std::max(0.05, v_max_);

  Eigen::Matrix2d R_wb;
  R_wb << std::cos(cur_yaw_), -std::sin(cur_yaw_),
          std::sin(cur_yaw_),  std::cos(cur_yaw_);

  bool using_upper_guides = false;
  double upper_guides_age = -1.0;
  Eigen::Vector2d wp0_world = ref[std::max(0, std::min(wp0_idx, static_cast<int>(ref.size()) - 1))];
  Eigen::Vector2d wp1_world = ref[std::max(0, std::min(wp1_idx, static_cast<int>(ref.size()) - 1))];

  if (use_upper_guides_) {
    std::lock_guard<std::mutex> lk(upper_guides_mtx_);
    if (have_upper_guides_ && upper_guides_world_.size() >= 2) {
      upper_guides_age =
        (last_upper_guides_rx_time_.nanoseconds() > 0) ?
        (now - last_upper_guides_rx_time_).seconds() : -1.0;
      const double guides_age =
        upper_guides_age;
      if (upper_guides_timeout_sec_ <= 0.0 || guides_age <= upper_guides_timeout_sec_) {
        wp0_world = upper_guides_world_[0];
        wp1_world = upper_guides_world_[1];
        using_upper_guides = (wp1_world - wp0_world).norm() > 1e-6;
      }
    }
  }

  if (!using_upper_guides) {
    if (!waypoint_seeded) {
      wp0_idx = 0;
      wp1_idx = std::min(1, static_cast<int>(ref.size()) - 1);
      waypoint_seeded = true;
    }
    const std::array<int, 2> wp_pair =
      findWaypointPair(ref, wp0_idx, pos_cur, vel_world, wp_switch_eps_);
    wp0_idx = wp_pair[0];
    wp1_idx = wp_pair[1];
    wp0_world = ref[wp0_idx];
    wp1_world = ref[wp1_idx];
    {
      std::lock_guard<std::mutex> lk(map_mtx_);
      wp0_index_ = wp0_idx;
      wp1_index_ = wp1_idx;
      waypoint_seeded_ = waypoint_seeded;
    }
  }

  // body frame preview
  const int preview_steps = 6;
  const double delta_s = std::max(0.05, dt * v_ref);
  const Eigen::Vector2d wp0_body = R_wb.transpose() * (wp0_world - pos_cur);
  const Eigen::Vector2d wp1_body = R_wb.transpose() * (wp1_world - pos_cur);

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 2000,
    "guide source: %s wp0=(%.3f,%.3f) wp1=(%.3f,%.3f)",
    using_upper_guides ? "upper_guides" : "global_path",
    wp0_world.x(), wp0_world.y(), wp1_world.x(), wp1_world.y());

  double preview_seed_size_for_trace = 0.0;
  const PreviewFallbackDirection fallback_dir =
    computePreviewFallbackDirection(wp0_body, wp1_body);
  const bool preview_wp0_usable = fallback_dir.wp0_usable;
  double preview_yaw0_before_clamp =
    std::numeric_limits<double>::quiet_NaN();
  double preview_yaw0_after_clamp =
    std::numeric_limits<double>::quiet_NaN();

  if (preview_seed_body_.size() < 2) {
    preview_yaw0_before_clamp = fallback_dir.yaw_before_clamp;
    preview_yaw0_after_clamp = fallback_dir.yaw_after_clamp;
  }

  if (preview_reset_on_bad_seed_) {
    bool bad_seed = false;
    for (const Eigen::Vector2d &p_seed : preview_seed_body_) {
      if (!std::isfinite(p_seed.x()) ||
        !std::isfinite(p_seed.y()) ||
        p_seed.x() < preview_seed_min_forward_x_m_ ||
        std::abs(p_seed.y()) > preview_seed_max_lateral_y_m_)
      {
        bad_seed = true;
        break;
      }
    }

    if (bad_seed) {
      preview_seed_body_.clear();
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "preview seed reset: nonfinite or outside body bounds");
    }
  }
  preview_seed_size_for_trace = static_cast<double>(preview_seed_body_.size());

  std::vector<Eigen::Vector2d> preview_body;
  std::vector<double> preview_psi_rad;
  buildPreviewFromPredictionSeed(
    wp0_body,
    wp1_body,
    preview_steps,
    delta_s,
    preview_seed_body_,
    preview_body,
    preview_psi_rad);
  if (!preview_psi_rad.empty()) {
    if (!std::isfinite(preview_yaw0_before_clamp)) {
      preview_yaw0_before_clamp = preview_psi_rad.front();
    }
    preview_yaw0_after_clamp = preview_psi_rad.front();
  }

  std::vector<Eigen::Vector2d> preview_world;
  preview_world.reserve(preview_body.size());
  for (const Eigen::Vector2d &p_body : preview_body) {
    preview_world.emplace_back(pos_cur + R_wb * p_body);
  }
  publishPath(preview_world, "map");

  // ---- DAQP MIQP  ----
  GridMapSnapshot grid_snapshot;
  bool have_grid = false;
  {
    std::lock_guard<std::mutex> lk(grid_mtx_);
    grid_snapshot = grid_map_;
    have_grid = have_grid_map_;
  }

  if (input_timeout_sec_ > 0.0 && last_grid_rx_time_.nanoseconds() > 0) {
    const double grid_age = (now - last_grid_rx_time_).seconds();
    if (grid_age > input_timeout_sec_) {
      have_grid = false;
    }
  }

  if (require_grid_map_ && !have_grid) {
    publishZeroDebugCmd("grid map not received/invalid (required)");
    return;
  }

  if (!have_grid) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000 /*ms*/,
      "grid map not received/invalid, using lane-offset fallback");
  }
  const GridMapSnapshot* grid_ptr = have_grid ? &grid_snapshot : nullptr;

  const PreviewConstraintData preview_c = makePreviewConstraints2(
    preview_body, preview_psi_rad, preview_steps, pos_cur, R_wb, lane_offsets_, lane_width_, grid_ptr);
  const int N_pred = preview_c.N_pred;
  if (N_pred <= 0) {
    publishZeroDebugCmd("preview constraints invalid or empty");
    return;
  }

  // ny = sum(Nck(2:end))
  std::vector<int> z_step_offset(N_pred + 1, 0);
  for (int k = 1; k <= N_pred; ++k) {
    const int nck_k = (k < static_cast<int>(preview_c.Nck.size())) ? preview_c.Nck[k] : 0;
    z_step_offset[k] = z_step_offset[k - 1] + std::max(0, nck_k);
  }

  const int nu = 2 * N_pred;
  const int ny = z_step_offset[N_pred];
  const int nv = nu + ny;
  if (ny <= 0 || nv <= 0) {
    publishZeroDebugCmd("invalid MIQP dimensions");
    return;
  }

  // G matrix X_con = G * U
  std::vector<Matrix<double>> Bk;
  Bk.reserve(N_pred);
  for (int k = 0; k < N_pred; ++k) {
    const double psi_k = preview_c.psirk[k];
    Matrix<double> Bk_k;
    Bk_k.resize(0.0, 2, 2);
    Bk_k[0][0] = dt * std::cos(psi_k);
    Bk_k[0][1] = -dt * v_ref * std::sin(psi_k);
    Bk_k[1][0] = dt * std::sin(psi_k);
    Bk_k[1][1] = dt * v_ref * std::cos(psi_k);
    Bk.push_back(Bk_k);
  }

  Matrix<double> G;
  G.resize(0.0, 2 * (N_pred + 1), nv);
  for (int k = 0; k < N_pred; ++k) {
    const int r0 = 2 * (k + 1);
    for (int j = 0; j <= k; ++j) {
      const int c0 = 2 * j;
      const Matrix<double> &Bj = Bk[j];
      for (int rr = 0; rr < 2; ++rr) {
        for (int cc = 0; cc < 2; ++cc) {
          G[r0 + rr][c0 + cc] = Bj[rr][cc];
        }
      }
    }
  }

  Matrix<double> Qblk;
  Qblk.resize(0.0, 2 * (N_pred + 1), 2 * (N_pred + 1));
  constexpr double qx = 0.1;
  for (int k = 0; k <= N_pred; ++k) {
    Qblk[2 * k + 0][2 * k + 0] = qx;
    Qblk[2 * k + 1][2 * k + 1] = qx;
  }

  Matrix<double> Rblk;
  Rblk.resize(0.0, nv, nv);
  constexpr double ru = 1.0;
  constexpr double binary_reg = 1e-6;
  for (int k = 0; k < N_pred; ++k) {
    Rblk[2 * k + 0][2 * k + 0] = ru;
    Rblk[2 * k + 1][2 * k + 1] = ru;
  }
  const double segment_pref_weight = std::max(w_segment_preference_, binary_reg);
  if (w_segment_preference_ <= 0.0) {
    RCLCPP_WARN_ONCE(
      this->get_logger(),
      "w_segment_preference <= 0. Applying %.1e binary regularization so the DAQP Hessian stays positive definite.",
      binary_reg);
  }
  for (int i = 0; i < ny; ++i) {
    Rblk[nu + i][nu + i] = segment_pref_weight;
  }

  const Matrix<double> Gt = transpose(G);
  const Matrix<double> QG = multiply(Qblk, G);
  Matrix<double> Hqp = multiply(Gt, QG);
  for (int r = 0; r < nv; ++r) {
    for (int c = 0; c < nv; ++c) {
      Hqp[r][c] += Rblk[r][c];
    }
  }
  for (int r = 0; r < nv; ++r) {
    for (int c = r + 1; c < nv; ++c) {
      const double sym = 0.5 * (Hqp[r][c] + Hqp[c][r]);
      Hqp[r][c] = sym;
      Hqp[c][r] = sym;
    }
    Hqp[r][r] += 1e-10;
  }
  Vector<double> fqp(0.0, nv);  // X1=zeros => fqp=0

  // Urk from preview yaw
  // urk(1,k)=0, urk(2,k)=psirk(k)-psirk(k-1), psirk(0)=0.
  Vector<double> Urk(0.0, nu);
  for (int k = 0; k < N_pred; ++k) {
    const double psir_k = preview_c.psirk[k];
    const double psir_prev = (k == 0) ? 0.0 : preview_c.psirk[k - 1];
    Urk[2 * k + 0] = 0.0;
    Urk[2 * k + 1] = psir_k - psir_prev;
  }

  // Input inequality HU/hu 
  Matrix<double> HU(0.0, 4 * N_pred, nu);
  Vector<double> hu(0.0, 4 * N_pred);
  const double steer_limit = std::max(std::abs(max_steer_left_), std::abs(max_steer_right_));
  const double phi_M = std::tan(steer_limit) * dt / std::max(1e-3, wheelbase_);
  const double v1 = std::max(1.0, v);
  for (int k = 0; k < N_pred; ++k) {
    HU[k][2 * k + 0] = 1.0;
    HU[N_pred + k][2 * k + 0] = -1.0;
    for (int j = 0; j <= k; ++j) {
      HU[2 * N_pred + k][2 * j + 0] = -phi_M;
      HU[3 * N_pred + k][2 * j + 0] = -phi_M;
    }
    HU[2 * N_pred + k][2 * k + 1] = 1.0;
    HU[3 * N_pred + k][2 * k + 1] = -1.0;

    hu[k] = a_max_ * dt;
    hu[N_pred + k] = a_max_ * dt;
    hu[2 * N_pred + k] = phi_M * v1;
    hu[3 * N_pred + k] = phi_M * v1;
  }

  Vector<double> hu_shifted(0.0, 4 * N_pred);
  for (int r = 0; r < 4 * N_pred; ++r) {
    double acc = 0.0;
    for (int c = 0; c < nu; ++c) {
      acc += HU[r][c] * Urk[c];
    }
    hu_shifted[r] = hu[r] - acc;
  }

  // Hx/hx from makePreviewConstraints2 output (Big-M)
  const int n_lane_ineq = 2 * ny;
  const int n_input_ineq = 4 * N_pred;
  const bool use_dpsi_delta_constraint =
    std::isfinite(dpsi_delta_max_rad_) && dpsi_delta_max_rad_ > 0.0;
  const int n_dpsi_delta_ineq = use_dpsi_delta_constraint ? 2 * N_pred : 0;
  const int n_ineq = n_lane_ineq + n_input_ineq + n_dpsi_delta_ineq;
  Matrix<double> Aineq(0.0, n_ineq, nv);
  Vector<double> bineq(0.0, n_ineq);

  int rineq = 0;
  for (int k = 1; k <= N_pred; ++k) {
    const int row_state = 2 * k;
    const int nck_k = preview_c.Nck[k];
    if (nck_k <= 0 ||
      static_cast<int>(preview_c.pmk[k].size()) < nck_k ||
      static_cast<int>(preview_c.pMk[k].size()) < nck_k)
    {
      publishZeroDebugCmd("invalid Nck/pmk/pMk while building MIQP");
      return;
    }

    const double mk_k = preview_c.mk[k];
    const double nk_k = preview_c.nk[k];
    const double ck_k = preview_c.ck[k];
    const Eigen::Vector2d prk_k = preview_c.prk[k];

    
    // Rc = [nk^2, -mk*nk; -mk*nk, mk^2], qc = -ck*[mk;nk]
    const double Rc00 = nk_k * nk_k;
    const double Rc01 = -mk_k * nk_k;
    const double Rc10 = -mk_k * nk_k;
    const double Rc11 = mk_k * mk_k;
    const double qc0 = -ck_k * mk_k;
    const double qc1 = -ck_k * nk_k;

    const Eigen::Vector2d &pm_first = preview_c.pmk[k].front();
    const Eigen::Vector2d &pM_last = preview_c.pMk[k].back();
    const double dx = pM_last.x() - pm_first.x();
    const double dy = pM_last.y() - pm_first.y();
    const double dc2 = dx * dx + dy * dy;
    if (dc2 < 1e-9) {
      publishZeroDebugCmd("degenerate lane segment while building MIQP");
      return;
    }

    const double RL0 = dx / dc2;
    const double RL1 = dy / dc2;
    const double qL = -(pm_first.x() * dx + pm_first.y() * dy) / dc2;

    const double RL_Rc0 = RL0 * Rc00 + RL1 * Rc10;
    const double RL_Rc1 = RL0 * Rc01 + RL1 * Rc11;
    const double RL_qc = RL0 * qc0 + RL1 * qc1;
    const double RL_Rc_pr = RL_Rc0 * prk_k.x() + RL_Rc1 * prk_k.y();

    for (int i = 0; i < nck_k; ++i) {
      const int zidx = nu + z_step_offset[k - 1] + i;
      const Eigen::Vector2d &pmik = preview_c.pmk[k][i];
      const Eigen::Vector2d &pMik = preview_c.pMk[k][i];
      const double lMik = RL0 * pMik.x() + RL1 * pMik.y() + qL;
      const double lmik = RL0 * pmik.x() + RL1 * pmik.y() + qL;

      const double hDik0 = RL_Rc_pr + RL_qc + qL - lMik;
      const double hDik1 = -RL_Rc_pr - RL_qc - qL + lmik;

      for (int j = 0; j < nu; ++j) {
        Aineq[rineq][j] = RL_Rc0 * G[row_state + 0][j] + RL_Rc1 * G[row_state + 1][j];
      }
      Aineq[rineq][zidx] += miqp_big_m_;
      bineq[rineq++] = miqp_big_m_ - hDik0;

      for (int j = 0; j < nu; ++j) {
        Aineq[rineq][j] = -RL_Rc0 * G[row_state + 0][j] - RL_Rc1 * G[row_state + 1][j];
      }
      Aineq[rineq][zidx] += miqp_big_m_;
      bineq[rineq++] = miqp_big_m_ - hDik1;
    }
  }

  for (int r = 0; r < n_input_ineq; ++r) {
    for (int c = 0; c < nu; ++c) {
      Aineq[rineq + r][c] = HU[r][c];
    }
    bineq[rineq + r] = hu_shifted[r];
  }
  rineq += n_input_ineq;

  if (use_dpsi_delta_constraint) {
    const double limit = dpsi_delta_max_rad_;
    const double prev_dpsi = have_prev_first_dpsi_ ? prev_first_dpsi_ : 0.0;

    Aineq[rineq][1] = 1.0;
    bineq[rineq++] = limit + prev_dpsi - Urk[1];

    Aineq[rineq][1] = -1.0;
    bineq[rineq++] = limit - prev_dpsi + Urk[1];

    for (int k = 1; k < N_pred; ++k) {
      const int i = 2 * k + 1;
      const int j = 2 * (k - 1) + 1;
      const double d_urk = Urk[i] - Urk[j];

      Aineq[rineq][i] = 1.0;
      Aineq[rineq][j] = -1.0;
      bineq[rineq++] = limit - d_urk;

      Aineq[rineq][i] = -1.0;
      Aineq[rineq][j] = 1.0;
      bineq[rineq++] = limit + d_urk;
    }
  }

  if (rineq != n_ineq) {
    publishZeroDebugCmd("MIQP row build mismatch");
    return;
  }

  Matrix<double> Aeq(0.0, N_pred, nv);
  Vector<double> beq(1.0, N_pred);
  for (int k = 1; k <= N_pred; ++k) {
    const int nck_k = preview_c.Nck[k];
    if (nck_k <= 0) {
      publishZeroDebugCmd("invalid Nck while building MIQP equalities");
      return;
    }
    for (int i = 0; i < nck_k; ++i) {
      Aeq[k - 1][nu + z_step_offset[k - 1] + i] = 1.0;
    }
  }

  QuadraticProblem miqp_solver(false);
  Variable *uvar = miqp_solver.vector_variable(nv, "U");
  if (!miqp_solver.add_variable(uvar)) {
    publishZeroDebugCmd("DAQP setup failed: cannot add variable");
    return;
  }
  const Var U = miqp_solver.get_variable(uvar);
  const Var main_var = miqp_solver.get_main_variable();

  Constraint cineq(main_var);
  cineq.set_constraint_variable(U, Aineq);
  cineq.set_known_term(bineq);
  miqp_solver.add_leq_constraint(cineq);

  Constraint ceq(main_var);
  ceq.set_constraint_variable(U, Aeq);
  ceq.set_known_term(beq);
  miqp_solver.add_equality_constraint(ceq);

  miqp_solver.set_Q_matrix(Hqp);
  miqp_solver.set_q0_vector(fqp);

  for (int k = 1; k <= N_pred; ++k) {
    const int nck_k = preview_c.Nck[k];
    for (int i = 0; i < nck_k; ++i) {
      const int zidx = nu + z_step_offset[k - 1] + i;
      miqp_solver.set_binary_var(U[zidx]);
    }
  }

  Vector<double> arg;
  const double fval = miqp_solver.solve_problem(arg);
  if (!std::isfinite(fval) || arg.size() < static_cast<unsigned int>(nu)) {
    publishZeroDebugCmd("DAQP MIQP infeasible");
    return;
  }

  RCLCPP_WARN_THROTTLE(
    this->get_logger(), *this->get_clock(), 500 /*ms*/,
    "DAQP MIQP solved: wp0=%d wp1=%d N_pred=%d obj=%.4f",
    wp0_idx, wp1_idx, N_pred, fval);

  std::vector<Eigen::Vector2d> p_pred_body(N_pred + 1, Eigen::Vector2d::Zero());
  for (int k = 0; k <= N_pred; ++k) {
    const int row = 2 * k;
    double ex = 0.0;
    double ey = 0.0;
    for (int j = 0; j < nv; ++j) {
      ex += G[row + 0][j] * arg[j];
      ey += G[row + 1][j] * arg[j];
    }
    p_pred_body[k] = preview_c.prk[k] + Eigen::Vector2d(ex, ey);
  }

  preview_seed_body_.clear();
  preview_seed_body_.reserve(std::max(1, N_pred));
  preview_seed_body_.push_back(Eigen::Vector2d::Zero());
  if (N_pred >= 2) {
    const double psi1 = arg[1] + Urk[1];
    const double c = std::cos(psi1);
    const double s1 = std::sin(psi1);
    for (int k = 1; k < N_pred; ++k) {
      const Eigen::Vector2d rel = p_pred_body[k + 1] - p_pred_body[1];
      const Eigen::Vector2d rel_rot(c * rel.x() + s1 * rel.y(),
                                    -s1 * rel.x() + c * rel.y());
      preview_seed_body_.push_back(rel_rot);
    }
  }

  const auto make_control_step =
    [this, dt, v1, steer_limit](double dv_cmd, double dpsi_cmd) {
      const double yaw_rate_cmd = dpsi_cmd / std::max(1e-6, dt);
      double steer_cmd = 0.0;
      if (std::abs(v1) > 1e-4) {
        steer_cmd = std::atan((wheelbase_ * yaw_rate_cmd) / v1);
      }
      SolverControlStep step;
      step.dv_mps = dv_cmd;
      step.dpsi_rad = dpsi_cmd;
      step.steer_norm = clampd(
        steer_cmd / std::max(1e-6, steer_limit), -1.0, 1.0);
      return step;
    };

  std::vector<SolverControlStep> solver_cmds;
  solver_cmds.reserve(static_cast<size_t>(N_pred));
  for (int k = 0; k < N_pred; ++k) {
    solver_cmds.push_back(make_control_step(
      arg[2 * k + 0] + Urk[2 * k + 0],
      arg[2 * k + 1] + Urk[2 * k + 1]));
  }

  if (solver_cmds.empty()) {
    publishZeroDebugCmd("solver returned empty command sequence");
    return;
  }

  previous_solver_cmds_.assign(
    solver_cmds.begin() + (solver_cmds.size() > 1 ? 1 : 0),
    solver_cmds.end());
  previous_solver_cmd_index_ = 0;

  if (mpc_trace_pub_) {
    const double nan = std::numeric_limits<double>::quiet_NaN();

    auto safe_prk_x = [&](int k) -> double {
      return (k >= 0 && k < static_cast<int>(preview_c.prk.size())) ? preview_c.prk[k].x() : nan;
    };

    auto safe_prk_y = [&](int k) -> double {
      return (k >= 0 && k < static_cast<int>(preview_c.prk.size())) ? preview_c.prk[k].y() : nan;
    };

    auto safe_Nck = [&](int k) -> double {
      return (k >= 0 && k < static_cast<int>(preview_c.Nck.size())) ?
        static_cast<double>(preview_c.Nck[k]) : 0.0;
    };

    auto selected_corridor = [&](int k) -> std::pair<double, double> {
      if (k <= 0 || k > N_pred) {
        return {-1.0, nan};
      }

      const int nck_k = (k < static_cast<int>(preview_c.Nck.size())) ? preview_c.Nck[k] : 0;
      if (nck_k <= 0) {
        return {-1.0, nan};
      }

      double selected_idx = -1.0;
      double center_y = nan;

      for (int i = 0; i < nck_k; ++i) {
        const int zidx = nu + z_step_offset[k - 1] + i;
        if (zidx >= 0 && zidx < static_cast<int>(arg.size()) && arg[zidx] > 0.5) {
          selected_idx = static_cast<double>(i);

          if (k < static_cast<int>(preview_c.pmk.size()) &&
              k < static_cast<int>(preview_c.pMk.size()) &&
              i < static_cast<int>(preview_c.pmk[k].size()) &&
              i < static_cast<int>(preview_c.pMk[k].size()))
          {
            center_y = 0.5 * (preview_c.pmk[k][i].y() + preview_c.pMk[k][i].y());
          }
          break;
        }
      }

      return {selected_idx, center_y};
    };

    const double pose_age =
      (last_pose_rx_time_.nanoseconds() > 0) ? (now - last_pose_rx_time_).seconds() : -1.0;

    const double grid_age =
      (last_grid_rx_time_.nanoseconds() > 0) ? (now - last_grid_rx_time_).seconds() : -1.0;

    const double guide_angle0 = std::atan2(wp0_body.y(), wp0_body.x());
    const double guide_angle1 = std::atan2(wp1_body.y(), wp1_body.x());

    const double preview_psi0 = (!preview_c.psirk.empty()) ? preview_c.psirk[0] : nan;
    const double preview_psi1 = (preview_c.psirk.size() > 1) ? preview_c.psirk[1] : nan;
    const double preview_psi2 = (preview_c.psirk.size() > 2) ? preview_c.psirk[2] : nan;

    const double Urk_dv0 = (Urk.size() > 0) ? Urk[0] : nan;
    const double Urk_dpsi0 = (Urk.size() > 1) ? Urk[1] : nan;

    const double arg_dv0 = (arg.size() > 0) ? arg[0] : nan;
    const double arg_dpsi0 = (arg.size() > 1) ? arg[1] : nan;

    const double cmd_dv0 = arg_dv0 + Urk_dv0;
    const double cmd_dpsi0 = arg_dpsi0 + Urk_dpsi0;

    const double prev_first_dpsi = have_prev_first_dpsi_ ? prev_first_dpsi_ : 0.0;
    const double dpsi_delta = cmd_dpsi0 - prev_first_dpsi;
    const double dpsi_delta_limit =
      (std::isfinite(dpsi_delta_max_rad_) && dpsi_delta_max_rad_ > 0.0) ?
      dpsi_delta_max_rad_ : nan;

    const double yaw_rate_cmd = cmd_dpsi0 / std::max(1e-6, dt);

    const double steer_angle_raw =
      (std::abs(v1) > 1e-4) ?
      std::atan((wheelbase_ * yaw_rate_cmd) / v1) : 0.0;

    const double steer_norm_raw =
      steer_angle_raw / std::max(1e-6, steer_limit);

    const double steer_norm_clamped =
      clampd(steer_norm_raw, -1.0, 1.0);

    const double mavlink_steering_norm =
      -clampd(mavlink_steer_sign_ * steer_norm_clamped, -1.0, 1.0);

    const double throttle_norm = computeThrottleNorm(cmd_dv0, true);
    const double output_steer_max = std::max(0.0, std::abs(steer_norm_max_));
    const double raw_steer_before_filter =
      clampd(solver_cmds.front().steer_norm, -output_steer_max, output_steer_max);
    const double applied_steer_after_filter =
      computeFilteredSteerNorm(solver_cmds.front().steer_norm, now);

    const auto sel1 = selected_corridor(1);
    const auto sel2 = selected_corridor(2);
    const auto sel3 = selected_corridor(3);

    std_msgs::msg::Float64MultiArray trace;
    trace.data = {
      now.seconds(),
      1.0,
      cur_x_,
      cur_y_,
      cur_yaw_,
      cur_speed_,
      pose_age,
      grid_age,
      upper_guides_age,
      using_upper_guides ? 1.0 : 0.0,
      wp0_body.x(),
      wp0_body.y(),
      wp1_body.x(),
      wp1_body.y(),
      guide_angle0,
      guide_angle1,
      preview_psi0,
      preview_psi1,
      preview_psi2,
      Urk_dv0,
      Urk_dpsi0,
      arg_dv0,
      arg_dpsi0,
      cmd_dv0,
      cmd_dpsi0,
      prev_first_dpsi,
      dpsi_delta,
      dpsi_delta_limit,
      dt,
      v,
      v1,
      v_ref,
      wheelbase_,
      steer_limit,
      yaw_rate_cmd,
      steer_angle_raw,
      steer_norm_raw,
      steer_norm_clamped,
      mavlink_steering_norm,
      throttle_norm,
      fval,
      static_cast<double>(N_pred),
      static_cast<double>(ny),
      safe_Nck(1),
      safe_Nck(2),
      safe_Nck(3),
      sel1.first,
      sel2.first,
      sel3.first,
      sel1.second,
      sel2.second,
      sel3.second,
      safe_prk_x(1),
      safe_prk_y(1),
      safe_prk_x(2),
      safe_prk_y(2),
      have_grid ? 1.0 : 0.0,
      have_grid ? grid_snapshot.origin_x : nan,
      have_grid ? grid_snapshot.origin_y : nan,
      have_grid ? grid_snapshot.origin_yaw : nan,
      have_grid ? grid_snapshot.resolution : nan,
      have_grid ? static_cast<double>(grid_snapshot.width) : 0.0,
      have_grid ? static_cast<double>(grid_snapshot.height) : 0.0,
      preview_seed_size_for_trace,
      preview_wp0_usable ? 1.0 : 0.0,
      preview_yaw0_before_clamp,
      preview_yaw0_after_clamp,
      raw_steer_before_filter,
      applied_steer_after_filter
    };

    mpc_trace_pub_->publish(trace);
  }

  const SolverControlStep & cmd = solver_cmds.front();
  publishcontrolCmd(cmd.dv_mps, cmd.dpsi_rad, cmd.steer_norm, true);
  prev_first_dpsi_ = cmd.dpsi_rad;
  have_prev_first_dpsi_ = true;

}
// find_wp 
std::array<int, 2> TrackingControllerNode::findWaypointPair(
  const std::vector<Eigen::Vector2d>& waypoints,
  int wp0_idx,
  const Eigen::Vector2d& position,
  const Eigen::Vector2d& velocity,
  double eps) const
{
  if (waypoints.empty()) {
    return {0, 0};
  }

  const int n_wp = static_cast<int>(waypoints.size());
  if (n_wp == 1) {
    return {0, 0};
  }

  int i0 = clampi(wp0_idx, 0, n_wp - 2);

  Eigen::Vector2d moving_dir = velocity;
  const double v_norm = moving_dir.norm();
  if (v_norm >= 1e-9) {
    moving_dir /= v_norm;
    if ((waypoints[i0] - position).dot(moving_dir) < eps) {
      i0 = std::min(i0 + 1, n_wp - 2);
    }
  }
  return {i0, i0 + 1};
}

TrackingControllerNode::PreviewFallbackDirection
TrackingControllerNode::computePreviewFallbackDirection(
  const Eigen::Vector2d& wp0_body,
  const Eigen::Vector2d& wp1_body) const
{
  PreviewFallbackDirection result;
  result.dir_before_clamp = Eigen::Vector2d(1.0, 0.0);

  const double wp0_norm = wp0_body.norm();
  result.wp0_usable =
    std::isfinite(wp0_body.x()) &&
    std::isfinite(wp0_body.y()) &&
    wp0_norm >= preview_min_wp_norm_m_ &&
    wp0_body.x() >= preview_min_forward_x_m_ &&
    std::abs(wp0_body.y()) <= preview_max_lateral_y_m_;

  Eigen::Vector2d tangent = wp1_body - wp0_body;
  const bool tangent_usable =
    std::isfinite(tangent.x()) &&
    std::isfinite(tangent.y()) &&
    tangent.norm() > 1e-6;

  if (preview_use_path_tangent_when_seed_empty_ && tangent_usable) {
    if (tangent.x() < 0.0) {
      tangent = -tangent;
    }
    result.dir_before_clamp = tangent.normalized();
  } else if (result.wp0_usable) {
    result.dir_before_clamp = wp0_body / wp0_norm;
  } else if (tangent_usable) {
    if (tangent.x() < 0.0) {
      tangent = -tangent;
    }
    result.dir_before_clamp = tangent.normalized();
  }

  const double yaw_limit = std::max(1e-3, std::abs(preview_max_yaw_rad_));
  result.yaw_before_clamp =
    std::atan2(result.dir_before_clamp.y(), result.dir_before_clamp.x());
  result.yaw_after_clamp =
    clampd(result.yaw_before_clamp, -yaw_limit, yaw_limit);

  return result;
}

void TrackingControllerNode::buildPreviewFromPredictionSeed(
  const Eigen::Vector2d& wp0_body,
  const Eigen::Vector2d& wp1_body,
  int horizon_steps,
  double delta_s,
  const std::vector<Eigen::Vector2d>& p_seed_body,
  std::vector<Eigen::Vector2d>& preview_pts_body,
  std::vector<double>& preview_psi_rad) const
{
  const int n = std::max(1, horizon_steps);
  const double ds = std::max(1e-3, delta_s);

  preview_pts_body.assign(n + 1, Eigen::Vector2d::Zero());
  preview_psi_rad.assign(n + 1, 0.0);

  if (p_seed_body.size() < 2) {
    const PreviewFallbackDirection fallback_dir =
      computePreviewFallbackDirection(wp0_body, wp1_body);
    const Eigen::Vector2d dir0(
      std::cos(fallback_dir.yaw_after_clamp),
      std::sin(fallback_dir.yaw_after_clamp));

    for (int k = 0; k <= n; ++k) {
      preview_pts_body[k] = dir0 * (ds * static_cast<double>(k));
    }
  } else {
    const Eigen::Vector2d& plast = p_seed_body.back();
    const Eigen::Vector2d& pprev = p_seed_body[p_seed_body.size() - 2];
    const Eigen::Vector2d to_wp0 = wp0_body - plast;
    const Eigen::Vector2d travel = plast - pprev;
    const bool switch_to_wp1 = to_wp0.dot(travel) < ds;
    const Eigen::Vector2d target = switch_to_wp1 ? wp1_body : wp0_body;

    Eigen::Vector2d wpB = plast;
    const Eigen::Vector2d dir = target - plast;
    const double nd = dir.norm();
    if (nd > 1e-6) {
      wpB += ds * dir / nd;
    }

    std::vector<Eigen::Vector2d> trj = p_seed_body;
    trj.push_back(wpB);
    for (int k = 0; k <= n; ++k) {
      const int idx = std::min(k, static_cast<int>(trj.size()) - 1);
      preview_pts_body[k] = trj[idx];
    }
  }

  for (int k = 0; k < n; ++k) {
    const Eigen::Vector2d dp = preview_pts_body[k + 1] - preview_pts_body[k];
    if (dp.norm() > 1e-9) {
      preview_psi_rad[k] = std::atan2(dp.y(), dp.x());
    } else if (k > 0) {
      preview_psi_rad[k] = preview_psi_rad[k - 1];
    }
  }
  preview_psi_rad[n] = preview_psi_rad[n - 1];

  const double yaw_limit = std::max(1e-3, std::abs(preview_max_yaw_rad_));
  for (double &psi : preview_psi_rad) {
    if (!std::isfinite(psi)) {
      psi = 0.0;
    }
    psi = clampd(psi, -yaw_limit, yaw_limit);
  }
}

// nav_msgs/Path 
void TrackingControllerNode::publishPath(
  const std::vector<Eigen::Vector2d> & pts,
  const std::string & frame_id)
{
  if (!local_curve_pub_) {
    return;
  }

  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = frame_id;
  path.poses.resize(pts.size());

  for (size_t i = 0; i < pts.size(); ++i) {
    path.poses[i].header = path.header;
    path.poses[i].pose.position.x = pts[i].x();
    path.poses[i].pose.position.y = pts[i].y();
    path.poses[i].pose.position.z = 0.0;
  }

  local_curve_pub_->publish(path);
}

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<imac_ctrl::TrackingControllerNode>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
