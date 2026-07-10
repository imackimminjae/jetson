#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <chrono>
#include <cstring>
#include <limits>
#include <mutex>
#include <string>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <imac_interfaces/msg/virtual_control_command.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

extern "C" {
#include <mavlink/v2.0/common/mavlink.h>
}

namespace imac_ctrl
{

namespace
{

double clampd(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

int clampi(int v, int lo, int hi)
{
  return std::max(lo, std::min(hi, v));
}

#ifndef MAV_CMD_DO_SET_ACTUATOR
#define MAV_CMD_DO_SET_ACTUATOR 187
#endif

#ifndef MAV_CMD_COMPONENT_ARM_DISARM
#define MAV_CMD_COMPONENT_ARM_DISARM 400
#endif

constexpr uint8_t kPx4CustomMainModeManual = 1;

uint32_t px4CustomMode(uint8_t main_mode, uint8_t sub_mode = 0)
{
  return (static_cast<uint32_t>(sub_mode) << 24) |
         (static_cast<uint32_t>(main_mode) << 16);
}

}  // namespace

class LowerDutyNode final : public rclcpp::Node
{
public:
  LowerDutyNode()
  : Node("lower_duty_node")
  {
    declare_parameter<std::string>("solver_cmd_topic", "/control/solver_cmd");
    declare_parameter<std::string>("applied_cmd_topic", "/control/applied_cmd");
    declare_parameter<std::string>("duty_topic", "/control/duty_cmd");
    declare_parameter<std::string>("pwm_topic", "/cmd_pwm");
    declare_parameter<std::string>("virtual_cmd_topic", "/cmd_control");
    declare_parameter<double>("loop_rate_hz", 20.0);
    declare_parameter<double>("input_timeout_sec", 0.30);
    declare_parameter<double>("hold_last_valid_sec", 0.30);
    declare_parameter<bool>("send_neutral_on_invalid", true);
    declare_parameter<bool>("publish_applied_cmd_when_invalid", true);
    declare_parameter<bool>("enable_output_filter", true);
    declare_parameter<double>("steer_norm_max", 1.0);
    declare_parameter<double>("steer_norm_rate_max", 2.0);
    declare_parameter<double>("steer_norm_lpf_alpha", 0.8);
    declare_parameter<double>("target_speed_mps", 1.0);
    declare_parameter<double>("target_accel_mps2", 0.0);
    declare_parameter<double>("speed_kp", 0.45);
    declare_parameter<double>("speed_ki", 0.05);
    declare_parameter<double>("throttle_ff", 0.08);
    declare_parameter<double>("max_virtual_throttle", 0.35);
    declare_parameter<double>("max_virtual_brake", 0.70);
    declare_parameter<double>("speed_deadband_mps", 0.03);
    declare_parameter<double>("speed_integral_limit", 2.0);
    declare_parameter<double>("throttle_max_norm", 0.17);
    declare_parameter<double>("steer_sign", 1.0);
    declare_parameter<bool>("publish_pwm", true);
    declare_parameter<int>("pwm_min", 1000);
    declare_parameter<int>("pwm_center", 1500);
    declare_parameter<int>("pwm_max", 2000);
    declare_parameter<int>("throttle_pwm_min", 1500);
    declare_parameter<int>("throttle_pwm_max", 2000);
    declare_parameter<int>("steering_pwm_min", 1000);
    declare_parameter<int>("steering_pwm_max", 2000);

    declare_parameter<bool>("mavlink_enable", true);
    declare_parameter<std::string>("mavlink_bind_ip", "0.0.0.0");
    declare_parameter<int>("mavlink_bind_port", 14540);
    declare_parameter<int>("mavlink_source_system", 245);
    declare_parameter<int>("mavlink_source_component", 190);
    declare_parameter<int>("mavlink_target_system", 1);
    declare_parameter<int>("mavlink_target_component", 1);
    declare_parameter<bool>("mavlink_auto_manual_mode", true);
    declare_parameter<bool>("mavlink_auto_arm", true);
    declare_parameter<bool>("mavlink_force_arm", true);
    declare_parameter<bool>("mavlink_disarm_on_shutdown", true);

    get_parameter("solver_cmd_topic", solver_cmd_topic_);
    get_parameter("applied_cmd_topic", applied_cmd_topic_);
    get_parameter("duty_topic", duty_topic_);
    get_parameter("pwm_topic", pwm_topic_);
    get_parameter("virtual_cmd_topic", virtual_cmd_topic_);
    get_parameter("loop_rate_hz", loop_rate_hz_);
    get_parameter("input_timeout_sec", input_timeout_sec_);
    get_parameter("hold_last_valid_sec", hold_last_valid_sec_);
    get_parameter("send_neutral_on_invalid", send_neutral_on_invalid_);
    get_parameter("publish_applied_cmd_when_invalid", publish_applied_cmd_when_invalid_);
    get_parameter("enable_output_filter", enable_output_filter_);
    get_parameter("steer_norm_max", steer_norm_max_);
    get_parameter("steer_norm_rate_max", steer_norm_rate_max_);
    get_parameter("steer_norm_lpf_alpha", steer_norm_lpf_alpha_);
    get_parameter("target_speed_mps", target_speed_mps_);
    get_parameter("target_accel_mps2", target_accel_mps2_);
    get_parameter("speed_kp", speed_kp_);
    get_parameter("speed_ki", speed_ki_);
    get_parameter("throttle_ff", throttle_ff_);
    get_parameter("max_virtual_throttle", max_virtual_throttle_);
    get_parameter("max_virtual_brake", max_virtual_brake_);
    get_parameter("speed_deadband_mps", speed_deadband_mps_);
    get_parameter("speed_integral_limit", speed_integral_limit_);
    get_parameter("throttle_max_norm", throttle_max_norm_);
    get_parameter("steer_sign", steer_sign_);
    get_parameter("publish_pwm", publish_pwm_);
    get_parameter("pwm_min", pwm_min_);
    get_parameter("pwm_center", pwm_center_);
    get_parameter("pwm_max", pwm_max_);
    get_parameter("throttle_pwm_min", throttle_pwm_min_);
    get_parameter("throttle_pwm_max", throttle_pwm_max_);
    get_parameter("steering_pwm_min", steering_pwm_min_);
    get_parameter("steering_pwm_max", steering_pwm_max_);

    get_parameter("mavlink_enable", mavlink_enable_);
    get_parameter("mavlink_bind_ip", mavlink_bind_ip_);
    get_parameter("mavlink_bind_port", mavlink_bind_port_);
    get_parameter("mavlink_source_system", mavlink_source_system_);
    get_parameter("mavlink_source_component", mavlink_source_component_);
    get_parameter("mavlink_target_system", mavlink_target_system_);
    get_parameter("mavlink_target_component", mavlink_target_component_);
    get_parameter("mavlink_auto_manual_mode", mavlink_auto_manual_mode_);
    get_parameter("mavlink_auto_arm", mavlink_auto_arm_);
    get_parameter("mavlink_force_arm", mavlink_force_arm_);
    get_parameter("mavlink_disarm_on_shutdown", mavlink_disarm_on_shutdown_);
    target_speed_profile_mps_ = std::max(0.0, target_speed_mps_);
    target_speed_profile_initialized_ = true;

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      solver_cmd_topic_,
      10,
      std::bind(&LowerDutyNode::commandCallback, this, std::placeholders::_1));
    applied_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(applied_cmd_topic_, 10);
    virtual_cmd_pub_ =
      create_publisher<imac_interfaces::msg::VirtualControlCommand>(virtual_cmd_topic_, 10);
    duty_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(duty_topic_, 10);
    if (publish_pwm_) {
      pwm_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>(pwm_topic_, 10);
    }

    if (mavlink_enable_) {
      mavlinkInitUdp();
    }

    const auto period = std::chrono::duration<double>(1.0 / std::max(1e-6, loop_rate_hz_));
    timer_ = create_wall_timer(period, std::bind(&LowerDutyNode::timerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "LowerDutyNode ready: input=%s duty=%s pwm=%s mavlink=%d throttle_max=%.3f steer_sign=%.2f",
      solver_cmd_topic_.c_str(),
      duty_topic_.c_str(),
      publish_pwm_ ? pwm_topic_.c_str() : "(disabled)",
      mavlink_enable_ ? 1 : 0,
      throttle_max_norm_,
      steer_sign_);
  }

  ~LowerDutyNode() override
  {
    timer_.reset();

    rememberDuty(0.0, 0.0, false);
    if (mavlink_enable_) {
      mavlinkSendActuator(0.0, 0.0);
      if (mavlink_disarm_on_shutdown_) {
        mavlinkSendArmCommand(false, false);
      }
    }
    mavlinkClose();
  }

private:
  struct VirtualDriveCmd
  {
    double steer{0.0};
    double throttle{0.0};
    double brake{0.0};
  };

  void commandCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_rx_time_ = now();
    applySolverCommand(*msg, "command");
  }

  void timerCallback()
  {
    if (mavlink_enable_) {
      mavlinkPoll();
      mavlinkSendManualNeutral();
      mavlinkManageModeAndArm();
    }

    const rclcpp::Time t = now();
    const bool stale =
      last_cmd_rx_time_.nanoseconds() == 0 ||
      (t - last_cmd_rx_time_).seconds() > input_timeout_sec_;
    if (stale) {
      geometry_msgs::msg::Twist msg;
      applySolverCommand(msg, "stale command input");
    }

    double throttle = 0.0;
    double steering = 0.0;
    bool valid = false;
    {
      std::lock_guard<std::mutex> lk(duty_mtx_);
      throttle = last_throttle_norm_;
      steering = last_steering_norm_;
      valid = last_valid_;
    }

    publishDuty(throttle, steering, valid);
    if (mavlink_enable_) {
      mavlinkSendActuator(throttle, steering);
    }
  }

  void applySolverCommand(const geometry_msgs::msg::Twist & solver_msg, const char * source)
  {
    const bool valid_cmd = solver_msg.linear.z > 0.5;
    const rclcpp::Time t = now();

    geometry_msgs::msg::Twist applied_msg = solver_msg;
    if (valid_cmd) {
      applied_msg.linear.y = computeFilteredSteerNorm(solver_msg.linear.y, t);
    } else {
      applied_msg.linear.x = 0.0;
      applied_msg.linear.y = 0.0;
      applied_msg.linear.z = 0.0;
      applied_msg.angular.z = 0.0;
    }

    if (applied_cmd_pub_ && (valid_cmd || publish_applied_cmd_when_invalid_)) {
      applied_cmd_pub_->publish(applied_msg);
    }

    if (valid_cmd && applied_msg.linear.x < -1e-6) {
      publishVirtualStopCommand(solver_msg.angular.x);
    } else {
      publishVirtualControlCommand(applied_msg.linear.y, valid_cmd, solver_msg.angular.x);
    }

    applyDutyCommand(applied_msg.linear.x, applied_msg.linear.y, valid_cmd, source);
  }

  void applyDutyCommand(double dv_mps, double steer_norm, bool valid_cmd, const char * source)
  {
    const rclcpp::Time t = now();

    if (valid_cmd) {
      const double throttle_norm = computeThrottleNorm(dv_mps);
      const double steering_norm = -clampd(steer_sign_ * steer_norm, -1.0, 1.0);
      last_valid_steering_norm_ = steering_norm;
      last_valid_cmd_time_ = t;
      rememberDuty(throttle_norm, steering_norm, true);
      return;
    }

    const bool have_recent_valid =
      last_valid_cmd_time_.nanoseconds() > 0 &&
      (t - last_valid_cmd_time_).seconds() < hold_last_valid_sec_;

    if (have_recent_valid) {
      rememberDuty(0.0, last_valid_steering_norm_, false);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "%s invalid: holding last steering %.3f for %.2f sec",
        source,
        last_valid_steering_norm_,
        hold_last_valid_sec_);
      return;
    }

    if (send_neutral_on_invalid_) {
      rememberDuty(0.0, 0.0, false);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "%s invalid: sending neutral throttle/steering",
        source);
    }
  }

  double computeThrottleNorm(double dv_mps) const
  {
    if (dv_mps <= 0.0) {
      return 0.0;
    }
    return clampd(throttle_max_norm_, 0.0, 1.0);
  }

  double computeFilteredSteerNorm(double steer_norm, const rclcpp::Time & t)
  {
    const double steer_max = std::max(0.0, std::abs(steer_norm_max_));
    const double raw_steer = clampd(steer_norm, -steer_max, steer_max);

    if (!enable_output_filter_) {
      rememberFilteredSteer(raw_steer, t);
      return raw_steer;
    }

    double dt = 1.0 / std::max(1e-6, loop_rate_hz_);
    if (have_prev_applied_cmd_ && last_applied_cmd_time_.nanoseconds() > 0) {
      const double measured_dt = (t - last_applied_cmd_time_).seconds();
      if (measured_dt > 1e-4 && measured_dt < 1.0) {
        dt = measured_dt;
      }
    }

    const double prev = have_prev_applied_cmd_ ? prev_applied_steer_norm_ : raw_steer;
    const double max_step = std::max(0.0, steer_norm_rate_max_) * dt;
    const double rate_limited = clampd(raw_steer, prev - max_step, prev + max_step);
    const double alpha = clampd(steer_norm_lpf_alpha_, 0.0, 1.0);
    const double filtered =
      clampd(alpha * rate_limited + (1.0 - alpha) * prev, -steer_max, steer_max);
    rememberFilteredSteer(filtered, t);
    return filtered;
  }

  void rememberFilteredSteer(double steer_norm, const rclcpp::Time & t)
  {
    prev_applied_steer_norm_ = steer_norm;
    last_applied_cmd_time_ = t;
    have_prev_applied_cmd_ = true;
  }

  void updateTargetSpeedProfile(double dt)
  {
    if (!target_speed_profile_initialized_) {
      target_speed_profile_mps_ = std::max(0.0, target_speed_mps_);
      target_speed_profile_initialized_ = true;
    }

    if (std::abs(target_accel_mps2_) > 1e-9) {
      target_speed_profile_mps_ += target_accel_mps2_ * dt;
    } else {
      target_speed_profile_mps_ = target_speed_mps_;
    }

    target_speed_profile_mps_ = std::max(0.0, target_speed_profile_mps_);
  }

  VirtualDriveCmd computeVirtualDriveCmd(
    double steer_norm,
    bool valid_cmd,
    double current_speed_mps,
    double dt)
  {
    VirtualDriveCmd out;

    if (!valid_cmd) {
      speed_integral_ = 0.0;
      return out;
    }

    out.steer = clampd(steer_norm, -1.0, 1.0);

    const double v_now = std::max(0.0, current_speed_mps);
    const double speed_error = target_speed_profile_mps_ - v_now;

    if (std::abs(speed_error) <= speed_deadband_mps_) {
      out.throttle = clampd(throttle_ff_, 0.0, max_virtual_throttle_);
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
    } else {
      out.brake = clampd(-u, 0.0, max_virtual_brake_);
    }

    return out;
  }

  void publishVirtualControlCommand(
    double steer_norm,
    bool valid_cmd,
    double current_speed_mps)
  {
    if (!virtual_cmd_pub_) {
      return;
    }

    const double dt = 1.0 / std::max(1e-6, loop_rate_hz_);
    if (valid_cmd) {
      updateTargetSpeedProfile(dt);
    }

    const VirtualDriveCmd cmd =
      computeVirtualDriveCmd(steer_norm, valid_cmd, current_speed_mps, dt);

    imac_interfaces::msg::VirtualControlCommand msg;
    msg.steer = static_cast<float>(cmd.steer);
    msg.throttle = static_cast<float>(cmd.throttle);
    msg.brake = static_cast<float>(cmd.brake);
    virtual_cmd_pub_->publish(msg);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      "VirtualControlCommand: topic=%s v_ref=%.3f v_now=%.3f steer=%.3f throttle=%.3f brake=%.3f valid=%d",
      virtual_cmd_topic_.c_str(),
      target_speed_profile_mps_,
      current_speed_mps,
      cmd.steer,
      cmd.throttle,
      cmd.brake,
      valid_cmd ? 1 : 0);
  }

  void publishVirtualStopCommand(double current_speed_mps)
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
      get_logger(), *get_clock(), 500,
      "VirtualControlCommand STOP: topic=%s v_now=%.3f brake=%.3f",
      virtual_cmd_topic_.c_str(),
      current_speed_mps,
      static_cast<double>(msg.brake));
  }

  void rememberDuty(double throttle_norm, double steering_norm, bool valid)
  {
    std::lock_guard<std::mutex> lk(duty_mtx_);
    last_throttle_norm_ = clampd(throttle_norm, -1.0, 1.0);
    last_steering_norm_ = clampd(steering_norm, -1.0, 1.0);
    last_valid_ = valid;
  }

  void publishDuty(double throttle_norm, double steering_norm, bool valid)
  {
    if (duty_pub_) {
      std_msgs::msg::Float64MultiArray msg;
      msg.data = {throttle_norm, steering_norm, valid ? 1.0 : 0.0};
      duty_pub_->publish(msg);
    }

    if (pwm_pub_) {
      std_msgs::msg::Int32MultiArray msg;
      msg.data.resize(2);
      msg.data[0] = normToPwm(steering_norm, steering_pwm_min_, steering_pwm_max_);
      msg.data[1] = normToPwm(throttle_norm, throttle_pwm_min_, throttle_pwm_max_);
      pwm_pub_->publish(msg);
    }
  }

  int normToPwm(double norm, int lo, int hi) const
  {
    const int min_pwm = std::max(pwm_min_, std::min(lo, hi));
    const int max_pwm = std::min(pwm_max_, std::max(lo, hi));
    const int span = std::max(0, max_pwm - pwm_center_);
    const int pwm = static_cast<int>(std::lround(
      static_cast<double>(pwm_center_) + clampd(norm, -1.0, 1.0) * static_cast<double>(span)));
    return clampi(pwm, min_pwm, max_pwm);
  }

  void mavlinkInitUdp()
  {
    std::lock_guard<std::mutex> lk(mavlink_mtx_);

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
        mavlink_bind_ip_.c_str(),
        mavlink_bind_port_,
        std::strerror(errno));
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
    mavlink_last_mode_request_time_ = now();
    mavlink_last_arm_request_time_ = now();

    RCLCPP_INFO(
      get_logger(),
      "MAVLink UDP bound at %s:%d. Waiting for MAVProxy peer. Use MAVProxy --out=127.0.0.1:%d",
      mavlink_bind_ip_.c_str(),
      mavlink_bind_port_,
      mavlink_bind_port_);
  }

  void mavlinkClose()
  {
    std::lock_guard<std::mutex> lk(mavlink_mtx_);
    if (mavlink_socket_ >= 0) {
      ::close(mavlink_socket_);
      mavlink_socket_ = -1;
    }
    mavlink_peer_known_ = false;
  }

  void mavlinkPoll()
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
          handleMavlinkMessage(msg, src_addr);
        }
      }
    }
  }

  void handleMavlinkMessage(const mavlink_message_t & msg, const sockaddr_in & src_addr)
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

      mavlink_px4_manual_ = px4_main_mode == kPx4CustomMainModeManual;
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

  bool mavlinkSendMessage(const mavlink_message_t & msg)
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
        sent,
        static_cast<unsigned>(len),
        std::strerror(errno));
      return false;
    }

    return true;
  }

  void mavlinkSendManualNeutral()
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

  void mavlinkSendSetManualMode()
  {
    if (!mavlink_enable_) {
      return;
    }

    const uint32_t custom_mode = px4CustomMode(kPx4CustomMainModeManual);

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
      custom_mode,
      ok ? 1 : 0);
  }

  void mavlinkSendArmCommand(bool arm, bool force)
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

  void mavlinkManageModeAndArm()
  {
    if (!mavlink_enable_) {
      return;
    }

    const rclcpp::Time t = now();
    bool send_manual = false;
    bool send_arm = false;

    {
      std::lock_guard<std::mutex> lk(mavlink_mtx_);
      if (!mavlink_peer_known_) {
        return;
      }

      if (mavlink_auto_manual_mode_ && !mavlink_px4_manual_) {
        if (mavlink_last_mode_request_time_.nanoseconds() == 0 ||
          (t - mavlink_last_mode_request_time_).seconds() > 1.0)
        {
          send_manual = true;
          mavlink_last_mode_request_time_ = t;
        }
      }

      if (mavlink_auto_arm_ && mavlink_px4_manual_ && !mavlink_px4_armed_) {
        if (mavlink_last_arm_request_time_.nanoseconds() == 0 ||
          (t - mavlink_last_arm_request_time_).seconds() > 1.0)
        {
          send_arm = true;
          mavlink_last_arm_request_time_ = t;
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

  void mavlinkSendActuator(double throttle_norm, double steering_norm)
  {
    if (!mavlink_enable_) {
      return;
    }

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

  std::string solver_cmd_topic_{"/control/solver_cmd"};
  std::string applied_cmd_topic_{"/control/applied_cmd"};
  std::string duty_topic_{"/control/duty_cmd"};
  std::string pwm_topic_{"/cmd_pwm"};
  std::string virtual_cmd_topic_{"/cmd_control"};
  double loop_rate_hz_{20.0};
  double input_timeout_sec_{0.30};
  double hold_last_valid_sec_{0.30};
  bool send_neutral_on_invalid_{true};
  bool publish_applied_cmd_when_invalid_{true};
  bool enable_output_filter_{true};
  double steer_norm_max_{1.0};
  double steer_norm_rate_max_{2.0};
  double steer_norm_lpf_alpha_{0.2};
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
  double throttle_max_norm_{0.17};
  double steer_sign_{1.0};
  bool publish_pwm_{true};
  int pwm_min_{1000};
  int pwm_center_{1500};
  int pwm_max_{2000};
  int throttle_pwm_min_{1500};
  int throttle_pwm_max_{2000};
  int steering_pwm_min_{1000};
  int steering_pwm_max_{2000};

  bool mavlink_enable_{false};
  std::string mavlink_bind_ip_{"0.0.0.0"};
  int mavlink_bind_port_{14540};
  int mavlink_source_system_{245};
  int mavlink_source_component_{190};
  int mavlink_target_system_{1};
  int mavlink_target_component_{1};
  bool mavlink_auto_manual_mode_{true};
  bool mavlink_auto_arm_{false};
  bool mavlink_force_arm_{false};
  bool mavlink_disarm_on_shutdown_{true};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr applied_cmd_pub_;
  rclcpp::Publisher<imac_interfaces::msg::VirtualControlCommand>::SharedPtr virtual_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr duty_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pwm_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time last_cmd_rx_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_valid_cmd_time_{0, 0, RCL_ROS_TIME};
  double last_valid_steering_norm_{0.0};
  double last_throttle_norm_{0.0};
  double last_steering_norm_{0.0};
  bool last_valid_{false};
  std::mutex duty_mtx_;
  double prev_applied_steer_norm_{0.0};
  bool have_prev_applied_cmd_{false};
  rclcpp::Time last_applied_cmd_time_{0, 0, RCL_ROS_TIME};

  int mavlink_socket_{-1};
  bool mavlink_peer_known_{false};
  sockaddr_in mavlink_peer_addr_{};
  mavlink_status_t mavlink_rx_status_{};
  bool mavlink_px4_manual_{false};
  bool mavlink_px4_armed_{false};
  rclcpp::Time mavlink_last_mode_request_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time mavlink_last_arm_request_time_{0, 0, RCL_ROS_TIME};
  std::mutex mavlink_mtx_;
};

}  // namespace imac_ctrl

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<imac_ctrl::LowerDutyNode>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
