#include "virtual_control/mc_test.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <exception>
#include <functional>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
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

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_srvs/srv/trigger.hpp>

#ifndef MAV_CMD_DO_SET_SERVO
#define MAV_CMD_DO_SET_SERVO 183
#endif

#ifndef MAV_CMD_DO_SET_ACTUATOR
#define MAV_CMD_DO_SET_ACTUATOR 187
#endif

namespace mocap_straight_pwm
{

using Trigger = std_srvs::srv::Trigger;
using namespace std::chrono_literals;

static constexpr uint8_t PX4_CUSTOM_MAIN_MODE_MANUAL = 1;
static constexpr double kPi = 3.14159265358979323846;

static inline double clampd(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

static inline int clampi(int v, int lo, int hi)
{
  return std::max(lo, std::min(hi, v));
}

static inline double wrapPi(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

static inline double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

static inline uint32_t px4CustomMode(uint8_t main_mode, uint8_t sub_mode = 0)
{
  return (static_cast<uint32_t>(sub_mode) << 24) |
         (static_cast<uint32_t>(main_mode) << 16);
}

struct MavlinkUdpBridge
{
  bool enable{true};
  std::string bind_ip{"0.0.0.0"};
  int bind_port{14540};

  int source_system{245};
  int source_component{190};
  int target_system{1};
  int target_component{1};

  int sock{-1};
  bool peer_known{false};
  sockaddr_in peer_addr{};
  mavlink_status_t rx_status{};

  bool px4_manual{false};
  bool px4_armed{false};
  uint8_t last_base_mode{0};
  uint32_t last_custom_mode{0};

  rclcpp::Time last_rx_time;
  rclcpp::Time last_mode_request_time;
  rclcpp::Time last_arm_request_time;
  std::mutex mtx;

  bool init(rclcpp::Node * node)
  {
    std::lock_guard<std::mutex> lk(mtx);

    if (!enable) {
      return false;
    }
    if (sock >= 0) {
      return true;
    }

    sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
      RCLCPP_ERROR(node->get_logger(), "MAVLink UDP socket creation failed: %s", std::strerror(errno));
      return false;
    }

    int reuse = 1;
    ::setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in local_addr{};
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(static_cast<uint16_t>(bind_port));

    if (bind_ip == "0.0.0.0") {
      local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    } else if (::inet_pton(AF_INET, bind_ip.c_str(), &local_addr.sin_addr) != 1) {
      RCLCPP_ERROR(node->get_logger(), "Invalid mavlink_bind_ip: %s", bind_ip.c_str());
      ::close(sock);
      sock = -1;
      return false;
    }

    if (::bind(sock, reinterpret_cast<sockaddr *>(&local_addr), sizeof(local_addr)) < 0) {
      RCLCPP_ERROR(
        node->get_logger(),
        "MAVLink UDP bind failed on %s:%d: %s",
        bind_ip.c_str(), bind_port, std::strerror(errno));
      ::close(sock);
      sock = -1;
      return false;
    }

    const int flags = ::fcntl(sock, F_GETFL, 0);
    if (flags >= 0) {
      ::fcntl(sock, F_SETFL, flags | O_NONBLOCK);
    }

    peer_known = false;
    std::memset(&peer_addr, 0, sizeof(peer_addr));
    std::memset(&rx_status, 0, sizeof(rx_status));
    px4_manual = false;
    px4_armed = false;
    last_rx_time = node->now();
    last_mode_request_time = node->now();
    last_arm_request_time = node->now();

    RCLCPP_INFO(
      node->get_logger(),
      "MAVLink UDP bound at %s:%d. Example bridge: MAVProxy --master=/dev/ttyACM0,115200 --out=127.0.0.1:%d",
      bind_ip.c_str(), bind_port, bind_port);

    return true;
  }

  void closeSocket()
  {
    std::lock_guard<std::mutex> lk(mtx);
    if (sock >= 0) {
      ::close(sock);
      sock = -1;
    }
    peer_known = false;
  }

  void poll(rclcpp::Node * node)
  {
    std::lock_guard<std::mutex> lk(mtx);

    if (!enable || sock < 0) {
      return;
    }

    std::array<uint8_t, 2048> buffer{};

    while (true) {
      sockaddr_in src_addr{};
      socklen_t src_len = sizeof(src_addr);
      const ssize_t n = ::recvfrom(
        sock,
        buffer.data(),
        buffer.size(),
        0,
        reinterpret_cast<sockaddr *>(&src_addr),
        &src_len);

      if (n < 0) {
        if (errno == EWOULDBLOCK || errno == EAGAIN) {
          break;
        }
        RCLCPP_WARN_THROTTLE(
          node->get_logger(), *node->get_clock(), 2000,
          "MAVLink recvfrom failed: %s", std::strerror(errno));
        break;
      }

      for (ssize_t i = 0; i < n; ++i) {
        mavlink_message_t msg{};
        if (mavlink_parse_char(MAVLINK_COMM_0, buffer[static_cast<size_t>(i)], &msg, &rx_status)) {
          if (!peer_known) {
            peer_addr = src_addr;
            peer_known = true;

            char ip_str[INET_ADDRSTRLEN] = {};
            ::inet_ntop(AF_INET, &src_addr.sin_addr, ip_str, sizeof(ip_str));
            RCLCPP_INFO(
              node->get_logger(),
              "MAVLink peer detected: %s:%d",
              ip_str,
              ntohs(src_addr.sin_port));
          }

          last_rx_time = node->now();

          if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
            mavlink_heartbeat_t hb{};
            mavlink_msg_heartbeat_decode(&msg, &hb);

            target_system = msg.sysid;
            target_component = 1;
            last_base_mode = hb.base_mode;
            last_custom_mode = hb.custom_mode;

            const uint8_t px4_main_mode = static_cast<uint8_t>((hb.custom_mode >> 16) & 0xFF);
            px4_manual = px4_main_mode == PX4_CUSTOM_MAIN_MODE_MANUAL;
            px4_armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
          } else if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
            mavlink_command_ack_t ack{};
            mavlink_msg_command_ack_decode(&msg, &ack);
            RCLCPP_INFO_THROTTLE(
              node->get_logger(), *node->get_clock(), 1000,
              "MAVLink COMMAND_ACK: command=%u result=%u",
              static_cast<unsigned>(ack.command),
              static_cast<unsigned>(ack.result));
          } else if (msg.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
            mavlink_statustext_t st{};
            mavlink_msg_statustext_decode(&msg, &st);
            char text[51] = {};
            std::memcpy(text, st.text, 50);
            RCLCPP_WARN_THROTTLE(
              node->get_logger(), *node->get_clock(), 1000,
              "PX4 STATUSTEXT: severity=%u text=%s",
              static_cast<unsigned>(st.severity), text);
          }
        }
      }
    }
  }

  bool sendMessage(rclcpp::Node * node, const mavlink_message_t & msg)
  {
    std::lock_guard<std::mutex> lk(mtx);

    if (!enable || sock < 0) {
      return false;
    }
    if (!peer_known) {
      RCLCPP_WARN_THROTTLE(
        node->get_logger(), *node->get_clock(), 2000,
        "MAVLink peer is not known yet. Check MAVProxy --out=127.0.0.1:%d",
        bind_port);
      return false;
    }

    std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> txbuf{};
    const uint16_t len = mavlink_msg_to_send_buffer(txbuf.data(), &msg);
    const ssize_t sent = ::sendto(
      sock,
      txbuf.data(),
      len,
      0,
      reinterpret_cast<sockaddr *>(&peer_addr),
      sizeof(peer_addr));

    return sent == static_cast<ssize_t>(len);
  }

  bool sendSetManualMode(rclcpp::Node * node)
  {
    const uint32_t custom_mode = px4CustomMode(PX4_CUSTOM_MAIN_MODE_MANUAL);

    mavlink_message_t msg{};
    mavlink_msg_set_mode_pack(
      static_cast<uint8_t>(source_system),
      static_cast<uint8_t>(source_component),
      &msg,
      static_cast<uint8_t>(target_system),
      MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
      custom_mode);

    const bool ok = sendMessage(node, msg);
    RCLCPP_INFO_THROTTLE(
      node->get_logger(), *node->get_clock(), 1000,
      "MAVLink SET_MODE MANUAL sent: custom_mode=%u ok=%d",
      custom_mode, ok ? 1 : 0);
    return ok;
  }

  bool sendArmCommand(rclcpp::Node * node, bool arm, bool force)
  {
    const float force_code = force ? 21196.0f : 0.0f;

    mavlink_message_t msg{};
    mavlink_msg_command_long_pack(
      static_cast<uint8_t>(source_system),
      static_cast<uint8_t>(source_component),
      &msg,
      static_cast<uint8_t>(target_system),
      static_cast<uint8_t>(target_component),
      MAV_CMD_COMPONENT_ARM_DISARM,
      0,
      arm ? 1.0f : 0.0f,
      force_code,
      0.0f,
      0.0f,
      0.0f,
      0.0f,
      0.0f);

    const bool ok = sendMessage(node, msg);
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *node->get_clock(), 1000,
      "MAVLink %s sent: force=%d ok=%d",
      arm ? "ARM" : "DISARM", force ? 1 : 0, ok ? 1 : 0);
    return ok;
  }

  bool sendDoSetServo(rclcpp::Node * node, int servo_number, int pwm_us)
  {
    mavlink_message_t msg{};
    mavlink_msg_command_long_pack(
      static_cast<uint8_t>(source_system),
      static_cast<uint8_t>(source_component),
      &msg,
      static_cast<uint8_t>(target_system),
      static_cast<uint8_t>(target_component),
      MAV_CMD_DO_SET_SERVO,
      0,
      static_cast<float>(servo_number),
      static_cast<float>(pwm_us),
      0.0f,
      0.0f,
      0.0f,
      0.0f,
      0.0f);

    return sendMessage(node, msg);
  }

  bool sendDoSetActuator(
    rclcpp::Node * node,
    double throttle_norm,
    double steering_norm,
    int throttle_actuator_index,
    int steering_actuator_index,
    int actuator_set_index)
  {
    std::array<float, 6> params{};
    const float nan_f = std::numeric_limits<float>::quiet_NaN();
    params.fill(nan_f);

    if (throttle_actuator_index >= 1 && throttle_actuator_index <= 6) {
      params[static_cast<size_t>(throttle_actuator_index - 1)] = static_cast<float>(clampd(throttle_norm, -1.0, 1.0));
    }
    if (steering_actuator_index >= 1 && steering_actuator_index <= 6) {
      params[static_cast<size_t>(steering_actuator_index - 1)] = static_cast<float>(clampd(steering_norm, -1.0, 1.0));
    }

    mavlink_message_t msg{};
    mavlink_msg_command_long_pack(
      static_cast<uint8_t>(source_system),
      static_cast<uint8_t>(source_component),
      &msg,
      static_cast<uint8_t>(target_system),
      static_cast<uint8_t>(target_component),
      MAV_CMD_DO_SET_ACTUATOR,
      0,
      params[0],
      params[1],
      params[2],
      params[3],
      params[4],
      params[5],
      static_cast<float>(actuator_set_index));

    return sendMessage(node, msg);
  }

  void manageModeAndArm(rclcpp::Node * node, bool auto_manual_mode, bool auto_arm, bool force_arm)
  {
    if (!enable || !peer_known) {
      return;
    }

    const rclcpp::Time now = node->now();

    if (auto_manual_mode && !px4_manual && (now - last_mode_request_time).seconds() > 1.0) {
      sendSetManualMode(node);
      last_mode_request_time = now;
    }

    if (auto_arm && px4_manual && !px4_armed && (now - last_arm_request_time).seconds() > 1.0) {
      sendArmCommand(node, true, force_arm);
      last_arm_request_time = now;
    }
  }
};

struct PathPoint
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double s{0.0};
  double v_ref{0.0};
};

struct PathProjection
{
  bool valid{false};
  size_t segment_index{0};
  double t{0.0};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double s{0.0};
  double v_ref{0.0};
  double distance{std::numeric_limits<double>::infinity()};
};

class StraightMocapPwmNode final : public rclcpp::Node
{
public:
  StraightMocapPwmNode()
  : Node(kStraightMocapPwmNodeName)
  {
    declareParameters();
    loadParameters();

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_,
      10,
      std::bind(&StraightMocapPwmNode::poseCallback, this, std::placeholders::_1));

    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_debug_topic_, rclcpp::QoS(1).transient_local());

    createServices();

    if (mavlink_.enable) {
      mavlink_.init(this);
    }

    initializeReferencePath();
    publishReferencePath();

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, loop_rate_hz_));
    loop_timer_ = create_wall_timer(period, std::bind(&StraightMocapPwmNode::controlLoop, this));

    path_timer_ = create_wall_timer(1s, std::bind(&StraightMocapPwmNode::publishReferencePath, this));

    RCLCPP_INFO(
      get_logger(),
      "straight_mocap_pwm_node ready: pose_topic=%s, path_mode=%s, waypoints=%zu, target_speed=%.2f m/s, output_mode=%s",
      pose_topic_.c_str(),
      using_csv_path_ ? "csv" : "internal_straight",
      ref_path_.size(),
      target_speed_mps_,
      mavlink_output_mode_.c_str());
  }

  ~StraightMocapPwmNode() override
  {
    if (send_neutral_when_stopped_) {
      sendNeutral();
    }
    if (disarm_on_shutdown_) {
      mavlink_.sendArmCommand(this, false, false);
    }
    mavlink_.closeSocket();
  }

private:
  void declareParameters()
  {
    declare_parameter<std::string>("pose_topic", "/pose");
    declare_parameter<std::string>("service_prefix", "/straight_pwm");
    declare_parameter<std::string>("path_debug_topic", "/straight_pwm/path");
    declare_parameter<std::string>("frame_id", "map");

    declare_parameter<double>("path_start_x", 0.20);
    declare_parameter<double>("path_start_y", 1.00);
    declare_parameter<double>("path_end_x", 1.80);
    declare_parameter<double>("path_end_y", 1.00);
    declare_parameter<std::string>("path_csv_file", "");
    declare_parameter<double>("goal_tolerance_m", 0.08);
    declare_parameter<double>("slow_down_distance_m", 0.35);

    declare_parameter<double>("loop_rate_hz", 20.0);
    declare_parameter<double>("pose_timeout_sec", 0.35);
    declare_parameter<double>("wheelbase_m", 0.33);
    declare_parameter<double>("target_speed_mps", 0.25);
    declare_parameter<bool>("use_csv_speed_ref", true);
    declare_parameter<double>("min_command_speed_mps", 0.05);
    declare_parameter<double>("stanley_k", 0.8);
    declare_parameter<double>("stanley_softening_mps", 0.10);
    declare_parameter<double>("max_steer_left_rad", 0.52);
    declare_parameter<double>("max_steer_right_rad", -0.52);

    declare_parameter<int>("throttle_stop_pwm", 1500);
    declare_parameter<int>("throttle_drive_min_pwm", 1550);
    declare_parameter<int>("throttle_drive_max_pwm", 1600);
    declare_parameter<double>("throttle_speed_at_max_pwm_mps", 0.35);

    declare_parameter<int>("steering_center_pwm", 1500);
    declare_parameter<int>("steering_pwm_span", 400);
    declare_parameter<int>("steering_pwm_min", 1000);
    declare_parameter<int>("steering_pwm_max", 2000);
    declare_parameter<double>("steering_pwm_sign", -1.0);

    declare_parameter<bool>("mavlink_enable", true);
    declare_parameter<std::string>("mavlink_bind_ip", "0.0.0.0");
    declare_parameter<int>("mavlink_bind_port", 14540);
    declare_parameter<int>("mavlink_source_system", 245);
    declare_parameter<int>("mavlink_source_component", 190);
    declare_parameter<int>("mavlink_target_system", 1);
    declare_parameter<int>("mavlink_target_component", 1);
    declare_parameter<std::string>("mavlink_output_mode", "do_set_servo");
    declare_parameter<int>("steering_servo_number", 1);
    declare_parameter<int>("throttle_servo_number", 2);
    declare_parameter<int>("throttle_actuator_index", 1);
    declare_parameter<int>("steering_actuator_index", 2);
    declare_parameter<int>("actuator_set_index", 0);
    declare_parameter<int>("actuator_pwm_min", 1000);
    declare_parameter<int>("actuator_pwm_max", 2000);

    declare_parameter<bool>("drive_enabled_on_start", false);
    declare_parameter<bool>("require_manual_mode", false);
    declare_parameter<bool>("require_armed", false);
    declare_parameter<bool>("send_neutral_when_stopped", true);
    declare_parameter<bool>("auto_manual_mode", false);
    declare_parameter<bool>("auto_arm", false);
    declare_parameter<bool>("force_arm", false);
    declare_parameter<bool>("disarm_on_shutdown", false);
  }

  void loadParameters()
  {
    pose_topic_ = get_parameter("pose_topic").as_string();
    service_prefix_ = get_parameter("service_prefix").as_string();
    path_debug_topic_ = get_parameter("path_debug_topic").as_string();
    frame_id_ = get_parameter("frame_id").as_string();

    path_start_x_ = get_parameter("path_start_x").as_double();
    path_start_y_ = get_parameter("path_start_y").as_double();
    path_end_x_ = get_parameter("path_end_x").as_double();
    path_end_y_ = get_parameter("path_end_y").as_double();
    path_csv_file_ = get_parameter("path_csv_file").as_string();
    goal_tolerance_m_ = get_parameter("goal_tolerance_m").as_double();
    slow_down_distance_m_ = get_parameter("slow_down_distance_m").as_double();

    loop_rate_hz_ = get_parameter("loop_rate_hz").as_double();
    pose_timeout_sec_ = get_parameter("pose_timeout_sec").as_double();
    wheelbase_m_ = get_parameter("wheelbase_m").as_double();
    target_speed_mps_ = get_parameter("target_speed_mps").as_double();
    use_csv_speed_ref_ = get_parameter("use_csv_speed_ref").as_bool();
    min_command_speed_mps_ = get_parameter("min_command_speed_mps").as_double();
    stanley_k_ = get_parameter("stanley_k").as_double();
    stanley_softening_mps_ = get_parameter("stanley_softening_mps").as_double();
    max_steer_left_rad_ = get_parameter("max_steer_left_rad").as_double();
    max_steer_right_rad_ = get_parameter("max_steer_right_rad").as_double();

    throttle_stop_pwm_ = static_cast<int>(get_parameter("throttle_stop_pwm").as_int());
    throttle_drive_min_pwm_ = static_cast<int>(get_parameter("throttle_drive_min_pwm").as_int());
    throttle_drive_max_pwm_ = static_cast<int>(get_parameter("throttle_drive_max_pwm").as_int());
    throttle_speed_at_max_pwm_mps_ = get_parameter("throttle_speed_at_max_pwm_mps").as_double();

    steering_center_pwm_ = static_cast<int>(get_parameter("steering_center_pwm").as_int());
    steering_pwm_span_ = static_cast<int>(get_parameter("steering_pwm_span").as_int());
    steering_pwm_min_ = static_cast<int>(get_parameter("steering_pwm_min").as_int());
    steering_pwm_max_ = static_cast<int>(get_parameter("steering_pwm_max").as_int());
    steering_pwm_sign_ = get_parameter("steering_pwm_sign").as_double();

    mavlink_.enable = get_parameter("mavlink_enable").as_bool();
    mavlink_.bind_ip = get_parameter("mavlink_bind_ip").as_string();
    mavlink_.bind_port = static_cast<int>(get_parameter("mavlink_bind_port").as_int());
    mavlink_.source_system = static_cast<int>(get_parameter("mavlink_source_system").as_int());
    mavlink_.source_component = static_cast<int>(get_parameter("mavlink_source_component").as_int());
    mavlink_.target_system = static_cast<int>(get_parameter("mavlink_target_system").as_int());
    mavlink_.target_component = static_cast<int>(get_parameter("mavlink_target_component").as_int());

    mavlink_output_mode_ = get_parameter("mavlink_output_mode").as_string();
    steering_servo_number_ = static_cast<int>(get_parameter("steering_servo_number").as_int());
    throttle_servo_number_ = static_cast<int>(get_parameter("throttle_servo_number").as_int());
    throttle_actuator_index_ = static_cast<int>(get_parameter("throttle_actuator_index").as_int());
    steering_actuator_index_ = static_cast<int>(get_parameter("steering_actuator_index").as_int());
    actuator_set_index_ = static_cast<int>(get_parameter("actuator_set_index").as_int());
    actuator_pwm_min_ = static_cast<int>(get_parameter("actuator_pwm_min").as_int());
    actuator_pwm_max_ = static_cast<int>(get_parameter("actuator_pwm_max").as_int());

    drive_enabled_ = get_parameter("drive_enabled_on_start").as_bool();
    require_manual_mode_ = get_parameter("require_manual_mode").as_bool();
    require_armed_ = get_parameter("require_armed").as_bool();
    send_neutral_when_stopped_ = get_parameter("send_neutral_when_stopped").as_bool();
    auto_manual_mode_ = get_parameter("auto_manual_mode").as_bool();
    auto_arm_ = get_parameter("auto_arm").as_bool();
    force_arm_ = get_parameter("force_arm").as_bool();
    disarm_on_shutdown_ = get_parameter("disarm_on_shutdown").as_bool();

    service_prefix_ = trimTrailingSlash(service_prefix_);
  }

  static std::string trimTrailingSlash(const std::string & in)
  {
    if (in.size() > 1 && in.back() == '/') {
      return in.substr(0, in.size() - 1);
    }
    return in;
  }

  static std::string trimWhitespace(const std::string & in)
  {
    const auto first = in.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
      return "";
    }
    const auto last = in.find_last_not_of(" \t\r\n");
    return in.substr(first, last - first + 1);
  }

  static std::vector<std::string> splitCsvLine(const std::string & line)
  {
    std::vector<std::string> tokens;
    std::stringstream ss(line);
    std::string token;
    while (std::getline(ss, token, ',')) {
      tokens.push_back(trimWhitespace(token));
    }
    return tokens;
  }

  static bool parseFiniteDouble(const std::vector<std::string> & tokens, size_t index, double & value)
  {
    if (index >= tokens.size() || tokens[index].empty()) {
      return false;
    }

    try {
      size_t parsed = 0;
      value = std::stod(tokens[index], &parsed);
      return parsed == tokens[index].size() && std::isfinite(value);
    } catch (const std::exception &) {
      return false;
    }
  }

  bool loadPathCsv(const std::string & csv_path)
  {
    std::ifstream file(csv_path);
    if (!file.is_open()) {
      RCLCPP_ERROR(get_logger(), "failed to open path CSV: %s", csv_path.c_str());
      return false;
    }

    std::string line;
    if (!std::getline(file, line)) {
      RCLCPP_ERROR(get_logger(), "path CSV is empty: %s", csv_path.c_str());
      return false;
    }

    std::vector<PathPoint> loaded_path;
    size_t row_number = 1;
    while (std::getline(file, line)) {
      ++row_number;
      if (trimWhitespace(line).empty()) {
        continue;
      }

      const auto tokens = splitCsvLine(line);
      double x = 0.0;
      double y = 0.0;
      double yaw = 0.0;
      double s = 0.0;
      double v_ref = 0.0;
      if (
        tokens.size() < 7 ||
        !parseFiniteDouble(tokens, 1, x) ||
        !parseFiniteDouble(tokens, 2, y) ||
        !parseFiniteDouble(tokens, 4, yaw) ||
        !parseFiniteDouble(tokens, 5, s) ||
        !parseFiniteDouble(tokens, 6, v_ref))
      {
        RCLCPP_WARN(
          get_logger(),
          "skipping malformed CSV row %zu in %s",
          row_number,
          csv_path.c_str());
        continue;
      }

      if (v_ref < 0.0) {
        RCLCPP_WARN(
          get_logger(),
          "skipping CSV row %zu with negative v_ref_mps=%.3f",
          row_number,
          v_ref);
        continue;
      }

      loaded_path.push_back(PathPoint{x, y, wrapPi(yaw), s, v_ref});
    }

    if (loaded_path.size() < 2) {
      RCLCPP_ERROR(
        get_logger(),
        "path CSV has fewer than 2 valid waypoints: %s valid=%zu",
        csv_path.c_str(),
        loaded_path.size());
      return false;
    }

    for (size_t i = 1; i < loaded_path.size(); ++i) {
      if (loaded_path[i].s + 1e-9 < loaded_path[i - 1].s) {
        RCLCPP_ERROR(
          get_logger(),
          "path CSV s_m must be nondecreasing: row_index=%zu prev_s=%.6f s=%.6f",
          i,
          loaded_path[i - 1].s,
          loaded_path[i].s);
        return false;
      }
    }

    const double total_length = loaded_path.back().s - loaded_path.front().s;
    if (total_length <= 1e-6) {
      RCLCPP_ERROR(
        get_logger(),
        "path CSV total length is invalid: %s length=%.6f",
        csv_path.c_str(),
        total_length);
      return false;
    }

    ref_path_ = std::move(loaded_path);
    using_csv_path_ = true;
    path_valid_ = true;

    const auto & start = ref_path_.front();
    const auto & end = ref_path_.back();
    RCLCPP_INFO(
      get_logger(),
      "loaded path CSV: file=%s waypoints=%zu start=(%.3f, %.3f, s=%.3f) end=(%.3f, %.3f, s=%.3f) total_length=%.3f m",
      csv_path.c_str(),
      ref_path_.size(),
      start.x,
      start.y,
      start.s,
      end.x,
      end.y,
      end.s,
      total_length);

    return true;
  }

  void buildInternalStraightPath()
  {
    ref_path_.clear();
    using_csv_path_ = false;
    path_valid_ = false;

    const double dx = path_end_x_ - path_start_x_;
    const double dy = path_end_y_ - path_start_y_;
    const double length = std::hypot(dx, dy);
    if (length <= 1e-6) {
      RCLCPP_ERROR(get_logger(), "invalid internal straight path: length is zero");
      return;
    }

    constexpr int waypoint_count = 33;
    const double yaw = std::atan2(dy, dx);
    ref_path_.reserve(waypoint_count);
    for (int i = 0; i < waypoint_count; ++i) {
      const double ratio = static_cast<double>(i) / static_cast<double>(waypoint_count - 1);
      ref_path_.push_back(PathPoint{
        path_start_x_ + ratio * dx,
        path_start_y_ + ratio * dy,
        yaw,
        ratio * length,
        std::max(0.0, target_speed_mps_)});
    }

    path_valid_ = true;
    RCLCPP_INFO(
      get_logger(),
      "generated internal straight path: waypoints=%zu start=(%.3f, %.3f) end=(%.3f, %.3f) total_length=%.3f m",
      ref_path_.size(),
      path_start_x_,
      path_start_y_,
      path_end_x_,
      path_end_y_,
      length);
  }

  void initializeReferencePath()
  {
    ref_path_.clear();
    path_valid_ = false;
    using_csv_path_ = false;

    if (path_csv_file_.empty()) {
      buildInternalStraightPath();
      return;
    }

    if (!loadPathCsv(path_csv_file_)) {
      drive_enabled_ = false;
      sendNeutral();
      RCLCPP_ERROR(
        get_logger(),
        "CSV path is invalid. Drive remains disabled and neutral PWM will be sent.");
    }
  }

  void createServices()
  {
    set_manual_srv_ = create_service<Trigger>(
      service_prefix_ + "/set_manual_mode",
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
        const bool ok = mavlink_.sendSetManualMode(this);
        res->success = ok;
        res->message = ok ? "MANUAL mode command sent" : "failed to send MANUAL mode command";
      });

    arm_srv_ = create_service<Trigger>(
      service_prefix_ + "/arm",
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
        const bool ok = mavlink_.sendArmCommand(this, true, force_arm_);
        res->success = ok;
        res->message = ok ? "ARM command sent" : "failed to send ARM command";
      });

    disarm_srv_ = create_service<Trigger>(
      service_prefix_ + "/disarm",
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
        drive_enabled_ = false;
        sendNeutralBurst();
        const bool ok = mavlink_.sendArmCommand(this, false, false);
        res->success = ok;
        res->message = ok ? "DISARM command sent" : "failed to send DISARM command";
      });

    start_srv_ = create_service<Trigger>(
      service_prefix_ + "/start",
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
        if (!path_valid_) {
          drive_enabled_ = false;
          sendNeutralBurst();
          res->success = false;
          res->message = "path invalid; drive remains disabled";
          return;
        }
        goal_reached_ = false;
        drive_enabled_ = true;
        res->success = true;
        res->message = "straight drive enabled";
      });

    stop_srv_ = create_service<Trigger>(
      service_prefix_ + "/stop",
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
        drive_enabled_ = false;
        sendNeutralBurst();
        res->success = true;
        res->message = "straight drive stopped and neutral PWM sent repeatedly";
      });
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    const rclcpp::Time now = this->now();

    const double x = msg->pose.position.x;
    const double y = msg->pose.position.y;
    const double yaw = yawFromQuaternion(msg->pose.orientation);

    if (have_pose_ && last_pose_time_.nanoseconds() > 0) {
      const double dt = (now - last_pose_time_).seconds();
      if (dt > 1e-4 && dt < 0.5) {
        const double instant_speed = std::hypot(x - cur_x_, y - cur_y_) / dt;
        const double alpha = 0.30;
        cur_speed_mps_ = alpha * instant_speed + (1.0 - alpha) * cur_speed_mps_;
      }
    }

    cur_x_ = x;
    cur_y_ = y;
    cur_z_ = msg->pose.position.z;
    cur_yaw_ = yaw;
    last_pose_time_ = now;
    have_pose_ = true;
  }

  void controlLoop()
  {
    if (mavlink_.enable) {
      mavlink_.poll(this);
      mavlink_.manageModeAndArm(this, auto_manual_mode_, auto_arm_, force_arm_);
    }

    const rclcpp::Time now = this->now();

    if (!drive_enabled_) {
      if (send_neutral_when_stopped_) {
        sendNeutral();
      }
      return;
    }

    if (require_manual_mode_ && !mavlink_.px4_manual) {
      sendNeutral();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "waiting for PX4 MANUAL mode");
      return;
    }

    if (require_armed_ && !mavlink_.px4_armed) {
      sendNeutral();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "waiting for PX4 armed state");
      return;
    }

    if (!have_pose_ || last_pose_time_.nanoseconds() == 0) {
      sendNeutral();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "waiting for mocap PoseStamped");
      return;
    }

    const double pose_age = (now - last_pose_time_).seconds();
    if (pose_timeout_sec_ > 0.0 && pose_age > pose_timeout_sec_) {
      sendNeutral();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "mocap pose timeout: %.3f sec", pose_age);
      return;
    }

    if (!path_valid_ || ref_path_.size() < 2) {
      sendNeutral();
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "reference path is invalid; sending neutral PWM");
      return;
    }

    PathProjection pose_projection;
    if (!computePathProjection(cur_x_, cur_y_, pose_projection)) {
      sendNeutral();
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "failed to project current pose onto path");
      return;
    }

    const double progress_s = pose_projection.s;
    const double remaining_s = ref_path_.back().s - progress_s;

    if (remaining_s <= goal_tolerance_m_) {
      goal_reached_ = true;
      drive_enabled_ = false;
      sendNeutralBurst();
      RCLCPP_INFO(
        get_logger(),
        "final waypoint reached: progress=%.3f / %.3f m. Drive disabled and neutral PWM sent.",
        progress_s,
        ref_path_.back().s);
      return;
    }

    const double front_x = cur_x_ + wheelbase_m_ * std::cos(cur_yaw_);
    const double front_y = cur_y_ + wheelbase_m_ * std::sin(cur_yaw_);
    PathProjection front_projection;
    if (!computePathProjection(front_x, front_y, front_projection)) {
      sendNeutral();
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "failed to project front axle onto path");
      return;
    }

    const double speed_ref_mps =
      (using_csv_path_ && use_csv_speed_ref_) ? pose_projection.v_ref : target_speed_mps_;
    const double desired_speed_mps = computeDesiredSpeed(remaining_s, speed_ref_mps);
    const double target_yaw = front_projection.yaw;
    const double heading_error = wrapPi(target_yaw - cur_yaw_);

    // Stanley cross-track error using front-axle point.
    // For path_yaw=0, vehicle at y>path_y gives cte<0, matching the uploaded bench code sign convention.
    const double cte =
      std::sin(front_projection.yaw) * (front_x - front_projection.x) -
      std::cos(front_projection.yaw) * (front_y - front_projection.y);
    const double control_speed = std::max({cur_speed_mps_, desired_speed_mps, 0.05});

    double steer_cmd_rad = heading_error + std::atan2(
      stanley_k_ * cte,
      control_speed + stanley_softening_mps_);
    steer_cmd_rad = wrapPi(steer_cmd_rad);
    steer_cmd_rad = clampd(steer_cmd_rad, max_steer_right_rad_, max_steer_left_rad_);

    const int steering_pwm = steeringToPwm(steer_cmd_rad);
    const int throttle_pwm = speedToThrottlePwm(desired_speed_mps);

    sendPwm(steering_pwm, throttle_pwm);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "pos=(%.2f, %.2f) nearest_seg=%zu s=%.2f/%.2f rem=%.2f cte=%.3f yaw_err=%.1fdeg v_des=%.2f v=%.2f steer_pwm=%d throttle_pwm=%d manual=%d armed=%d",
      cur_x_,
      cur_y_,
      pose_projection.segment_index,
      progress_s,
      ref_path_.back().s,
      remaining_s,
      cte,
      heading_error * 180.0 / kPi,
      desired_speed_mps,
      cur_speed_mps_,
      steering_pwm,
      throttle_pwm,
      mavlink_.px4_manual ? 1 : 0,
      mavlink_.px4_armed ? 1 : 0);
  }

  bool computePathProjection(double x, double y, PathProjection & projection) const
  {
    if (!path_valid_ || ref_path_.size() < 2) {
      return false;
    }

    PathProjection best;
    double best_distance_sq = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i + 1 < ref_path_.size(); ++i) {
      const auto & p0 = ref_path_[i];
      const auto & p1 = ref_path_[i + 1];
      const double dx = p1.x - p0.x;
      const double dy = p1.y - p0.y;
      const double length_sq = dx * dx + dy * dy;
      if (length_sq <= 1e-12) {
        continue;
      }

      const double raw_t = ((x - p0.x) * dx + (y - p0.y) * dy) / length_sq;
      const double t = clampd(raw_t, 0.0, 1.0);
      const double proj_x = p0.x + t * dx;
      const double proj_y = p0.y + t * dy;
      const double err_x = x - proj_x;
      const double err_y = y - proj_y;
      const double distance_sq = err_x * err_x + err_y * err_y;

      if (distance_sq < best_distance_sq) {
        const double yaw = wrapPi(p0.yaw + t * wrapPi(p1.yaw - p0.yaw));
        best.valid = true;
        best.segment_index = i;
        best.t = t;
        best.x = proj_x;
        best.y = proj_y;
        best.yaw = yaw;
        best.s = p0.s + t * (p1.s - p0.s);
        best.v_ref = p0.v_ref + t * (p1.v_ref - p0.v_ref);
        best.distance = std::sqrt(distance_sq);
        best_distance_sq = distance_sq;
      }
    }

    if (!best.valid) {
      return false;
    }

    projection = best;
    return true;
  }

  double computeDesiredSpeed(double remaining_s, double speed_ref_mps) const
  {
    if (speed_ref_mps <= 0.0) {
      return 0.0;
    }

    double scale = 1.0;
    if (slow_down_distance_m_ > 1e-6) {
      scale = clampd(remaining_s / slow_down_distance_m_, 0.0, 1.0);
    }

    double desired = speed_ref_mps * scale;
    if (desired > 1e-6) {
      desired = std::max(min_command_speed_mps_, desired);
    }
    return desired;
  }

  int steeringToPwm(double steer_cmd_rad) const
  {
    const double max_abs_steer = std::max(std::abs(max_steer_left_rad_), std::abs(max_steer_right_rad_));
    const double steer_norm = clampd(steer_cmd_rad / std::max(1e-6, max_abs_steer), -1.0, 1.0);
    const int pwm = static_cast<int>(std::lround(
      static_cast<double>(steering_center_pwm_) + steering_pwm_sign_ * steer_norm * static_cast<double>(steering_pwm_span_)));
    return clampi(pwm, steering_pwm_min_, steering_pwm_max_);
  }

  int speedToThrottlePwm(double desired_speed_mps)
  {
    if (desired_speed_mps <= 1e-5) {
      return throttle_stop_pwm_;
    }

    const double speed_scale = clampd(
      desired_speed_mps / std::max(1e-6, throttle_speed_at_max_pwm_mps_),
      0.0,
      1.0);

    double pwm = static_cast<double>(throttle_drive_min_pwm_) +
      speed_scale * static_cast<double>(throttle_drive_max_pwm_ - throttle_drive_min_pwm_);

    const int lo = std::min(throttle_stop_pwm_, std::min(throttle_drive_min_pwm_, throttle_drive_max_pwm_));
    const int hi = std::max(throttle_stop_pwm_, std::max(throttle_drive_min_pwm_, throttle_drive_max_pwm_));
    return clampi(static_cast<int>(std::lround(pwm)), lo, hi);
  }

  double pwmToNorm(int pwm_us) const
  {
    const double lo = static_cast<double>(actuator_pwm_min_);
    const double hi = static_cast<double>(actuator_pwm_max_);
    if (hi <= lo + 1e-6) {
      return 0.0;
    }
    return clampd(2.0 * (static_cast<double>(pwm_us) - lo) / (hi - lo) - 1.0, -1.0, 1.0);
  }

  void sendPwm(int steering_pwm, int throttle_pwm)
  {
    if (!mavlink_.enable) {
      return;
    }

    bool ok = false;
    if (mavlink_output_mode_ == "do_set_servo") {
      const bool ok_steer = mavlink_.sendDoSetServo(this, steering_servo_number_, steering_pwm);
      const bool ok_throttle = mavlink_.sendDoSetServo(this, throttle_servo_number_, throttle_pwm);
      ok = ok_steer && ok_throttle;
    } else if (mavlink_output_mode_ == "do_set_actuator") {
      const double throttle_norm = pwmToNorm(throttle_pwm);
      const double steering_norm = pwmToNorm(steering_pwm);
      ok = mavlink_.sendDoSetActuator(
        this,
        throttle_norm,
        steering_norm,
        throttle_actuator_index_,
        steering_actuator_index_,
        actuator_set_index_);
    } else {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "unknown mavlink_output_mode=%s. Use do_set_servo or do_set_actuator.",
        mavlink_output_mode_.c_str());
      return;
    }

    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 500,
      "MAVLink PWM tx: steer=%d throttle=%d ok=%d",
      steering_pwm, throttle_pwm, ok ? 1 : 0);
  }

  void sendNeutral()
  {
    sendPwm(steering_center_pwm_, throttle_stop_pwm_);
  }

  void sendNeutralBurst(int repeat_count = 5)
  {
    for (int i = 0; i < std::max(1, repeat_count); ++i) {
      sendNeutral();
    }
  }

  void publishReferencePath()
  {
    if (!path_pub_ || !path_valid_) {
      return;
    }

    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = frame_id_;

    path.poses.resize(ref_path_.size());
    for (size_t i = 0; i < ref_path_.size(); ++i) {
      const auto & point = ref_path_[i];
      auto & ps = path.poses[i];
      ps.header = path.header;
      ps.pose.position.x = point.x;
      ps.pose.position.y = point.y;
      ps.pose.position.z = 0.0;
      ps.pose.orientation.z = std::sin(0.5 * point.yaw);
      ps.pose.orientation.w = std::cos(0.5 * point.yaw);
    }

    path_pub_->publish(path);
  }

  // ROS parameters/state
  std::string pose_topic_;
  std::string service_prefix_;
  std::string path_debug_topic_;
  std::string frame_id_;

  double path_start_x_{0.20};
  double path_start_y_{1.00};
  double path_end_x_{1.80};
  double path_end_y_{1.00};
  std::string path_csv_file_;
  double goal_tolerance_m_{0.08};
  double slow_down_distance_m_{0.35};

  double loop_rate_hz_{20.0};
  double pose_timeout_sec_{0.35};
  double wheelbase_m_{0.33};
  double target_speed_mps_{0.25};
  bool use_csv_speed_ref_{true};
  double min_command_speed_mps_{0.05};
  double stanley_k_{0.8};
  double stanley_softening_mps_{0.10};
  double max_steer_left_rad_{0.52};
  double max_steer_right_rad_{-0.52};

  int throttle_stop_pwm_{1500};
  int throttle_drive_min_pwm_{1550};
  int throttle_drive_max_pwm_{1600};
  double throttle_speed_at_max_pwm_mps_{0.35};

  int steering_center_pwm_{1500};
  int steering_pwm_span_{400};
  int steering_pwm_min_{1000};
  int steering_pwm_max_{2000};
  double steering_pwm_sign_{-1.0};

  std::string mavlink_output_mode_{"do_set_servo"};
  int steering_servo_number_{1};
  int throttle_servo_number_{2};
  int throttle_actuator_index_{1};
  int steering_actuator_index_{2};
  int actuator_set_index_{0};
  int actuator_pwm_min_{1000};
  int actuator_pwm_max_{2000};

  bool drive_enabled_{false};
  bool require_manual_mode_{false};
  bool require_armed_{false};
  bool send_neutral_when_stopped_{true};
  bool auto_manual_mode_{false};
  bool auto_arm_{false};
  bool force_arm_{false};
  bool disarm_on_shutdown_{false};
  bool goal_reached_{false};
  bool path_valid_{false};
  bool using_csv_path_{false};
  std::vector<PathPoint> ref_path_;

  // Pose/speed state
  bool have_pose_{false};
  double cur_x_{0.0};
  double cur_y_{0.0};
  double cur_z_{0.0};
  double cur_yaw_{0.0};
  double cur_speed_mps_{0.0};
  rclcpp::Time last_pose_time_;

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr loop_timer_;
  rclcpp::TimerBase::SharedPtr path_timer_;

  rclcpp::Service<Trigger>::SharedPtr set_manual_srv_;
  rclcpp::Service<Trigger>::SharedPtr arm_srv_;
  rclcpp::Service<Trigger>::SharedPtr disarm_srv_;
  rclcpp::Service<Trigger>::SharedPtr start_srv_;
  rclcpp::Service<Trigger>::SharedPtr stop_srv_;

  MavlinkUdpBridge mavlink_;
};

}  // namespace mocap_straight_pwm

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mocap_straight_pwm::StraightMocapPwmNode>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
