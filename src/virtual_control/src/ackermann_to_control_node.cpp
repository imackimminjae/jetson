#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

#include <imac_interfaces/msg/virtual_control_command.hpp>

#include <algorithm>
#include <cmath>
#include <chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;

class AckermannToControlNode : public rclcpp::Node
{
public:
  AckermannToControlNode()
  : rclcpp::Node("ackermann_to_control_node")
  {
    // 파라미터 선언
    this->declare_parameter<double>("loop_rate_hz", 50.0);
    this->declare_parameter<double>("steer_limit_rad", 0.5);  // 최대 조향각 [rad]
    this->declare_parameter<double>("kp_throttle", 0.5);      // 속도 P게인(가속)
    this->declare_parameter<double>("kp_brake", 0.5);         // 속도 P게인(감속)

    // 파라미터 읽기
    this->get_parameter("loop_rate_hz", loop_rate_hz_);
    this->get_parameter("steer_limit_rad", steer_limit_rad_);
    this->get_parameter("kp_throttle", kp_throttle_);
    this->get_parameter("kp_brake", kp_brake_);

    RCLCPP_INFO(
      this->get_logger(),
      "AckermannToControlNode @ %.1f Hz (steer_limit=%.3f rad, kp_th=%.2f, kp_br=%.2f)",
      loop_rate_hz_, steer_limit_rad_, kp_throttle_, kp_brake_);

    // 구독자: /odom
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&AckermannToControlNode::odomCallback, this, _1));

    // 구독자: /ackermann_cmd
    ackermann_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      "/ackermann_cmd", 10,
      std::bind(&AckermannToControlNode::ackermannCallback, this, _1));

    // 퍼블리셔: /cmd_control (imac_interfaces/VirtualControlCommand)
    control_pub_ = this->create_publisher<imac_interfaces::msg::VirtualControlCommand>(
      "/cmd_control", 10);

    // 타이머
    auto period = std::chrono::duration<double>(1.0 / loop_rate_hz_);
    timer_ = this->create_wall_timer(
      period, std::bind(&AckermannToControlNode::onTimer, this));
  }

private:
  // /odom 콜백: 현재 속도만 저장
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    double v = std::sqrt(vx * vx + vy * vy);

    // 전방(+x) 기준으로 후진이면 음수 표시하고 싶다면:
    if (vx < 0.0) v = -v;

    current_speed_ = v;
    have_odom_ = true;
  }

  // /ackermann_cmd 콜백: 목표 조향/속도 갱신
  void ackermannCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
  {
    cmd_steer_angle_ = msg->drive.steering_angle;  // [rad]
    cmd_speed_       = msg->drive.speed;           // [m/s]
    have_cmd_ = true;
  }

  void onTimer()
  {
    imac_interfaces::msg::VirtualControlCommand out;
    out.steer    = 0.0f;
    out.throttle = 0.0f;
    out.brake    = 0.0f;

    // 아직 Ackermann 명령이 안 들어왔으면 0 출력
    if (!have_cmd_) {
      control_pub_->publish(out);
      return;
    }

    // 1) 조향: 최대 조향각으로 나눠서 [-1, 1] 정규화
    if (std::abs(steer_limit_rad_) > 1e-6) {
      double steer_norm = cmd_steer_angle_ / steer_limit_rad_;
      steer_norm = std::clamp(steer_norm, -1.0, 1.0);
      out.steer = static_cast<float>(steer_norm);
    } else {
      out.steer = 0.0f;
    }

    // 2) 속도 제어: P 제어로 throttle/brake
    //  - odom이 아직 안 들어왔으면 current_speed_는 0.0 그대로 사용
    double e = cmd_speed_ - current_speed_;  // 속도 오차: 목표 - 현재

    double throttle = 0.0;
    double brake = 0.0;

    if (e > 0.0) {
      // 더 빨라져야 -> throttle
      throttle = kp_throttle_ * e;
      throttle = std::clamp(throttle, 0.0, 1.0);
      brake = 0.0;
    } else {
      // 더 느려져야 -> brake
      brake = kp_brake_ * (-e);
      brake = std::clamp(brake, 0.0, 1.0);
      throttle = 0.0;
    }

    out.throttle = static_cast<float>(throttle);
    out.brake    = static_cast<float>(brake);

    control_pub_->publish(out);

    // 디버그용 (원하면 주석 해제)
    // RCLCPP_INFO_THROTTLE(
    //   this->get_logger(), *this->get_clock(), 1000,
    //   "ack2ctrl: v_des=%.2f, v_cur=%.2f, e=%.2f -> th=%.2f, br=%.2f, steer=%.2f",
    //   cmd_speed_, current_speed_, e, throttle, brake, out.steer);
  }

private:
  // 파라미터
  double loop_rate_hz_{50.0};
  double steer_limit_rad_{0.5};
  double kp_throttle_{0.5};
  double kp_brake_{0.5};

  // 상태
  bool   have_odom_{false};
  bool   have_cmd_{false};
  double current_speed_{0.0};
  double cmd_steer_angle_{0.0};
  double cmd_speed_{0.0};

  // ROS 핸들들
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_sub_;
  rclcpp::Publisher<imac_interfaces::msg::VirtualControlCommand>::SharedPtr control_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AckermannToControlNode>());
  rclcpp::shutdown();
  return 0;
}

