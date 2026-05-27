#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

#include <cmath>
#include <vector>
#include <limits>
#include <chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;

class PathTrackingAckermannNode : public rclcpp::Node
{
public:
  PathTrackingAckermannNode()
  : rclcpp::Node("path_tracking_ackermann_node")
  {
    // 파라미터 선언
    this->declare_parameter<double>("loop_rate_hz", 50.0);
    this->declare_parameter<double>("stanley_k", 1.0);
    this->declare_parameter<double>("v_set", 1.0);          // 목표 속도 [m/s]
    this->declare_parameter<double>("max_steer_left", 0.5); // +최대 조향각 [rad]
    this->declare_parameter<double>("max_steer_right", 0.5);// -최대 조향각 [rad]

    // 파라미터 로드
    this->get_parameter("loop_rate_hz", loop_rate_hz_);
    this->get_parameter("stanley_k", stanley_k_);
    this->get_parameter("v_set", v_set_);
    this->get_parameter("max_steer_left", max_steer_left_);
    this->get_parameter("max_steer_right", max_steer_right_);

    RCLCPP_INFO(this->get_logger(),
      "PathTrackingAckermannNode @ %.1f Hz (v_set=%.2f, k=%.2f, maxL=%.2f, maxR=%.2f)",
      loop_rate_hz_, v_set_, stanley_k_, max_steer_left_, max_steer_right_);

    // 구독: /odom
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&PathTrackingAckermannNode::odomCallback, this, _1));

    // 구독: /controller/local_curve
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/controller/local_curve", 10, std::bind(&PathTrackingAckermannNode::pathCallback, this, _1));

    // 퍼블리셔: /cmd_ackermann_raw
    ackermann_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      "/cmd_ackermann_raw", 10);

    // 제어 루프 타이머
    auto period = std::chrono::duration<double>(1.0 / loop_rate_hz_);
    timer_ = this->create_wall_timer(
      period, std::bind(&PathTrackingAckermannNode::controlLoop, this));
  }

private:
  // 쿼터니언 → yaw
  static double yawFromQuat(double x, double y, double z, double w)
  {
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  static double normalizeAngle(double a)
  {
    return std::atan2(std::sin(a), std::cos(a));
  }

  // /odom 콜백: 현재 상태 갱신
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    cur_x_ = msg->pose.pose.position.x;
    cur_y_ = msg->pose.pose.position.y;

    const auto & q = msg->pose.pose.orientation;
    cur_yaw_ = yawFromQuat(q.x, q.y, q.z, q.w);

    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    double v = std::sqrt(vx * vx + vy * vy);
    if (vx < 0.0) v = -v;

    cur_speed_ = v;
    have_odom_ = true;
  }

  // /controller/local_curve 콜백: 로컬 경로 저장
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    path_ = msg;
    have_path_ = true;
  }

  // 메인 제어 루프
  void controlLoop()
  {
    if (!have_odom_ || !have_path_) {
      return;
    }
    const auto & poses = path_->poses;
    if (poses.size() < 2) {
      return;
    }

    // 현재 위치/자세
    double px = cur_x_;
    double py = cur_y_;
    double yaw = cur_yaw_;

    // 경로 포인트들을 2D로 보면서 최근접 포인트 찾기
    size_t nearest_idx = 0;
    double dmin = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < poses.size(); ++i) {
      double x = poses[i].pose.position.x;
      double y = poses[i].pose.position.y;
      double dx = x - px;
      double dy = y - py;
      double d2 = dx * dx + dy * dy;
      if (d2 < dmin) {
        dmin = d2;
        nearest_idx = i;
      }
    }

    // 경로 진행 방향(path_yaw) 계산
    size_t i_prev = (nearest_idx > 0) ? nearest_idx - 1 : nearest_idx;
    size_t i_next = std::min(nearest_idx + 1, poses.size() - 1);

    double path_yaw;
    if (i_next != nearest_idx) {
      double x0 = poses[nearest_idx].pose.position.x;
      double y0 = poses[nearest_idx].pose.position.y;
      double x1 = poses[i_next].pose.position.x;
      double y1 = poses[i_next].pose.position.y;
      path_yaw = std::atan2(y1 - y0, x1 - x0);
    } else if (i_prev != nearest_idx) {
      double x0 = poses[i_prev].pose.position.x;
      double y0 = poses[i_prev].pose.position.y;
      double x1 = poses[nearest_idx].pose.position.x;
      double y1 = poses[nearest_idx].pose.position.y;
      path_yaw = std::atan2(y1 - y0, x1 - x0);
    } else {
      // 포인트가 1개뿐인 경우
      return;
    }

    // CTE 계산 (경로 좌표계에서 y 방향 오차)
    {
      double x_ref = poses[nearest_idx].pose.position.x;
      double y_ref = poses[nearest_idx].pose.position.y;

      double dx = px - x_ref;
      double dy = py - y_ref;

      // 경로 방향(path_yaw)을 기준으로 회전했을 때, y축 성분이 CTE
      cte_ = -dx * std::sin(path_yaw) + dy * std::cos(path_yaw);
    }

    // 헤딩 에러
    double heading_err = normalizeAngle(path_yaw - yaw);

    // Stanley 조향 법칙
    double v_for = std::max(0.1, std::abs(cur_speed_)); // 너무 느리면 수치 문제 방지
    double steer_cmd = heading_err + std::atan2(stanley_k_ * cte_, v_for);

    // 조향 saturate
    if (steer_cmd > 0.0) {
      steer_cmd = std::min(steer_cmd, max_steer_left_);
    } else {
      steer_cmd = std::max(steer_cmd, -max_steer_right_);
    }

    // Ackermann 메시지로 publish
    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "base_link";  // 필요에 따라 수정

    cmd.drive.steering_angle = steer_cmd; // [rad]
    cmd.drive.speed          = v_set_;    // [m/s]

    ackermann_pub_->publish(cmd);

    // 디버깅 필요하면:
    // RCLCPP_INFO(this->get_logger(),
    //   "steer=%.3f rad, v=%.2f m/s (cte=%.3f, heading_err=%.3f)",
    //   steer_cmd, v_set_, cte_, heading_err);
  }

private:
  // 파라미터
  double loop_rate_hz_{50.0};
  double stanley_k_{1.0};
  double v_set_{1.0};
  double max_steer_left_{0.5};
  double max_steer_right_{0.5};

  // 상태
  bool have_odom_{false};
  bool have_path_{false};

  double cur_x_{0.0};
  double cur_y_{0.0};
  double cur_yaw_{0.0};
  double cur_speed_{0.0};

  double cte_{0.0};

  // ROS
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Path::SharedPtr path_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathTrackingAckermannNode>());
  rclcpp::shutdown();
  return 0;
}
