#include "virtual_control/tracking_controller_node.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <limits>
#include <mutex>
#include <vector>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>

using std::placeholders::_1;
using Eigen::Vector2d;

namespace imac_ctrl
{

inline double deg2rad(double d)
{
  constexpr double PI = 3.14159265358979323846;
  return d * PI / 180.0;
}

// [-pi, pi] wrap
static double wraptopi(double angle)
{
  const double two_pi = 2.0 * M_PI;
  const double pi = M_PI;
  double wrapped_angle = std::fmod(angle, two_pi);
  if (wrapped_angle > pi) {
    wrapped_angle -= two_pi;
  } else if (wrapped_angle < -pi) {
    wrapped_angle += two_pi;
  }
  return wrapped_angle;
}

static inline int clampi(int v, int lo, int hi)
{
  return std::max(lo, std::min(hi, v));
}

TrackingControllerNode::TrackingControllerNode()
: Node("tracking_controller_node")
{
  // ---- Declare parameters ----
  this->declare_parameter<double>("s_step_init", 0.20);
  this->declare_parameter<double>("kp", 1.0);
  this->declare_parameter<double>("ki", 0.1);
  this->declare_parameter<double>("kd", 0.1);
  this->declare_parameter<double>("loop_rate_hz_ctrl", 20.0);
  this->declare_parameter<double>("loop_rate_hz_trj", 20.0);
  this->declare_parameter<double>("stanley_k", 1.0);
  this->declare_parameter<double>("max_steer_left", 0.52);
  this->declare_parameter<double>("max_steer_right", -0.52);
  this->declare_parameter<double>("v_goal", 0.8);
  this->declare_parameter<std::vector<double>>("dpsi_deg", std::vector<double>{0.0});

  // ---- Get parameters ----
  this->get_parameter("kp", kp_);
  this->get_parameter("ki", ki_);
  this->get_parameter("kd", kd_);
  this->get_parameter("loop_rate_hz_ctrl", loop_rate_hz_ctrl_);
  this->get_parameter("loop_rate_hz_trj", loop_rate_hz_trj_);
  this->get_parameter("s_step_init", s_step_init_);
  this->get_parameter("stanley_k", stanley_k_);
  this->get_parameter("max_steer_left", max_steer_left_);
  this->get_parameter("max_steer_right", max_steer_right_);
  this->get_parameter("v_goal", v_goal_);

  v_set_ = v_goal_;

  auto dpsi_deg_vec = this->get_parameter("dpsi_deg").as_double_array();
  dpsi_set_.reserve(dpsi_deg_vec.size());
  for (double d : dpsi_deg_vec) {
    dpsi_set_.push_back(deg2rad(d));
  }

  // ---- 초기 상태 설정 ----
  if (!sim_initialized_) {
    cur_x_ = 0.0;
    cur_y_ = 0.0;
    cur_z_ = 0.0;
    cur_yaw_ = 0.0;
    cur_speed_ = 0.0;

    sim_x_ = 0.0;
    sim_y_ = 0.0;
    sim_yaw_ = 0.0;

    sim_vx_body_ = 0.0;
    sim_vy_body_ = 0.0;
    sim_yaw_rate_ = 0.0;
    sim_speed_ = 0.0;

    sim_initialized_ = true;
    have_pose_ = true;
  }

  RCLCPP_INFO(
    get_logger(),
    "TrackingControllerNode @ ctrl=%.1f Hz, v_goal=%.2f",
    loop_rate_hz_ctrl_, v_goal_);

  // ---- Subscribers ----
  state_sim_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, std::bind(&TrackingControllerNode::simposeCallback, this, _1));

  global_path_sub_ = this->create_subscription<imac_interfaces::msg::GlobalPath>(
    "/global_path", 10, std::bind(&TrackingControllerNode::globalPathCallback, this, _1));

  // ---- Publishers ----
  control_command_pub_ =
    this->create_publisher<imac_interfaces::msg::VirtualControlCommand>("/cmd_control", 10);

  // (추가) PWM 전용 토픽 퍼블리셔
  pwm_pub_ =
    this->create_publisher<std_msgs::msg::Int32MultiArray>("/cmd_pwm", 10);

  local_curve_pub_ =
    this->create_publisher<nav_msgs::msg::Path>("/controller/local_curve", 10);
  status_hils_pub_ =
    this->create_publisher<imac_interfaces::msg::VehicleStatusHils>("/status_hils", 10);

  // ---- Control loop timer ----
  auto period = std::chrono::duration<double>(1.0 / loop_rate_hz_ctrl_);
  loop_timer_ = this->create_wall_timer(
    period, std::bind(&TrackingControllerNode::controlLoop, this));

  RCLCPP_INFO(this->get_logger(), "TrackingControllerNode controlLoop at %.1f Hz", loop_rate_hz_ctrl_);
}

void TrackingControllerNode::poseCallback(
  const imac_interfaces::msg::VehicleStatusMav::SharedPtr msg)
{
  cur_x_ = msg->x;
  cur_y_ = msg->y;
  cur_z_ = msg->z;
  cur_speed_ = msg->speed;
  cur_yaw_ = msg->heading;
  have_pose_ = true;
}

// 시뮬레이터 odom 콜백
void TrackingControllerNode::simposeCallback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  sim_x_ = msg->pose.pose.position.x;
  sim_y_ = msg->pose.pose.position.y;

  const auto & q = msg->pose.pose.orientation;
  tf2::Quaternion quat(q.x, q.y, q.z, q.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  sim_yaw_ = yaw;

  // body-frame velocities
  sim_vx_body_ = msg->twist.twist.linear.x;
  sim_vy_body_ = msg->twist.twist.linear.y;

  // yaw rate (rad/s)
  sim_yaw_rate_ = msg->twist.twist.angular.z;

  // speed scalar
  sim_speed_ = std::hypot(sim_vx_body_, sim_vy_body_);

  sim_have_pose_ = true;
}

// Global path 콜백
void TrackingControllerNode::globalPathCallback(
  const imac_interfaces::msg::GlobalPath::SharedPtr msg)
{
  if (msg->g_x.size() != msg->g_y.size()) {
    RCLCPP_WARN(
      this->get_logger(),
      "[global_path] size mismatch: g_x=%zu g_y=%zu",
      msg->g_x.size(), msg->g_y.size());
    return;
  }

  std::vector<Eigen::Vector2d> g;
  g.reserve(msg->g_x.size());
  for (size_t i = 0; i < msg->g_x.size(); ++i) {
    g.emplace_back(msg->g_x[i], msg->g_y[i]);
  }

  {
    std::lock_guard<std::mutex> lk(map_mtx_);
    global_path_.swap(g);
    ref_path_ = global_path_;
    ref_path_ok_ = !ref_path_.empty();
    global_ok_ = !global_path_.empty();
  }

  RCLCPP_INFO(this->get_logger(), "Received global_path: %zu points", ref_path_.size());
}

// 메인 control loop
void TrackingControllerNode::controlLoop()
{
  if (!sim_have_pose_) {
    return;
  }
  if (!global_ok_) {
    return;
  }

  std::vector<Eigen::Vector2d> ref;
  {
    std::lock_guard<std::mutex> lk(map_mtx_);
    if (!ref_path_ok_ || ref_path_.size() < 2) {
      return;
    }
    ref = ref_path_;
  }

  // ---- 현재 차량 위치 (sim 기준) ----
  Vector2d pos_sim(sim_x_, sim_y_);

  // ---- 현재 경로에서 가까운 포인트 찾기 ----
  current_path_idx_ = nearestIndex(ref, pos_sim);

  // ---- 근방 일부 구간만 잘라서 quarticFit 수행 ----
  int i0 = current_path_idx_;
  int i1 = std::min(static_cast<int>(ref.size()) - 1, i0 + 10);
  std::vector<Eigen::Vector2d> slice(ref.begin() + i0, ref.begin() + i1 + 1);

  auto local_curve = quarticFit(slice, pos_sim, static_cast<int>(slice.size()));
  publishPath(local_curve, "map");

  // ---- Stanley steering 제어 ----
  const auto & pts = ref;
  if (pts.size() < 2) {
    return;
  }

  size_t i_near = 0;
  double dmin = 1e18;
  for (size_t i = 0; i < pts.size(); ++i) {
    double d2 = (pts[i] - pos_sim).squaredNorm();
    if (d2 < dmin) {
      dmin = d2;
      i_near = i;
    }
  }

  const size_t i_prev = (i_near > 0) ? i_near - 1 : i_near;
  const size_t i_next = std::min(i_near + 3, pts.size() - 1);

  double path_yaw;
  if (i_next != i_near) {
    path_yaw = wraptopi(std::atan2(
      pts[i_next].y() - pts[i_near].y(),
      pts[i_next].x() - pts[i_near].x()));
  } else if (i_prev != i_near) {
    path_yaw = wraptopi(std::atan2(
      pts[i_near].y() - pts[i_prev].y(),
      pts[i_near].x() - pts[i_prev].x()));
  } else {
    return;
  }

  // CTE
  double dx = pts[i_near].x() - pos_sim.x();
  double dy = pts[i_near].y() - pos_sim.y();
  double cte = -dx * std::sin(path_yaw) + dy * std::cos(path_yaw);

  auto normAng = [](double a){ return std::atan2(std::sin(a), std::cos(a)); };

  // 헤딩 에러
  double heading_err = normAng(path_yaw - sim_yaw_);

  // Stanley formula
  const double v_for = std::max(0.05, v_set_);
  double steer_cmd = heading_err + std::atan2(stanley_k_ * cte, v_for + 0.1);

  // 조향 한계 적용
  if (steer_cmd > 0.0) {
    steer_cmd = std::min(steer_cmd, max_steer_left_);
  } else {
    steer_cmd = std::max(steer_cmd, max_steer_right_);
  }

  // ---- PID 속도 제어 ----
  const double dt = 1.0 / loop_rate_hz_ctrl_;
  double v_err = v_goal_ - sim_speed_;

  integral_ += v_err * dt;
  double deriv = (v_err - prev_err_) / dt;
  prev_err_ = v_err;

  double raw_accel = kp_ * v_err + ki_ * integral_ + kd_ * deriv;

  const double a_limit = 3.0;
  double accel = std::max(-a_limit, std::min(a_limit, raw_accel));

  // ---- Cmd 구성 ----
  Cmd cmd{};
  cmd.steer = steer_cmd / max_steer_left_;  // [-1,1] 근사

  if (accel >= 0.0) {
    cmd.throttle = accel / a_limit; // [0,1]
    cmd.brake = 0.0;
  } else {
    cmd.throttle = 0.0;
    cmd.brake = -accel / a_limit;
  }

  // ---- VirtualControlCommand 퍼블리시 (기존 유지) ----
  imac_interfaces::msg::VirtualControlCommand ctrl_msg;
  ctrl_msg.steer = static_cast<float>(cmd.steer);
  ctrl_msg.throttle = static_cast<float>(cmd.throttle);
  ctrl_msg.brake = static_cast<float>(cmd.brake);
  control_command_pub_->publish(ctrl_msg);

  // ==========================================================
  // (추가) PWM 토픽 /cmd_pwm 발행
  // data[0] = steer_pwm
  // data[1] = throttle_pwm  (여기서는 cmd.throttle이 아니라 sim_speed_로 매핑)
  // ==========================================================
  // steer: -1~1 -> 1000~2000 (1500 center)
  int steer_pwm = static_cast<int>(std::lround(1500.0 + (-cmd.steer) * 500.0));
  steer_pwm = clampi(steer_pwm, 1000, 2000);

  // throttle: sim_speed_ -> 1550~1600 (v_goal 기준)
  const double v_ref = std::max(1e-3, v_goal_);
  double speed_ratio = sim_speed_ / v_ref;
  speed_ratio = std::max(0.0, std::min(1.0, speed_ratio));

  int throttle_pwm = static_cast<int>(std::lround(1550.0 + speed_ratio * 50.0));
  throttle_pwm = clampi(throttle_pwm, 1550, 1600);

  std_msgs::msg::Int32MultiArray pwm_msg;
  pwm_msg.data.resize(2);
  pwm_msg.data[0] = steer_pwm;
  pwm_msg.data[1] = throttle_pwm;
  pwm_pub_->publish(pwm_msg);

  // ---- HILS 상태 업데이트 (odom twist 기반 적분) ----
  const double vbx = sim_vx_body_;
  const double vby = sim_vy_body_;

  const double vx_world = vbx * std::cos(cur_yaw_) - vby * std::sin(cur_yaw_);
  const double vy_world = vbx * std::sin(cur_yaw_) + vby * std::cos(cur_yaw_);

  cur_x_ += vx_world * dt;
  cur_y_ += vy_world * dt;

  cur_yaw_ = normAng(cur_yaw_ + sim_yaw_rate_ * dt);
  cur_speed_ = sim_speed_;

  imac_interfaces::msg::VehicleStatusHils st;
  st.x = cur_x_;
  st.y = cur_y_;
  st.z = 0.0;
  st.speed = cur_speed_;
  st.heading = cur_yaw_;
  status_hils_pub_->publish(st);
}

// 최근접 포인트 인덱스
int TrackingControllerNode::nearestIndex(
  const std::vector<Eigen::Vector2d> & path,
  const Eigen::Vector2d & p) const
{
  if (path.empty()) {
    return 0;
  }
  int idx = 0;
  double best = std::numeric_limits<double>::infinity();
  for (int i = 0; i < static_cast<int>(path.size()); ++i) {
    double d = (path[i] - p).norm();
    if (d < best) {
      best = d;
      idx = i;
    }
  }
  return idx;
}

// quartic polynomial fitting으로 local curve 생성
std::vector<Eigen::Vector2d> TrackingControllerNode::quarticFit(
  const std::vector<Eigen::Vector2d> & path_ref,
  const Eigen::Vector2d & p,
  int maxPts) const
{
  std::vector<Eigen::Vector2d> P;
  std::vector<double> S;

  if (path_ref.empty()) {
    return P;
  }

  int ind_start = nearestIndex(path_ref, p);

  P.reserve(maxPts);
  S.reserve(maxPts);

  P.push_back(p);
  S.push_back(0.0);

  for (int i = 0; i < maxPts - 1; ++i) {
    int idx = ind_start + i;
    if (idx >= static_cast<int>(path_ref.size())) {
      break;
    }
    P.push_back(path_ref[idx]);
    S.push_back((i + 1) * s_step_init_);
  }

  int M = static_cast<int>(P.size());
  if (M < 5) {
    return P;
  }

  Eigen::MatrixXd A(M, 5);
  Eigen::VectorXd X(M), Y(M);

  for (int i = 0; i < M; ++i) {
    double s = S[i];
    A(i, 0) = 1.0;
    A(i, 1) = s;
    A(i, 2) = s * s;
    A(i, 3) = s * s * s;
    A(i, 4) = s * s * s * s;
    X(i) = P[i].x();
    Y(i) = P[i].y();
  }

  Eigen::VectorXd cx = A.colPivHouseholderQr().solve(X);
  Eigen::VectorXd cy = A.colPivHouseholderQr().solve(Y);

  int Nd = std::max(10, M * 2);
  std::vector<Eigen::Vector2d> fit;
  fit.reserve(Nd);

  for (int i = 0; i < Nd; ++i) {
    double s = S.back() * (static_cast<double>(i) / static_cast<double>(Nd - 1));
    Eigen::RowVectorXd bb(5);
    bb << 1.0, s, s * s, s * s * s, s * s * s * s;
    double x = (bb * cx)(0);
    double y = (bb * cy)(0);
    fit.emplace_back(x, y);
  }

  return fit;
}

// nav_msgs/Path 퍼블리시
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

}  // namespace imac_ctrl

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<imac_ctrl::TrackingControllerNode>());
  rclcpp::shutdown();
  return 0;
}

