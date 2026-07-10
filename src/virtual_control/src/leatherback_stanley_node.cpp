#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

#include <cmath>
#include <vector>
#include <utility>
#include <limits>
#include <string>

using std_msgs::msg::Float64;
using nav_msgs::msg::Odometry;
using nav_msgs::msg::Path;
using ackermann_msgs::msg::AckermannDriveStamped;

static double wrap_pi(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

static double yaw_from_quat(double x, double y, double z, double w)
{
  // roll ≈ pitch ≈ 0 가정
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

// ----------------- PID -----------------
class PID
{
public:
  PID(double kp, double ki, double kd, double u_min, double u_max)
  : kp_(kp), ki_(ki), kd_(kd), u_min_(u_min), u_max_(u_max),
    i_(0.0), prev_e_(std::numeric_limits<double>::quiet_NaN())
  {}

  void reset()
  {
    i_ = 0.0;
    prev_e_ = std::numeric_limits<double>::quiet_NaN();
  }

  double step(double e, double dt)
  {
    if (dt <= 0.0) return 0.0;

    i_ += e * dt;
    double d = 0.0;
    if (!std::isnan(prev_e_)) {
      d = (e - prev_e_) / dt;
    }
    prev_e_ = e;

    double u = kp_ * e + ki_ * i_ + kd_ * d;
    if (u < u_min_) u = u_min_;
    if (u > u_max_) u = u_max_;
    return u;
  }

private:
  double kp_, ki_, kd_;
  double u_min_, u_max_;
  double i_;
  double prev_e_;
};

// ----------------- Node -----------------
class LeatherbackStanleyNode : public rclcpp::Node
{
public:
  LeatherbackStanleyNode()
  : rclcpp::Node("leatherback_stanley"),
    pid_(0.8, 0.2, 0.05, -2.0, 2.0)   // a_max는 아래에서 덮어씀
  {
    // ---- Parameters ----
    this->declare_parameter<std::string>("odom_topic", "/odom");
    this->declare_parameter<std::string>("path_topic", "/path");
    this->declare_parameter<std::string>("cmd_topic", "/ackermann_cmd");

    this->declare_parameter<double>("wheelbase", 0.32);
    this->declare_parameter<double>("steer_max_deg", 30.0);
    this->declare_parameter<double>("steer_rate_deg", 120.0);
    this->declare_parameter<bool>("invert_steering", false);
    this->declare_parameter<bool>("invert_cte_sign", false);
    this->declare_parameter<bool>("steer_is_degrees", false);

    this->declare_parameter<double>("v_ref", 3.0);
    this->declare_parameter<double>("v_ref_end", 0.8);
    this->declare_parameter<double>("v_max", 6.0);
    this->declare_parameter<double>("a_max", 2.0);

    this->declare_parameter<double>("k_cte", 1.2);
    this->declare_parameter<double>("eps", 0.1);

    this->declare_parameter<double>("lookahead", 1.5);
    this->declare_parameter<double>("control_rate", 30.0);
    this->declare_parameter<double>("goal_tolerance", 0.5);
    this->declare_parameter<bool>("use_twist", true);
    this->declare_parameter<bool>("align_path_to_start", true);

    this->declare_parameter<double>("yaw_body_offset_deg", -1.57);
    this->declare_parameter<double>("debug_log_rate_hz", 1.0);

    // ---- Load parameters ----
    odom_topic_ = this->get_parameter("odom_topic").as_string();
    path_topic_ = this->get_parameter("path_topic").as_string();
    cmd_topic_  = this->get_parameter("cmd_topic").as_string();

    L_   = this->get_parameter("wheelbase").as_double();
    k_   = this->get_parameter("k_cte").as_double();
    eps_ = this->get_parameter("eps").as_double();

    v_ref_     = this->get_parameter("v_ref").as_double();
    v_ref_end_ = this->get_parameter("v_ref_end").as_double();
    v_max_     = this->get_parameter("v_max").as_double();
    a_max_     = this->get_parameter("a_max").as_double();

    steer_max_  = this->deg2rad(this->get_parameter("steer_max_deg").as_double());
    steer_rate_ = this->deg2rad(this->get_parameter("steer_rate_deg").as_double());
    lookahead_  = this->get_parameter("lookahead").as_double();
    ctrl_rate_  = this->get_parameter("control_rate").as_double();
    goal_tol_   = this->get_parameter("goal_tolerance").as_double();
    use_twist_  = this->get_parameter("use_twist").as_bool();
    invert_steer_ = this->get_parameter("invert_steering").as_bool();
    invert_cte_sign_ = this->get_parameter("invert_cte_sign").as_bool();
    align_path_ = this->get_parameter("align_path_to_start").as_bool();
    steer_is_degrees_ = this->get_parameter("steer_is_degrees").as_bool();
    debug_log_rate_hz_ = this->get_parameter("debug_log_rate_hz").as_double();

    yaw_body_offset_ = this->deg2rad(this->get_parameter("yaw_body_offset_deg").as_double());

    // PID 출력 제한도 a_max로 다시 설정
    pid_ = PID(0.8, 0.2, 0.05, -a_max_, a_max_);

    // ---- State 초기값 ----
    x_ = y_ = yaw_ = v_meas_ = 0.0;
    prev_x_ = prev_y_ = std::numeric_limits<double>::quiet_NaN();
    prev_t_ = std::numeric_limits<double>::quiet_NaN();
    delta_cmd_ = 0.0;
    v_cmd_ = 0.0;

    have_odom_ = false;
    path_aligned_ = false;

    path_idx_ = 0;

    // 디버그 스냅샷 초기화
    dbg_cte_ = dbg_psi_err_ = dbg_delta_ = dbg_v_ = dbg_goal_ = dbg_yaw_ = dbg_psi_ref_ = 0.0;

    // ---- ROS I/O ----
    sub_odom_ = this->create_subscription<Odometry>(
      odom_topic_, 30,
      std::bind(&LeatherbackStanleyNode::onOdom, this, std::placeholders::_1));

    sub_path_ = this->create_subscription<Path>(
      path_topic_, 5,
      std::bind(&LeatherbackStanleyNode::onPath, this, std::placeholders::_1));

    pub_cmd_ = this->create_publisher<AckermannDriveStamped>(cmd_topic_, 10);

    pub_dbg_cte_  = this->create_publisher<Float64>("/dbg/cte", 10);
    pub_dbg_psi_  = this->create_publisher<Float64>("/dbg/psi_err", 10);
    pub_dbg_del_  = this->create_publisher<Float64>("/dbg/delta_cmd", 10);
    pub_dbg_v_    = this->create_publisher<Float64>("/dbg/v_cmd", 10);
    pub_dbg_goal_ = this->create_publisher<Float64>("/dbg/goal_dist", 10);
    pub_dbg_yaw_  = this->create_publisher<Float64>("/dbg/yaw", 10);
    pub_dbg_psir_ = this->create_publisher<Float64>("/dbg/psi_ref", 10);

    dt_ = 1.0 / ctrl_rate_;
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(dt_),
      std::bind(&LeatherbackStanleyNode::onTimer, this));

    if (debug_log_rate_hz_ > 1e-6) {
      log_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / debug_log_rate_hz_),
        std::bind(&LeatherbackStanleyNode::onLogTimer, this));
    }

    RCLCPP_INFO(
      this->get_logger(),
      "[LeatherbackStanley] L=%.3f, k=%.3f, v_ref=%.2f | topics: odom=%s, path=%s, cmd=%s | "
      "align=%d, invert_steer=%d, invert_cte=%d, steer_deg=%d",
      L_, k_, v_ref_, odom_topic_.c_str(), path_topic_.c_str(), cmd_topic_.c_str(),
      align_path_, invert_steer_, invert_cte_sign_, steer_is_degrees_);
  }

private:
  double deg2rad(double d) const { return d * M_PI / 180.0; }

  // ----------------- Callbacks -----------------
  void onOdom(const Odometry::SharedPtr msg)
  {
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
    const auto & q = msg->pose.pose.orientation;
    yaw_ = yaw_from_quat(q.x, q.y, q.z, q.w);
    have_odom_ = true;

    // 바디-맵 오프셋 적용
    double yaw_eff = yaw_ - yaw_body_offset_;

    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;

    if (use_twist_ && (std::fabs(vx) > 1e-5 || std::fabs(vy) > 1e-5)) {
      v_meas_ = std::cos(yaw_eff) * vx + std::sin(yaw_eff) * vy;
    } else {
      // pose 미분으로 속도 추정
      double now = this->now().seconds();
      if (!std::isnan(prev_t_)) {
        double dt = now - prev_t_;
        if (dt > 1e-4) {
          double dx = x_ - prev_x_;
          double dy = y_ - prev_y_;
          v_meas_ = (std::cos(yaw_eff) * dx + std::sin(yaw_eff) * dy) / dt;
        }
      }
      prev_x_ = x_;
      prev_y_ = y_;
      prev_t_ = now;
    }
  }

  void onPath(const Path::SharedPtr msg)
  {
    std::vector<std::pair<double,double>> pts;
    pts.reserve(msg->poses.size());
    for (const auto & ps : msg->poses) {
      pts.emplace_back(ps.pose.position.x, ps.pose.position.y);
    }

    if (pts.size() < 2) {
      RCLCPP_WARN(this->get_logger(), "path has fewer than 2 points; skip");
      return;
    }

    std::vector<std::pair<double,double>> out = pts;

    // 첫 수신 시 정렬
    if (align_path_ && have_odom_ && !path_aligned_) {
      auto [x0, y0] = pts[0];
      auto [x1, y1] = pts[1];
      double phi_path = std::atan2(y1 - y0, x1 - x0);
      double theta = yaw_ - phi_path;
      double c = std::cos(theta);
      double s = std::sin(theta);
      out.clear();
      out.reserve(pts.size());
      for (auto [x, y] : pts) {
        double dx = x - x0;
        double dy = y - y0;
        double X = c * dx - s * dy + x_;
        double Y = s * dx + c * dy + y_;
        out.emplace_back(X, Y);
      }
      path_aligned_ = true;
      RCLCPP_INFO(
        this->get_logger(),
        "aligned path to start pose (yaw=%.3f rad, phi_path=%.3f rad)",
        yaw_, phi_path);
    } else if (align_path_ && !have_odom_) {
      RCLCPP_WARN(
        this->get_logger(),
        "received path but odom not yet available; keeping raw path");
    }

    path_xy_ = std::move(out);
    path_idx_ = 0;
    RCLCPP_INFO(
      this->get_logger(),
      "received path: %zu points (aligned=%d)",
      path_xy_.size(), path_aligned_);
  }

  // ----------------- Helpers -----------------
  int nearestIndex() const
  {
    const int n = static_cast<int>(path_xy_.size());
    if (n == 0) return 0;

    int start = std::max(0, path_idx_ - 50);
    int end   = std::min(n - 1, path_idx_ + 400);

    int best_i = start;
    double best_d2 = std::numeric_limits<double>::infinity();

    for (int i = start; i <= end; ++i) {
      double dx = path_xy_[i].first - x_;
      double dy = path_xy_[i].second - y_;
      double d2 = dx*dx + dy*dy;
      if (d2 < best_d2) {
        best_d2 = d2;
        best_i = i;
      }
    }
    return best_i;
  }

  double signedCte(double xi, double yi, double psi_ref_eff) const
  {
    double dx = x_ - xi;
    double dy = y_ - yi;
    double x_local = std::cos(psi_ref_eff) * dx + std::sin(psi_ref_eff) * dy;
    double y_local = -std::sin(psi_ref_eff) * dx + std::cos(psi_ref_eff) * dy;
    double cte = y_local;  // 좌(+), 우(-)

    if (invert_cte_sign_) cte = -cte;
    return cte;
  }

  double clamp(double x, double lo, double hi) const
  {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
  }

  // ----------------- Control Loop -----------------
  void onTimer()
  {
    if (path_xy_.empty() || !have_odom_) return;

    const int n = static_cast<int>(path_xy_.size());
    int i_near = nearestIndex();
    path_idx_ = i_near;

    // "바로 앞"이 아니라, **10개 이후 점**을 참고
    int j = std::min(i_near + 5, n - 1);

    double xi = path_xy_[i_near].first;
    double yi = path_xy_[i_near].second;
    double xj = path_xy_[j].first;
    double yj = path_xy_[j].second;

    double psi_ref = std::atan2(yj - yi, xj - xi);

    // yaw_body_offset 반영
    double psi_ref_eff = psi_ref + yaw_body_offset_;
    double yaw_eff     = yaw_   + yaw_body_offset_;

    // CTE
    double cte = signedCte(xi, yi, psi_ref_eff);
    cte = clamp(cte, -5.0, 5.0); // 큰 값 클램프

    double psi_err = wrap_pi(psi_ref_eff - yaw_eff);

    // Stanley steering
    double v_abs = std::fabs(v_meas_);
    double delta_des = psi_err + std::atan2(k_ * cte, v_abs + eps_);
    if (invert_steer_) delta_des = -delta_des;
    delta_des = clamp(delta_des, -steer_max_, steer_max_);

    // rate limit
    double max_step = steer_rate_ * dt_;
    delta_cmd_ = clamp(
      delta_des,
      delta_cmd_ - max_step,
      delta_cmd_ + max_step);

    double out_delta = delta_cmd_;
    if (steer_is_degrees_) {
      out_delta = delta_cmd_ * 180.0 / M_PI;
    }

    // 목표점 거리
    double goal_dx = path_xy_.back().first  - x_;
    double goal_dy = path_xy_.back().second - y_;
    double goal_dist = std::hypot(goal_dx, goal_dy);

    double v_target =
      (goal_dist > 3.0 * goal_tol_) ? v_ref_ : v_ref_end_;

    // 속도 PID
    double a_cmd = pid_.step(v_target - v_meas_, dt_);
    v_cmd_ = clamp(v_cmd_ + a_cmd * dt_, 0.0, v_max_);

    // 명령 publish
    AckermannDriveStamped cmd;
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "base_link"; // 필요 시 수정
    cmd.drive.speed = static_cast<float>(v_cmd_);
    cmd.drive.steering_angle = static_cast<float>(out_delta);
    pub_cmd_->publish(cmd);

    // 디버그 스냅샷
    dbg_cte_ = cte;
    dbg_psi_err_ = psi_err;
    dbg_delta_ = delta_cmd_;
    dbg_v_ = v_cmd_;
    dbg_goal_ = goal_dist;
    dbg_yaw_ = yaw_;
    dbg_psi_ref_ = psi_ref;

    pubDebug();

    // 목표 도달 시 정지 명령 한번 더
    if (goal_dist <= goal_tol_ && std::fabs(v_meas_) < 0.1) {
      AckermannDriveStamped stop;
      stop.header.stamp = this->now();
      stop.header.frame_id = "base_link";
      stop.drive.speed = 0.0f;
      stop.drive.steering_angle = static_cast<float>(out_delta);
      pub_cmd_->publish(stop);
    }
  }

  void pubDebug()
  {
    Float64 m;

    m.data = dbg_cte_;   pub_dbg_cte_->publish(m);
    m.data = dbg_psi_err_; pub_dbg_psi_->publish(m);
    m.data = dbg_delta_; pub_dbg_del_->publish(m);
    m.data = dbg_v_;     pub_dbg_v_->publish(m);
    m.data = dbg_goal_;  pub_dbg_goal_->publish(m);
    m.data = dbg_yaw_;   pub_dbg_yaw_->publish(m);
    m.data = dbg_psi_ref_; pub_dbg_psir_->publish(m);
  }

  void onLogTimer()
  {
    RCLCPP_INFO(
      this->get_logger(),
      "CTE=%+.3f m | psi_err=%+.1f deg | delta=%+.1f deg | v=%.2f m/s | goal=%.2f m",
      dbg_cte_,
      dbg_psi_err_ * 180.0 / M_PI,
      dbg_delta_   * 180.0 / M_PI,
      dbg_v_,
      dbg_goal_);
  }

private:
  // 파라미터/상태
  std::string odom_topic_, path_topic_, cmd_topic_;

  double L_{0.32};
  double k_{1.2};
  double eps_{0.1};
  double v_ref_{3.0};
  double v_ref_end_{0.8};
  double v_max_{6.0};
  double a_max_{2.0};
  double steer_max_{M_PI/6};
  double steer_rate_{2.0 * M_PI};
  double lookahead_{1.5};
  double ctrl_rate_{30.0};
  double goal_tol_{0.5};
  bool use_twist_{true};
  bool invert_steer_{false};
  bool invert_cte_sign_{false};
  bool align_path_{true};
  bool steer_is_degrees_{false};
  double yaw_body_offset_{0.0};
  double debug_log_rate_hz_{1.0};

  double x_{0.0}, y_{0.0}, yaw_{0.0}, v_meas_{0.0};
  double prev_x_, prev_y_, prev_t_;
  bool have_odom_{false};
  bool path_aligned_{false};

  std::vector<std::pair<double,double>> path_xy_;
  int path_idx_{0};

  double delta_cmd_{0.0};
  double v_cmd_{0.0};

  double dt_{0.0333};

  PID pid_;

  // 디버그 값
  double dbg_cte_, dbg_psi_err_, dbg_delta_, dbg_v_, dbg_goal_, dbg_yaw_, dbg_psi_ref_;

  // ROS 핸들
  rclcpp::Subscription<Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<Path>::SharedPtr sub_path_;
  rclcpp::Publisher<AckermannDriveStamped>::SharedPtr pub_cmd_;

  rclcpp::Publisher<Float64>::SharedPtr pub_dbg_cte_, pub_dbg_psi_, pub_dbg_del_,
                                        pub_dbg_v_, pub_dbg_goal_, pub_dbg_yaw_, pub_dbg_psir_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr log_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LeatherbackStanleyNode>());
  rclcpp::shutdown();
  return 0;
}

