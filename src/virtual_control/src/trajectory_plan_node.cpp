#include "virtual_control/trajectory_plan_node.hpp"

#include <algorithm>
#include <limits>
#include <chrono>
#include <cmath>
#include <mutex>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <imac_interfaces/msg/global_path.hpp>
#include <imac_interfaces/msg/obstacles.hpp>
#include <imac_interfaces/msg/hils_obstacles.hpp>

#include <Eigen/Dense>


using Eigen::Vector2d;
using std::placeholders::_1;

static inline double deg2rad(double d) { return d * M_PI / 180.0; }

// ===================== ctor =====================
TrajectoryPlanNode::TrajectoryPlanNode()
: Node("trajectory_plan_node")
{
  // Parameters
  this->declare_parameter("loop_rate_hz", 20.0);
  this->declare_parameter("s_step_init", 0.20);
  this->declare_parameter<std::vector<double>>(
      "dpsi_deg",
      std::vector<double>{-17.5, -15, -12.5, -10, -7.5, -5, -2.5, 0,
                          2.5,  5,   7.5,   10,  12.5, 15, 17.5, 20});

  this->get_parameter("loop_rate_hz", loop_rate_hz_);
  this->get_parameter("s_step_init", s_step_init_);

  auto dpsi_deg_vec = this->get_parameter("dpsi_deg").as_double_array();
  dpsi_set_.reserve(dpsi_deg_vec.size());
  for (double d : dpsi_deg_vec) {
    dpsi_set_.push_back(deg2rad(d));
  }

  // 회피 영역은 일단 항상 사용 (원하면 파라미터화 가능)
  avoid_zone_ = true;

  // publisher: Stanley가 따라갈 로컬 경로
  local_curve_pub_ =
      this->create_publisher<nav_msgs::msg::Path>("/controller/local_curve", 10);

  // subscribers
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 20, std::bind(&TrajectoryPlanNode::odomCallback, this, _1));

  global_path_sub_ =
      this->create_subscription<imac_interfaces::msg::GlobalPath>(
          "/global_path", 10,
          std::bind(&TrajectoryPlanNode::globalPathCallback, this, _1));

  obstacles_sub_ =
      this->create_subscription<imac_interfaces::msg::Obstacles>(
          "/obstacles", 10,
          std::bind(&TrajectoryPlanNode::obstaclesCallback, this, _1));

  hils_obstacles_sub_ =
      this->create_subscription<imac_interfaces::msg::HilsObstacles>(
          "/hils_obstacles", 10,
          std::bind(&TrajectoryPlanNode::hilsobstaclesCallback, this, _1));

  // Timer
  plan_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / loop_rate_hz_),
      std::bind(&TrajectoryPlanNode::planLoop, this));

  RCLCPP_INFO(this->get_logger(),
              "TrajectoryPlanNode running at %.1f Hz (odom+global_path mode)",
              loop_rate_hz_);
}

// ===================== Callbacks =====================

// /odom → 상태 업데이트
void TrajectoryPlanNode::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
  status_x_ = msg->pose.pose.position.x;
  status_y_ = msg->pose.pose.position.y;

  const auto & q = msg->pose.pose.orientation;
  // roll, pitch ~ 0 가정 (평면 주행)
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  status_yaw_ = std::atan2(siny_cosp, cosy_cosp);

  // 속도: twist를 yaw 방향으로 투영
  double vx = msg->twist.twist.linear.x;
  double vy = msg->twist.twist.linear.y;
  current_speed_ = std::cos(status_yaw_) * vx + std::sin(status_yaw_) * vy;

  status_ok_ = true;
}

// /global_path → global_path_ 채우기
void TrajectoryPlanNode::globalPathCallback(
    const imac_interfaces::msg::GlobalPath::SharedPtr msg)
{
  if (msg->g_x.size() != msg->g_y.size()) {
    RCLCPP_WARN(this->get_logger(),
                "[global_path] size mismatch: g_x=%zu g_y=%zu",
                msg->g_x.size(), msg->g_y.size());
    return;
  }

  std::vector<Vector2d> g;
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
}

// /obstacles → p_obs_list_ 채우기 (실측 장애물)
void TrajectoryPlanNode::obstaclesCallback(
    const imac_interfaces::msg::Obstacles::SharedPtr msg)
{
  std::vector<Vector2d> obs;
  std::vector<double> dists;

  const size_t n = std::min(msg->obs_x.size(), msg->obs_y.size());
  obs.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    obs.emplace_back(msg->obs_x[i], msg->obs_y[i]);
  }

  dists.assign(msg->obs_distance.begin(), msg->obs_distance.end());
  if (dists.size() != n) {
    dists.clear();
  }

  std::lock_guard<std::mutex> lk(map_mtx_);
  p_obs_list_.swap(obs);
  obs_distances_.swap(dists);
  total_obs_ = static_cast<int>(p_obs_list_.size());

  if (total_obs_ == 0) {
    has_curr_obs_ = false;
    next_obs_ptr_ = 0;
  } else if (next_obs_ptr_ >= total_obs_) {
    next_obs_ptr_ = total_obs_ - 1;
  }
}

// /hils_obstacles → p_obs_list_ 채우기 (시뮬 장애물)
void TrajectoryPlanNode::hilsobstaclesCallback(
    const imac_interfaces::msg::HilsObstacles::SharedPtr msg)
{
  std::vector<Vector2d> obs;
  std::vector<double> dists;

  const size_t n = std::min(msg->obs_x.size(), msg->obs_y.size());
  obs.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    obs.emplace_back(msg->obs_x[i], msg->obs_y[i]);
  }

  dists.assign(msg->obs_distance.begin(), msg->obs_distance.end());
  if (dists.size() != n) {
    dists.clear();
  }

  std::lock_guard<std::mutex> lk(map_mtx_);
  p_obs_list_.swap(obs);
  obs_distances_.swap(dists);
  total_obs_ = static_cast<int>(p_obs_list_.size());

  if (total_obs_ == 0) {
    has_curr_obs_ = false;
    next_obs_ptr_ = 0;
  } else if (next_obs_ptr_ >= total_obs_) {
    next_obs_ptr_ = total_obs_ - 1;
  }
}

// ===================== Main loop =====================
void TrajectoryPlanNode::planLoop()
{
  if (!status_ok_) return;
  {
    std::lock_guard<std::mutex> lk(map_mtx_);
    if (!ref_path_ok_) return;
  }

  Vector2d car_pos(status_x_, status_y_);
  double car_yaw = status_yaw_;

  std::vector<Vector2d> obs_snapshot;
  int total_obs_snap = 0;
  {
    std::lock_guard<std::mutex> lk(map_mtx_);
    obs_snapshot = p_obs_list_;
    total_obs_snap = total_obs_;
  }

  current_path_idx_ = nearestIndex(ref_path_, car_pos);

  // --- 최근접 장애물(차량 기준 유클리드) 선택 ---
  int sel_idx = -1;
  double sel_dist = std::numeric_limits<double>::infinity();
  for (int i = 0; i < (int)obs_snapshot.size(); ++i) {
    double d = (obs_snapshot[i] - car_pos).norm();
    if (d < sel_dist) {
      sel_dist = d;
      sel_idx = i;
    }
  }

  // 단일 거리식 트리거 (차량 위치 기준)
  const double entry_thresh = 1.19;

  if (avoid_zone_ && !avoid_ && sel_idx != -1 && sel_dist <= entry_thresh) {
    p_obs_curr_ = obs_snapshot[sel_idx];
    has_curr_obs_ = true;

    next_obs_ptr_ = sel_idx;
    avoid_ = true;
    finished_ = false;
    pos_gen_ = car_pos;
    psi_gen_ = car_yaw;
    s_step_ = s_step_init_;

    // 현재까지 전면 경로 (현재까지 ref_path 앞부분)
    path_front_.assign(ref_path_.begin(),
                       ref_path_.begin() + std::max(1, current_path_idx_));

    int i1 = 0, i2 = 0;
    auto seg_cur = findAvoid(ref_path_, p_obs_curr_, i1, i2);
    idx_e_cur_ = i2;
    if (seg_cur.empty()) {
      // ref_path 근처에 영향 구간이 없으면, 그냥 장애물 위치 근처를 목표처럼
      p_goal_curr_ = p_obs_curr_;
      back_path_curr_.assign(ref_path_.begin() + std::max(0, current_path_idx_),
                             ref_path_.end());
    } else {
      p_goal_curr_ = seg_cur.back();
      back_path_curr_.assign(
          ref_path_.begin() + std::min((int)ref_path_.size(), idx_e_cur_ + 1),
          ref_path_.end());
    }

    RCLCPP_INFO(this->get_logger(),
                "[AVOID] start (euclid trigger): sel=%d dist=%.2f thr=%.2f "
                "idx_e=%d",
                sel_idx, sel_dist, entry_thresh, idx_e_cur_);
  }

  // ----- 회피 경로 유지/연장 -----
  if (avoid_zone_ && avoid_ && !finished_) {
    while ((int)path_front_.size() - current_path_idx_ < 8) {
      bool ok = false;
      Vector2d p2;
      double psi2;
      double s_step_out = s_step_;

      std::tie(p2, psi2, s_step_out, ok) =
          nextMicroFront(pos_gen_, psi_gen_, p_obs_curr_,
                         dpsi_set_, s_step_, p_goal_curr_, ref_path_);

      s_step_ = s_step_out;
      if (!ok) {
        if (s_step_ < 0.02) {
          RCLCPP_WARN(this->get_logger(),
                      "Avoidance failed: no candidate (s_step < 0.02)");
          break;
        }
        continue;
      }

      pos_gen_ = p2;
      psi_gen_ = psi2;
      if (path_front_.empty() ||
          (p2 - path_front_.back()).norm() > 0.4) {
        path_front_.push_back(p2);
      }

      // 다중 장애물 자동 전환
      if (next_obs_ptr_ < total_obs_snap) {
        int imin = -1;
        double dmin = std::numeric_limits<double>::infinity();
        for (int k = next_obs_ptr_ + 1; k < total_obs_snap; ++k) {
          double d = (obs_snapshot[k] - pos_gen_).norm();
          if (d < dmin) {
            dmin = d;
            imin = k;
          }
        }
        if (imin != -1 && dmin < 0.8) {
          next_obs_ptr_ = imin;
          p_obs_curr_ = obs_snapshot[next_obs_ptr_];
          has_curr_obs_ = true;

          int i1 = 0, i2 = 0;
          auto seg_cur = findAvoid(ref_path_, p_obs_curr_, i1, i2);
          idx_e_cur_ = i2;
          if (seg_cur.empty()) {
            p_goal_curr_ = p_obs_curr_;
            back_path_curr_.assign(
                ref_path_.begin() + std::max(0, current_path_idx_),
                ref_path_.end());
          } else {
            p_goal_curr_ = seg_cur.back();
            back_path_curr_.assign(
                ref_path_.begin() +
                    std::min((int)ref_path_.size(), idx_e_cur_ + 1),
                ref_path_.end());
          }
          finished_ = false;
        }
      }

      // 종료 조건: 회피 중심점 근처까지 오면
      if ((pos_gen_ - p_obs_curr_).norm() <= 1.19) {
        finished_ = true;

        // 백 경로 재결합
        if (!back_path_curr_.empty()) {
          Vector2d tail = path_front_.back();
          Vector2d head = back_path_curr_.front();
          if ((tail - head).norm() < 0.25) {
            path_front_.insert(path_front_.end(),
                               back_path_curr_.begin(),
                               back_path_curr_.end());
          } else {
            // 선형 보간 뒤 붙이기
            const int M = 10;
            for (int i = 1; i < M - 1; ++i) {
              double t = static_cast<double>(i) / (M - 1);
              path_front_.push_back(tail + t * (head - tail));
            }
            path_front_.insert(path_front_.end(),
                               back_path_curr_.begin(),
                               back_path_curr_.end());
          }
        }

        avoid_ = false;
        finished_ = false;
        if (next_obs_ptr_ < total_obs_) next_obs_ptr_++;
        break;
      }
    }
  }

  // ----- 소스 경로 선택 → 슬라이스 → quarticFit → 퍼블리시 -----
  const auto & src_path = (avoid_ ? path_front_ : ref_path_);
  if (src_path.size() < 2) return;

  int i0 = nearestIndex(src_path, car_pos);
  int i1 = std::min((int)src_path.size() - 1, i0 + 8);
  std::vector<Vector2d> slice(src_path.begin() + i0,
                              src_path.begin() + i1 + 1);

  (void)current_speed_; // 필요시 v_for_fit 등으로 활용 가능
  auto local_curve = quarticFit(slice, car_pos, (int)slice.size());
  publishPath(local_curve_pub_, local_curve, "map");
}

// ===================== Helpers =====================

int TrajectoryPlanNode::nearestIndex(
    const std::vector<Vector2d> & path,
    const Vector2d & p)
{
  if (path.empty()) return 0;
  int idx = 0;
  double best = std::numeric_limits<double>::infinity();
  for (int i = 0; i < (int)path.size(); ++i) {
    double d = (path[i] - p).norm();
    if (d < best) {
      best = d;
      idx = i;
    }
  }
  return idx;
}

double TrajectoryPlanNode::minDistanceToPath(
    const std::vector<Vector2d> & path,
    const Vector2d & p)
{
  if (path.empty()) return std::numeric_limits<double>::infinity();
  double best = std::numeric_limits<double>::infinity();
  for (const auto & q : path) {
    double d = (q - p).norm();
    if (d < best) best = d;
  }
  return best;
}

std::tuple<Vector2d, double, double, bool>
TrajectoryPlanNode::nextMicroFront(
    const Vector2d & p, double psi, const Vector2d & p_obs,
    const std::vector<double> & dpsi_set, double s_step,
    const Vector2d & goal, const std::vector<Vector2d> & path_ref)
{
  Vector2d head(std::cos(psi), std::sin(psi));
  bool ok = false;
  double best = std::numeric_limits<double>::infinity();
  Vector2d p2 = p;
  double psi2 = psi;

  for (double d : dpsi_set) {
    double psi_c = psi + d;
    Vector2d step(std::cos(psi_c), std::sin(psi_c));
    Vector2d pc = p + s_step * step;

    // 진행 방향이 뒤로 가지 않도록
    if ((pc - p).dot(head) <= 0.0) continue;

    double clr = (pc - p_obs).norm() - 0.40;
    if (clr <= 0.4) continue;

    double d_path = minDistanceToPath(path_ref, pc);
    if (d_path > 1.5) continue;

    double J = 1.0 * (pc - goal).norm()
             + 0.5 / std::max(1e-3, clr)
             + 0.3 * d_path;

    if (J < best) {
      best = J;
      p2 = pc;
      psi2 = psi_c;
      ok = true;
    }
  }

  if (!ok) {
    s_step *= 0.7;
  }
  return {p2, psi2, s_step, ok};
}

std::vector<Vector2d>
TrajectoryPlanNode::findAvoid(
    const std::vector<Vector2d> & ref_path,
    const Vector2d & p_obs,
    int & i1, int & i2)
{
  const double R = 0.6;

  // 1) 글로벌 경로 스냅샷
  std::vector<Vector2d> G;
  {
    std::lock_guard<std::mutex> lk(map_mtx_);
    G = global_path_;
  }

  // 2) 글로벌에서 영향 구간 찾기
  if (!G.empty()) {
    int i1g = -1, i2g = -1;
    for (int k = 0; k < (int)G.size(); ++k) {
      if ((G[k] - p_obs).norm() < R) {
        if (i1g == -1) i1g = k;
        i2g = k;
      }
    }

    if (i1g != -1) {
      i1g = std::max(0, i1g - 5);
      i2g = std::min((int)G.size() - 1, i2g + 5);

      // 3) 글로벌 i1g/i2g를 로컬 ref_path 인덱스로 매핑
      int i1l = nearestIndex(ref_path, G[i1g]);
      int i2l = nearestIndex(ref_path, G[i2g]);
      if (i1l > i2l) std::swap(i1l, i2l);

      i1 = std::max(0, i1l);
      i2 = std::min((int)ref_path.size() - 1, i2l);

      std::vector<Vector2d> seg;
      if (!ref_path.empty() && i2 >= i1) {
        seg.insert(seg.end(),
                   ref_path.begin() + i1,
                   ref_path.begin() + i2 + 1);
      }
      return seg;
    }
    // 글로벌에선 히트가 없으면 아래 로컬 fallback
  }

  // 4) fallback: ref_path만으로 영향 구간 찾기
  std::vector<int> hit;
  hit.reserve(ref_path.size());
  for (int k = 0; k < (int)ref_path.size(); ++k) {
    if ((ref_path[k] - p_obs).norm() < R) {
      hit.push_back(k);
    }
  }

  if (hit.empty()) {
    i1 = 0;
    i2 = 0;
    return {};
  }
  i1 = std::max(hit.front() - 5, 0);
  i2 = std::min(hit.back() + 5, (int)ref_path.size() - 1);

  std::vector<Vector2d> seg;
  seg.insert(seg.end(),
             ref_path.begin() + i1,
             ref_path.begin() + i2 + 1);
  return seg;
}

std::vector<Vector2d>
TrajectoryPlanNode::quarticFit(
    const std::vector<Vector2d> & path_ref,
    const Vector2d & p,
    int maxPts)
{
  std::vector<Vector2d> P;
  std::vector<double> S;
  if (path_ref.empty()) return P;

  int ind_start = nearestIndex(path_ref, p);

  P.reserve(maxPts);
  S.reserve(maxPts);
  P.push_back(p);
  S.push_back(0.0);

  for (int i = 0; i < maxPts - 1; ++i) {
    int idx = ind_start + i;
    if (idx >= (int)path_ref.size()) break;
    P.push_back(path_ref[idx]);
    S.push_back((i + 1) * 0.04);
  }
  int M = (int)P.size();
  if (M < 5) return P;

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
  std::vector<Vector2d> fit;
  fit.reserve(Nd);
  for (int i = 0; i < Nd; ++i) {
    double s = S.back() * (double(i) / double(Nd - 1));
    Eigen::RowVectorXd bb(5);
    bb << 1.0, s, s * s, s * s * s, s * s * s * s;
    fit.emplace_back((bb * cx)(0), (bb * cy)(0));
  }
  return fit;
}

void TrajectoryPlanNode::publishPath(
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub,
    const std::vector<Vector2d> & pts,
    const std::string & frame_id)
{
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
  pub->publish(path);
}

// ===================== main =====================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryPlanNode>());
  rclcpp::shutdown();
  return 0;
}

