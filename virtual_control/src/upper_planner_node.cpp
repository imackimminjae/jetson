#include "virtual_control/upper_planner_node.hpp"

#include "QuadraticProblem.h"
#include "matrix_utils.h"

#include <Eigen/Dense>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace imac_ctrl
{

SdMapUpperPlanner::SdMapUpperPlanner(const SdMapUpperPlannerConfig & config)
: config_(config)
{
}

void SdMapUpperPlanner::setConfig(const SdMapUpperPlannerConfig & config)
{
  config_ = config;
}

const SdMapUpperPlannerConfig & SdMapUpperPlanner::config() const
{
  return config_;
}

void SdMapUpperPlanner::setMap(const SdMap & map)
{
  map_ = map;
}

const SdMap & SdMapUpperPlanner::map() const
{
  return map_;
}

bool SdMapUpperPlanner::hasMap() const
{
  return !map_.edges.empty();
}

double SdMapUpperPlanner::clampd(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

std::vector<double> SdMapUpperPlanner::buildArclength(const std::vector<Eigen::Vector2d> & pts)
{
  std::vector<double> s(pts.size(), 0.0);
  for (size_t i = 1; i < pts.size(); ++i) {
    s[i] = s[i - 1] + (pts[i] - pts[i - 1]).norm();
  }
  return s;
}

double SdMapUpperPlanner::polylineLength(const std::vector<Eigen::Vector2d> & pts)
{
  if (pts.size() < 2) {
    return 0.0;
  }
  double total = 0.0;
  for (size_t i = 1; i < pts.size(); ++i) {
    total += (pts[i] - pts[i - 1]).norm();
  }
  return total;
}

std::vector<Eigen::Vector2d> SdMapUpperPlanner::dedupPolyline(
  const std::vector<Eigen::Vector2d> & pts)
{
  if (pts.size() < 2) {
    return pts;
  }
  std::vector<Eigen::Vector2d> out;
  out.reserve(pts.size());
  out.push_back(pts.front());
  for (size_t i = 1; i < pts.size(); ++i) {
    if ((pts[i] - out.back()).norm() > 1e-9) {
      out.push_back(pts[i]);
    }
  }
  return out;
}

Eigen::Vector2d SdMapUpperPlanner::samplePolylineByArclength(
  const std::vector<Eigen::Vector2d> & pts,
  double s_query)
{
  if (pts.empty()) {
    return Eigen::Vector2d::Zero();
  }

  const auto clean = dedupPolyline(pts);
  if (clean.size() == 1) {
    return clean.front();
  }

  const auto s = buildArclength(clean);
  const double q = clampd(s_query, 0.0, s.back());
  for (size_t i = 0; i + 1 < clean.size(); ++i) {
    if (q <= s[i + 1] + 1e-9) {
      const double ds = std::max(1e-9, s[i + 1] - s[i]);
      const double tau = clampd((q - s[i]) / ds, 0.0, 1.0);
      return clean[i] + tau * (clean[i + 1] - clean[i]);
    }
  }
  return clean.back();
}

Eigen::Vector2d SdMapUpperPlanner::samplePolylineTangent(
  const std::vector<Eigen::Vector2d> & pts,
  double s_query)
{
  if (pts.size() < 2) {
    return Eigen::Vector2d::UnitX();
  }
  const auto s = buildArclength(pts);
  const double total = s.back();
  const double q = clampd(s_query, 0.0, total);
  const double ds = std::min(0.5, std::max(0.05, 0.1 * std::max(1e-3, total)));
  const double s1 = std::max(0.0, q - ds);
  const double s2 = std::min(total, q + ds);
  Eigen::Vector2d tangent = samplePolylineByArclength(pts, s2) - samplePolylineByArclength(pts, s1);
  if (tangent.norm() < 1e-9) {
    tangent = pts.back() - pts.front();
  }
  if (tangent.norm() < 1e-9) {
    return Eigen::Vector2d::UnitX();
  }
  return tangent.normalized();
}

std::vector<Eigen::Vector2d> SdMapUpperPlanner::resamplePolyline(
  const std::vector<Eigen::Vector2d> & pts,
  double ds)
{
  const auto clean = dedupPolyline(pts);
  if (clean.size() < 2) {
    return clean;
  }
  const auto s = buildArclength(clean);
  const double total = s.back();
  const double step = std::max(1e-3, ds);
  std::vector<Eigen::Vector2d> out;
  for (double q = 0.0; q < total; q += step) {
    out.push_back(samplePolylineByArclength(clean, q));
  }
  out.push_back(clean.back());
  return dedupPolyline(out);
}

std::vector<Eigen::Vector2d> SdMapUpperPlanner::trimPolylineFromS(
  const std::vector<Eigen::Vector2d> & pts,
  double s0)
{
  if (pts.size() < 2) {
    return pts;
  }
  const auto s = buildArclength(pts);
  const double q0 = clampd(s0, 0.0, s.back());

  std::vector<Eigen::Vector2d> out;
  out.push_back(samplePolylineByArclength(pts, q0));
  for (size_t i = 0; i < pts.size(); ++i) {
    if (s[i] > q0 + 1e-9) {
      out.push_back(pts[i]);
    }
  }
  return dedupPolyline(out);
}

SdMapUpperPlanner::Projection SdMapUpperPlanner::projectPointToPolyline(
  const std::vector<Eigen::Vector2d> & pts,
  const Eigen::Vector2d & point_world)
{
  Projection proj;
  if (pts.empty()) {
    return proj;
  }
  if (pts.size() == 1) {
    proj.point = pts.front();
    proj.dist = (pts.front() - point_world).norm();
    proj.s_on_edge = 0.0;
    proj.tangent = Eigen::Vector2d::UnitX();
    return proj;
  }

  const auto s = buildArclength(pts);
  double best_dist = std::numeric_limits<double>::infinity();

  for (size_t k = 0; k + 1 < pts.size(); ++k) {
    const Eigen::Vector2d a = pts[k];
    const Eigen::Vector2d b = pts[k + 1];
    const Eigen::Vector2d ab = b - a;
    const double lab2 = ab.squaredNorm();
    if (lab2 < 1e-12) {
      continue;
    }
    double tau = (point_world - a).dot(ab) / lab2;
    tau = clampd(tau, 0.0, 1.0);
    const Eigen::Vector2d q = a + tau * ab;
    const double d = (q - point_world).norm();
    if (d < best_dist) {
      best_dist = d;
      proj.point = q;
      proj.dist = d;
      proj.s_on_edge = s[k] + tau * std::sqrt(lab2);
      proj.tangent = ab.normalized();
    }
  }
  return proj;
}

std::vector<Eigen::Vector2d> SdMapUpperPlanner::worldToBody(
  const std::vector<Eigen::Vector2d> & pts_world,
  const Eigen::Vector2d & ego_pos_world,
  double ego_yaw_rad)
{
  std::vector<Eigen::Vector2d> out;
  out.reserve(pts_world.size());
  const double c = std::cos(ego_yaw_rad);
  const double s = std::sin(ego_yaw_rad);
  for (const auto & p_world : pts_world) {
    const Eigen::Vector2d d = p_world - ego_pos_world;
    out.emplace_back(c * d.x() + s * d.y(), -s * d.x() + c * d.y());
  }
  return out;
}

Eigen::Vector2d SdMapUpperPlanner::worldToBodyPoint(
  const Eigen::Vector2d & p_world,
  const Eigen::Vector2d & ego_pos_world,
  double ego_yaw_rad)
{
  const double c = std::cos(ego_yaw_rad);
  const double s = std::sin(ego_yaw_rad);
  const Eigen::Vector2d d = p_world - ego_pos_world;
  return Eigen::Vector2d(c * d.x() + s * d.y(), -s * d.x() + c * d.y());
}

Eigen::Vector2d SdMapUpperPlanner::bodyToWorldPoint(
  const Eigen::Vector2d & p_body,
  const Eigen::Vector2d & ego_pos_world,
  double ego_yaw_rad)
{
  const double c = std::cos(ego_yaw_rad);
  const double s = std::sin(ego_yaw_rad);
  return ego_pos_world + Eigen::Vector2d(
    c * p_body.x() - s * p_body.y(),
    s * p_body.x() + c * p_body.y());
}

Eigen::Vector2d SdMapUpperPlanner::rotateWorldToBodyVector(
  const Eigen::Vector2d & v_world,
  double ego_yaw_rad)
{
  const double c = std::cos(ego_yaw_rad);
  const double s = std::sin(ego_yaw_rad);
  return Eigen::Vector2d(c * v_world.x() + s * v_world.y(),
                         -s * v_world.x() + c * v_world.y());
}

std::vector<double> SdMapUpperPlanner::computeHeadingRad(
  const std::vector<Eigen::Vector2d> & body_pts)
{
  if (body_pts.empty()) {
    return {};
  }
  std::vector<double> psi(body_pts.size(), 0.0);
  if (body_pts.size() == 1) {
    return psi;
  }
  for (size_t k = 0; k + 1 < body_pts.size(); ++k) {
    const Eigen::Vector2d dp = body_pts[k + 1] - body_pts[k];
    if (dp.norm() < 1e-9) {
      psi[k] = (k == 0) ? 0.0 : psi[k - 1];
    } else {
      psi[k] = std::atan2(dp.y(), dp.x());
    }
  }
  psi.back() = psi[psi.size() - 2];
  return psi;
}

int SdMapUpperPlanner::findClosestPointIndex(
  const std::vector<Eigen::Vector2d> & pts_world,
  const Eigen::Vector2d & query_world)
{
  if (pts_world.empty()) {
    return -1;
  }
  int best_idx = 0;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < pts_world.size(); ++i) {
    const double d2 = (pts_world[i] - query_world).squaredNorm();
    if (d2 < best_d2) {
      best_d2 = d2;
      best_idx = static_cast<int>(i);
    }
  }
  return best_idx;
}

int SdMapUpperPlanner::edgeIdToIndex(int edge_id) const
{
  for (size_t i = 0; i < map_.edges.size(); ++i) {
    if (map_.edges[i].id == edge_id) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

Eigen::Vector2d SdMapUpperPlanner::goalWorld() const
{
  if (map_.goal_xy) {
    return *map_.goal_xy;
  }
  const int i_goal = findGoalEdgeIndex();
  return map_.edges[static_cast<size_t>(i_goal)].centerline.back();
}

bool SdMapUpperPlanner::goalReached(const Eigen::Vector2d & ego_pos_world, double tol) const
{
  return (goalWorld() - ego_pos_world).norm() <= tol;
}

int SdMapUpperPlanner::findGoalEdgeIndex() const
{
  if (map_.edges.empty()) {
    throw std::runtime_error("sdmap is empty");
  }
  if (map_.goal_edge_id) {
    const int idx = edgeIdToIndex(*map_.goal_edge_id);
    if (idx < 0) {
      throw std::runtime_error("sdmap.goal_edge_id does not exist in edges");
    }
    return idx;
  }
  if (map_.goal_xy) {
    double best_dist = std::numeric_limits<double>::infinity();
    int best_idx = -1;
    for (size_t i = 0; i < map_.edges.size(); ++i) {
      if (map_.edges[i].centerline.size() < 2) {
        continue;
      }
      const auto proj = projectPointToPolyline(map_.edges[i].centerline, *map_.goal_xy);
      if (proj.dist < best_dist) {
        best_dist = proj.dist;
        best_idx = static_cast<int>(i);
      }
    }
    if (best_idx < 0) {
      throw std::runtime_error("failed to infer goal edge from goal_xy");
    }
    return best_idx;
  }
  throw std::runtime_error("sdmap must contain a valid non-negative goal_edge_id or goal_xy");
}

int SdMapUpperPlanner::findCurrentEdge(
  const Eigen::Vector2d & ego_pos_world,
  double ego_yaw_rad,
  Projection * proj_out) const
{
  if (map_.edges.empty()) {
    throw std::runtime_error("sdmap is empty");
  }
  const Eigen::Vector2d h_ego(std::cos(ego_yaw_rad), std::sin(ego_yaw_rad));

  int best_idx = -1;
  double best_cost = std::numeric_limits<double>::infinity();
  Projection best_proj;

  for (size_t i = 0; i < map_.edges.size(); ++i) {
    if (map_.edges[i].centerline.size() < 2) {
      continue;
    }
    const auto proj = projectPointToPolyline(map_.edges[i].centerline, ego_pos_world);
    const Eigen::Vector2d h_edge = proj.tangent.normalized();
    const double heading_dot = clampd(h_ego.dot(h_edge), -1.0, 1.0);
    const double heading_cost = 1.0 - heading_dot;
    const double J = proj.dist * proj.dist + config_.heading_weight * heading_cost * heading_cost;
    if (J < best_cost) {
      best_cost = J;
      best_idx = static_cast<int>(i);
      best_proj = proj;
    }
  }

  if (best_idx < 0) {
    throw std::runtime_error("failed to identify the current edge from ego pose");
  }
  if (proj_out) {
    *proj_out = best_proj;
  }
  return best_idx;
}

bool SdMapUpperPlanner::searchEdgePath(
  int i_start,
  int i_goal,
  std::vector<int> * path_idx) const
{
  if (!path_idx) {
    return false;
  }
  path_idx->clear();

  const int N = static_cast<int>(map_.edges.size());
  if (i_start < 0 || i_goal < 0 || i_start >= N || i_goal >= N) {
    return false;
  }
  if (i_start == i_goal) {
    *path_idx = {i_start};
    return true;
  }

  std::vector<double> dist(static_cast<size_t>(N), std::numeric_limits<double>::infinity());
  std::vector<int> prev(static_cast<size_t>(N), -1);
  std::vector<bool> visited(static_cast<size_t>(N), false);
  dist[static_cast<size_t>(i_start)] = 0.0;

  while (true) {
    double best = std::numeric_limits<double>::infinity();
    int u = -1;
    for (int i = 0; i < N; ++i) {
      if (!visited[static_cast<size_t>(i)] && dist[static_cast<size_t>(i)] < best) {
        best = dist[static_cast<size_t>(i)];
        u = i;
      }
    }
    if (u < 0 || !std::isfinite(best)) {
      break;
    }
    if (u == i_goal) {
      break;
    }
    visited[static_cast<size_t>(u)] = true;

    for (int succ_id : map_.edges[static_cast<size_t>(u)].successors) {
      const int v = edgeIdToIndex(succ_id);
      if (v < 0) {
        continue;
      }
      const double alt = dist[static_cast<size_t>(u)] +
        polylineLength(map_.edges[static_cast<size_t>(v)].centerline);
      if (alt < dist[static_cast<size_t>(v)]) {
        dist[static_cast<size_t>(v)] = alt;
        prev[static_cast<size_t>(v)] = u;
      }
    }
  }

  if (!std::isfinite(dist[static_cast<size_t>(i_goal)])) {
    return false;
  }

  std::vector<int> seq;
  int u = i_goal;
  seq.push_back(u);
  while (u != i_start && prev[static_cast<size_t>(u)] >= 0) {
    u = prev[static_cast<size_t>(u)];
    seq.push_back(u);
  }
  if (seq.empty() || seq.back() != i_start) {
    return false;
  }
  std::reverse(seq.begin(), seq.end());
  *path_idx = seq;
  return true;
}

bool SdMapUpperPlanner::buildRouteEdgePath(
  int i_start,
  std::vector<int> * path_idx) const
{
  if (!path_idx) {
    return false;
  }
  path_idx->clear();

  if (map_.route_edge_ids.empty() ||
      i_start < 0 ||
      i_start >= static_cast<int>(map_.edges.size()))
  {
    return false;
  }

  const int start_edge_id = map_.edges[static_cast<size_t>(i_start)].id;
  auto it = std::find(map_.route_edge_ids.begin(), map_.route_edge_ids.end(), start_edge_id);
  if (it == map_.route_edge_ids.end()) {
    return false;
  }

  for (; it != map_.route_edge_ids.end(); ++it) {
    const int idx = edgeIdToIndex(*it);
    if (idx < 0) {
      path_idx->clear();
      return false;
    }
    if (path_idx->empty() || path_idx->back() != idx) {
      path_idx->push_back(idx);
    }
  }

  return !path_idx->empty();
}

void SdMapUpperPlanner::enumerateCandidatePaths(
  int i_start,
  int max_depth,
  std::vector<std::vector<int>> * paths_out) const
{
  if (!paths_out) {
    return;
  }
  paths_out->clear();

  std::function<void(std::vector<int>, int)> dfs =
    [&](std::vector<int> cur_path, int depth) {
      const int cur = cur_path.back();
      const auto & succ_ids = map_.edges[static_cast<size_t>(cur)].successors;
      if (succ_ids.empty() || depth >= max_depth) {
        paths_out->push_back(cur_path);
        return;
      }
      bool progressed = false;
      for (int succ_id : succ_ids) {
        const int nxt = edgeIdToIndex(succ_id);
        if (nxt < 0) {
          continue;
        }
        if (std::find(cur_path.begin(), cur_path.end(), nxt) != cur_path.end()) {
          continue;
        }
        progressed = true;
        auto next = cur_path;
        next.push_back(nxt);
        dfs(next, depth + 1);
      }
      if (!progressed) {
        paths_out->push_back(cur_path);
      }
    };

  dfs({i_start}, 1);
}

std::vector<Eigen::Vector2d> SdMapUpperPlanner::concatenateEdgePath(
  const std::vector<int> & path_idx,
  int i_start,
  double s_on_start_edge) const
{
  std::vector<Eigen::Vector2d> path_world;
  for (int idx : path_idx) {
    if (idx < 0 || idx >= static_cast<int>(map_.edges.size())) {
      continue;
    }
    std::vector<Eigen::Vector2d> pts = map_.edges[static_cast<size_t>(idx)].centerline;
    if (pts.size() < 2) {
      continue;
    }
    if (idx == i_start) {
      pts = trimPolylineFromS(pts, s_on_start_edge);
    }
    if (pts.empty()) {
      continue;
    }

    if (path_world.empty()) {
      path_world = pts;
    } else if ((path_world.back() - pts.front()).norm() < 1e-6) {
      path_world.insert(path_world.end(), pts.begin() + 1, pts.end());
    } else {
      path_world.insert(path_world.end(), pts.begin(), pts.end());
    }
  }
  return dedupPolyline(path_world);
}

SdMapUpperPlanner::CandidatePath SdMapUpperPlanner::buildCandidatePathFromEdgePath(
  const std::vector<int> & idx_seq,
  int i_start,
  double s_on_start_edge) const
{
  if (idx_seq.empty()) {
    throw std::runtime_error("candidate edge path is empty");
  }

  CandidatePath cand;
  cand.path_idx = idx_seq;
  cand.path_world = resamplePolyline(
    concatenateEdgePath(idx_seq, i_start, s_on_start_edge),
    config_.path_resample_ds);
  if (cand.path_world.size() < 2) {
    throw std::runtime_error("candidate path became too short after trimming/resampling");
  }
  cand.length = polylineLength(cand.path_world);
  cand.terminal_edge_id = map_.edges[static_cast<size_t>(idx_seq.back())].id;

  double min_width = std::numeric_limits<double>::infinity();
  for (int idx : idx_seq) {
    const double w = map_.edges[static_cast<size_t>(idx)].lane_width > 0.0 ?
      map_.edges[static_cast<size_t>(idx)].lane_width : config_.lane_width_default;
    min_width = std::min(min_width, w);
  }
  cand.lane_width = std::isfinite(min_width) ? min_width : config_.lane_width_default;
  return cand;
}

std::vector<SdMapUpperPlanner::CandidatePath> SdMapUpperPlanner::buildCandidatePaths(
  int i_start,
  double s_on_start_edge) const
{
  std::vector<std::vector<int>> cand_idx;
  enumerateCandidatePaths(i_start, config_.max_branch_depth, &cand_idx);
  if (cand_idx.empty()) {
    throw std::runtime_error("no candidate edge paths were enumerated");
  }

  std::vector<CandidatePath> candidates;
  for (const auto & idx_seq : cand_idx) {
    try {
      candidates.push_back(buildCandidatePathFromEdgePath(idx_seq, i_start, s_on_start_edge));
    } catch (const std::exception &) {
      continue;
    }
  }

  if (candidates.empty()) {
    throw std::runtime_error("all candidate paths became invalid after trimming/resampling");
  }
  return candidates;
}

SdMapUpperPreviewData SdMapUpperPlanner::makePreviewConstraints(
  const Eigen::Vector2d & ego_pos_world,
  double ego_yaw_rad,
  int prev_terminal_edge_id) const
{
  if (map_.edges.empty()) {
    throw std::runtime_error("sdmap.edges is missing or empty");
  }

  Projection proj_start;
  const int i_start = findCurrentEdge(ego_pos_world, ego_yaw_rad, &proj_start);
  const Eigen::Vector2d goal_world = goalWorld();

  std::vector<int> goal_path_idx;
  const bool using_route = buildRouteEdgePath(i_start, &goal_path_idx);
  if (!using_route) {
    const int i_goal = findGoalEdgeIndex();
    const bool ok_goal = searchEdgePath(i_start, i_goal, &goal_path_idx);
    if (!ok_goal || goal_path_idx.empty()) {
      std::ostringstream oss;
      oss << "no reachable path exists from current edge " << map_.edges[static_cast<size_t>(i_start)].id
          << " to goal edge " << map_.edges[static_cast<size_t>(i_goal)].id;
      throw std::runtime_error(oss.str());
    }
  }

  auto goal_path_world = concatenateEdgePath(goal_path_idx, i_start, proj_start.s_on_edge);
  goal_path_world = resamplePolyline(goal_path_world, config_.path_resample_ds);
  if (goal_path_world.size() < 2) {
    throw std::runtime_error("goal-directed path is shorter than two samples after trimming/resampling");
  }

  std::vector<CandidatePath> candidates;
  if (using_route) {
    candidates.push_back(buildCandidatePathFromEdgePath(
      goal_path_idx, i_start, proj_start.s_on_edge));
  } else {
    candidates = buildCandidatePaths(i_start, proj_start.s_on_edge);
  }

  double max_len = 0.0;
  for (const auto & cand : candidates) {
    max_len = std::max(max_len, cand.length);
  }
  double lookahead = std::min(config_.lookahead_max, max_len);
  lookahead = std::max(
    std::min(lookahead, max_len),
    std::min(config_.lookahead_min, max_len));
  if (lookahead < config_.min_required_lookahead) {
    lookahead = max_len;
  }
  if (lookahead <= 1e-6) {
    std::ostringstream oss;
    oss << "maximum candidate length " << max_len << " m is too short for preview generation";
    throw std::runtime_error(oss.str());
  }

  SdMapUpperPreviewData out;
  out.N_pred = std::max(2, static_cast<int>(std::round(lookahead / config_.ds_nominal)));
  std::vector<double> s_grid(static_cast<size_t>(out.N_pred + 1), 0.0);
  if (out.N_pred > 0) {
    for (int k = 0; k <= out.N_pred; ++k) {
      s_grid[static_cast<size_t>(k)] = lookahead * static_cast<double>(k) / static_cast<double>(out.N_pred);
    }
    out.ds_eff = s_grid[1] - s_grid[0];
  }

  out.prk.reserve(static_cast<size_t>(out.N_pred + 1));
  std::vector<Eigen::Vector2d> prk_world;
  prk_world.reserve(static_cast<size_t>(out.N_pred + 1));
  for (int k = 0; k <= out.N_pred; ++k) {
    prk_world.push_back(samplePolylineByArclength(goal_path_world, s_grid[static_cast<size_t>(k)]));
  }
  out.prk = worldToBody(prk_world, ego_pos_world, ego_yaw_rad);
  out.psirk_rad = computeHeadingRad(out.prk);

  out.mk.resize(static_cast<size_t>(out.N_pred + 1), 0.0);
  out.nk.resize(static_cast<size_t>(out.N_pred + 1), 0.0);
  out.ck.resize(static_cast<size_t>(out.N_pred + 1), 0.0);
  for (int k = 0; k <= out.N_pred; ++k) {
    out.mk[static_cast<size_t>(k)] = std::cos(out.psirk_rad[static_cast<size_t>(k)]);
    out.nk[static_cast<size_t>(k)] = std::sin(out.psirk_rad[static_cast<size_t>(k)]);
    out.ck[static_cast<size_t>(k)] = -(
      out.mk[static_cast<size_t>(k)] * out.prk[static_cast<size_t>(k)].x() +
      out.nk[static_cast<size_t>(k)] * out.prk[static_cast<size_t>(k)].y());
  }

  out.pmk.resize(static_cast<size_t>(out.N_pred + 1));
  out.pMk.resize(static_cast<size_t>(out.N_pred + 1));
  out.branch_info.resize(static_cast<size_t>(out.N_pred + 1));
  out.Nck.resize(static_cast<size_t>(out.N_pred + 1), 0);

  out.pmk[0].push_back(out.prk[0]);
  out.pMk[0].push_back(out.prk[0]);
  out.branch_info[0].push_back(0);
  out.Nck[0] = 1;

  const int num_cand = static_cast<int>(candidates.size());
  out.branch_cost.assign(static_cast<size_t>(num_cand), 0.0);
  out.candidate_terminal_edge_ids.resize(static_cast<size_t>(num_cand), -1);
  const Eigen::Vector2d goal_body = worldToBodyPoint(goal_world, ego_pos_world, ego_yaw_rad);

  for (int b = 0; b < num_cand; ++b) {
    const Eigen::Vector2d end_body =
      worldToBodyPoint(candidates[static_cast<size_t>(b)].path_world.back(), ego_pos_world, ego_yaw_rad);
    double cost = config_.goal_weight * (end_body - goal_body).norm();
    if (prev_terminal_edge_id >= 0 &&
        candidates[static_cast<size_t>(b)].terminal_edge_id != prev_terminal_edge_id)
    {
      cost += config_.hysteresis_weight;
    }
    if (candidates[static_cast<size_t>(b)].length < lookahead) {
      cost += config_.short_path_weight * (lookahead - candidates[static_cast<size_t>(b)].length);
    }
    out.branch_cost[static_cast<size_t>(b)] = cost;
    out.candidate_terminal_edge_ids[static_cast<size_t>(b)] =
      candidates[static_cast<size_t>(b)].terminal_edge_id;
  }

  for (int k = 1; k <= out.N_pred; ++k) {
    const Eigen::Vector2d pref = out.prk[static_cast<size_t>(k)];
    Eigen::Vector2d q(-out.nk[static_cast<size_t>(k)], out.mk[static_cast<size_t>(k)]);
    const double qn = q.norm();
    q = (qn < 1e-9) ? Eigen::Vector2d(0.0, 1.0) : q / qn;

    std::vector<double> mids(static_cast<size_t>(num_cand), 0.0);
    std::vector<Eigen::Vector2d> pmin(static_cast<size_t>(num_cand), Eigen::Vector2d::Zero());
    std::vector<Eigen::Vector2d> pmax(static_cast<size_t>(num_cand), Eigen::Vector2d::Zero());
    std::vector<int> labels(static_cast<size_t>(num_cand), 0);

    for (int b = 0; b < num_cand; ++b) {
      labels[static_cast<size_t>(b)] = b;
      const Eigen::Vector2d c_world =
        samplePolylineByArclength(candidates[static_cast<size_t>(b)].path_world, s_grid[static_cast<size_t>(k)]);
      const Eigen::Vector2d c_body = worldToBodyPoint(c_world, ego_pos_world, ego_yaw_rad);

      Eigen::Vector2d t_world =
        samplePolylineTangent(candidates[static_cast<size_t>(b)].path_world, s_grid[static_cast<size_t>(k)]);
      Eigen::Vector2d t_body = rotateWorldToBodyVector(t_world, ego_yaw_rad);
      if (t_body.norm() < 1e-9) {
        t_body = Eigen::Vector2d(
          std::cos(out.psirk_rad[static_cast<size_t>(k)]),
          std::sin(out.psirk_rad[static_cast<size_t>(k)]));
      } else {
        t_body.normalize();
      }

      const Eigen::Vector2d n_body(-t_body.y(), t_body.x());
      const double half_width = std::max(
        config_.min_half_width,
        0.5 * candidates[static_cast<size_t>(b)].lane_width - config_.corridor_margin);

      const double a = n_body.dot(pref - c_body);
      const double bcoef = n_body.dot(q);
      const double lambda_center = q.dot(c_body - pref);

      double lam1 = 0.0;
      double lam2 = 0.0;
      if (std::abs(bcoef) < config_.parallel_eps) {
        const double half_span = std::min(config_.parallel_interval_half_len, half_width);
        lam1 = lambda_center - half_span;
        lam2 = lambda_center + half_span;
      } else {
        lam1 = (-half_width - a) / bcoef;
        lam2 = ( half_width - a) / bcoef;
        if (lam1 > lam2) {
          std::swap(lam1, lam2);
        }
        if ((lam2 - lam1) > 2.0 * config_.max_interval_half_len) {
          lam1 = lambda_center - config_.max_interval_half_len;
          lam2 = lambda_center + config_.max_interval_half_len;
        }
      }

      pmin[static_cast<size_t>(b)] = pref + lam1 * q;
      pmax[static_cast<size_t>(b)] = pref + lam2 * q;
      mids[static_cast<size_t>(b)] = 0.5 * (lam1 + lam2);
    }

    std::vector<int> order(static_cast<size_t>(num_cand), 0);
    for (int i = 0; i < num_cand; ++i) {
      order[static_cast<size_t>(i)] = i;
    }
    std::sort(order.begin(), order.end(), [&](int lhs, int rhs) {
      return mids[static_cast<size_t>(lhs)] < mids[static_cast<size_t>(rhs)];
    });

    out.pmk[static_cast<size_t>(k)].reserve(static_cast<size_t>(num_cand));
    out.pMk[static_cast<size_t>(k)].reserve(static_cast<size_t>(num_cand));
    out.branch_info[static_cast<size_t>(k)].reserve(static_cast<size_t>(num_cand));
    for (int ord_idx : order) {
      out.pmk[static_cast<size_t>(k)].push_back(pmin[static_cast<size_t>(ord_idx)]);
      out.pMk[static_cast<size_t>(k)].push_back(pmax[static_cast<size_t>(ord_idx)]);
      out.branch_info[static_cast<size_t>(k)].push_back(labels[static_cast<size_t>(ord_idx)]);
    }
    out.Nck[static_cast<size_t>(k)] = num_cand;
  }

  return out;
}

std::pair<std::vector<Eigen::Vector2d>, SdMapUpperMiqpDebug> SdMapUpperPlanner::solveUpperMiqp(
  const SdMapUpperPreviewData & preview,
  double vr,
  double Ts,
  double a_M,
  double phi_M,
  double big_m,
  double v1) const
{
  if (preview.N_pred < 1) {
    throw std::runtime_error("upper MIQP: N_pred must be at least 1");
  }
  if (preview.Nck.size() < static_cast<size_t>(preview.N_pred + 1)) {
    throw std::runtime_error("upper MIQP: Nck size is inconsistent with N_pred");
  }

  int ny = 0;
  for (int k = 1; k <= preview.N_pred; ++k) {
    if (preview.Nck[static_cast<size_t>(k)] <= 0) {
      throw std::runtime_error("upper MIQP: one of the preview steps has zero intervals");
    }
    ny += preview.Nck[static_cast<size_t>(k)];
  }
  if (ny <= 0) {
    throw std::runtime_error("upper MIQP: zero interval binaries");
  }

  const int nz = static_cast<int>(preview.branch_cost.size());
  if (nz <= 0) {
    throw std::runtime_error("upper MIQP: zero branch binaries");
  }

  const int cont_dim = 2 * preview.N_pred;
  const int y_offset = cont_dim;
  const int z_offset = cont_dim + ny;
  const int nv = cont_dim + ny + nz;

  std::vector<Matrix<double>> B(preview.N_pred + 1);
  for (int k = 0; k <= preview.N_pred; ++k) {
    Matrix<double> Bk;
    Bk.resize(0.0, 2, 2);
    const double psi = preview.psirk_rad[static_cast<size_t>(k)];
    Bk[0][0] = Ts * std::cos(psi);
    Bk[0][1] = -Ts * vr * std::sin(psi);
    Bk[1][0] = Ts * std::sin(psi);
    Bk[1][1] = Ts * vr * std::cos(psi);
    B[static_cast<size_t>(k)] = Bk;
  }

  Matrix<double> G;
  G.resize(0.0, 2 * (preview.N_pred + 1), nv);
  for (int k = 1; k <= preview.N_pred; ++k) {
    const int row = 2 * k;
    for (int j = 0; j < k; ++j) {
      G[row + 0][2 * j + 0] = B[static_cast<size_t>(j)][0][0];
      G[row + 0][2 * j + 1] = B[static_cast<size_t>(j)][0][1];
      G[row + 1][2 * j + 0] = B[static_cast<size_t>(j)][1][0];
      G[row + 1][2 * j + 1] = B[static_cast<size_t>(j)][1][1];
    }
  }

  Matrix<double> Qblk;
  Qblk.resize(0.0, 2 * (preview.N_pred + 1), 2 * (preview.N_pred + 1));
  for (int k = 0; k <= preview.N_pred; ++k) {
    Qblk[2 * k + 0][2 * k + 0] = config_.upper_qx_x;
    Qblk[2 * k + 1][2 * k + 1] = config_.upper_qx_y;
  }

  Matrix<double> Rblk;
  Rblk.resize(0.0, nv, nv);
  for (int k = 0; k < preview.N_pred; ++k) {
    Rblk[2 * k + 0][2 * k + 0] = config_.upper_ru_dv;
    Rblk[2 * k + 1][2 * k + 1] = config_.upper_ru_dpsi;
  }
  for (int i = y_offset; i < nv; ++i) {
    Rblk[i][i] = config_.binary_regularization;
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

  Vector<double> fqp(0.0, nv);
  for (int b = 0; b < nz; ++b) {
    fqp[z_offset + b] = preview.branch_cost[static_cast<size_t>(b)];
  }

  Vector<double> Urk(0.0, cont_dim);
  for (int k = 0; k < preview.N_pred; ++k) {
    const double psi_prev = (k == 0) ? 0.0 : preview.psirk_rad[static_cast<size_t>(k - 1)];
    const double dpsi_ref = preview.psirk_rad[static_cast<size_t>(k)] - psi_prev;
    Urk[2 * k + 0] = 0.0;
    Urk[2 * k + 1] = dpsi_ref;
  }

  std::vector<int> y_step_offset(static_cast<size_t>(preview.N_pred + 1), 0);
  for (int k = 1; k <= preview.N_pred; ++k) {
    y_step_offset[static_cast<size_t>(k)] =
      y_step_offset[static_cast<size_t>(k - 1)] + preview.Nck[static_cast<size_t>(k)];
  }

  std::vector<int> y_branch(static_cast<size_t>(ny), -1);
  Matrix<double> Aineq;
  const int n_ineq = 2 * ny + 4 * preview.N_pred + ny;
  Aineq.resize(0.0, n_ineq, nv);
  Vector<double> bineq(0.0, n_ineq);

  int row_ineq = 0;
  for (int k = 1; k <= preview.N_pred; ++k) {
    const int nck_k = preview.Nck[static_cast<size_t>(k)];
    const auto & pmk_k = preview.pmk[static_cast<size_t>(k)];
    const auto & pMk_k = preview.pMk[static_cast<size_t>(k)];
    const auto & branch_k = preview.branch_info[static_cast<size_t>(k)];

    const double mk_k = preview.mk[static_cast<size_t>(k)];
    const double nk_k = preview.nk[static_cast<size_t>(k)];
    const double ck_k = preview.ck[static_cast<size_t>(k)];

    const double Rc00 = nk_k * nk_k;
    const double Rc01 = -mk_k * nk_k;
    const double Rc10 = -mk_k * nk_k;
    const double Rc11 = mk_k * mk_k;
    const double qc0 = -ck_k * mk_k;
    const double qc1 = -ck_k * nk_k;

    const Eigen::Vector2d & pmin_first = pmk_k.front();
    const Eigen::Vector2d & pmax_last = pMk_k.back();
    const double dx = pmax_last.x() - pmin_first.x();
    const double dy = pmax_last.y() - pmin_first.y();
    const double dc2 = dx * dx + dy * dy;
    if (dc2 < 1e-9) {
      throw std::runtime_error("upper MIQP: degenerate common preview line segment");
    }

    const double RL0 = dx / dc2;
    const double RL1 = dy / dc2;
    const double qL = -(pmin_first.x() * dx + pmin_first.y() * dy) / dc2;

    const double RL_Rc0 = RL0 * Rc00 + RL1 * Rc10;
    const double RL_Rc1 = RL0 * Rc01 + RL1 * Rc11;
    const double RL_qc = RL0 * qc0 + RL1 * qc1;
    const Eigen::Vector2d & prk_k = preview.prk[static_cast<size_t>(k)];
    const double RL_Rc_pr = RL_Rc0 * prk_k.x() + RL_Rc1 * prk_k.y();

    const int state_row = 2 * k;
    for (int i = 0; i < nck_k; ++i) {
      const int ycol = y_step_offset[static_cast<size_t>(k - 1)] + i;
      y_branch[static_cast<size_t>(ycol)] = branch_k[static_cast<size_t>(i)];

      const Eigen::Vector2d & pmik = pmk_k[static_cast<size_t>(i)];
      const Eigen::Vector2d & pMik = pMk_k[static_cast<size_t>(i)];
      const double lMik = RL0 * pMik.x() + RL1 * pMik.y() + qL;
      const double lmik = RL0 * pmik.x() + RL1 * pmik.y() + qL;
      const double hD0 = RL_Rc_pr + RL_qc + qL - lMik;
      const double hD1 = -RL_Rc_pr - RL_qc - qL + lmik;

      for (int c = 0; c < cont_dim; ++c) {
        Aineq[row_ineq][c] =
          RL_Rc0 * G[state_row + 0][c] + RL_Rc1 * G[state_row + 1][c];
      }
      Aineq[row_ineq][y_offset + ycol] = big_m;
      bineq[row_ineq] = big_m - hD0;
      ++row_ineq;

      for (int c = 0; c < cont_dim; ++c) {
        Aineq[row_ineq][c] =
          -RL_Rc0 * G[state_row + 0][c] - RL_Rc1 * G[state_row + 1][c];
      }
      Aineq[row_ineq][y_offset + ycol] = big_m;
      bineq[row_ineq] = big_m - hD1;
      ++row_ineq;
    }
  }

  Matrix<double> HU;
  HU.resize(0.0, 4 * preview.N_pred, cont_dim);
  Vector<double> hu(0.0, 4 * preview.N_pred);
  for (int k = 0; k < preview.N_pred; ++k) {
    HU[k][2 * k + 0] = 1.0;
    HU[preview.N_pred + k][2 * k + 0] = -1.0;
    for (int j = 0; j <= k; ++j) {
      HU[2 * preview.N_pred + k][2 * j + 0] = -phi_M;
      HU[3 * preview.N_pred + k][2 * j + 0] = -phi_M;
    }
    HU[2 * preview.N_pred + k][2 * k + 1] = 1.0;
    HU[3 * preview.N_pred + k][2 * k + 1] = -1.0;

    hu[k] = a_M * Ts;
    hu[preview.N_pred + k] = a_M * Ts;
    hu[2 * preview.N_pred + k] = phi_M * v1;
    hu[3 * preview.N_pred + k] = phi_M * v1;
  }
  Vector<double> hu_shifted(0.0, 4 * preview.N_pred);
  for (int r = 0; r < 4 * preview.N_pred; ++r) {
    double acc = 0.0;
    for (int c = 0; c < cont_dim; ++c) {
      acc += HU[r][c] * Urk[c];
    }
    hu_shifted[r] = hu[r] - acc;
  }
  for (int r = 0; r < 4 * preview.N_pred; ++r) {
    for (int c = 0; c < cont_dim; ++c) {
      Aineq[row_ineq + r][c] = HU[r][c];
    }
    bineq[row_ineq + r] = hu_shifted[r];
  }
  row_ineq += 4 * preview.N_pred;

  for (int ycol = 0; ycol < ny; ++ycol) {
    const int b = y_branch[static_cast<size_t>(ycol)];
    if (b < 0 || b >= nz) {
      throw std::runtime_error("upper MIQP: invalid branch label in interval coupling");
    }
    Aineq[row_ineq][y_offset + ycol] = 1.0;
    Aineq[row_ineq][z_offset + b] = -1.0;
    bineq[row_ineq] = 0.0;
    ++row_ineq;
  }

  Matrix<double> Aeq;
  Aeq.resize(0.0, preview.N_pred + 1, nv);
  Vector<double> beq(0.0, preview.N_pred + 1);

  for (int k = 1; k <= preview.N_pred; ++k) {
    for (int i = 0; i < preview.Nck[static_cast<size_t>(k)]; ++i) {
      const int ycol = y_step_offset[static_cast<size_t>(k - 1)] + i;
      Aeq[k - 1][y_offset + ycol] = 1.0;
    }
    beq[k - 1] = 1.0;
  }
  for (int b = 0; b < nz; ++b) {
    Aeq[preview.N_pred][z_offset + b] = 1.0;
  }
  beq[preview.N_pred] = 1.0;

  QuadraticProblem miqp_solver(false);
  Variable * uvar = miqp_solver.vector_variable(nv, "U");
  if (!miqp_solver.add_variable(uvar)) {
    throw std::runtime_error("upper MIQP: failed to add optimization variable");
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

  for (int idx = y_offset; idx < nv; ++idx) {
    miqp_solver.set_binary_var(U[idx]);
  }

  Vector<double> arg;
  const double fval = miqp_solver.solve_problem(arg);
  if (!std::isfinite(fval) || arg.size() < static_cast<unsigned int>(nv)) {
    throw std::runtime_error("upper MIQP: solver failed or returned infeasible solution");
  }

  std::vector<Eigen::Vector2d> p_pred(static_cast<size_t>(preview.N_pred + 1), Eigen::Vector2d::Zero());
  for (int k = 0; k <= preview.N_pred; ++k) {
    const int row = 2 * k;
    double ex = 0.0;
    double ey = 0.0;
    for (int j = 0; j < nv; ++j) {
      ex += G[row + 0][j] * arg[j];
      ey += G[row + 1][j] * arg[j];
    }
    p_pred[static_cast<size_t>(k)] = preview.prk[static_cast<size_t>(k)] + Eigen::Vector2d(ex, ey);
    if (!std::isfinite(p_pred[static_cast<size_t>(k)].x()) ||
        !std::isfinite(p_pred[static_cast<size_t>(k)].y()))
    {
      throw std::runtime_error("upper MIQP: predicted points contain non-finite values");
    }
  }

  SdMapUpperMiqpDebug debug;
  debug.y_branch = y_branch;
  debug.z.resize(static_cast<size_t>(nz), 0.0);
  debug.y.resize(static_cast<size_t>(ny), 0.0);
  for (int b = 0; b < nz; ++b) {
    debug.z[static_cast<size_t>(b)] = arg[z_offset + b];
  }
  for (int ycol = 0; ycol < ny; ++ycol) {
    debug.y[static_cast<size_t>(ycol)] = arg[y_offset + ycol];
  }

  double best_z = -std::numeric_limits<double>::infinity();
  int selected_branch = -1;
  for (int b = 0; b < nz; ++b) {
    if (debug.z[static_cast<size_t>(b)] > best_z) {
      best_z = debug.z[static_cast<size_t>(b)];
      selected_branch = b;
    }
  }
  if (selected_branch < 0) {
    throw std::runtime_error("upper MIQP: failed to select a branch");
  }
  debug.selected_branch_idx = selected_branch;

  debug.selected_interval.resize(static_cast<size_t>(preview.N_pred), -1);
  int cursor = 0;
  for (int k = 1; k <= preview.N_pred; ++k) {
    double best_y = -std::numeric_limits<double>::infinity();
    int best_idx = -1;
    for (int i = 0; i < preview.Nck[static_cast<size_t>(k)]; ++i) {
      const double val = debug.y[static_cast<size_t>(cursor + i)];
      if (val > best_y) {
        best_y = val;
        best_idx = i;
      }
    }
    debug.selected_interval[static_cast<size_t>(k - 1)] = best_idx;
    cursor += preview.Nck[static_cast<size_t>(k)];
  }

  return {p_pred, debug};
}

SdMapUpperPlanResult SdMapUpperPlanner::plan(
  const Eigen::Vector2d & ego_pos_world,
  double ego_yaw_rad,
  double vr,
  double delta_M_deg,
  double wheelbase,
  double a_M,
  double big_m,
  int prev_terminal_edge_id,
  double v1) const
{
  const auto preview = makePreviewConstraints(ego_pos_world, ego_yaw_rad, prev_terminal_edge_id);
  const double Ts = preview.ds_eff / std::max(vr, 1e-6);
  const double phi_M = std::tan(delta_M_deg * 3.14159265358979323846 / 180.0) * Ts / std::max(1e-6, wheelbase);
  const double v_eff = (v1 > 0.0) ? v1 : vr;

  auto [p_pred_body, dbg] = solveUpperMiqp(preview, vr, Ts, a_M, phi_M, big_m, v_eff);

  SdMapUpperPlanResult result;
  result.valid = true;
  result.preview = preview;
  result.miqp_debug = dbg;
  result.selected_branch_idx = dbg.selected_branch_idx;
  result.ds_eff = preview.ds_eff;
  result.lookahead = preview.ds_eff * static_cast<double>(preview.N_pred);

  result.dense_world.reserve(p_pred_body.size());
  for (const auto & p_body : p_pred_body) {
    result.dense_world.push_back(bodyToWorldPoint(p_body, ego_pos_world, ego_yaw_rad));
  }

  if (result.selected_branch_idx >= 0 &&
      result.selected_branch_idx < static_cast<int>(preview.candidate_terminal_edge_ids.size()))
  {
    result.selected_terminal_edge_id =
      preview.candidate_terminal_edge_ids[static_cast<size_t>(result.selected_branch_idx)];
  }

  return result;
}

std::vector<Eigen::Vector2d> SdMapUpperPlanner::extractAheadWorldPath(
  const std::vector<Eigen::Vector2d> & world_path,
  const Eigen::Vector2d & ego_pos_world)
{
  if (world_path.size() < 2) {
    return {};
  }
  const int idx = findClosestPointIndex(world_path, ego_pos_world);
  if (idx < 0) {
    return {};
  }
  if (idx >= static_cast<int>(world_path.size()) - 1) {
    return {world_path.back()};
  }
  return std::vector<Eigen::Vector2d>(
    world_path.begin() + idx,
    world_path.end());
}

SdMapLowerGuide SdMapUpperPlanner::makeLowerGuidesFromUpper(
  const std::vector<Eigen::Vector2d> & upper_dense_world,
  const Eigen::Vector2d & ego_pos_world,
  double ego_yaw_rad,
  double lower_wp_min_separation) const
{
  const auto upper_dense_body = worldToBody(upper_dense_world, ego_pos_world, ego_yaw_rad);
  if (upper_dense_body.size() < 2) {
    throw std::runtime_error("upper dense path has fewer than two samples");
  }

  SdMapLowerGuide guide;
  guide.remaining_length = polylineLength(upper_dense_body);
  if (guide.remaining_length < lower_wp_min_separation) {
    std::ostringstream oss;
    oss << "remaining upper path length " << guide.remaining_length
        << " m is shorter than the minimum lower guide separation "
        << lower_wp_min_separation;
    throw std::runtime_error(oss.str());
  }

  guide.s1 = std::min(config_.lower_wp1_s, guide.remaining_length);
  guide.s0 = std::min(config_.lower_wp0_s, std::max(0.0, guide.s1 - lower_wp_min_separation));
  if ((guide.s1 - guide.s0) < lower_wp_min_separation) {
    guide.s0 = std::max(0.0, guide.s1 - lower_wp_min_separation);
  }

  guide.wp0_body = samplePolylineByArclength(upper_dense_body, guide.s0);
  guide.wp1_body = samplePolylineByArclength(upper_dense_body, guide.s1);

  if ((guide.wp1_body - guide.wp0_body).norm() < 1e-9) {
    throw std::runtime_error("lower guides extracted from upper path are degenerate");
  }

  return guide;
}


namespace
{

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw_rad)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw_rad);
  geometry_msgs::msg::Quaternion out;
  out.x = q.x();
  out.y = q.y();
  out.z = q.z();
  out.w = q.w();
  return out;
}

Eigen::Vector2d bodyPointToWorld(
  const Eigen::Vector2d & p_body,
  const Eigen::Vector2d & ego_pos_world,
  double ego_yaw_rad)
{
  const double c = std::cos(ego_yaw_rad);
  const double s = std::sin(ego_yaw_rad);
  return ego_pos_world + Eigen::Vector2d(
    c * p_body.x() - s * p_body.y(),
    s * p_body.x() + c * p_body.y());
}

double normalizeAngle(double a)
{
  constexpr double PI = 3.14159265358979323846;
  while (a > PI) {
    a -= 2.0 * PI;
  }
  while (a < -PI) {
    a += 2.0 * PI;
  }
  return a;
}

}  // namespace


struct UpperPlannerNode::Impl
{
  explicit Impl(UpperPlannerNode & node)
  : node_(node)
  {
  }

  static bool pathsApproximatelyEqual(
    const std::vector<Eigen::Vector2d> & a,
    const std::vector<Eigen::Vector2d> & b,
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

  static bool sdMapsApproximatelyEqual(const SdMap & a, const SdMap & b)
  {
    if (a.goal_edge_id != b.goal_edge_id) {
      return false;
    }
    if (static_cast<bool>(a.goal_xy) != static_cast<bool>(b.goal_xy)) {
      return false;
    }
    if (a.goal_xy && b.goal_xy && (*a.goal_xy - *b.goal_xy).norm() > 1e-6) {
      return false;
    }
    if (a.edges.size() != b.edges.size()) {
      return false;
    }
    if (a.route_edge_ids != b.route_edge_ids) {
      return false;
    }
    for (size_t i = 0; i < a.edges.size(); ++i) {
      const auto & ea = a.edges[i];
      const auto & eb = b.edges[i];
      if (ea.id != eb.id || ea.successors != eb.successors ||
          std::abs(ea.lane_width - eb.lane_width) > 1e-6 ||
          !pathsApproximatelyEqual(ea.centerline, eb.centerline, 1e-6))
      {
        return false;
      }
    }
    return true;
  }

  static std::string trim(const std::string & s)
  {
    const auto first = s.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
      return "";
    }
    const auto last = s.find_last_not_of(" \t\r\n");
    return s.substr(first, last - first + 1);
  }

  static std::vector<std::string> splitCsvLine(const std::string & line)
  {
    std::vector<std::string> out;
    std::string cur;
    bool in_quotes = false;
    for (size_t i = 0; i < line.size(); ++i) {
      const char ch = line[i];
      if (ch == '"') {
        if (in_quotes && i + 1 < line.size() && line[i + 1] == '"') {
          cur.push_back('"');
          ++i;
        } else {
          in_quotes = !in_quotes;
        }
      } else if (ch == ',' && !in_quotes) {
        out.push_back(trim(cur));
        cur.clear();
      } else {
        cur.push_back(ch);
      }
    }
    out.push_back(trim(cur));
    return out;
  }

  static std::string cell(
    const std::vector<std::string> & row,
    const std::map<std::string, size_t> & col,
    const std::string & name)
  {
    const auto it = col.find(name);
    if (it == col.end() || it->second >= row.size()) {
      return "";
    }
    return row[it->second];
  }

  static bool parseInt(const std::string & s, int * out)
  {
    if (s.empty()) {
      return false;
    }
    try {
      size_t pos = 0;
      const int v = std::stoi(s, &pos);
      if (pos != s.size()) {
        return false;
      }
      *out = v;
      return true;
    } catch (...) {
      return false;
    }
  }

  static bool parseDouble(const std::string & s, double * out)
  {
    if (s.empty()) {
      return false;
    }
    try {
      size_t pos = 0;
      const double v = std::stod(s, &pos);
      if (pos != s.size() || !std::isfinite(v)) {
        return false;
      }
      *out = v;
      return true;
    } catch (...) {
      return false;
    }
  }

  static bool parseBoolish(const std::string & s)
  {
    const std::string v = trim(s);
    return v == "1" || v == "true" || v == "TRUE" || v == "True" || v == "yes";
  }

  static std::vector<int> parseIntList(const std::string & s)
  {
    std::vector<int> out;
    std::stringstream ss(s);
    std::string token;
    while (std::getline(ss, token, '|')) {
      int v = 0;
      if (parseInt(trim(token), &v)) {
        out.push_back(v);
      }
    }
    return out;
  }

  static std::string resolveCsvPath(const std::string & csv_file)
  {
    namespace fs = std::filesystem;
    if (csv_file.empty()) {
      return "";
    }

    std::vector<fs::path> candidates;
    const fs::path raw(csv_file);
    candidates.push_back(raw);
    if (!raw.is_absolute()) {
      candidates.push_back(fs::path("data") / raw);
      try {
        const fs::path share = ament_index_cpp::get_package_share_directory("virtual_control");
        candidates.push_back(share / raw);
        candidates.push_back(share / "data" / raw);
      } catch (...) {
      }
    }

    for (const auto & p : candidates) {
      std::error_code ec;
      if (fs::exists(p, ec) && fs::is_regular_file(p, ec)) {
        return p.string();
      }
    }
    return "";
  }

  struct CsvEdgeDraft
  {
    SdMapEdge edge;
    bool is_route{false};
    bool is_goal{false};
  };

  static bool loadSdMapCsv(
    const std::string & csv_path,
    double lane_width_default,
    SdMap * out,
    std::string * error)
  {
    std::ifstream file(csv_path);
    if (!file.is_open()) {
      if (error) {
        *error = "failed to open file";
      }
      return false;
    }

    std::string header_line;
    if (!std::getline(file, header_line)) {
      if (error) {
        *error = "empty file";
      }
      return false;
    }

    const auto header = splitCsvLine(header_line);
    std::map<std::string, size_t> col;
    for (size_t i = 0; i < header.size(); ++i) {
      col[header[i]] = i;
    }

    std::map<int, CsvEdgeDraft> edge_drafts;
    std::map<int, std::vector<std::pair<int, Eigen::Vector2d>>> edge_points;
    std::vector<int> route_edge_ids;
    std::optional<int> goal_edge_id;
    std::optional<Eigen::Vector2d> goal_xy;

    std::string line;
    int line_no = 1;
    while (std::getline(file, line)) {
      ++line_no;
      if (trim(line).empty()) {
        continue;
      }
      const auto row = splitCsvLine(line);
      const std::string record_type = cell(row, col, "record_type");

      if (record_type == "route") {
        const auto route_ids = parseIntList(cell(row, col, "route_edge_ids"));
        if (!route_ids.empty()) {
          route_edge_ids = route_ids;
        }
        int gid = 0;
        if (parseInt(cell(row, col, "goal_edge_id"), &gid)) {
          goal_edge_id = gid;
        }
        double gx = 0.0;
        double gy = 0.0;
        if (parseDouble(cell(row, col, "goal_x"), &gx) &&
            parseDouble(cell(row, col, "goal_y"), &gy))
        {
          goal_xy = Eigen::Vector2d(gx, gy);
        }
      } else if (record_type == "edge") {
        int edge_id = 0;
        if (!parseInt(cell(row, col, "edge_id"), &edge_id) &&
            !parseInt(cell(row, col, "id"), &edge_id))
        {
          if (error) {
            *error = "edge row missing edge_id at line " + std::to_string(line_no);
          }
          return false;
        }

        auto & draft = edge_drafts[edge_id];
        draft.edge.id = edge_id;
        draft.edge.successors = parseIntList(cell(row, col, "next_edge_ids"));
        draft.edge.lane_width = lane_width_default;
        double lane_width = 0.0;
        if (parseDouble(cell(row, col, "road_width"), &lane_width) && lane_width > 0.0) {
          draft.edge.lane_width = lane_width;
        }
        draft.is_route = parseBoolish(cell(row, col, "is_route"));
        draft.is_goal = parseBoolish(cell(row, col, "is_goal"));
        if (draft.is_goal && !goal_edge_id) {
          goal_edge_id = edge_id;
        }
      } else if (record_type == "edge_point") {
        int edge_id = 0;
        int seq = 0;
        double x = 0.0;
        double y = 0.0;
        if (!parseInt(cell(row, col, "edge_id"), &edge_id) ||
            !parseInt(cell(row, col, "seq"), &seq) ||
            !parseDouble(cell(row, col, "x"), &x) ||
            !parseDouble(cell(row, col, "y"), &y))
        {
          if (error) {
            *error = "bad edge_point row at line " + std::to_string(line_no);
          }
          return false;
        }
        edge_points[edge_id].push_back({seq, Eigen::Vector2d(x, y)});
        auto & draft = edge_drafts[edge_id];
        draft.edge.id = edge_id;
        if (draft.edge.lane_width <= 0.0) {
          draft.edge.lane_width = lane_width_default;
        }
      }
    }

    SdMap decoded;
    decoded.route_edge_ids = route_edge_ids;
    decoded.goal_edge_id = goal_edge_id;
    decoded.goal_xy = goal_xy;
    std::vector<int> route_edge_ids_from_edges;

    for (auto & kv : edge_drafts) {
      const int edge_id = kv.first;
      auto & draft = kv.second;
      auto pts_it = edge_points.find(edge_id);
      if (pts_it != edge_points.end()) {
        auto & pts = pts_it->second;
        std::sort(pts.begin(), pts.end(), [](const auto & a, const auto & b) {
          return a.first < b.first;
        });
        draft.edge.centerline.clear();
        draft.edge.centerline.reserve(pts.size());
        for (const auto & p : pts) {
          draft.edge.centerline.push_back(p.second);
        }
      }

      if (draft.edge.centerline.size() < 2) {
        if (error) {
          *error = "edge " + std::to_string(edge_id) + " has fewer than 2 points";
        }
        return false;
      }
      if (draft.edge.lane_width <= 0.0) {
        draft.edge.lane_width = lane_width_default;
      }
      if (draft.is_route) {
        route_edge_ids_from_edges.push_back(edge_id);
      }
      decoded.edges.push_back(draft.edge);
    }

    std::sort(decoded.edges.begin(), decoded.edges.end(), [](const auto & a, const auto & b) {
      return a.id < b.id;
    });
    if (decoded.route_edge_ids.empty()) {
      decoded.route_edge_ids = route_edge_ids_from_edges;
    }

    if (!decoded.goal_edge_id && !decoded.route_edge_ids.empty()) {
      decoded.goal_edge_id = decoded.route_edge_ids.back();
    }
    if (!decoded.goal_xy && decoded.goal_edge_id) {
      for (const auto & edge : decoded.edges) {
        if (edge.id == *decoded.goal_edge_id && !edge.centerline.empty()) {
          decoded.goal_xy = edge.centerline.back();
          break;
        }
      }
    }

    if (decoded.edges.empty()) {
      if (error) {
        *error = "no edge records found";
      }
      return false;
    }
    if (!decoded.goal_edge_id && !decoded.goal_xy) {
      if (error) {
        *error = "missing goal_edge_id/goal_xy";
      }
      return false;
    }

    *out = std::move(decoded);
    return true;
  }

  bool loadCsvSdMapIfConfigured()
  {
    if (!use_csv_sdmap_) {
      return false;
    }

    const std::string resolved = resolveCsvPath(csv_sdmap_file_);
    if (resolved.empty()) {
      RCLCPP_WARN(
        node_.get_logger(),
        "CSV SD-map enabled but file was not found: '%s'",
        csv_sdmap_file_.c_str());
      return false;
    }

    SdMap decoded;
    std::string error;
    if (!loadSdMapCsv(resolved, planner_.config().lane_width_default, &decoded, &error)) {
      RCLCPP_ERROR(
        node_.get_logger(),
        "failed to load CSV SD-map '%s': %s",
        resolved.c_str(), error.c_str());
      return false;
    }

    planner_.setMap(decoded);
    planner_ready_ = planner_.hasMap();
    upper_valid_ = false;
    upper_dense_world_cache_.clear();
    last_published_guides_.clear();
    selected_terminal_edge_id_ = -1;
    csv_sdmap_loaded_ = planner_ready_;

    RCLCPP_INFO(
      node_.get_logger(),
      "CSV SD-map loaded: file=%s edges=%zu route_edges=%zu goal_edge=%d goal_xy=%d",
      resolved.c_str(),
      decoded.edges.size(),
      decoded.route_edge_ids.size(),
      decoded.goal_edge_id.value_or(-1),
      decoded.goal_xy ? 1 : 0);
    publishStatus("upper_csv_sdmap_ready");
    return planner_ready_;
  }

  void init()
  {
    node_.declare_parameter<double>("timer_hz", 10.0);
    node_.declare_parameter<int>("input_timeout_ms", 500);
    node_.declare_parameter<bool>("upper_keep_last_valid_on_failure", true);
    node_.declare_parameter<bool>("publish_debug_mirror_path", true);
    node_.declare_parameter<bool>("debug_log_every_update", true);
    node_.declare_parameter<int>("upper_update_period_steps", 3);
    node_.declare_parameter<double>("v_ref_mps", 0.4);
    node_.declare_parameter<double>("delta_M_deg", 20.0);
    node_.declare_parameter<double>("wheelbase", 0.3);
    node_.declare_parameter<double>("a_max_mps2", 1.0);
    node_.declare_parameter<double>("miqp_big_m", 20.0);
    node_.declare_parameter<std::string>("path_frame_id", "map");
    node_.declare_parameter<std::string>("publish_topic", "/planner/upper_guides");
    node_.declare_parameter<std::string>("debug_publish_topic", "/planner/upper_dense_path");
    node_.declare_parameter<std::string>("pose_stamped_topic", "/motive/vehicle/pose");
    node_.declare_parameter<double>("pose_x_offset", 0.0);
    node_.declare_parameter<double>("pose_y_offset", 0.0);
    node_.declare_parameter<double>("pose_z_offset", 0.0);
    node_.declare_parameter<double>("pose_yaw_offset_rad", 0.0);
    node_.declare_parameter<double>("pose_position_scale", 1.0);
    node_.declare_parameter<bool>("pose_swap_xy", false);
    node_.declare_parameter<bool>("pose_invert_x", false);
    node_.declare_parameter<bool>("pose_invert_y", false);
    node_.declare_parameter<std::string>("sdmap_edges_topic", "/planner/sdmap_edges_xy");
    node_.declare_parameter<std::string>("sdmap_meta_ids_topic", "/planner/sdmap_meta_ids");
    node_.declare_parameter<std::string>("sdmap_meta_vals_topic", "/planner/sdmap_meta_vals");
    node_.declare_parameter<bool>("use_csv_sdmap", true);
    node_.declare_parameter<std::string>("csv_sdmap_file", "local_map.csv");
    node_.declare_parameter<int>("path_republish_heartbeat_steps", 20);

    SdMapUpperPlannerConfig cfg;
    node_.declare_parameter<double>("upper_lookahead_min", cfg.lookahead_min);
    node_.declare_parameter<double>("upper_lookahead_max", cfg.lookahead_max);
    node_.declare_parameter<double>("upper_ds_nominal", cfg.ds_nominal);
    node_.declare_parameter<double>("upper_path_resample_ds", cfg.path_resample_ds);
    node_.declare_parameter<int>("upper_max_branch_depth", cfg.max_branch_depth);
    node_.declare_parameter<double>("upper_goal_weight", cfg.goal_weight);
    node_.declare_parameter<double>("upper_hysteresis_weight", cfg.hysteresis_weight);
    node_.declare_parameter<double>("upper_short_path_weight", cfg.short_path_weight);
    node_.declare_parameter<double>("upper_qx_x", cfg.upper_qx_x);
    node_.declare_parameter<double>("upper_qx_y", cfg.upper_qx_y);
    node_.declare_parameter<double>("upper_ru_dv", cfg.upper_ru_dv);
    node_.declare_parameter<double>("upper_ru_dpsi", cfg.upper_ru_dpsi);
    node_.declare_parameter<double>("upper_lower_wp0_s", cfg.lower_wp0_s);
    node_.declare_parameter<double>("upper_lower_wp1_s", cfg.lower_wp1_s);
    node_.declare_parameter<double>("upper_replan_margin_s", cfg.upper_replan_margin_s);
    node_.declare_parameter<double>("upper_goal_reached_tol", cfg.goal_reached_tol);
    node_.declare_parameter<double>("upper_goal_zone_tol", cfg.goal_zone_tol);
    node_.declare_parameter<double>("upper_min_required_lookahead", cfg.min_required_lookahead);
    node_.declare_parameter<double>("upper_heading_weight", cfg.heading_weight);
    node_.declare_parameter<double>("upper_lane_width_default", cfg.lane_width_default);
    node_.declare_parameter<double>("upper_corridor_margin", cfg.corridor_margin);
    node_.declare_parameter<double>("upper_min_half_width", cfg.min_half_width);
    node_.declare_parameter<double>("upper_parallel_interval_half_len", cfg.parallel_interval_half_len);
    node_.declare_parameter<double>("upper_max_interval_half_len", cfg.max_interval_half_len);
    node_.declare_parameter<double>("upper_parallel_eps", cfg.parallel_eps);
    node_.declare_parameter<double>("upper_binary_regularization", cfg.binary_regularization);

    node_.get_parameter("timer_hz", timer_hz_);
    node_.get_parameter("input_timeout_ms", input_timeout_ms_);
    node_.get_parameter("upper_keep_last_valid_on_failure", upper_keep_last_valid_on_failure_);
    node_.get_parameter("publish_debug_mirror_path", publish_debug_mirror_path_);
    node_.get_parameter("debug_log_every_update", debug_log_every_update_);
    node_.get_parameter("upper_update_period_steps", upper_update_period_steps_);
    node_.get_parameter("v_ref_mps", vr_mps_);
    node_.get_parameter("delta_M_deg", delta_M_deg_);
    node_.get_parameter("wheelbase", wheelbase_);
    node_.get_parameter("a_max_mps2", a_M_);
    node_.get_parameter("miqp_big_m", miqp_big_m_);
    node_.get_parameter("path_frame_id", path_frame_id_);
    node_.get_parameter("publish_topic", publish_topic_);
    node_.get_parameter("debug_publish_topic", debug_publish_topic_);
    node_.get_parameter("pose_stamped_topic", pose_stamped_topic_);
    node_.get_parameter("pose_x_offset", pose_x_offset_);
    node_.get_parameter("pose_y_offset", pose_y_offset_);
    node_.get_parameter("pose_z_offset", pose_z_offset_);
    node_.get_parameter("pose_yaw_offset_rad", pose_yaw_offset_rad_);
    node_.get_parameter("pose_position_scale", pose_position_scale_);
    node_.get_parameter("pose_swap_xy", pose_swap_xy_);
    node_.get_parameter("pose_invert_x", pose_invert_x_);
    node_.get_parameter("pose_invert_y", pose_invert_y_);
    node_.get_parameter("sdmap_edges_topic", sdmap_edges_topic_);
    node_.get_parameter("sdmap_meta_ids_topic", sdmap_meta_ids_topic_);
    node_.get_parameter("sdmap_meta_vals_topic", sdmap_meta_vals_topic_);
    node_.get_parameter("use_csv_sdmap", use_csv_sdmap_);
    node_.get_parameter("csv_sdmap_file", csv_sdmap_file_);
    node_.get_parameter("path_republish_heartbeat_steps", path_republish_heartbeat_steps_);

    node_.get_parameter("upper_lookahead_min", cfg.lookahead_min);
    node_.get_parameter("upper_lookahead_max", cfg.lookahead_max);
    node_.get_parameter("upper_ds_nominal", cfg.ds_nominal);
    node_.get_parameter("upper_path_resample_ds", cfg.path_resample_ds);
    node_.get_parameter("upper_max_branch_depth", cfg.max_branch_depth);
    node_.get_parameter("upper_goal_weight", cfg.goal_weight);
    node_.get_parameter("upper_hysteresis_weight", cfg.hysteresis_weight);
    node_.get_parameter("upper_short_path_weight", cfg.short_path_weight);
    node_.get_parameter("upper_qx_x", cfg.upper_qx_x);
    node_.get_parameter("upper_qx_y", cfg.upper_qx_y);
    node_.get_parameter("upper_ru_dv", cfg.upper_ru_dv);
    node_.get_parameter("upper_ru_dpsi", cfg.upper_ru_dpsi);
    node_.get_parameter("upper_lower_wp0_s", cfg.lower_wp0_s);
    node_.get_parameter("upper_lower_wp1_s", cfg.lower_wp1_s);
    node_.get_parameter("upper_replan_margin_s", cfg.upper_replan_margin_s);
    node_.get_parameter("upper_goal_reached_tol", cfg.goal_reached_tol);
    node_.get_parameter("upper_goal_zone_tol", cfg.goal_zone_tol);
    node_.get_parameter("upper_min_required_lookahead", cfg.min_required_lookahead);
    node_.get_parameter("upper_heading_weight", cfg.heading_weight);
    node_.get_parameter("upper_lane_width_default", cfg.lane_width_default);
    node_.get_parameter("upper_corridor_margin", cfg.corridor_margin);
    node_.get_parameter("upper_min_half_width", cfg.min_half_width);
    node_.get_parameter("upper_parallel_interval_half_len", cfg.parallel_interval_half_len);
    node_.get_parameter("upper_max_interval_half_len", cfg.max_interval_half_len);
    node_.get_parameter("upper_parallel_eps", cfg.parallel_eps);
    node_.get_parameter("upper_binary_regularization", cfg.binary_regularization);
    planner_.setConfig(cfg);

    pose_stamped_sub_ = node_.create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_stamped_topic_, 10,
      std::bind(&Impl::poseStampedCallback, this, std::placeholders::_1));
    RCLCPP_INFO(
      node_.get_logger(),
      "UpperPlannerNode state input: PoseStamped topic=%s yaw_source=orientation_z",
      pose_stamped_topic_.c_str());

    rclcpp::QoS qos_sdmap(10);
    sdmap_edges_sub_ = node_.create_subscription<std_msgs::msg::Float64MultiArray>(
      sdmap_edges_topic_, qos_sdmap,
      std::bind(&Impl::sdmapEdgesCallback, this, std::placeholders::_1));
    sdmap_meta_ids_sub_ = node_.create_subscription<std_msgs::msg::Int32MultiArray>(
      sdmap_meta_ids_topic_, qos_sdmap,
      std::bind(&Impl::sdmapMetaIdsCallback, this, std::placeholders::_1));
    sdmap_meta_vals_sub_ = node_.create_subscription<std_msgs::msg::Float64MultiArray>(
      sdmap_meta_vals_topic_, qos_sdmap,
      std::bind(&Impl::sdmapMetaValsCallback, this, std::placeholders::_1));

    auto path_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    path_pub_ = node_.create_publisher<nav_msgs::msg::Path>(publish_topic_, path_qos);
    if (publish_debug_mirror_path_) {
      debug_path_pub_ = node_.create_publisher<nav_msgs::msg::Path>(debug_publish_topic_, 10);
    }
    terminal_edge_pub_ =
      node_.create_publisher<std_msgs::msg::Int32>("/planner/selected_terminal_edge_id", 10);
    plan_seq_pub_ =
      node_.create_publisher<std_msgs::msg::Int32>("/planner/upper_plan_seq", 10);
    status_pub_ =
      node_.create_publisher<std_msgs::msg::String>("/planner/status", 10);

    loadCsvSdMapIfConfigured();

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, timer_hz_));
    timer_ = node_.create_wall_timer(period, std::bind(&Impl::timerCallback, this));

    RCLCPP_INFO(
      node_.get_logger(),
      "UpperPlannerNode ready: planner_ready=%d csv_loaded=%d csv_file=%s state_input=PoseStamped pose_topic=%s yaw_source=%s publish_topic=%s update_period_steps=%d timer_hz=%.1f v_ref=%.2f lookahead=[%.2f,%.2f] ds=%.2f lane_width_default=%.2f sdmap_topics=(%s,%s,%s)",
      planner_ready_ ? 1 : 0, csv_sdmap_loaded_ ? 1 : 0, csv_sdmap_file_.c_str(),
      pose_stamped_topic_.c_str(), "orientation_z", publish_topic_.c_str(),
      upper_update_period_steps_, timer_hz_,
      vr_mps_, planner_.config().lookahead_min, planner_.config().lookahead_max,
      planner_.config().ds_nominal, planner_.config().lane_width_default,
      sdmap_edges_topic_.c_str(), sdmap_meta_ids_topic_.c_str(), sdmap_meta_vals_topic_.c_str());
  }

  void publishStatus(const std::string & text) const
  {
    if (!status_pub_) {
      return;
    }
    std_msgs::msg::String msg;
    msg.data = text;
    status_pub_->publish(msg);
  }

  void sdmapEdgesCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(sdmap_mtx_);
    last_sdmap_edges_msg_ = msg;
    if (tryAssembleSdMap()) {
      publishStatus("upper_sdmap_ready");
    }
  }

  void sdmapMetaIdsCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(sdmap_mtx_);
    last_sdmap_meta_ids_msg_ = msg;
    if (tryAssembleSdMap()) {
      publishStatus("upper_sdmap_ready");
    }
  }

  void sdmapMetaValsCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(sdmap_mtx_);
    last_sdmap_meta_vals_msg_ = msg;
    if (tryAssembleSdMap()) {
      publishStatus("upper_sdmap_ready");
    }
  }

  bool tryAssembleSdMap()
  {
    if (!last_sdmap_edges_msg_ || !last_sdmap_meta_ids_msg_) {
      return false;
    }

    const auto & xy = last_sdmap_edges_msg_->data;
    const auto & ids = last_sdmap_meta_ids_msg_->data;
    const std::vector<double> empty_vals;
    const auto & vals = last_sdmap_meta_vals_msg_ ? last_sdmap_meta_vals_msg_->data : empty_vals;

    if (ids.size() < 2) {
      return false;
    }

    size_t cursor = 0;
    const int num_edges = ids[cursor++];
    const int goal_edge_id = ids[cursor++];
    if (num_edges <= 0) {
      return false;
    }

    SdMap decoded;
    if (goal_edge_id >= 0) {
      decoded.goal_edge_id = goal_edge_id;
    }
    if (vals.size() >= 2) {
      decoded.goal_xy = Eigen::Vector2d(vals[0], vals[1]);
    } else if (!decoded.goal_edge_id) {
      return false;
    }
    decoded.edges.reserve(static_cast<size_t>(num_edges));
    size_t lane_width_cursor = 2;

    for (int e = 0; e < num_edges; ++e) {
      if (cursor + 4 > ids.size()) {
        return false;
      }

      SdMapEdge edge;
      edge.id = ids[cursor++];
      const int start_pt_idx = ids[cursor++];
      const int num_pts = ids[cursor++];
      const int num_succ = ids[cursor++];
      edge.lane_width = planner_.config().lane_width_default;
      if (lane_width_cursor < vals.size() && std::isfinite(vals[lane_width_cursor]) &&
          vals[lane_width_cursor] > 0.0)
      {
        edge.lane_width = vals[lane_width_cursor];
      }
      ++lane_width_cursor;

      if (start_pt_idx < 0 || num_pts < 2) {
        return false;
      }
      const int xy_start = 2 * start_pt_idx;
      const int xy_count = 2 * num_pts;
      if (xy_start + xy_count > static_cast<int>(xy.size())) {
        return false;
      }

      edge.centerline.reserve(static_cast<size_t>(num_pts));
      for (int k = 0; k < num_pts; ++k) {
        const double x = xy[xy_start + 2 * k + 0];
        const double y = xy[xy_start + 2 * k + 1];
        edge.centerline.emplace_back(x, y);
      }

      if (cursor + num_succ > ids.size()) {
        return false;
      }
      edge.successors.reserve(static_cast<size_t>(num_succ));
      for (int s = 0; s < num_succ; ++s) {
        edge.successors.push_back(ids[cursor++]);
      }

      decoded.edges.push_back(std::move(edge));
    }

    if (cursor < ids.size()) {
      const int num_route_edges = ids[cursor++];
      if (num_route_edges < 0 || cursor + static_cast<size_t>(num_route_edges) > ids.size()) {
        return false;
      }
      decoded.route_edge_ids.reserve(static_cast<size_t>(num_route_edges));
      for (int i = 0; i < num_route_edges; ++i) {
        decoded.route_edge_ids.push_back(ids[cursor++]);
      }
    }

    if (planner_.hasMap() && sdMapsApproximatelyEqual(planner_.map(), decoded)) {
      planner_ready_ = true;
      RCLCPP_DEBUG_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 2000,
        "SD-map republish unchanged: edges=%zu", decoded.edges.size());
      return true;
    }

    planner_.setMap(decoded);
    planner_ready_ = planner_.hasMap();
    upper_valid_ = false;
    upper_dense_world_cache_.clear();
    last_published_guides_.clear();
    selected_terminal_edge_id_ = -1;

    RCLCPP_INFO_THROTTLE(
      node_.get_logger(), *node_.get_clock(), 2000,
      "SD-map received: edges=%zu route_edges=%zu goal_edge=%d goal_xy=%d lane_width_values=%zu",
      decoded.edges.size(),
      decoded.route_edge_ids.size(),
      decoded.goal_edge_id.value_or(-1),
      decoded.goal_xy ? 1 : 0,
      (vals.size() > 2) ? (vals.size() - 2) : 0);
    return planner_ready_;
  }

  void publishCurrentPath(bool force = false)
  {
    if (!path_pub_ || upper_dense_world_cache_.size() < 2) {
      return;
    }

    const auto & cfg = planner_.config();
    const double configured_sep = std::max(0.05, cfg.lower_wp1_s - cfg.lower_wp0_s);
    const double dynamic_sep = std::max(0.05, vr_mps_ / std::max(1.0, timer_hz_));
    const double lower_wp_min_separation = std::max(configured_sep, dynamic_sep);
    const auto ahead = SdMapUpperPlanner::extractAheadWorldPath(upper_dense_world_cache_, ego_pos_world_);
    const auto & guide_source = (ahead.size() >= 2) ? ahead : upper_dense_world_cache_;

    SdMapLowerGuide guide;
    try {
      guide = planner_.makeLowerGuidesFromUpper(
        guide_source, ego_pos_world_, ego_yaw_rad_, lower_wp_min_separation);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 1000,
        "upper guide extraction failed: %s", e.what());
      return;
    }

    const Eigen::Vector2d wp0_world = bodyPointToWorld(guide.wp0_body, ego_pos_world_, ego_yaw_rad_);
    const Eigen::Vector2d wp1_world = bodyPointToWorld(guide.wp1_body, ego_pos_world_, ego_yaw_rad_);
    const std::vector<Eigen::Vector2d> guides_world{wp0_world, wp1_world};

    if (!force && !last_published_guides_.empty() &&
        pathsApproximatelyEqual(last_published_guides_, guides_world) &&
        (step_count_ - last_publish_step_) < std::max(1, path_republish_heartbeat_steps_)) {
      return;
    }

    nav_msgs::msg::Path path;
    path.header.stamp = node_.now();
    path.header.frame_id = path_frame_id_;
    path.poses.resize(2);
    path.poses[0].header = path.header;
    path.poses[0].pose.position.x = wp0_world.x();
    path.poses[0].pose.position.y = wp0_world.y();
    path.poses[0].pose.position.z = 0.0;
    path.poses[1].header = path.header;
    path.poses[1].pose.position.x = wp1_world.x();
    path.poses[1].pose.position.y = wp1_world.y();
    path.poses[1].pose.position.z = 0.0;
    const double yaw_rad = std::atan2((wp1_world - wp0_world).y(), (wp1_world - wp0_world).x());
    path.poses[0].pose.orientation = yawToQuaternion(yaw_rad);
    path.poses[1].pose.orientation = path.poses[0].pose.orientation;

    path_pub_->publish(path);

    if (debug_path_pub_) {
      nav_msgs::msg::Path dense_path;
      dense_path.header = path.header;
      dense_path.poses.reserve(upper_dense_world_cache_.size());
      for (size_t i = 0; i < upper_dense_world_cache_.size(); ++i) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = dense_path.header;
        ps.pose.position.x = upper_dense_world_cache_[i].x();
        ps.pose.position.y = upper_dense_world_cache_[i].y();
        ps.pose.position.z = 0.0;
        double dense_yaw_rad = 0.0;
        if (upper_dense_world_cache_.size() >= 2) {
          if (i + 1 < upper_dense_world_cache_.size()) {
            const Eigen::Vector2d dp = upper_dense_world_cache_[i + 1] - upper_dense_world_cache_[i];
            dense_yaw_rad = std::atan2(dp.y(), dp.x());
          } else {
            const Eigen::Vector2d dp = upper_dense_world_cache_[i] - upper_dense_world_cache_[i - 1];
            dense_yaw_rad = std::atan2(dp.y(), dp.x());
          }
        }
        ps.pose.orientation = yawToQuaternion(dense_yaw_rad);
        dense_path.poses.push_back(ps);
      }
      debug_path_pub_->publish(dense_path);
    }

    last_published_guides_ = guides_world;
    last_publish_step_ = step_count_;

    std_msgs::msg::Int32 seq_msg;
    seq_msg.data = plan_seq_;
    plan_seq_pub_->publish(seq_msg);

    std_msgs::msg::Int32 edge_msg;
    edge_msg.data = selected_terminal_edge_id_;
    terminal_edge_pub_->publish(edge_msg);
  }

  void poseStampedCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;

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
    (void)z;

    double yaw = msg->pose.orientation.z;
    yaw = normalizeAngle(yaw + pose_yaw_offset_rad_);

    ego_pos_world_.x() = x;
    ego_pos_world_.y() = y;
    ego_yaw_rad_ = yaw;
    have_pose_ = true;
    last_pose_rx_time_ = node_.now();

    RCLCPP_DEBUG_THROTTLE(
      node_.get_logger(), *node_.get_clock(), 500,
      "Upper PoseStamped state: frame=%s x=%.3f y=%.3f yaw=%.3f source=%s",
      msg->header.frame_id.c_str(),
      ego_pos_world_.x(),
      ego_pos_world_.y(),
      ego_yaw_rad_,
      "orientation_z");
  }

  bool maybeReplan()
  {
    const auto & cfg = planner_.config();
    bool need_update =
      !upper_valid_ ||
      (((step_count_ - 1) % std::max(1, upper_update_period_steps_)) == 0);

    if (upper_valid_) {
      const auto ahead = SdMapUpperPlanner::extractAheadWorldPath(upper_dense_world_cache_, ego_pos_world_);
      const double remaining_ahead = SdMapUpperPlanner::polylineLength(ahead);
      if (remaining_ahead < (cfg.lower_wp1_s + cfg.upper_replan_margin_s)) {
        need_update = true;
      }
    }

    if (!need_update) {
      return upper_valid_;
    }

    const int prev_terminal = upper_valid_ ? selected_terminal_edge_id_ : -1;

    try {
      const auto result = planner_.plan(
        ego_pos_world_, ego_yaw_rad_, std::max(0.05, vr_mps_), delta_M_deg_, wheelbase_,
        a_M_, miqp_big_m_, prev_terminal, std::max(0.05, vr_mps_));

      if (!result.valid || result.dense_world.size() < 2) {
        throw std::runtime_error("upper plan result is invalid or too short");
      }

      upper_dense_world_cache_ = result.dense_world;
      selected_terminal_edge_id_ = result.selected_terminal_edge_id;
      upper_valid_ = true;
      ++plan_seq_;

      if (debug_log_every_update_) {
        RCLCPP_INFO(
          node_.get_logger(),
          "upper replan success: seq=%d branch=%d terminal_edge=%d dense_pts=%zu ds_eff=%.3f",
          plan_seq_, result.selected_branch_idx, selected_terminal_edge_id_,
          upper_dense_world_cache_.size(), result.ds_eff);
      }
      publishStatus("upper_ok");
      return true;
    } catch (const std::exception & e) {
      std::ostringstream oss;
      if (upper_keep_last_valid_on_failure_ && upper_valid_) {
        oss << "upper_replan_failed_keep_last_valid: " << e.what();
        publishStatus(oss.str());
        RCLCPP_WARN(node_.get_logger(), "%s", oss.str().c_str());
        return true;
      }
      upper_valid_ = false;
      upper_dense_world_cache_.clear();
      selected_terminal_edge_id_ = -1;
      oss << "upper_replan_failed: " << e.what();
      publishStatus(oss.str());
      RCLCPP_ERROR(node_.get_logger(), "%s", oss.str().c_str());
      return false;
    }
  }

  void timerCallback()
  {
    ++step_count_;

    if (!planner_ready_) {
      publishStatus("upper_not_ready_no_map");
      return;
    }

    if (!have_pose_) {
      publishStatus("upper_waiting_for_pose");
      return;
    }

    if (input_timeout_ms_ > 0) {
      const double age_ms = (node_.now() - last_pose_rx_time_).seconds() * 1000.0;
      if (age_ms > static_cast<double>(input_timeout_ms_)) {
        if (upper_valid_) {
          publishStatus("upper_stale_pose_keep_last_valid");
          publishCurrentPath();
        } else {
          publishStatus("upper_stale_pose_no_valid_path");
        }
        return;
      }
    }

    if (planner_.goalReached(ego_pos_world_, planner_.config().goal_reached_tol)) {
      publishStatus("upper_goal_reached");
      if (upper_valid_) {
        publishCurrentPath(true);
      }
      return;
    }

    if (!maybeReplan()) {
      return;
    }

    publishCurrentPath();
  }

  UpperPlannerNode & node_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sdmap_edges_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sdmap_meta_ids_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sdmap_meta_vals_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr debug_path_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr terminal_edge_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr plan_seq_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex sdmap_mtx_;
  std_msgs::msg::Float64MultiArray::SharedPtr last_sdmap_edges_msg_;
  std_msgs::msg::Int32MultiArray::SharedPtr last_sdmap_meta_ids_msg_;
  std_msgs::msg::Float64MultiArray::SharedPtr last_sdmap_meta_vals_msg_;

  SdMapUpperPlanner planner_;

  bool planner_ready_{false};
  bool have_pose_{false};
  bool upper_valid_{false};
  bool upper_keep_last_valid_on_failure_{true};
  bool publish_debug_mirror_path_{true};
  bool debug_log_every_update_{true};
  bool use_csv_sdmap_{true};
  bool csv_sdmap_loaded_{false};

  int step_count_{0};
  int upper_update_period_steps_{3};
  int input_timeout_ms_{500};
  int plan_seq_{0};
  int selected_terminal_edge_id_{-1};
  int last_publish_step_{-1000000};
  int path_republish_heartbeat_steps_{20};

  double timer_hz_{10.0};
  double vr_mps_{1.0};
  double delta_M_deg_{20.0};
  double wheelbase_{0.3};
  double a_M_{1.0};
  double miqp_big_m_{20.0};

  std::string path_frame_id_{"map"};
  std::string publish_topic_{"/planner/upper_guides"};
  std::string debug_publish_topic_{"/planner/upper_dense_path"};
  std::string pose_stamped_topic_{"/motive/vehicle/pose"};
  std::string sdmap_edges_topic_{"/planner/sdmap_edges_xy"};
  std::string sdmap_meta_ids_topic_{"/planner/sdmap_meta_ids"};
  std::string sdmap_meta_vals_topic_{"/planner/sdmap_meta_vals"};
  std::string csv_sdmap_file_{"local_map.csv"};

  double pose_x_offset_{0.0};
  double pose_y_offset_{0.0};
  double pose_z_offset_{0.0};
  double pose_yaw_offset_rad_{0.0};
  double pose_position_scale_{1.0};
  bool pose_swap_xy_{false};
  bool pose_invert_x_{false};
  bool pose_invert_y_{false};

  Eigen::Vector2d ego_pos_world_{Eigen::Vector2d::Zero()};
  double ego_yaw_rad_{0.0};
  rclcpp::Time last_pose_rx_time_{0, 0, RCL_ROS_TIME};

  std::vector<Eigen::Vector2d> upper_dense_world_cache_;
  std::vector<Eigen::Vector2d> last_published_guides_;
};

UpperPlannerNode::UpperPlannerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("upper_planner_node", options),
  impl_(std::make_unique<Impl>(*this))
{
  impl_->init();
}

UpperPlannerNode::~UpperPlannerNode() = default;

}  // namespace imac_ctrl


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<imac_ctrl::UpperPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
