#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace imac_ctrl::egocentric_planner_geometry
{

struct BranchDetection
{
  int turn_step{-1};
  std::vector<int> candidate_counts;
};

// Count original observed spans before contraction. Only increases from k1->k2
// onward activate turn costs; k0->k1 is excluded. Road constraints are retained.
inline BranchDetection detectBranch(
  const std::vector<std::vector<double>> & raw_lengths, double minimum_width_m,
  bool require_count_increase = true, bool include_k0_transition = false)
{
  BranchDetection result;
  result.candidate_counts.resize(raw_lengths.size(), 0);
  for (std::size_t k = 0; k < raw_lengths.size(); ++k) {
    for (const double length : raw_lengths[k]) {
      if (std::isfinite(length) && length > 0.0 && length + 1e-9 >= minimum_width_m) {
        ++result.candidate_counts[k];
      }
    }
    const bool future_step = k >= (include_k0_transition ? 1U : 2U);
    const bool split = future_step && (require_count_increase ?
      (result.candidate_counts[k - 1] >= 1 &&
      result.candidate_counts[k] > result.candidate_counts[k - 1]) :
      result.candidate_counts[k] >= 2);
    if (split && result.turn_step < 0)
    {
      result.turn_step = static_cast<int>(k);
    }
  }
  return result;
}

// Optional historical diagnostic: override the anchor when one eligible
// section lies within the continuity distance of at least two next sections.
// This changes only the turn-cost anchor, never the retained road candidates.
inline BranchDetection detectBranchWithContinuity(
  const std::vector<std::vector<Eigen::Vector2d>> & starts,
  const std::vector<std::vector<Eigen::Vector2d>> & ends,
  const std::vector<std::vector<double>> & raw_lengths, double minimum_width_m,
  bool require_count_increase, bool include_k0_transition,
  bool use_continuity, double continuity_m)
{
  auto result = detectBranch(
    raw_lengths, minimum_width_m, require_count_increase, include_k0_transition);
  if (!use_continuity) {return result;}
  const auto eligible = [minimum_width_m](double width) {
      return std::isfinite(width) && width > 0.0 && width + 1e-9 >= minimum_width_m;
    };
  const auto point_segment = [](const Eigen::Vector2d & p,
      const Eigen::Vector2d & a, const Eigen::Vector2d & b) {
      const Eigen::Vector2d d = b - a;
      const double u = d.squaredNorm() > 1e-12 ?
        std::clamp((p - a).dot(d) / d.squaredNorm(), 0.0, 1.0) : 0.0;
      return (p - a - u * d).norm();
    };
  const auto distance = [&point_segment](const Eigen::Vector2d & a,
      const Eigen::Vector2d & b, const Eigen::Vector2d & c, const Eigen::Vector2d & d) {
      return std::min({point_segment(a, c, d), point_segment(b, c, d),
        point_segment(c, a, b), point_segment(d, a, b)});
    };
  const std::size_t count = std::min({starts.size(), ends.size(), raw_lengths.size()});
  for (std::size_t k = include_k0_transition ? 1U : 2U; k < count; ++k) {
    const std::size_t previous_count = k == 1 ? 1U :
      std::min({starts[k - 1].size(), ends[k - 1].size(), raw_lengths[k - 1].size()});
    for (std::size_t i = 0; i < previous_count; ++i) {
      if (k > 1 && !eligible(raw_lengths[k - 1][i])) {continue;}
      const Eigen::Vector2d a = k == 1 ? Eigen::Vector2d::Zero().eval() : starts[k - 1][i];
      const Eigen::Vector2d b = k == 1 ? Eigen::Vector2d::Zero().eval() : ends[k - 1][i];
      int continuing = 0;
      const auto candidates = std::min({starts[k].size(), ends[k].size(), raw_lengths[k].size()});
      for (std::size_t j = 0; j < candidates; ++j) {
        if (eligible(raw_lengths[k][j]) &&
          distance(a, b, starts[k][j], ends[k][j]) <= continuity_m) {++continuing;}
      }
      if (continuing >= 2) {result.turn_step = static_cast<int>(k); return result;}
    }
  }
  return result;
}

inline bool gridValueIsDrivable(
  int value,
  bool positive_is_drivable,
  int threshold)
{
  if (value < 0) {
    return false;
  }
  return positive_is_drivable ? value > threshold : value <= threshold;
}

inline bool referenceAllowsNominalFallback(
  bool sample_available,
  int value,
  bool positive_is_drivable,
  int threshold)
{
  return !sample_available || value < 0 ||
         gridValueIsDrivable(value, positive_is_drivable, threshold);
}

inline int retainMatlabPreviewHorizon(
  int maximum_horizon,
  std::size_t previous_path_point_count,
  bool restore_full_horizon = false)
{
  if (restore_full_horizon || maximum_horizon <= 0 || previous_path_point_count < 3U) {
    return maximum_horizon;
  }
  const int previous_horizon = static_cast<int>(previous_path_point_count) - 1;
  return std::min(maximum_horizon, previous_horizon);
}

inline int previewHorizonAfterMissingInterval(
  int missing_step,
  int current_horizon)
{
  if (current_horizon <= 0) {
    return 0;
  }
  // The MIQP constrains predicted stages k=1,...,N. The k=0 sample is the
  // measured ego state and must not invalidate otherwise usable future
  // intervals merely because the body origin is marked outside the road.
  if (missing_step <= 0) {
    return current_horizon;
  }
  return std::min(current_horizon, missing_step - 1);
}

inline std::array<int, 2> findWaypointPair(
  const std::vector<Eigen::Vector2d> & waypoints,
  int wp0_index,
  const Eigen::Vector2d & position,
  const Eigen::Vector2d & velocity,
  double switch_epsilon_m,
  double switch_distance_m)
{
  if (waypoints.size() < 2) {
    return {0, 0};
  }

  const int last_pair_start = static_cast<int>(waypoints.size()) - 2;
  int index = std::max(0, std::min(wp0_index, last_pair_start));
  const double velocity_norm = velocity.norm();
  if (velocity_norm > 1e-9) {
    const Eigen::Vector2d moving_direction = velocity / velocity_norm;
    // Match find_wp.m: wp0 is the progress gate. This lets the first cycle
    // advance from [0,1] to [1,2] when the ego starts at route[0], so the
    // outgoing segment after the current target is visible before the turn.
    const Eigen::Vector2d waypoint_delta =
      waypoints[static_cast<std::size_t>(index)] - position;
    if (index < last_pair_start &&
      waypoint_delta.dot(moving_direction) < switch_epsilon_m &&
      waypoint_delta.norm() <= switch_distance_m)
    {
      ++index;
    }
  }

  return {index, std::min(index + 1, static_cast<int>(waypoints.size()) - 1)};
}

inline Eigen::Vector2d buildOneStepTerminalTarget(
  const Eigen::Vector2d & previous_reference,
  const Eigen::Vector2d & wp0_body,
  const Eigen::Vector2d & wp1_body,
  double delta_s,
  const Eigen::Vector2d & fallback_direction)
{
  const double fallback_norm = fallback_direction.norm();
  Eigen::Vector2d stable_direction = Eigen::Vector2d::UnitX();
  if (fallback_norm > 1e-9) {
    stable_direction = fallback_direction / fallback_norm;
  }
  const double wp0_forward_distance =
    (wp0_body - previous_reference).dot(stable_direction);
  const Eigen::Vector2d & selected_waypoint =
    wp0_forward_distance > delta_s ? wp0_body : wp1_body;
  const Eigen::Vector2d target_delta = selected_waypoint - previous_reference;
  if (target_delta.norm() > 1e-9) {
    return previous_reference + delta_s * target_delta.normalized();
  }

  return previous_reference + delta_s * stable_direction;
}

inline bool buildMeasuredFrameProgressPrefix(
  const std::vector<Eigen::Vector2d> & previous_path_world,
  const Eigen::Vector2d & current_position_world,
  double current_measured_yaw_rad, double spacing_m,
  int requested_horizon,
  std::vector<Eigen::Vector2d> & prefix_body)
{
  prefix_body.clear();
  if (requested_horizon < 2 || previous_path_world.size() < 2 ||
    !current_position_world.allFinite() || !std::isfinite(current_measured_yaw_rad) ||
    !std::isfinite(spacing_m) || spacing_m <= 0.0)
  {
    return false;
  }
  std::vector<Eigen::Vector2d> points;
  std::vector<double> arc;
  for (const auto & point : previous_path_world) {
    if (!point.allFinite()) {return false;}
    if (!points.empty() && (point - points.back()).norm() <= 1e-9) {continue;}
    arc.push_back(points.empty() ? 0.0 : arc.back() + (point - points.back()).norm());
    points.push_back(point);
  }
  if (points.size() < 2) {return false;}
  double best_distance = std::numeric_limits<double>::infinity();
  double progress = 0.0;
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    const Eigen::Vector2d segment = points[i + 1] - points[i];
    const double fraction = std::clamp(
      (current_position_world - points[i]).dot(segment) / segment.squaredNorm(), 0.0, 1.0);
    const double distance =
      (current_position_world - points[i] - fraction * segment).squaredNorm();
    if (distance < best_distance) {
      best_distance = distance;
      progress = arc[i] + fraction * (arc[i + 1] - arc[i]);
    }
  }
  const double cosine = std::cos(current_measured_yaw_rad);
  const double sine = std::sin(current_measured_yaw_rad);
  Eigen::Matrix2d rotation_body_world;
  rotation_body_world << cosine, sine, -sine, cosine;
  prefix_body.assign(static_cast<std::size_t>(requested_horizon + 1), Eigen::Vector2d::Zero());
  // r0 is the measured ego. Future points retain the old world geometry.
  // The last two points are subsequently extended by buildPreviewReference.
  for (int k = 1; k <= requested_horizon - 2; ++k) {
    const double query = progress + k * spacing_m;
    Eigen::Vector2d world;
    if (query >= arc.back()) {
      world = points.back() + (query - arc.back()) *
        (points.back() - points[points.size() - 2]).normalized();
    } else {
      const auto right = static_cast<std::size_t>(
        std::upper_bound(arc.begin(), arc.end(), query) - arc.begin());
      const double fraction = (query - arc[right - 1]) / (arc[right] - arc[right - 1]);
      world = points[right - 1] + fraction * (points[right] - points[right - 1]);
    }
    prefix_body[static_cast<std::size_t>(k)] =
      rotation_body_world * (world - current_position_world);
  }
  return true;
}

inline int previewStepsFromForwardExtent(
  double forward_extent_m,
  double delta_s,
  int maximum_steps,
  int reserve_steps = 1)
{
  if (!std::isfinite(forward_extent_m) || !std::isfinite(delta_s) ||
    delta_s <= 0.0 || maximum_steps <= 0 || reserve_steps < 0)
  {
    return 0;
  }
  // Default preserves main8.m's one-step reserve. A zero reserve allows all
  // complete steps inside the forward extent, subject to the configured cap.
  const int extent_limited =
    static_cast<int>(std::floor(forward_extent_m / delta_s)) - reserve_steps;
  return std::max(0, std::min(extent_limited, maximum_steps));
}

}  // namespace imac_ctrl::egocentric_planner_geometry
