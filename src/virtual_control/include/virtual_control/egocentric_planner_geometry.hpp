#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace imac_ctrl::egocentric_planner_geometry
{

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

inline bool buildMeasuredFrameShiftedPrefix(
  const std::vector<Eigen::Vector2d> & previous_path_world,
  double current_measured_yaw_rad,
  int requested_horizon,
  std::vector<Eigen::Vector2d> & shifted_prefix_body)
{
  shifted_prefix_body.clear();
  if (requested_horizon < 2 ||
    previous_path_world.size() != static_cast<std::size_t>(requested_horizon + 1) ||
    !std::isfinite(current_measured_yaw_rad))
  {
    return false;
  }
  for (const auto & point : previous_path_world) {
    if (!point.allFinite()) {return false;}
  }

  const double cosine = std::cos(current_measured_yaw_rad);
  const double sine = std::sin(current_measured_yaw_rad);
  Eigen::Matrix2d rotation_body_world;
  rotation_body_world << cosine, sine, -sine, cosine;
  const Eigen::Vector2d shift_origin = previous_path_world[1];

  shifted_prefix_body.assign(
    static_cast<std::size_t>(requested_horizon + 1), Eigen::Vector2d::Zero());
  // Old p1,...,pN-1 become new r0,...,rN-2. Old pN is intentionally excluded.
  for (int k = 0; k <= requested_horizon - 2; ++k) {
    shifted_prefix_body[static_cast<std::size_t>(k)] =
      rotation_body_world *
      (previous_path_world[static_cast<std::size_t>(k + 1)] - shift_origin);
  }
  return shifted_prefix_body.front().norm() <= 1e-9;
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
