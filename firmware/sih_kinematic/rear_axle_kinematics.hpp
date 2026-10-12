#pragma once

#include <cmath>

// Standalone motion core for integration into the custom PX4 SIH rover.
// This file is NOT connected to the running firmware or ROS publishers.
namespace sih_kinematic
{
constexpr double kPi = 3.14159265358979323846;

struct RearAxleState
{
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
  double forward_speed_mps{0.0};
};

struct MotionSample
{
  double vx_mps{0.0};
  double vy_mps{0.0};
  double yaw_rate_radps{0.0};
  // Physical acceleration resolved in the vehicle frame, for sensor synthesis.
  // Lateral velocity is zero; lateral acceleration during a turn is NOT zero.
  double forward_acceleration_mps2{0.0};
  double lateral_acceleration_mps2{0.0};
};

// Integrate a rear-axle-center pose with zero lateral velocity. Speed varies
// linearly from state.forward_speed_mps to end_speed_mps over the step. Applied
// steering is held for this one integration step. Feed the angle AFTER any
// actuator delay/lag/limits, not a normalized steering command.
//
// Keep the existing longitudinal and steering actuator models outside this
// function. The caller owns acceleration/steering limits and input timeouts.
// Axes may be ENU or NED, provided yaw and the XY axes use the same convention.
// Invalid inputs leave both outputs untouched.
inline bool advanceRearAxle(
  RearAxleState & state, MotionSample & sample, double end_speed_mps,
  double applied_steering_rad, double wheelbase_m, double dt_sec)
{
  if (!std::isfinite(state.x_m) || !std::isfinite(state.y_m) ||
      !std::isfinite(state.yaw_rad) || !std::isfinite(state.forward_speed_mps) ||
      !std::isfinite(end_speed_mps) || !std::isfinite(applied_steering_rad) ||
      !std::isfinite(wheelbase_m) || !std::isfinite(dt_sec) ||
      wheelbase_m <= 0.0 || dt_sec <= 0.0 ||
      std::abs(applied_steering_rad) >= 0.5 * kPi) {
    return false;
  }
  const double curvature = std::tan(applied_steering_rad) / wheelbase_m;
  const double distance = (0.5 * state.forward_speed_mps + 0.5 * end_speed_mps) * dt_sec;
  const double turn = distance * curvature;
  const double half_turn = 0.5 * turn;
  // Exact circular-arc integration, with a stable straight-line limit.
  const double sinc = std::abs(half_turn) < 1e-4 ?
    1.0 - half_turn * half_turn / 6.0 + std::pow(half_turn, 4) / 120.0 :
    std::sin(half_turn) / half_turn;
  const double chord = distance * sinc;
  RearAxleState next = state;
  next.x_m += chord * std::cos(state.yaw_rad + half_turn);
  next.y_m += chord * std::sin(state.yaw_rad + half_turn);
  next.yaw_rad = std::remainder(state.yaw_rad + turn, 2.0 * kPi);
  next.forward_speed_mps = end_speed_mps;
  MotionSample motion;
  motion.vx_mps = end_speed_mps * std::cos(next.yaw_rad);
  motion.vy_mps = end_speed_mps * std::sin(next.yaw_rad);
  motion.yaw_rate_radps = end_speed_mps * curvature;
  motion.forward_acceleration_mps2 = (end_speed_mps - state.forward_speed_mps) / dt_sec;
  motion.lateral_acceleration_mps2 = end_speed_mps * motion.yaw_rate_radps;
  if (!std::isfinite(next.x_m) || !std::isfinite(next.y_m) ||
      !std::isfinite(next.yaw_rad) || !std::isfinite(motion.vx_mps) ||
      !std::isfinite(motion.vy_mps) || !std::isfinite(motion.yaw_rate_radps) ||
      !std::isfinite(motion.forward_acceleration_mps2) ||
      !std::isfinite(motion.lateral_acceleration_mps2)) {
    return false;
  }
  state = next;
  sample = motion;
  return true;
}
}  // namespace sih_kinematic
