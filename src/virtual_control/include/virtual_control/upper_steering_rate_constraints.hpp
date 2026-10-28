#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace imac_ctrl::upper_steering_rate
{
struct Feedback
{
  bool valid{false};
  double angle{0.0};
  double stamp{0.0};
  double received{0.0};
  bool vehicle_valid{false};
  double command{0.0}, tau{0.0}, delay{0.0}, offset{0.0};
  bool actuator_enabled{false};
};
inline Feedback parseFeedback(const std::vector<double> & trace, double received)
{
  Feedback out;
  // lower_mpc_trace[19] is the effective-steering ESTIMATE, not wheel feedback.
  if (trace.size() < 22 || !std::isfinite(received) || !std::isfinite(trace[0]) ||
    trace[0] <= 0.0 || !std::isfinite(trace[19]) ||
    std::abs(trace[19]) >= 1.5707963267948966) {return out;}
  out = {true, trace[19], trace[0], received};
  if (trace.size() >= 32 && std::isfinite(trace[20]) && std::isfinite(trace[21]) &&
    std::isfinite(trace[22]) && std::isfinite(trace[23]) && std::isfinite(trace[30]) &&
    std::abs(trace[20]) < 1.5707963267948966 &&
    (trace[21] == 0.0 || trace[21] == 1.0) && trace[22] >= 0.0 && trace[30] >= 0.0 &&
    (trace[21] == 0.0 || trace[23] > 0.0))
  {
    out.vehicle_valid = true; out.command = trace[20]; out.tau = trace[23];
    out.delay = trace[22]; out.offset = trace[30]; out.actuator_enabled = trace[21] == 1.0;
  }
  return out;
}
inline bool fresh(const Feedback & f, double now, double timeout)
{
  return f.valid && std::isfinite(now) && std::isfinite(timeout) && timeout > 0.0 &&
    now >= f.stamp && now >= f.received &&
    now-f.stamp <= timeout && now-f.received <= timeout;
}
struct Settings
{
  double rate_radps{0.0};  // Zero disables the feature.
  double dt{0.5};
  double wheelbase{2.8};
  double speed{6.0};
  double max_dv{0.5};
  double max_angle{0.3};
  double initial_angle{0.0};
};
struct Row {std::vector<double> coefficients; double bound{0.0};};
struct Constraints
{
  bool valid{true};
  std::string status;
  std::vector<Row> rows;
};

// Rows act on TOTAL inputs [dv0, dpsi0, ...], including the reference increments.
// v[k] = speed + sum(dv[0:k]); delta[k] = atan(L*dpsi[k]/(dt*v[k])).
// The first angle bounds are exact. Later bounds are sufficient (conservative):
// |atan(q[k])-atan(q[k-1])| <= |q[k]-q[k-1]|
// <= (L/dt*|dpsi[k]-dpsi[k-1]| + tan(max_angle)*|dv[k]|)/v_min[k].
// Four linear sign combinations express the two absolute values without
// assuming that dv=0. Existing max-angle/acceleration bounds must also be present.
inline Constraints build(int horizon, const Settings & s)
{
  Constraints out;
  if (s.rate_radps == 0.0) {return out;}
  if (horizon < 1 || !std::isfinite(s.rate_radps) || s.rate_radps < 0.0 ||
    !std::isfinite(s.dt) || s.dt <= 0.0 || !std::isfinite(s.wheelbase) ||
    s.wheelbase <= 0.0 || !std::isfinite(s.speed) || s.speed <= 0.0 ||
    !std::isfinite(s.max_dv) || s.max_dv < 0.0 ||
    !std::isfinite(s.max_angle) || s.max_angle <= 0.0 ||
    s.max_angle >= 1.5707963267948966 || !std::isfinite(s.initial_angle) ||
    std::abs(s.initial_angle) >= 1.5707963267948966)
  {
    out.valid = false; out.status = "invalid steering-rate settings"; return out;
  }
  const double step = s.rate_radps * s.dt;
  if (!std::isfinite(step)) {
    out.valid = false; out.status = "nonfinite steering-rate step"; return out;
  }
  const double lo = std::max(-s.max_angle, s.initial_angle - step);
  const double hi = std::min(s.max_angle, s.initial_angle + step);
  if (lo > hi) {
    out.valid = false; out.status = "initial steering cannot reach angle bounds"; return out;
  }
  const int n = 2 * horizon;
  auto add = [&](double bound) -> std::vector<double> & {
      out.rows.push_back({std::vector<double>(n, 0.0), bound});
      return out.rows.back().coefficients;
    };
  const double factor = s.dt / s.wheelbase;
  auto & upper = add(factor * std::tan(hi) * s.speed);
  upper[0] = -factor * std::tan(hi); upper[1] = 1.0;
  auto & lower = add(-factor * std::tan(lo) * s.speed);
  lower[0] = factor * std::tan(lo); lower[1] = -1.0;
  const double floor = std::min(0.1, s.speed);
  for (int k = 0; k < horizon; ++k) {
    // Needed for a well-defined equivalent steering at very long horizons.
    auto & positive_speed = add(s.speed - floor);
    for (int j = 0; j <= k; ++j) {positive_speed[2*j] = -1.0;}
    if (k == 0) {continue;}
    const double minimum_speed = std::max(floor, s.speed - (k+1)*s.max_dv);
    for (double turn_sign : {-1.0, 1.0}) {
      for (double speed_sign : {-1.0, 1.0}) {
        auto & row = add(factor * minimum_speed * step);
        row[2*k+1] = turn_sign;
        row[2*(k-1)+1] = -turn_sign;
        row[2*k] = factor * std::tan(s.max_angle) * speed_sign;
      }
    }
  }
  return out;
}

struct Report
{
  bool valid{false};
  double max_rate{std::numeric_limits<double>::quiet_NaN()};
  std::vector<double> angles;
};
inline Report evaluate(const std::vector<double> & inputs, const Settings & s,
  double angle_tolerance = 1e-6)
{
  Report out;
  if (inputs.empty() || inputs.size()%2 || !build(inputs.size()/2, s).valid ||
    s.rate_radps <= 0.0 || !std::isfinite(angle_tolerance) || angle_tolerance < 0.0)
  {return out;}
  double speed = s.speed, previous = s.initial_angle;
  out.valid = true; out.max_rate = 0.0;
  for (std::size_t i = 0; i < inputs.size(); i += 2) {
    speed += inputs[i];
    if (!std::isfinite(speed) || speed < std::min(0.1, s.speed)-1e-9 ||
      !std::isfinite(inputs[i+1])) {out.valid = false; return out;}
    const double angle = std::atan(s.wheelbase*inputs[i+1]/(s.dt*speed));
    out.angles.push_back(angle);
    out.max_rate = std::max(out.max_rate, std::abs(angle-previous)/s.dt);
    if (std::abs(angle) > s.max_angle+angle_tolerance ||
      std::abs(angle-previous) > s.rate_radps*s.dt+angle_tolerance)
    {out.valid = false;}
    previous = angle;
  }
  return out;
}
template<typename Points>
Report evaluateGeometry(const Points & points, const Settings & s,
  double angle_tolerance = 1e-6)
{
  Report out;
  if (points.size() < 2 || !build(points.size()-1, s).valid || s.rate_radps <= 0.0) {
    return out;
  }
  double heading_before = 0.0, angle_before = s.initial_angle;
  out.valid = true; out.max_rate = 0.0;
  for (std::size_t k = 1; k < points.size(); ++k) {
    const double x = points[k][0]-points[k-1][0], y = points[k][1]-points[k-1][1];
    const double length = std::hypot(x, y);
    if (!std::isfinite(length) || length <= 1e-9) {out.valid = false; return out;}
    const double heading = std::atan2(y, x);
    const double turn = std::atan2(std::sin(heading-heading_before), std::cos(heading-heading_before));
    const double angle = std::atan(s.wheelbase*turn/length);
    out.angles.push_back(angle);
    out.max_rate = std::max(out.max_rate, std::abs(angle-angle_before)/s.dt);
    // The existing maximum-angle bound belongs to the upper model. This new
    // independent geometry check verifies RATE; it does not introduce a second,
    // different maximum-curvature bound on the legacy linearized polyline.
    if (std::abs(angle-angle_before) > s.rate_radps*s.dt+angle_tolerance) {out.valid = false;}
    heading_before = heading; angle_before = angle;
  }
  return out;
}
}  // namespace imac_ctrl::upper_steering_rate
