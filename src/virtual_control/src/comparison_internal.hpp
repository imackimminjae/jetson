#pragma once
#include "virtual_control/comparison_planner.hpp"
#include "virtual_control/lower_path_geometry.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace imac_ctrl::comparison
{
using Clock = std::chrono::steady_clock;
inline double milliseconds(Clock::time_point a, Clock::time_point b)
{return std::chrono::duration<double, std::milli>(b-a).count();}
inline bool sameRoute(const std::vector<Point> & a, const std::vector<Point> & b)
{
  if (a.size()!=b.size()) {return false;}
  for (size_t i=0; i<a.size(); ++i) {if ((a[i]-b[i]).norm()>1e-8) {return false;}}
  return true;
}
inline void validate(const Input & in)
{
  const auto & c=in.settings; const auto & v=in.geometry;
  const double values[]={c.period,c.dt,c.prediction_speed,c.lower_dt,c.min_spacing,c.min_length,
    c.extra_margin,c.corridor_scan,c.projection_back,c.projection_slack,c.ring_step,c.angular_step,
    c.arc_half_angle,c.attraction,c.repulsion,c.attraction_power,c.repulsion_power,c.valley_gradient,
    c.max_connection,c.q_contour,c.q_lag,c.reward_progress,c.r_steer,c.r_steer_change,
    c.max_virtual_speed,c.steer_trust,c.progress_trust,v.wheelbase,v.length,v.width,
    v.reference_to_rear,v.max_steer,v.max_steer_rate,in.actual_speed,in.time,in.vehicle.yaw};
  for (double x : values) {if (!std::isfinite(x)) {throw std::invalid_argument("nonfinite input/config");}}
  if (!in.grid.valid() || !in.vehicle.p.allFinite() || in.actual_speed<0 ||
    c.period<=0 || c.dt<=0 || c.steps<1 || c.steps>100 || c.sqp_iterations<1 ||
    c.prediction_speed<=0 || c.lower_dt<=0 || c.min_spacing<=0 || c.min_length<=0 ||
    c.extra_margin<0 || c.corridor_scan<=0 || c.projection_back<0 || c.projection_slack<0 ||
    c.ring_step<=0 || c.angular_step<=0 || c.angular_step>0.5 ||
    c.arc_half_angle<=0 || c.arc_half_angle>=1.570796327 || c.attraction<0 ||
    c.repulsion<=0 || c.attraction_power<=0 || c.repulsion_power<=0 ||
    c.valley_gradient<0 || c.max_connection<=0 || c.q_contour<=0 || c.q_lag<=0 ||
    c.reward_progress<0 || c.r_steer<=0 || c.r_steer_change<0 || c.max_virtual_speed<=0 ||
    c.steer_trust<=0 || c.progress_trust<=0 || v.wheelbase<=0 || v.length<=0 || v.width<=0 ||
    v.reference_to_rear<0 || v.reference_to_rear>v.wheelbase || v.max_steer<=0 ||
    v.max_steer>=1.4 || v.max_steer_rate<=0 ||
    (in.applied_steer && !std::isfinite(*in.applied_steer)) ||
    (in.progress && !std::isfinite(*in.progress))) {throw std::invalid_argument("invalid input/config");}
}
inline double projectProgress(const Input & in, const ReferenceCurve & curve,
  const std::optional<double> & previous, double previous_time)
{
  const auto seed=previous ? previous : in.progress;
  if (!seed) {return curve.project(in.vehicle.p, 0, curve.length());}
  const double elapsed=previous ? std::max(0.0,in.time-previous_time) : 0;
  return curve.project(in.vehicle.p, *seed-in.settings.projection_back,
    *seed+in.actual_speed*elapsed+in.settings.projection_slack);
}
inline void finish(Output & out, const Input & in)
{
  if (!densify(out.sparse, std::max(in.settings.min_spacing,
    in.settings.prediction_speed*in.settings.lower_dt), out.path, out.s) ||
    out.s.back()<in.settings.min_length) {throw std::runtime_error("path too short");}
  const auto geometry=prepareLowerPath(out.path, &out.s);
  if (!geometry.valid) {throw std::runtime_error(geometry.status);}
  // Densification can remove polyline corners. Recheck the emitted geometry.
  double previous_yaw=in.vehicle.yaw;
  for (size_t i=0; i+1<out.path.size(); ++i) {
    const double yaw=std::atan2(out.path[i+1].y()-out.path[i].y(),
      out.path[i+1].x()-out.path[i].x());
    if (!connectionFree(in, {out.path[i],previous_yaw}, {out.path[i],yaw}) ||
      !connectionFree(in, {out.path[i],yaw}, {out.path[i+1],yaw})) {
      throw std::runtime_error("emitted path connector collision");
    }
    previous_yaw=yaw;
  }
  out.success=true; out.reason="ok";
}
inline void fail(Output & out, const std::exception & error)
{
  out.success=false; out.reason=error.what(); out.path.clear(); out.s.clear();
  out.sparse.clear();
}
}  // namespace imac_ctrl::comparison
