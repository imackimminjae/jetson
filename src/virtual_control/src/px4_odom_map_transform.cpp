#include "virtual_control/px4_odom_map_transform.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace virtual_control
{
namespace px4_odom_map
{

namespace
{

bool finiteRawPose(const RawPose & pose)
{
  return std::isfinite(pose.x) && std::isfinite(pose.y) &&
         std::isfinite(pose.z) && std::isfinite(pose.yaw);
}

}  // namespace

double RouteAnchor::targetYaw() const
{
  return std::atan2(second[1] - first[1], second[0] - first[0]);
}

std::optional<RouteAnchor> routeAnchorForName(const std::string & route_name)
{
  // Spawn positions from data/knu_routes/knu_start_poses.json.
  // Translate both anchor points equally to preserve the existing yaw.
  // Mission waypoint CSVs remain unchanged.
  if (route_name == "scenario1") {
    return RouteAnchor{{150.0, -140.0, 0.0}, {130.0, -103.0, 0.0}};
  }
  if (route_name == "scenario2") {
    return RouteAnchor{{-252.789, 93.548, 0.0}, {-209.0, 86.0, 0.0}};
  }
  if (route_name == "scenario3") {
    return RouteAnchor{{187.0, 220.0, 0.0}, {187.0, 192.0, 0.0}};
  }
  if (route_name == "scenario4") {
    return RouteAnchor{{0.0, 7.0, 0.0}, {50.0, 5.0, 0.0}};
  }
  if (route_name == "scenario5") {
    return RouteAnchor{{-264.134000000, 99.010000000, 0.000000000}, {-259.423000000, 97.448000000, 0.000000000}};
  }
  if (route_name == "scenario6") {
    return RouteAnchor{{-196.057969795, 157.447255292, 0.000000000}, {-197.338969795, 152.668255292, 0.000000000}};
  }
  if (route_name == "scenario7") {
    return RouteAnchor{{-98.120930916, 147.747302441, 0.000000000}, {-99.594930916, 143.052302441, 0.000000000}};
  }
  if (route_name == "scenario8") {
    return RouteAnchor{{-152.533000000, -81.575000000, 0.000000000}, {-151.513000000, -76.743000000, 0.000000000}};
  }
  if (route_name == "scenario9") {
    return RouteAnchor{{-71.085000000, 69.126000000, 0.000000000}, {-75.911000000, 67.997000000, 0.000000000}};
  }
  if (route_name == "scenario10") {
    return RouteAnchor{{-135.165279652, 51.211274402, 0.000000000}, {-130.901279652, 48.929274402, 0.000000000}};
  }
  if (route_name == "scenario11") {
    return RouteAnchor{{-114.447939094, 60.604796596, 0.000000000}, {-113.466939094, 55.732796596, 0.000000000}};
  }
  if (route_name == "scenario12") {
    return RouteAnchor{{-101.089, 30.493, 0.0}, {-97.338, 27.262, 0.0}};
  }
  if (route_name == "scenario13") {
    return RouteAnchor{{197.601270328, -5.629844838, 0.000000000}, {192.764270328, -4.775844838, 0.000000000}};
  }
  if (route_name == "scenario14") {
    return RouteAnchor{{53.802, 9.079, 0.0}, {58.758, 8.954, 0.0}};
  }
  if (route_name == "scenario15") {
    return RouteAnchor{{189.401000000, 249.126000000, 0.000000000}, {189.217000000, 244.309000000, 0.000000000}};
  }
  if (route_name == "scenario16") {
    return RouteAnchor{{279.949458390, 174.262345750, 0.000000000}, {277.040458390, 170.235345750, 0.000000000}};
  }
  if (route_name == "scenario17") {
    return RouteAnchor{{157.559916070, 174.283537228, 0.000000000}, {162.443916070, 175.314537228, 0.000000000}};
  }
  if (route_name == "scenario18") {
    return RouteAnchor{{187.766898753, 249.091077624, 0.000000000}, {187.581898753, 244.190077624, 0.000000000}};
  }
  if (route_name == "scenario19") {
    return RouteAnchor{{144.185650196, -124.673628748, 0.000000000}, {143.542650196, -119.725628748, 0.000000000}};
  }
  if (route_name == "scenario20") {
    return RouteAnchor{{118.526000000, -8.623000000, 0.000000000}, {120.657000000, -4.193000000, 0.000000000}};
  }
  if (route_name == "scenario21") {
    return RouteAnchor{{-98.135040990, 147.690709055, 0.000000000}, {-99.587040990, 142.972709055, 0.000000000}};
  }
  return std::nullopt;
}

double wrapToPi(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

YawQuaternion yawQuaternion(double yaw)
{
  const double half_yaw = 0.5 * yaw;
  return YawQuaternion{std::sin(half_yaw), std::cos(half_yaw)};
}

PoseAlignment::PoseAlignment(const TransformConfig & config, const RouteAnchor & target)
: config_(config), target_(target), target_yaw_(target.targetYaw())
{
  if (!std::isfinite(config_.position_scale) ||
    std::abs(config_.position_scale) < 1e-12)
  {
    throw std::invalid_argument("position_scale must be finite and nonzero");
  }
  if (!std::isfinite(config_.x_offset) || !std::isfinite(config_.y_offset) ||
    !std::isfinite(config_.z_offset) || !std::isfinite(config_.yaw_offset_rad))
  {
    throw std::invalid_argument("axis transform offsets must be finite");
  }
  for (const double value : target_.first) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("target first waypoint must be finite");
    }
  }
  for (const double value : target_.second) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("target second waypoint must be finite");
    }
  }
  if (std::hypot(
      target_.second[0] - target_.first[0],
      target_.second[1] - target_.first[1]) < 1e-9)
  {
    throw std::invalid_argument("target waypoints must define a nonzero heading");
  }
}

SourcePose PoseAlignment::transformSource(const RawPose & raw_pose) const
{
  SourcePose source;
  if (!finiteRawPose(raw_pose)) {
    source.x = source.y = source.z = source.yaw =
      std::numeric_limits<double>::quiet_NaN();
    return source;
  }

  double x = raw_pose.x;
  double y = raw_pose.y;
  if (config_.swap_xy) {
    std::swap(x, y);
  }
  if (config_.invert_x) {
    x = -x;
  }
  if (config_.invert_y) {
    y = -y;
  }
  source.x = config_.position_scale * x + config_.x_offset;
  source.y = config_.position_scale * y + config_.y_offset;
  source.z = config_.position_scale * raw_pose.z + config_.z_offset;

  const double yaw_with_offset = wrapToPi(raw_pose.yaw + config_.yaw_offset_rad);
  double heading_x = std::cos(yaw_with_offset);
  double heading_y = std::sin(yaw_with_offset);
  if (config_.swap_xy) {
    std::swap(heading_x, heading_y);
  }
  if (config_.invert_x) {
    heading_x = -heading_x;
  }
  if (config_.invert_y) {
    heading_y = -heading_y;
  }
  heading_x *= config_.position_scale;
  heading_y *= config_.position_scale;
  source.yaw = std::atan2(heading_y, heading_x);
  return source;
}

bool PoseAlignment::transform(const RawPose & raw_pose, MapPose & map_pose)
{
  if (!finiteRawPose(raw_pose)) {
    return false;
  }

  const SourcePose source = transformSource(raw_pose);
  if (!std::isfinite(source.x) || !std::isfinite(source.y) ||
    !std::isfinite(source.z) || !std::isfinite(source.yaw))
  {
    return false;
  }

  if (!aligned_) {
    source_origin_ = source;
    yaw_delta_ = wrapToPi(target_yaw_ - source_origin_.yaw);
    aligned_ = true;
  }

  const double relative_x = source.x - source_origin_.x;
  const double relative_y = source.y - source_origin_.y;
  const double c = std::cos(yaw_delta_);
  const double s = std::sin(yaw_delta_);

  map_pose.x = target_.first[0] + c * relative_x - s * relative_y;
  map_pose.y = target_.first[1] + s * relative_x + c * relative_y;
  map_pose.z = target_.first[2] + source.z - source_origin_.z;
  map_pose.yaw = wrapToPi(target_yaw_ + source.yaw - source_origin_.yaw);

  return std::isfinite(map_pose.x) && std::isfinite(map_pose.y) &&
         std::isfinite(map_pose.z) && std::isfinite(map_pose.yaw);
}

void PoseAlignment::reset()
{
  source_origin_ = SourcePose{};
  yaw_delta_ = 0.0;
  aligned_ = false;
}

bool PoseAlignment::aligned() const
{
  return aligned_;
}

double PoseAlignment::yawDelta() const
{
  return yaw_delta_;
}

const SourcePose & PoseAlignment::sourceOrigin() const
{
  return source_origin_;
}

const RouteAnchor & PoseAlignment::target() const
{
  return target_;
}

}  // namespace px4_odom_map
}  // namespace virtual_control
