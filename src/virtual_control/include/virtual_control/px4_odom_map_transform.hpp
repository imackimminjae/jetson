#pragma once

#include <array>
#include <optional>
#include <string>

namespace virtual_control
{
namespace px4_odom_map
{

struct RouteAnchor
{
  std::array<double, 3> first{};
  std::array<double, 3> second{};

  double targetYaw() const;
};

struct TransformConfig
{
  double position_scale{1.0};
  double x_offset{0.0};
  double y_offset{0.0};
  double z_offset{0.0};
  double yaw_offset_rad{0.0};
  bool swap_xy{false};
  bool invert_x{false};
  bool invert_y{false};
};

struct RawPose
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
};

struct SourcePose
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
};

struct MapPose
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
};

struct YawQuaternion
{
  double z{0.0};
  double w{1.0};
};

std::optional<RouteAnchor> routeAnchorForName(const std::string & route_name);
double wrapToPi(double angle);
YawQuaternion yawQuaternion(double yaw);

class PoseAlignment
{
public:
  PoseAlignment(const TransformConfig & config, const RouteAnchor & target);

  bool transform(const RawPose & raw_pose, MapPose & map_pose);
  SourcePose transformSource(const RawPose & raw_pose) const;
  void reset();

  bool aligned() const;
  double yawDelta() const;
  const SourcePose & sourceOrigin() const;
  const RouteAnchor & target() const;

private:
  TransformConfig config_;
  RouteAnchor target_;
  SourcePose source_origin_{};
  double target_yaw_{0.0};
  double yaw_delta_{0.0};
  bool aligned_{false};
};

}  // namespace px4_odom_map
}  // namespace virtual_control
