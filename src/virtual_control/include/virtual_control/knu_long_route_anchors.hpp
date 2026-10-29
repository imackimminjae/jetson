#pragma once
#include "virtual_control/px4_odom_map_transform.hpp"
namespace virtual_control { namespace px4_odom_map {
inline std::optional<RouteAnchor> longRouteAnchorForName(const std::string & route_name)
{
  if (route_name == "scenario22") {
    return RouteAnchor{{-249.435000000000, 94.518000000000, 0.000000000000}, {-244.863528608545, 92.492747097988, 0.000000000000}};
  }
  if (route_name == "scenario23") {
    return RouteAnchor{{-100.604000000000, 139.907000000000, 0.000000000000}, {-101.076869501706, 134.929410780874, 0.000000000000}};
  }
  if (route_name == "scenario24") {
    return RouteAnchor{{137.499000000000, -113.874000000000, 0.000000000000}, {134.602351404322, -109.798533534286, 0.000000000000}};
  }
  if (route_name == "scenario25") {
    return RouteAnchor{{-152.030000000000, -63.355000000000, 0.000000000000}, {-150.980994124654, -58.466279648672, 0.000000000000}};
  }
  return std::nullopt;
}
} }  // namespace virtual_control::px4_odom_map
