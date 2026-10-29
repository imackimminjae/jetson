#pragma once
#include "virtual_control/px4_odom_map_transform.hpp"
namespace virtual_control { namespace px4_odom_map {
inline std::optional<RouteAnchor> longRouteAnchorForName(const std::string & route_name)
{
  if (route_name == "scenario22") {
    return RouteAnchor{{-250.407000000000, 92.324000000000, 0.000000000000}, {-245.835528608545, 90.298747097988, 0.000000000000}};
  }
  if (route_name == "scenario23") {
    return RouteAnchor{{-100.604000000000, 139.907000000000, 0.000000000000}, {-101.076869501706, 134.929410780874, 0.000000000000}};
  }
  if (route_name == "scenario24") {
    return RouteAnchor{{187.966000000000, 236.679000000000, 0.000000000000}, {187.845340739917, 231.680456077721, 0.000000000000}};
  }
  if (route_name == "scenario25") {
    return RouteAnchor{{-152.030000000000, -63.355000000000, 0.000000000000}, {-150.980994124654, -58.466279648672, 0.000000000000}};
  }
  return std::nullopt;
}
} }  // namespace virtual_control::px4_odom_map
