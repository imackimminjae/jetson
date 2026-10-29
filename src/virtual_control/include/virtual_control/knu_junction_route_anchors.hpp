#pragma once
#include "virtual_control/px4_odom_map_transform.hpp"
namespace virtual_control { namespace px4_odom_map {
inline std::optional<RouteAnchor> junctionRouteAnchorForName(const std::string & route_name)
{
  if (route_name == "scenario26") {
    return RouteAnchor{{-187.978185000000, 67.111154000000, 0.000000000000}, {-183.252597211115, 65.477501635039, 0.000000000000}};
  }
  if (route_name == "scenario27") {
    return RouteAnchor{{-144.773092000000, -30.418107000000, 0.000000000000}, {-143.327319798824, -25.631694696165, 0.000000000000}};
  }
  if (route_name == "scenario28") {
    return RouteAnchor{{38.471750000000, 6.866342000000, 0.000000000000}, {43.471430490394, 6.809817722269, 0.000000000000}};
  }
  if (route_name == "scenario29") {
    return RouteAnchor{{122.801920000000, -89.906410000000, 0.000000000000}, {120.491696408337, -85.472126861033, 0.000000000000}};
  }
  if (route_name == "scenario30") {
    return RouteAnchor{{160.648738000000, 71.444188000000, 0.000000000000}, {163.123021659611, 75.789060883271, 0.000000000000}};
  }
  if (route_name == "scenario31") {
    return RouteAnchor{{161.400745000000, 73.264209000000, 0.000000000000}, {163.855522365508, 77.620132333323, 0.000000000000}};
  }
  return std::nullopt;
}
} }  // namespace virtual_control::px4_odom_map
