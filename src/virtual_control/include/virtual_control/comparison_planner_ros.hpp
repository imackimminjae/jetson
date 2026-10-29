#pragma once
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

namespace imac_ctrl::comparison
{
std::shared_ptr<rclcpp::Node> makePlannerNode(const std::string & planner);
}  // namespace imac_ctrl::comparison
