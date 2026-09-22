#pragma once

#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace imac_ctrl
{

// High-rate lateral QP-MPC and the sole owner of virtual/Pixhawk actuation.
class LowerTrackingMpcNode : public rclcpp::Node
{
public:
  explicit LowerTrackingMpcNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~LowerTrackingMpcNode() override;

private:
  friend struct LowerTrackingMpcTestAccess;
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace imac_ctrl
