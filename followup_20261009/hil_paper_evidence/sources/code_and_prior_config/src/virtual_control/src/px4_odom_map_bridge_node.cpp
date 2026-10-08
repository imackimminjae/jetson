#include "virtual_control/px4_odom_map_transform.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>

namespace virtual_control
{

class Px4OdomMapBridge : public rclcpp::Node
{
public:
  Px4OdomMapBridge()
  : Node("px4_odom_map_bridge")
  {
    declare_parameter<std::string>("input_topic", "/px4/sih/odom");
    declare_parameter<std::string>("output_topic", "/px4/sih/odom_map");
    declare_parameter<bool>("publish_pose_stamped", false);
    declare_parameter<std::string>("output_pose_topic", "/motive/vehicle/pose");
    declare_parameter<std::string>("route_name", "scenario1");
    declare_parameter<bool>("enable_batch_reanchor", false);
    declare_parameter<double>("position_scale", 1.0);
    declare_parameter<double>("x_offset", 0.0);
    declare_parameter<double>("y_offset", 0.0);
    declare_parameter<double>("z_offset", 0.0);
    declare_parameter<double>("yaw_offset_rad", 0.0);
    declare_parameter<bool>("swap_xy", false);
    declare_parameter<bool>("invert_x", false);
    declare_parameter<bool>("invert_y", false);
    declare_parameter<double>("velocity_lpf_alpha", 0.4);
    declare_parameter<double>("yaw_rate_lpf_alpha", 0.2);
    declare_parameter<double>("max_velocity_dt_sec", 0.5);
    declare_parameter<double>("input_timeout_sec", 0.6);
    declare_parameter<bool>("output_yaw_is_orientation_z", false);

    input_topic_ = get_parameter("input_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    publish_pose_stamped_ = get_parameter("publish_pose_stamped").as_bool();
    output_pose_topic_ = get_parameter("output_pose_topic").as_string();
    route_name_ = get_parameter("route_name").as_string();
    transform_config_.position_scale = get_parameter("position_scale").as_double();
    transform_config_.x_offset = get_parameter("x_offset").as_double();
    transform_config_.y_offset = get_parameter("y_offset").as_double();
    transform_config_.z_offset = get_parameter("z_offset").as_double();
    transform_config_.yaw_offset_rad = get_parameter("yaw_offset_rad").as_double();
    transform_config_.swap_xy = get_parameter("swap_xy").as_bool();
    transform_config_.invert_x = get_parameter("invert_x").as_bool();
    transform_config_.invert_y = get_parameter("invert_y").as_bool();
    velocity_lpf_alpha_ = get_parameter("velocity_lpf_alpha").as_double();
    yaw_rate_lpf_alpha_ = get_parameter("yaw_rate_lpf_alpha").as_double();
    max_velocity_dt_sec_ = get_parameter("max_velocity_dt_sec").as_double();
    input_timeout_sec_ = get_parameter("input_timeout_sec").as_double();
    output_yaw_is_orientation_z_ =
      get_parameter("output_yaw_is_orientation_z").as_bool();

    validateParameters();
    const auto target = px4_odom_map::routeAnchorForName(route_name_);
    if (!target.has_value()) {
      throw std::invalid_argument(
              "route_name must be scenario1 through scenario21");
    }
    alignment_ = std::make_unique<px4_odom_map::PoseAlignment>(transform_config_, *target);
    if (get_parameter("enable_batch_reanchor").as_bool()) {
      route_parameter_callback_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> & parameters) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = false;
          if (parameters.size() != 1 || parameters[0].get_name() != "route_name" ||
            parameters[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING)
          {
            result.reason = "batch bridge accepts only a single route_name update";
            return result;
          }
          const auto name = parameters[0].as_string();
          const auto anchor = px4_odom_map::routeAnchorForName(name);
          if (!anchor) {
            result.reason = "unknown KNU route";
            return result;
          }
          std::lock_guard<std::mutex> lock(mutex_);
          alignment_ = std::make_unique<px4_odom_map::PoseAlignment>(transform_config_, *anchor);
          route_name_ = name;
          clearVelocityState();
          result.successful = true;
          return result;
        });
    }

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    publisher_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, qos);
    if (publish_pose_stamped_) {
      pose_publisher_ =
        create_publisher<geometry_msgs::msg::PoseStamped>(output_pose_topic_, qos);
    }
    subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      input_topic_, qos,
      std::bind(&Px4OdomMapBridge::poseCallback, this, std::placeholders::_1));
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "/px4/reset_map_alignment",
      std::bind(
        &Px4OdomMapBridge::resetAlignment, this,
        std::placeholders::_1, std::placeholders::_2));

    const double monitor_period_sec = std::clamp(input_timeout_sec_ * 0.5, 0.1, 1.0);
    monitor_timer_ = create_wall_timer(
      std::chrono::duration<double>(monitor_period_sec),
      std::bind(&Px4OdomMapBridge::monitorInput, this));

    RCLCPP_INFO(
      get_logger(),
      "PX4 odom map bridge ready: input=%s (yaw scalar z) odom=%s pose=%s route=%s "
      "target=(%.3f, %.3f, %.3f) target_yaw=%.6f rad odom_yaw=%s",
      input_topic_.c_str(), output_topic_.c_str(),
      publish_pose_stamped_ ? output_pose_topic_.c_str() : "disabled", route_name_.c_str(),
      target->first[0], target->first[1], target->first[2], target->targetYaw(),
      output_yaw_is_orientation_z_ ? "orientation.z scalar" : "quaternion");
  }

private:
  using SteadyClock = std::chrono::steady_clock;

  void validateParameters() const
  {
    if (input_topic_.empty() || output_topic_.empty() ||
      (publish_pose_stamped_ && output_pose_topic_.empty()))
    {
      throw std::invalid_argument("input_topic and output_topic must not be empty");
    }
    if (!std::isfinite(velocity_lpf_alpha_) ||
      velocity_lpf_alpha_ < 0.0 || velocity_lpf_alpha_ > 1.0)
    {
      throw std::invalid_argument("velocity_lpf_alpha must be in [0, 1]");
    }
    if (!std::isfinite(yaw_rate_lpf_alpha_) ||
      yaw_rate_lpf_alpha_ < 0.0 || yaw_rate_lpf_alpha_ > 1.0)
    {
      throw std::invalid_argument("yaw_rate_lpf_alpha must be in [0, 1]");
    }
    if (!std::isfinite(max_velocity_dt_sec_) || max_velocity_dt_sec_ <= 1e-4) {
      throw std::invalid_argument("max_velocity_dt_sec must be > 1e-4");
    }
    if (!std::isfinite(input_timeout_sec_) || input_timeout_sec_ <= 0.0) {
      throw std::invalid_argument("input_timeout_sec must be > 0");
    }
  }

  static int64_t stampNanoseconds(const builtin_interfaces::msg::Time & stamp)
  {
    if (stamp.sec < 0) {
      return 0;
    }
    return static_cast<int64_t>(stamp.sec) * 1000000000LL +
           static_cast<int64_t>(stamp.nanosec);
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    const auto receive_time = SteadyClock::now();
    const px4_odom_map::RawPose raw{
      message->pose.position.x,
      message->pose.position.y,
      message->pose.position.z,
      px4_odom_map::wrapToPi(message->pose.orientation.z)};

    if (!std::isfinite(raw.x) || !std::isfinite(raw.y) ||
      !std::isfinite(raw.z) || !std::isfinite(raw.yaw))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejected non-finite PX4 source pose");
      return;
    }

    nav_msgs::msg::Odometry output;
    geometry_msgs::msg::PoseStamped pose_output;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const bool was_aligned = alignment_->aligned();
      px4_odom_map::MapPose map_pose;
      if (!alignment_->transform(raw, map_pose)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "PX4 source pose could not be transformed");
        return;
      }

      if (!was_aligned) {
        const auto & source = alignment_->sourceOrigin();
        RCLCPP_INFO(
          get_logger(),
          "Map alignment initialized once: source=(%.6f, %.6f, %.6f, yaw=%.6f) "
          "yaw_delta=%.6f input_frame='%s' (treated as PX4 local)",
          source.x, source.y, source.z, source.yaw,
          alignment_->yawDelta(), message->header.frame_id.c_str());
      }

      output.header.stamp = message->header.stamp;
      output.header.frame_id = "map";
      output.child_frame_id = "base_link";
      output.pose.pose.position.x = map_pose.x;
      output.pose.pose.position.y = map_pose.y;
      output.pose.pose.position.z = map_pose.z;
      output.pose.pose.orientation.x = 0.0;
      output.pose.pose.orientation.y = 0.0;
      if (output_yaw_is_orientation_z_) {
        // Project convention: publish wrapped map yaw directly in radians.
        // This field is deliberately not a quaternion in scalar-yaw mode.
        output.pose.pose.orientation.z = map_pose.yaw;
        output.pose.pose.orientation.w = 0.0;
      } else {
        const auto quaternion = px4_odom_map::yawQuaternion(map_pose.yaw);
        output.pose.pose.orientation.z = quaternion.z;
        output.pose.pose.orientation.w = quaternion.w;
      }

      if (pose_publisher_) {
        pose_output.header = output.header;
        pose_output.pose.position = output.pose.pose.position;
        pose_output.pose.orientation.x = 0.0;
        pose_output.pose.orientation.y = 0.0;
        pose_output.pose.orientation.z = map_pose.yaw;
        pose_output.pose.orientation.w = 0.0;
      }

      setVelocity(message, receive_time, map_pose, output);
      previous_map_pose_ = map_pose;
      previous_receive_time_ = receive_time;
      const int64_t stamp_ns = stampNanoseconds(message->header.stamp);
      previous_stamp_ns_ = stamp_ns > 0 ? std::optional<int64_t>(stamp_ns) : std::nullopt;
      have_previous_pose_ = true;
      last_valid_receive_time_ = receive_time;
    }

    publisher_->publish(output);
    if (pose_publisher_) {
      pose_publisher_->publish(pose_output);
    }
  }

  void setVelocity(
    const geometry_msgs::msg::PoseStamped::SharedPtr & message,
    const SteadyClock::time_point & receive_time,
    const px4_odom_map::MapPose & map_pose,
    nav_msgs::msg::Odometry & output)
  {
    output.twist.twist.linear.x = 0.0;
    output.twist.twist.linear.y = 0.0;
    output.twist.twist.linear.z = 0.0;
    output.twist.twist.angular.z = 0.0;
    if (!have_previous_pose_) {
      have_filtered_speed_ = false;
      filtered_speed_ = 0.0;
      have_filtered_yaw_rate_ = false;
      filtered_yaw_rate_ = 0.0;
      return;
    }

    const int64_t current_stamp_ns = stampNanoseconds(message->header.stamp);
    double dt = std::chrono::duration<double>(receive_time - previous_receive_time_).count();
    if (current_stamp_ns > 0 && previous_stamp_ns_.has_value() &&
      current_stamp_ns > *previous_stamp_ns_)
    {
      dt = static_cast<double>(current_stamp_ns - *previous_stamp_ns_) * 1e-9;
    }

    if (!std::isfinite(dt) || dt <= 1e-4 || dt > max_velocity_dt_sec_) {
      have_filtered_speed_ = false;
      filtered_speed_ = 0.0;
      have_filtered_yaw_rate_ = false;
      filtered_yaw_rate_ = 0.0;
      return;
    }

    const double measured_vx = (map_pose.x - previous_map_pose_.x) / dt;
    const double measured_vy = (map_pose.y - previous_map_pose_.y) / dt;
    const double measured_vz = (map_pose.z - previous_map_pose_.z) / dt;
    const double measured_speed = std::hypot(measured_vx, measured_vy);
    const double yaw_rate =
      px4_odom_map::wrapToPi(map_pose.yaw - previous_map_pose_.yaw) / dt;
    if (!std::isfinite(measured_vx) || !std::isfinite(measured_vy) ||
      !std::isfinite(measured_vz) || !std::isfinite(measured_speed) ||
      !std::isfinite(yaw_rate))
    {
      have_filtered_speed_ = false;
      filtered_speed_ = 0.0;
      have_filtered_yaw_rate_ = false;
      filtered_yaw_rate_ = 0.0;
      return;
    }

    filtered_speed_ = have_filtered_speed_ ?
      velocity_lpf_alpha_ * measured_speed +
      (1.0 - velocity_lpf_alpha_) * filtered_speed_ : measured_speed;
    have_filtered_speed_ = true;
    // LOCAL_POSITION_NED and ATTITUDE are asynchronous. The PoseStamped yaw
    // can therefore repeat for several position samples and then jump,
    // producing alternating zero/impulse finite differences. Filter the
    // derivative without changing the pose or yaw alignment itself.
    filtered_yaw_rate_ = have_filtered_yaw_rate_ ?
      yaw_rate_lpf_alpha_ * yaw_rate +
      (1.0 - yaw_rate_lpf_alpha_) * filtered_yaw_rate_ : yaw_rate;
    have_filtered_yaw_rate_ = true;

    if (measured_speed > 1e-9) {
      const double speed_ratio = filtered_speed_ / measured_speed;
      output.twist.twist.linear.x = speed_ratio * measured_vx;
      output.twist.twist.linear.y = speed_ratio * measured_vy;
    }
    output.twist.twist.linear.z = measured_vz;
    output.twist.twist.angular.z = filtered_yaw_rate_;
  }

  void resetAlignment(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;
    std::lock_guard<std::mutex> lock(mutex_);
    alignment_->reset();
    clearVelocityState();
    response->success = true;
    response->message = "alignment reset; next valid PX4 pose will become the source origin";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
  }

  void clearVelocityState()
  {
    have_previous_pose_ = false;
    have_filtered_speed_ = false;
    filtered_speed_ = 0.0;
    have_filtered_yaw_rate_ = false;
    filtered_yaw_rate_ = 0.0;
    previous_stamp_ns_.reset();
  }

  void monitorInput()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!last_valid_receive_time_.has_value()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for valid PX4 source pose on %s", input_topic_.c_str());
      return;
    }
    const double age = std::chrono::duration<double>(
      SteadyClock::now() - *last_valid_receive_time_).count();
    if (age > input_timeout_sec_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "PX4 source pose is stale: topic=%s age=%.3f sec",
        input_topic_.c_str(), age);
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  bool publish_pose_stamped_{false};
  std::string output_pose_topic_;
  std::string route_name_;
  px4_odom_map::TransformConfig transform_config_;
  double velocity_lpf_alpha_{0.4};
  double yaw_rate_lpf_alpha_{0.2};
  double max_velocity_dt_sec_{0.5};
  double input_timeout_sec_{0.6};
  bool output_yaw_is_orientation_z_{false};

  std::unique_ptr<px4_odom_map::PoseAlignment> alignment_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr route_parameter_callback_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;

  std::mutex mutex_;
  bool have_previous_pose_{false};
  bool have_filtered_speed_{false};
  double filtered_speed_{0.0};
  bool have_filtered_yaw_rate_{false};
  double filtered_yaw_rate_{0.0};
  px4_odom_map::MapPose previous_map_pose_{};
  SteadyClock::time_point previous_receive_time_{};
  std::optional<int64_t> previous_stamp_ns_;
  std::optional<SteadyClock::time_point> last_valid_receive_time_;
};

}  // namespace virtual_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<virtual_control::Px4OdomMapBridge>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("px4_odom_map_bridge"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
