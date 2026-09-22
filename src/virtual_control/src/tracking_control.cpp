#include "virtual_control/tracking_control.hpp"

#include "virtual_control/egocentric_planner_geometry.hpp"
#include "virtual_control/miqp_solution_validation.hpp"

#include "QuadraticProblem.h"
#include "matrix_utils.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <functional>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <utility>

using std::placeholders::_1;

namespace
{

constexpr double kPi = 3.141592653589793238462643383279502884;

std::string branchJsonNumber(double value)
{
  if (!std::isfinite(value)) {return "null";}
  std::ostringstream out; out << std::setprecision(17) << value; return out.str();
}

std::string branchJsonString(const std::string & value)
{
  std::ostringstream out; out << '"';
  for (unsigned char ch : value) {
    if (ch == '"' || ch == '\\') {out << '\\' << ch;}
    else if (ch < 0x20) {out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(ch);}
    else {out << ch;}
  }
  out << '"'; return out.str();
}

double clampd(double value, double lower, double upper)
{
  return std::max(lower, std::min(value, upper));
}

int clampi(int value, int lower, int upper)
{
  return std::max(lower, std::min(value, upper));
}

double wrapToPi(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double radToDeg(double angle_rad)
{
  return angle_rad * 180.0 / kPi;
}

bool finitePoint(const Eigen::Vector2d & point)
{
  return std::isfinite(point.x()) && std::isfinite(point.y());
}

std::string trimCopy(const std::string & value)
{
  const auto begin = value.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return {};
  }
  const auto end = value.find_last_not_of(" \t\r\n");
  return value.substr(begin, end - begin + 1);
}

std::string normalizedFrameId(const std::string & frame_id)
{
  std::string normalized = trimCopy(frame_id);
  while (!normalized.empty() && normalized.front() == '/') {
    normalized.erase(normalized.begin());
  }
  return normalized;
}

bool frameIdsEquivalent(const std::string & lhs, const std::string & rhs)
{
  return normalizedFrameId(lhs) == normalizedFrameId(rhs);
}

std::vector<std::string> splitCsvLine(const std::string & line)
{
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) {
    fields.push_back(trimCopy(field));
  }
  return fields;
}

bool parseDouble(const std::string & text, double & value)
{
  try {
    std::size_t consumed = 0;
    value = std::stod(trimCopy(text), &consumed);
    return consumed > 0 && std::isfinite(value);
  } catch (...) {
    return false;
  }
}

bool csvTruthy(const std::string & text)
{
  const std::string value = trimCopy(text);
  return value == "1" || value == "true" || value == "TRUE" || value == "yes";
}

bool pathsExactlyEqual(
  const std::vector<Eigen::Vector2d> & lhs,
  const std::vector<Eigen::Vector2d> & rhs)
{
  if (lhs.size() != rhs.size()) {
    return false;
  }
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    if (lhs[i].x() != rhs[i].x() || lhs[i].y() != rhs[i].y()) {
      return false;
    }
  }
  return true;
}

int closestSegmentIndex(
  const std::vector<Eigen::Vector2d> & path,
  const Eigen::Vector2d & point)
{
  if (path.size() < 2 || !finitePoint(point)) {
    return 0;
  }
  int best_index = 0;
  double best_distance_squared = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const Eigen::Vector2d segment = path[i + 1] - path[i];
    const double length_squared = segment.squaredNorm();
    if (length_squared <= 1e-12) {
      continue;
    }
    const double ratio = clampd(
      (point - path[i]).dot(segment) / length_squared, 0.0, 1.0);
    const double distance_squared =
      (point - (path[i] + ratio * segment)).squaredNorm();
    if (distance_squared < best_distance_squared) {
      best_distance_squared = distance_squared;
      best_index = static_cast<int>(i);
    }
  }
  return best_index;
}

double quaternionYaw(const geometry_msgs::msg::Quaternion & orientation)
{
  tf2::Quaternion quaternion(
    orientation.x, orientation.y, orientation.z, orientation.w);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
  return yaw;
}

geometry_msgs::msg::Quaternion yawQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion orientation;
  orientation.z = std::sin(0.5 * yaw);
  orientation.w = std::cos(0.5 * yaw);
  return orientation;
}

}  // namespace

namespace imac_ctrl
{

bool SdMapUpperPlannerNode::hasIntervalReference(const IntervalCenteringState & state)
{
  return std::isfinite(state.reference_length) && state.reference_length > 0.0;
}

// Pure explicit state transition; called once after extracting the entire preview.
SdMapUpperPlannerNode::IntervalCenteringUpdate
SdMapUpperPlannerNode::updateIntervalCentering(
  const IntervalCenteringState & previous,
  const std::vector<std::vector<IntervalObservation>> & preview,
  const IntervalCenteringSettings & settings)
{
  IntervalCenteringUpdate info;
  info.state = previous;
  info.previous_reference_length = previous.reference_length;
  for (std::size_t k = 0; k < preview.size(); ++k) {
    const auto & intervals = preview[k];
    std::size_t containing_count = 0U;
    std::size_t containing_index = 0U;
    // Count containment before testing reliability: an unreliable overlapping
    // interval still makes the reference's membership ambiguous.
    for (std::size_t i = 0; i < intervals.size(); ++i) {
      if (intervals[i].reference_inside) {
        ++containing_count;
        containing_index = i;
      }
    }
    if (containing_count != 1U) {continue;}
    const auto & interval = intervals[containing_index];
    // Unknown/map-edge clipping is represented by an unobserved boundary.
    if (interval.nominal_fallback || !interval.start_boundary_observed ||
      !interval.end_boundary_observed || !std::isfinite(interval.length) || interval.length <= 0.0)
    {
      continue;
    }
    if (info.candidate_step < 0 || interval.length < info.candidate_length) {
      info.candidate_length = interval.length;
      info.candidate_step = static_cast<int>(k);
      info.candidate_interval = static_cast<int>(containing_index);
    }
  }

  if (info.candidate_step >= 0 &&
    (!hasIntervalReference(previous) ||
    info.candidate_length <= settings.relative_length_factor * previous.reference_length))
  {
    info.state.reference_length = info.candidate_length;
  }
  return info;
}

SdMapUpperPlannerNode::IntervalTreatment
SdMapUpperPlannerNode::intervalTreatment(
  const IntervalObservation & interval, const IntervalCenteringState & state,
  const IntervalCenteringSettings & settings, double soft_ratio,
  double boundary_margin, double fallback_max_centering_length)
{
  IntervalTreatment result;
  // Fallback keeps its independent threshold; all observed stages share 1.5*B.
  if (interval.nominal_fallback) {
    result.threshold = fallback_max_centering_length;
  } else if (hasIntervalReference(state)) {
    result.threshold = settings.relative_length_factor * state.reference_length;
  }
  result.centering_on = std::isfinite(result.threshold) && interval.length <= result.threshold;
  result.inset = result.centering_on ? (1.0 - soft_ratio) * interval.length :
    std::min(boundary_margin, 0.45 * interval.length);
  return result;
}

SdMapUpperPlannerNode::SdMapUpperPlannerNode(const rclcpp::NodeOptions & options)
: Node("upper_planner_node", options)
{
  declareAndLoadParameters();
  validateParameters();
  createInterfaces();
  csv_path_loaded_ = loadReferencePathCsvIfConfigured();

  const auto period = std::chrono::duration<double>(1.0 / upper_planner_rate_hz_);
  planner_timer_ = create_wall_timer(period, std::bind(&SdMapUpperPlannerNode::plannerLoop, this));

  RCLCPP_INFO(
    get_logger(),
    "Egocentric MIQP planner ready: rate=%.2fHz pred_dt=%.3fs Nmax=%d "
    "lower_dt=%.3fs target_v=%.3fm/s scale=%s "
    "wheelbase=%.2fm length=%.2fm "
    "bev=%.2fm margin=%.2fm state=%s grid=%s sparse=%s dense=%s",
    upper_planner_rate_hz_, upper_prediction_dt_sec_, upper_preview_steps_,
    lower_prediction_dt_sec_, target_speed_mps_, scale_mode_.c_str(), wheelbase_,
    vehicle_length_m_, bev_forward_m_,
    preview_interval_boundary_margin_m_, state_input_type_.c_str(),
    grid_map_topic_.c_str(), sparse_path_topic_.c_str(), dense_path_topic_.c_str());
}

void SdMapUpperPlannerNode::declareAndLoadParameters()
{
  declare_parameter<std::string>("scale_mode", scale_mode_);
  get_parameter("scale_mode", scale_mode_);
  if (scale_mode_ == "fullscale") {
    wheelbase_ = 2.80;
    vehicle_length_m_ = 4.70;
    bev_forward_m_ = 20.0;
    waypoint_switch_eps_m_ = 10.0;
    waypoint_switch_distance_m_ = 10.0;
  } else {
    // The model profile is also used temporarily for validation diagnostics
    // when an unsupported scale_mode is supplied.
    wheelbase_ = 0.30;
    vehicle_length_m_ = 0.42;
    bev_forward_m_ = 1.50;
    waypoint_switch_eps_m_ = 0.5;
    waypoint_switch_distance_m_ = 0.5;
  }

  declare_parameter<double>("upper_planner_rate_hz", upper_planner_rate_hz_);
  declare_parameter<double>("upper_prediction_dt_sec", upper_prediction_dt_sec_);
  declare_parameter<int>("upper_preview_steps", upper_preview_steps_);
  declare_parameter<bool>("preview_limit_to_grid_extent", preview_limit_to_grid_extent_);
  declare_parameter<int>("preview_grid_reserve_steps", preview_grid_reserve_steps_);
  declare_parameter<bool>("preview_restore_full_horizon", preview_restore_full_horizon_);
  declare_parameter<double>("lower_prediction_dt_sec", lower_prediction_dt_sec_);
  declare_parameter<double>("lower_min_path_spacing_m", lower_min_path_spacing_m_);
  declare_parameter<double>("target_speed_mps", target_speed_mps_);
  declare_parameter<double>("wheelbase", wheelbase_);
  declare_parameter<double>("vehicle_length_m", vehicle_length_m_);
  declare_parameter<double>("bev_forward_m", bev_forward_m_);
  declare_parameter<double>("max_steering_angle_rad", max_steering_angle_rad_);
  declare_parameter<double>("a_max_mps2", a_max_mps2_);

  declare_parameter<double>("miqp_big_m", miqp_big_m_);
  declare_parameter<double>("position_scale_m", position_scale_m_);
  declare_parameter<double>("relative_turn_scale_rad", relative_turn_scale_rad_);
  declare_parameter<double>("lambda_position", lambda_position_);
  declare_parameter<double>("lambda_relative_turn", lambda_relative_turn_);
  declare_parameter<double>(
    "terminal_buffer_max_turn_rad", terminal_buffer_max_turn_rad_);
  declare_parameter<double>(
    "terminal_position_weight_multiplier", terminal_position_weight_multiplier_);
  declare_parameter<double>("upper_r_dv", upper_r_dv_);
  declare_parameter<double>("upper_r_dpsi", upper_r_dpsi_);
  declare_parameter<bool>("normalize_upper_input_cost", normalize_upper_input_cost_);
  declare_parameter<int>("turn_preview_steps", turn_preview_steps_);
  declare_parameter<double>("turn_weight_growth", turn_weight_growth_);
  declare_parameter<double>("miqp_diagonal_regularization", miqp_diagonal_regularization_);
  declare_parameter<double>(
    "miqp_binary_diagonal_regularization", miqp_binary_diagonal_regularization_);
  declare_parameter<double>("miqp_feasibility_tolerance", miqp_feasibility_tolerance_);

  declare_parameter<double>("wp_switch_eps", waypoint_switch_eps_m_);
  declare_parameter<double>("wp_switch_max_distance_m", waypoint_switch_distance_m_);
  declare_parameter<double>("goal_stop_distance_m", goal_stop_distance_m_);
  declare_parameter<bool>("enable_waypoint_bias", enable_waypoint_bias_);
  declare_parameter<double>("waypoint_x_bias_m", waypoint_x_bias_m_);
  declare_parameter<double>("waypoint_y_bias_m", waypoint_y_bias_m_);
  declare_parameter<double>("waypoint_yaw_bias_rad", waypoint_yaw_bias_rad_);
  declare_parameter<double>("input_timeout_sec", input_timeout_sec_);
  declare_parameter<bool>("require_grid_map", require_grid_map_);

  declare_parameter<bool>("use_csv_global_path", use_csv_global_path_);
  declare_parameter<std::string>("csv_global_path_file", csv_global_path_file_);
  declare_parameter<bool>(
    "allow_topic_path_override_when_csv_loaded",
    allow_topic_path_override_when_csv_loaded_);
  declare_parameter<bool>("reset_waypoint_on_path_update", reset_waypoint_on_path_update_);

  declare_parameter<double>(
    "preview_min_target_distance_m", preview_min_target_distance_m_);
  declare_parameter<double>("preview_line_sample_m", preview_line_sample_m_);
  declare_parameter<int>("preview_min_segment_samples", preview_min_segment_samples_);
  declare_parameter<double>("preview_interval_soft_ratio", preview_interval_soft_ratio_);
  declare_parameter<double>(
    "preview_interval_relative_length_factor", interval_centering_settings_.relative_length_factor);
  declare_parameter<double>(
    "preview_interval_boundary_margin_m", preview_interval_boundary_margin_m_);
  declare_parameter<bool>(
    "preview_interval_enable_nominal_fallback",
    preview_interval_enable_nominal_fallback_);
  declare_parameter<double>(
    "preview_interval_nominal_fallback_width_m",
    preview_interval_nominal_fallback_width_m_);
  declare_parameter<double>(
    "preview_interval_fallback_max_centering_length_m",
    preview_interval_fallback_max_centering_length_m_);
  declare_parameter<bool>("preview_interval_debug", preview_interval_debug_);

  declare_parameter<int>("grid_value_threshold", grid_value_threshold_);
  declare_parameter<bool>("grid_positive_is_drivable", grid_positive_is_drivable_);
  declare_parameter<std::string>("expected_grid_frame_id", expected_grid_frame_id_);
  declare_parameter<bool>("require_grid_frame_match", require_grid_frame_match_);

  declare_parameter<std::string>("state_input_type", state_input_type_);
  declare_parameter<std::string>("odom_topic", odom_topic_);
  declare_parameter<bool>("odom_yaw_is_orientation_z", odom_yaw_is_orientation_z_);
  declare_parameter<std::string>("pose_stamped_topic", pose_stamped_topic_);
  declare_parameter<bool>(
    "pose_stamped_yaw_is_orientation_z", pose_stamped_yaw_is_orientation_z_);
  declare_parameter<double>("pose_x_offset", pose_x_offset_);
  declare_parameter<double>("pose_y_offset", pose_y_offset_);
  declare_parameter<double>("pose_yaw_offset_rad", pose_yaw_offset_rad_);
  declare_parameter<double>("pose_position_scale", pose_position_scale_);
  declare_parameter<bool>("pose_swap_xy", pose_swap_xy_);
  declare_parameter<bool>("pose_invert_x", pose_invert_x_);
  declare_parameter<bool>("pose_invert_y", pose_invert_y_);
  declare_parameter<double>("pose_speed_lpf_alpha", pose_speed_lpf_alpha_);
  declare_parameter<double>("pose_max_dt_for_speed", pose_max_dt_for_speed_);

  declare_parameter<std::string>("global_path_topic", global_path_topic_);
  declare_parameter<std::string>("grid_map_topic", grid_map_topic_);
  declare_parameter<std::string>("upper_sparse_path_topic", sparse_path_topic_);
  declare_parameter<std::string>("lower_reference_path_topic", dense_path_topic_);
  declare_parameter<std::string>("goal_reached_topic", goal_reached_topic_);
  declare_parameter<std::string>("path_frame_id", path_frame_id_);

  get_parameter("upper_planner_rate_hz", upper_planner_rate_hz_);
  get_parameter("upper_prediction_dt_sec", upper_prediction_dt_sec_);
  get_parameter("upper_preview_steps", upper_preview_steps_);
  get_parameter("preview_limit_to_grid_extent", preview_limit_to_grid_extent_);
  get_parameter("preview_grid_reserve_steps", preview_grid_reserve_steps_);
  get_parameter("preview_restore_full_horizon", preview_restore_full_horizon_);
  get_parameter("lower_prediction_dt_sec", lower_prediction_dt_sec_);
  get_parameter("lower_min_path_spacing_m", lower_min_path_spacing_m_);
  get_parameter("target_speed_mps", target_speed_mps_);
  get_parameter("wheelbase", wheelbase_);
  get_parameter("vehicle_length_m", vehicle_length_m_);
  get_parameter("bev_forward_m", bev_forward_m_);
  get_parameter("max_steering_angle_rad", max_steering_angle_rad_);
  get_parameter("a_max_mps2", a_max_mps2_);

  get_parameter("miqp_big_m", miqp_big_m_);
  get_parameter("position_scale_m", position_scale_m_);
  get_parameter("relative_turn_scale_rad", relative_turn_scale_rad_);
  get_parameter("lambda_position", lambda_position_);
  get_parameter("lambda_relative_turn", lambda_relative_turn_);
  get_parameter("terminal_buffer_max_turn_rad", terminal_buffer_max_turn_rad_);
  get_parameter(
    "terminal_position_weight_multiplier", terminal_position_weight_multiplier_);
  get_parameter("upper_r_dv", upper_r_dv_);
  get_parameter("upper_r_dpsi", upper_r_dpsi_);
  get_parameter("normalize_upper_input_cost", normalize_upper_input_cost_);
  get_parameter("turn_preview_steps", turn_preview_steps_);
  get_parameter("turn_weight_growth", turn_weight_growth_);
  get_parameter("miqp_diagonal_regularization", miqp_diagonal_regularization_);
  get_parameter(
    "miqp_binary_diagonal_regularization", miqp_binary_diagonal_regularization_);
  get_parameter("miqp_feasibility_tolerance", miqp_feasibility_tolerance_);

  get_parameter("wp_switch_eps", waypoint_switch_eps_m_);
  get_parameter("wp_switch_max_distance_m", waypoint_switch_distance_m_);
  get_parameter("goal_stop_distance_m", goal_stop_distance_m_);
  get_parameter("enable_waypoint_bias", enable_waypoint_bias_);
  get_parameter("waypoint_x_bias_m", waypoint_x_bias_m_);
  get_parameter("waypoint_y_bias_m", waypoint_y_bias_m_);
  get_parameter("waypoint_yaw_bias_rad", waypoint_yaw_bias_rad_);
  get_parameter("input_timeout_sec", input_timeout_sec_);
  get_parameter("require_grid_map", require_grid_map_);

  get_parameter("use_csv_global_path", use_csv_global_path_);
  get_parameter("csv_global_path_file", csv_global_path_file_);
  get_parameter(
    "allow_topic_path_override_when_csv_loaded",
    allow_topic_path_override_when_csv_loaded_);
  get_parameter("reset_waypoint_on_path_update", reset_waypoint_on_path_update_);

  get_parameter("preview_min_target_distance_m", preview_min_target_distance_m_);
  get_parameter("preview_line_sample_m", preview_line_sample_m_);
  get_parameter("preview_min_segment_samples", preview_min_segment_samples_);
  get_parameter("preview_interval_soft_ratio", preview_interval_soft_ratio_);
  get_parameter(
    "preview_interval_relative_length_factor", interval_centering_settings_.relative_length_factor);
  get_parameter("preview_interval_boundary_margin_m", preview_interval_boundary_margin_m_);
  get_parameter(
    "preview_interval_enable_nominal_fallback",
    preview_interval_enable_nominal_fallback_);
  get_parameter(
    "preview_interval_nominal_fallback_width_m",
    preview_interval_nominal_fallback_width_m_);
  get_parameter(
    "preview_interval_fallback_max_centering_length_m",
    preview_interval_fallback_max_centering_length_m_);
  get_parameter("preview_interval_debug", preview_interval_debug_);

  get_parameter("grid_value_threshold", grid_value_threshold_);
  get_parameter("grid_positive_is_drivable", grid_positive_is_drivable_);
  get_parameter("expected_grid_frame_id", expected_grid_frame_id_);
  get_parameter("require_grid_frame_match", require_grid_frame_match_);

  get_parameter("state_input_type", state_input_type_);
  get_parameter("odom_topic", odom_topic_);
  get_parameter("odom_yaw_is_orientation_z", odom_yaw_is_orientation_z_);
  get_parameter("pose_stamped_topic", pose_stamped_topic_);
  get_parameter("pose_stamped_yaw_is_orientation_z", pose_stamped_yaw_is_orientation_z_);
  get_parameter("pose_x_offset", pose_x_offset_);
  get_parameter("pose_y_offset", pose_y_offset_);
  get_parameter("pose_yaw_offset_rad", pose_yaw_offset_rad_);
  get_parameter("pose_position_scale", pose_position_scale_);
  get_parameter("pose_swap_xy", pose_swap_xy_);
  get_parameter("pose_invert_x", pose_invert_x_);
  get_parameter("pose_invert_y", pose_invert_y_);
  get_parameter("pose_speed_lpf_alpha", pose_speed_lpf_alpha_);
  get_parameter("pose_max_dt_for_speed", pose_max_dt_for_speed_);

  get_parameter("global_path_topic", global_path_topic_);
  get_parameter("grid_map_topic", grid_map_topic_);
  get_parameter("upper_sparse_path_topic", sparse_path_topic_);
  get_parameter("lower_reference_path_topic", dense_path_topic_);
  get_parameter("goal_reached_topic", goal_reached_topic_);
  get_parameter("path_frame_id", path_frame_id_);
}

void SdMapUpperPlannerNode::validateParameters() const
{
  auto fail = [this](const std::string & message) {
      RCLCPP_FATAL(get_logger(), "invalid egocentric planner parameter: %s", message.c_str());
      throw std::invalid_argument(message);
    };

  if (upper_planner_rate_hz_ <= 0.0) {fail("upper_planner_rate_hz must be > 0");}
  if (upper_prediction_dt_sec_ <= 0.0) {fail("upper_prediction_dt_sec must be > 0");}
  if (upper_preview_steps_ <= 0) {fail("upper_preview_steps must be > 0");}
  if (preview_grid_reserve_steps_ < 0) {fail("preview_grid_reserve_steps must be >= 0");}
  if (lower_prediction_dt_sec_ <= 0.0) {fail("lower_prediction_dt_sec must be > 0");}
  if (lower_min_path_spacing_m_ <= 0.0) {fail("lower_min_path_spacing_m must be > 0");}
  if (target_speed_mps_ <= 0.0) {fail("target_speed_mps must be > 0");}
  if (scale_mode_ != "model" && scale_mode_ != "fullscale") {
    fail("scale_mode must be 'model' or 'fullscale'");
  }
  if (wheelbase_ <= 0.0) {fail("wheelbase must be > 0");}
  if (vehicle_length_m_ <= 0.0) {fail("vehicle_length_m must be > 0");}
  if (bev_forward_m_ <= 0.0) {fail("bev_forward_m must be > 0");}
  if (max_steering_angle_rad_ <= 0.0 || max_steering_angle_rad_ >= 0.5 * kPi) {
    fail("max_steering_angle_rad must be in (0, pi/2)");
  }
  if (a_max_mps2_ <= 0.0) {fail("a_max_mps2 must be > 0");}
  if (miqp_big_m_ <= 0.0) {fail("miqp_big_m must be > 0");}
  if (position_scale_m_ <= 0.0) {fail("position_scale_m must be > 0");}
  if (relative_turn_scale_rad_ <= 0.0) {fail("relative_turn_scale_rad must be > 0");}
  if (lambda_position_ < 0.0 || lambda_relative_turn_ < 0.0) {
    fail("normalized objective weights must be >= 0");
  }
  if (!std::isfinite(terminal_buffer_max_turn_rad_) ||
    terminal_buffer_max_turn_rad_ <= 0.0 ||
    terminal_buffer_max_turn_rad_ > 0.5 * kPi)
  {
    fail("terminal_buffer_max_turn_rad must be in (0, pi/2]");
  }
  if (!std::isfinite(terminal_position_weight_multiplier_) ||
    terminal_position_weight_multiplier_ < 1.0)
  {
    fail("terminal_position_weight_multiplier must be >= 1");
  }
  if (upper_r_dv_ < 0.0 || upper_r_dpsi_ < 0.0) {
    fail("input regularization weights must be >= 0");
  }
  if (turn_preview_steps_ < 0) {fail("turn_preview_steps must be >= 0");}
  if (turn_weight_growth_ <= 0.0) {fail("turn_weight_growth must be > 0");}
  if (miqp_diagonal_regularization_ < 0.0) {
    fail("miqp_diagonal_regularization must be >= 0");
  }
  if (!std::isfinite(miqp_binary_diagonal_regularization_) ||
    miqp_binary_diagonal_regularization_ < 1e-4)
  {
    fail("miqp_binary_diagonal_regularization must be finite and >= 1e-4");
  }
  if (!std::isfinite(miqp_feasibility_tolerance_) ||
    miqp_feasibility_tolerance_ <= 0.0)
  {
    fail("miqp_feasibility_tolerance must be finite and > 0");
  }
  if (waypoint_switch_eps_m_ < 0.0) {fail("wp_switch_eps must be >= 0");}
  if (waypoint_switch_distance_m_ < 0.0) {
    fail("wp_switch_max_distance_m must be >= 0");
  }
  if (!std::isfinite(waypoint_x_bias_m_) || !std::isfinite(waypoint_y_bias_m_) ||
    !std::isfinite(waypoint_yaw_bias_rad_))
  {
    fail("waypoint bias parameters must be finite");
  }
  if (input_timeout_sec_ < 0.0) {fail("input_timeout_sec must be >= 0");}
  if (preview_min_target_distance_m_ <= 0.0) {
    fail("preview_min_target_distance_m must be > 0");
  }
  if (preview_line_sample_m_ <= 0.0) {fail("preview_line_sample_m must be > 0");}
  if (preview_min_segment_samples_ < 2) {
    fail("preview_min_segment_samples must be >= 2");
  }
  if (!std::isfinite(preview_interval_soft_ratio_) ||
    std::abs(preview_interval_soft_ratio_ - 0.7) > 1e-12)
  {
    fail("main9 requires fixed preview_interval_soft_ratio=0.7");
  }
  if (!std::isfinite(interval_centering_settings_.relative_length_factor) ||
    std::abs(interval_centering_settings_.relative_length_factor - 1.5) > 1e-12)
  {
    fail("interval centering requires fixed preview_interval_relative_length_factor=1.5");
  }
  if (!std::isfinite(preview_interval_boundary_margin_m_) ||
    preview_interval_boundary_margin_m_ < 0.0)
  {
    fail("preview_interval_boundary_margin_m must be finite and >= 0");
  }
  if (!std::isfinite(preview_interval_nominal_fallback_width_m_) ||
    preview_interval_nominal_fallback_width_m_ <= 0.0)
  {
    fail("preview_interval_nominal_fallback_width_m must be finite and > 0");
  }
  if (!std::isfinite(preview_interval_fallback_max_centering_length_m_) ||
    preview_interval_fallback_max_centering_length_m_ <= 0.0)
  {
    fail("preview_interval_fallback_max_centering_length_m must be finite and > 0");
  }
  if (grid_value_threshold_ < 0 || grid_value_threshold_ > 100) {
    fail("grid_value_threshold must be in [0, 100]");
  }
  if (require_grid_frame_match_ && normalizedFrameId(expected_grid_frame_id_).empty()) {
    fail("expected_grid_frame_id must be nonempty when require_grid_frame_match is true");
  }
  if (state_input_type_ != "pose_stamped" && state_input_type_ != "odom") {
    fail("state_input_type must be 'pose_stamped' or 'odom'");
  }
  if (!std::isfinite(pose_position_scale_) || std::abs(pose_position_scale_) < 1e-12) {
    fail("pose_position_scale must be finite and nonzero");
  }
  if (pose_speed_lpf_alpha_ < 0.0 || pose_speed_lpf_alpha_ > 1.0) {
    fail("pose_speed_lpf_alpha must be in [0, 1]");
  }
  if (pose_max_dt_for_speed_ <= 0.0) {fail("pose_max_dt_for_speed must be > 0");}
}

void SdMapUpperPlannerNode::createInterfaces()
{
  global_path_sub_ = create_subscription<nav_msgs::msg::Path>(
    global_path_topic_, rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&SdMapUpperPlannerNode::globalPathCallback, this, _1));
  grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    grid_map_topic_, 10, std::bind(&SdMapUpperPlannerNode::gridMapCallback, this, _1));

  if (state_input_type_ == "odom") {
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10, std::bind(&SdMapUpperPlannerNode::odomCallback, this, _1));
    RCLCPP_INFO(
      get_logger(), "planner state input: Odometry topic=%s yaw=%s",
      odom_topic_.c_str(),
      odom_yaw_is_orientation_z_ ? "orientation.z scalar" : "quaternion");
  } else {
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_stamped_topic_, 10,
      std::bind(&SdMapUpperPlannerNode::poseStampedCallback, this, _1));
    RCLCPP_INFO(
      get_logger(), "planner state input: PoseStamped topic=%s yaw=%s",
      pose_stamped_topic_.c_str(),
      pose_stamped_yaw_is_orientation_z_ ? "orientation.z scalar" : "quaternion");
  }

  auto path_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  sparse_path_pub_ = create_publisher<nav_msgs::msg::Path>(sparse_path_topic_, path_qos);
  dense_path_pub_ = create_publisher<nav_msgs::msg::Path>(dense_path_topic_, path_qos);
  dense_arc_path_pub_ = create_publisher<imac_interfaces::msg::PathWithArcLength>(
    dense_path_topic_ + "/with_arclength", path_qos);
  preview_debug_pub_ = create_publisher<nav_msgs::msg::Path>(
    "/debug/upper_preview_reference", 10);
  goal_reached_pub_ = create_publisher<std_msgs::msg::Bool>(
    goal_reached_topic_, rclcpp::QoS(1).reliable().transient_local());
  interval_debug_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    "/debug/upper_constraint_intervals", 10);
  interval_info_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    "/debug/upper_interval_info", 10);
  interval_state_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    "/debug/upper_interval_centering_state", 10);
  upper_trace_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    "/debug/upper_miqp_trace", 10);
  branch_event_pub_ = create_publisher<std_msgs::msg::String>(
    "/debug/upper_branch_event", 10);
  upper_timing_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    "/debug/upper_solver_timing", 10);
  upper_solver_time_pub_ = create_publisher<std_msgs::msg::Float64>(
    "/debug/upper_solver_time_ms", 10);
}

void SdMapUpperPlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  PoseSnapshot next;
  next.position = Eigen::Vector2d(msg->pose.pose.position.x, msg->pose.pose.position.y);
  const double raw_yaw = odom_yaw_is_orientation_z_ ?
    msg->pose.pose.orientation.z : quaternionYaw(msg->pose.pose.orientation);
  next.yaw_rad = wrapToPi(raw_yaw);
  next.speed_mps = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
  next.frame_id = msg->header.frame_id;
  next.received = now();
  next.speed_valid = std::isfinite(next.speed_mps);
  next.valid = finitePoint(next.position) && std::isfinite(next.yaw_rad) && next.speed_valid;
  std::lock_guard<std::mutex> lock(pose_mtx_);
  pose_ = next;
}

void SdMapUpperPlannerNode::poseStampedCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  double x = msg->pose.position.x;
  double y = msg->pose.position.y;
  if (pose_swap_xy_) {std::swap(x, y);}
  if (pose_invert_x_) {x = -x;}
  if (pose_invert_y_) {y = -y;}
  x = pose_position_scale_ * x + pose_x_offset_;
  y = pose_position_scale_ * y + pose_y_offset_;

  PoseSnapshot next;
  next.position = Eigen::Vector2d(x, y);
  const double raw_yaw = pose_stamped_yaw_is_orientation_z_ ?
    msg->pose.orientation.z : quaternionYaw(msg->pose.orientation);
  next.yaw_rad = wrapToPi(raw_yaw + pose_yaw_offset_rad_);
  next.frame_id = msg->header.frame_id;
  next.received = now();
  next.valid = finitePoint(next.position) && std::isfinite(next.yaw_rad);

  {
    std::lock_guard<std::mutex> lock(pose_mtx_);
    if (!next.valid) {
      next.speed_mps = 0.0;
      next.speed_valid = false;
      have_previous_pose_measurement_ = false;
    } else if (have_previous_pose_measurement_) {
      const double dt = (next.received - previous_pose_time_).seconds();
      if (dt > 1e-4 && dt <= pose_max_dt_for_speed_) {
        const double measured_speed =
          (next.position - previous_pose_measurement_).norm() / dt;
        if (std::isfinite(measured_speed)) {
          const double alpha = clampd(pose_speed_lpf_alpha_, 0.0, 1.0);
          const double previous_speed = pose_.speed_valid && std::isfinite(pose_.speed_mps) ?
            pose_.speed_mps : measured_speed;
          next.speed_mps = alpha * measured_speed + (1.0 - alpha) * previous_speed;
          next.speed_valid = std::isfinite(next.speed_mps);
        }
      } else {
        // Do not keep an old finite-difference estimate across a long or
        // non-monotonic sample interval. The next valid interval re-seeds it.
        next.speed_mps = 0.0;
        next.speed_valid = false;
      }
    } else {
      next.speed_mps = 0.0;
      next.speed_valid = false;
    }
    if (next.valid) {
      previous_pose_measurement_ = next.position;
      previous_pose_time_ = next.received;
      have_previous_pose_measurement_ = true;
    }
    pose_ = next;
  }
}

void SdMapUpperPlannerNode::globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (csv_path_loaded_ && !allow_topic_path_override_when_csv_loaded_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "ignoring global path topic because the configured CSV path is active");
    return;
  }

  std::vector<Eigen::Vector2d> path;
  path.reserve(msg->poses.size());
  for (const auto & pose : msg->poses) {
    const Eigen::Vector2d point(pose.pose.position.x, pose.pose.position.y);
    if (finitePoint(point)) {
      path.push_back(point);
    }
  }
  applyReferencePath(
    path, msg->header.frame_id.empty() ? path_frame_id_ : msg->header.frame_id,
    global_path_topic_);
}

std::vector<Eigen::Vector2d> SdMapUpperPlannerNode::applyConfiguredWaypointBias(
  const std::vector<Eigen::Vector2d> & path) const
{
  if (!enable_waypoint_bias_ || path.empty()) {
    return path;
  }

  const Eigen::Vector2d anchor = path.front();
  const double cosine = std::cos(waypoint_yaw_bias_rad_);
  const double sine = std::sin(waypoint_yaw_bias_rad_);
  Eigen::Matrix2d rotation;
  rotation << cosine, -sine, sine, cosine;
  const Eigen::Vector2d translation(waypoint_x_bias_m_, waypoint_y_bias_m_);

  std::vector<Eigen::Vector2d> biased;
  biased.reserve(path.size());
  for (const auto & point : path) {
    biased.push_back(anchor + rotation * (point - anchor) + translation);
  }
  return biased;
}

void SdMapUpperPlannerNode::applyReferencePath(
  const std::vector<Eigen::Vector2d> & path,
  const std::string & frame_id,
  const std::string & source_label)
{
  if (path.size() < 2) {
    RCLCPP_WARN(get_logger(), "ignored invalid path from %s", source_label.c_str());
    return;
  }

  const auto configured_path = applyConfiguredWaypointBias(path);
  const std::string resolved_frame = frame_id.empty() ? path_frame_id_ : frame_id;
  {
    std::lock_guard<std::mutex> lock(path_mtx_);
    const bool same_frame = frameIdsEquivalent(global_path_frame_id_, resolved_frame);
    if (same_frame && pathsExactlyEqual(global_path_, configured_path)) {
      // The global path publisher republishes the route as a heartbeat. An identical
      // path is not a new mission and must never reset waypoint progress.
      return;
    }

    const bool first_path = global_path_.size() < 2;
    bool preserve_seed = false;
    int preserved_wp0 = 0;
    if (!first_path && !reset_waypoint_on_path_update_) {
      const int old_anchor_index = clampi(
        wp0_index_, 0, static_cast<int>(global_path_.size()) - 1);
      preserved_wp0 = closestSegmentIndex(
        configured_path, global_path_[static_cast<std::size_t>(old_anchor_index)]);
      preserve_seed = true;
    }

    global_path_ = configured_path;
    global_path_frame_id_ = resolved_frame;
    if (preserve_seed) {
      wp0_index_ = clampi(
        preserved_wp0, 0, static_cast<int>(configured_path.size()) - 2);
      wp1_index_ = wp0_index_ + 1;
    } else {
      wp0_index_ = 0;
      wp1_index_ = 1;
    }
    goal_reached_ = false;
    ++planning_revision_;
  }
  // Do not nest path_mtx_ and previous_solution_mtx_. A changed path must
  // invalidate the locally re-anchored reference before another cycle can
  // consider it, while identical heartbeat paths return above unchanged.
  invalidatePreviousReference();
  publishGoalReached(false);
  RCLCPP_INFO(
    get_logger(),
    "reference path loaded from %s: %zu points frame=%s waypoint_bias=%s",
    source_label.c_str(), configured_path.size(), resolved_frame.c_str(),
    enable_waypoint_bias_ ? "enabled" : "disabled");
}

void SdMapUpperPlannerNode::invalidatePreviousReference()
{
  std::lock_guard<std::mutex> lock(previous_solution_mtx_);
  previous_optimal_path_world_.clear();
  previous_solution_valid_ = false;
}

void SdMapUpperPlannerNode::gridMapCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  GridMapSnapshot next;
  next.resolution = msg->info.resolution;
  next.width = static_cast<int>(msg->info.width);
  next.height = static_cast<int>(msg->info.height);
  next.origin_body = Eigen::Vector2d(
    msg->info.origin.position.x, msg->info.origin.position.y);
  next.origin_yaw_rad = quaternionYaw(msg->info.origin.orientation);
  next.frame_id = msg->header.frame_id;
  next.data.assign(msg->data.begin(), msg->data.end());
  next.received = now();
  next.valid = next.resolution > 0.0 && next.width > 0 && next.height > 0 &&
    static_cast<std::size_t>(next.width * next.height) == next.data.size() &&
    finitePoint(next.origin_body) && std::isfinite(next.origin_yaw_rad);
  const bool grid_frame_matches = normalizedFrameId(expected_grid_frame_id_).empty() ||
    (!normalizedFrameId(next.frame_id).empty() &&
    frameIdsEquivalent(next.frame_id, expected_grid_frame_id_));
  if (!grid_frame_matches) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "OccupancyGrid frame '%s' differs from expected ego frame '%s'",
      next.frame_id.c_str(), expected_grid_frame_id_.c_str());
    if (require_grid_frame_match_) {
      next.valid = false;
    }
  }
  {
    std::lock_guard<std::mutex> lock(grid_mtx_);
    grid_ = next;
  }
  if (!next.valid) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "invalid OccupancyGrid: resolution=%.4f size=%dx%d data=%zu",
      next.resolution, next.width, next.height, next.data.size());
  }
}

bool SdMapUpperPlannerNode::loadReferencePathCsvIfConfigured()
{
  if (!use_csv_global_path_ || csv_global_path_file_.empty()) {
    return false;
  }
  namespace fs = std::filesystem;
  const fs::path configured(csv_global_path_file_);
  std::vector<fs::path> candidates{configured};
  if (!configured.is_absolute()) {
    candidates.emplace_back(fs::path("data") / configured);
    try {
      const fs::path share = ament_index_cpp::get_package_share_directory("virtual_control");
      candidates.emplace_back(share / configured);
      candidates.emplace_back(share / "data" / configured);
    } catch (...) {
    }
  }
  fs::path resolved;
  for (const auto & candidate : candidates) {
    std::error_code error;
    if (fs::is_regular_file(candidate, error)) {
      resolved = candidate;
      break;
    }
  }
  if (resolved.empty()) {
    RCLCPP_WARN(
      get_logger(), "CSV global path not found: %s", csv_global_path_file_.c_str());
    return false;
  }

  std::ifstream file(resolved);
  std::string header_line;
  if (!file.is_open() || !std::getline(file, header_line)) {
    RCLCPP_ERROR(get_logger(), "cannot read CSV global path: %s", resolved.c_str());
    return false;
  }
  const auto header = splitCsvLine(header_line);
  auto column = [&header](const std::string & name) {
      for (std::size_t i = 0; i < header.size(); ++i) {
        if (trimCopy(header[i]) == name) {return static_cast<int>(i);}
      }
      return -1;
    };
  const int record_col = column("record_type");
  const int x_col = column("x");
  const int y_col = column("y");
  const int route_col = column("is_route");
  std::vector<Eigen::Vector2d> route_points;
  std::vector<Eigen::Vector2d> plain_points;
  std::string line;
  while (std::getline(file, line)) {
    const auto row = splitCsvLine(line);
    auto cell = [&row](int index) -> std::string {
        return index >= 0 && index < static_cast<int>(row.size()) ?
               row[static_cast<std::size_t>(index)] : std::string();
      };
    double x = 0.0;
    double y = 0.0;
    if (record_col >= 0) {
      if (cell(record_col) != "waypoint" ||
        (route_col >= 0 && !csvTruthy(cell(route_col))))
      {
        continue;
      }
      if (parseDouble(cell(x_col), x) && parseDouble(cell(y_col), y)) {
        route_points.emplace_back(x, y);
      }
    } else {
      if (parseDouble(cell(x_col >= 0 ? x_col : 0), x) &&
        parseDouble(cell(y_col >= 0 ? y_col : 1), y))
      {
        plain_points.emplace_back(x, y);
      }
    }
  }
  const auto & selected = route_points.size() >= 2 ? route_points : plain_points;
  if (selected.size() < 2) {
    RCLCPP_ERROR(get_logger(), "CSV path contains fewer than two points: %s", resolved.c_str());
    return false;
  }
  applyReferencePath(selected, path_frame_id_, "csv:" + resolved.string());
  return true;
}

int SdMapUpperPlannerNode::computeRequestedPreviewSteps(
  const GridMapSnapshot * grid,
  double planning_speed_mps) const
{
  int requested = upper_preview_steps_;
  if (!preview_limit_to_grid_extent_ || grid == nullptr || !grid->valid) {
    return requested;
  }

  const double width_m = static_cast<double>(grid->width) * grid->resolution;
  const double height_m = static_cast<double>(grid->height) * grid->resolution;
  const double cosine = std::cos(grid->origin_yaw_rad);
  const double sine = std::sin(grid->origin_yaw_rad);
  Eigen::Matrix2d rotation;
  rotation << cosine, -sine, sine, cosine;

  const std::array<Eigen::Vector2d, 4> local_corners{{
    Eigen::Vector2d(0.0, 0.0),
    Eigen::Vector2d(width_m, 0.0),
    Eigen::Vector2d(0.0, height_m),
    Eigen::Vector2d(width_m, height_m)}};

  double max_x_body = -std::numeric_limits<double>::infinity();
  for (const auto & local : local_corners) {
    const Eigen::Vector2d body = grid->origin_body + rotation * local;
    max_x_body = std::max(max_x_body, body.x());
  }

  const double delta_s = planning_speed_mps * upper_prediction_dt_sec_;
  if (!std::isfinite(max_x_body) || delta_s <= 1e-9) {
    return 0;
  }

  const double usable_forward_m = std::min(max_x_body, bev_forward_m_);
  requested = egocentric_planner_geometry::previewStepsFromForwardExtent(
    usable_forward_m, delta_s, upper_preview_steps_, preview_grid_reserve_steps_);
  return requested;
}

SdMapUpperPlannerNode::PreviewReferenceData
SdMapUpperPlannerNode::buildPreviewReference(
  const Eigen::Vector2d & wp_prev_body,
  const Eigen::Vector2d & wp0_body,
  const Eigen::Vector2d & wp1_body,
  bool has_previous_waypoint,
  double planning_speed_mps,
  int requested_horizon,
  const std::vector<Eigen::Vector2d> & previous_reference_body) const
{
  PreviewReferenceData output;
  if (requested_horizon <= 0) {
    output.status = "requested preview horizon is zero";
    return output;
  }
  if (!finitePoint(wp0_body) || !finitePoint(wp1_body) ||
    (has_previous_waypoint && !finitePoint(wp_prev_body)))
  {
    output.status = "non-finite body-frame waypoint";
    return output;
  }

  const double target_norm = wp1_body.norm();
  if (target_norm < preview_min_target_distance_m_) {
    output.status = "wp1 is too close to the ego origin";
    return output;
  }

  const Eigen::Vector2d outgoing = wp1_body - wp0_body;
  if (outgoing.norm() < preview_min_target_distance_m_) {
    output.status = "wp0 and wp1 do not define a nonzero outgoing road segment";
    return output;
  }
  double psi_in = 0.0;
  if (has_previous_waypoint) {
    const Eigen::Vector2d incoming = wp0_body - wp_prev_body;
    if (incoming.norm() < preview_min_target_distance_m_) {
      output.status = "wpPrev and wp0 do not define a nonzero incoming road segment";
      return output;
    }
    psi_in = std::atan2(incoming.y(), incoming.x());
  }

  const double psi_out_raw = std::atan2(outgoing.y(), outgoing.x());
  const double psi_out = psi_in + wrapToPi(psi_out_raw - psi_in);
  output.road_segment_heading_rad = {{psi_in, psi_out}};
  output.delta_psi_road_rad = wrapToPi(psi_out - psi_in);

  const Eigen::Vector2d reference_direction = wp1_body / target_norm;
  const double waypoint_heading =
    std::atan2(reference_direction.y(), reference_direction.x());
  const double delta_s = planning_speed_mps * upper_prediction_dt_sec_;
  const std::size_t reference_size = static_cast<std::size_t>(requested_horizon + 1);

  bool previous_solution_usable = requested_horizon >= 2 &&
    previous_reference_body.size() == reference_size &&
    finitePoint(previous_reference_body.front()) &&
    previous_reference_body.front().norm() <= 1e-9;
  if (previous_solution_usable) {
    // Old p1,...,pN-1 were re-expressed using the current measured yaw.
    // Only r0,...,rN-2 are populated; old pN is deliberately absent.
    for (int k = 0; k <= requested_horizon - 2; ++k) {
      if (!finitePoint(previous_reference_body[static_cast<std::size_t>(k)])) {
        previous_solution_usable = false;
        break;
      }
    }
  }

  output.points_body.assign(reference_size, Eigen::Vector2d::Zero());
  if (previous_solution_usable) {
    for (int k = 0; k <= requested_horizon - 2; ++k) {
      output.points_body[static_cast<std::size_t>(k)] =
        previous_reference_body[static_cast<std::size_t>(k)];
    }
  } else {
    // First cycle, route revision, or dimension mismatch: main8.m creates a
    // fresh straight reference from the measured body origin toward wp1_B.
    for (int k = 0; k <= requested_horizon; ++k) {
      output.points_body[static_cast<std::size_t>(k)] =
        static_cast<double>(k) * delta_s * reference_direction;
    }
  }

  constexpr double kHeadingEpsilon = 1e-9;
  double stable_heading = waypoint_heading;
  if (previous_solution_usable) {
    Eigen::Vector2d direction_sum = Eigen::Vector2d::Zero();
    Eigen::Vector2d last_valid_direction = reference_direction;
    int direction_count = 0;
    for (int i = requested_horizon - 2; i >= 1 && direction_count < 3; --i) {
      const Eigen::Vector2d segment =
        output.points_body[static_cast<std::size_t>(i)] -
        output.points_body[static_cast<std::size_t>(i - 1)];
      const double segment_norm = segment.norm();
      if (segment_norm > kHeadingEpsilon) {
        const Eigen::Vector2d direction = segment / segment_norm;
        if (direction_count == 0) {last_valid_direction = direction;}
        direction_sum += direction;
        ++direction_count;
      }
    }
    if (direction_count > 0) {
      const Eigen::Vector2d stable_direction =
        direction_sum.norm() > kHeadingEpsilon ?
        direction_sum.normalized() : last_valid_direction;
      stable_heading = std::atan2(stable_direction.y(), stable_direction.x());
    }
    const Eigen::Vector2d stable_step = delta_s * Eigen::Vector2d(
      std::cos(stable_heading), std::sin(stable_heading));
    output.points_body[static_cast<std::size_t>(requested_horizon - 1)] =
      output.points_body[static_cast<std::size_t>(requested_horizon - 2)] + stable_step;
    output.points_body[static_cast<std::size_t>(requested_horizon)] =
      output.points_body[static_cast<std::size_t>(requested_horizon - 1)] + stable_step;
  }

  // make_preview_constraints8.m recomputes point-tangent headings every
  // cycle after the old world path has been expressed in the measured frame.
  output.heading_rad.assign(reference_size, stable_heading);
  for (int k = 0; k < requested_horizon; ++k) {
    const Eigen::Vector2d segment =
      output.points_body[static_cast<std::size_t>(k + 1)] -
      output.points_body[static_cast<std::size_t>(k)];
    double heading = k > 0 ? output.heading_rad[static_cast<std::size_t>(k - 1)] :
      stable_heading;
    if (segment.norm() > kHeadingEpsilon) {
      const double raw_heading = std::atan2(segment.y(), segment.x());
      heading = k == 0 ? raw_heading :
        output.heading_rad[static_cast<std::size_t>(k - 1)] +
        wrapToPi(raw_heading - output.heading_rad[static_cast<std::size_t>(k - 1)]);
    }
    output.heading_rad[static_cast<std::size_t>(k)] = heading;
  }
  output.heading_rad.back() = output.heading_rad[reference_size - 2];

  output.stable_heading_rad = stable_heading;
  output.waypoint_bearing_rad = waypoint_heading;
  output.terminal_buffer_turn_rad = 0.0;
  output.used_previous_solution = previous_solution_usable;
  output.status = previous_solution_usable ?
    "valid previous optimum in current measured frame" :
    "valid initial straight seed with one-step terminal target";
  output.valid = true;
  return output;
}

bool SdMapUpperPlannerNode::bodyPointToGrid(
  const Eigen::Vector2d & point_body,
  const GridMapSnapshot & grid,
  int & gx,
  int & gy) const
{
  const double cosine = std::cos(grid.origin_yaw_rad);
  const double sine = std::sin(grid.origin_yaw_rad);
  const Eigen::Vector2d relative = point_body - grid.origin_body;
  const Eigen::Vector2d local(
    cosine * relative.x() + sine * relative.y(),
    -sine * relative.x() + cosine * relative.y());
  const double width_m = static_cast<double>(grid.width) * grid.resolution;
  const double height_m = static_cast<double>(grid.height) * grid.resolution;
  if (!finitePoint(local) || local.x() < 0.0 || local.y() < 0.0 ||
    local.x() >= width_m || local.y() >= height_m)
  {
    gx = -1;
    gy = -1;
    return false;
  }
  gx = static_cast<int>(std::floor(local.x() / grid.resolution));
  gy = static_cast<int>(std::floor(local.y() / grid.resolution));
  return gx >= 0 && gx < grid.width && gy >= 0 && gy < grid.height;
}

bool SdMapUpperPlannerNode::sampleGridValue(
  const Eigen::Vector2d & point_body,
  const GridMapSnapshot & grid,
  int8_t & value) const
{
  int gx = -1;
  int gy = -1;
  if (!bodyPointToGrid(point_body, grid, gx, gy)) {
    return false;
  }
  const int index = gy * grid.width + gx;
  if (index < 0 || index >= static_cast<int>(grid.data.size())) {
    return false;
  }
  value = grid.data[static_cast<std::size_t>(index)];
  return true;
}

bool SdMapUpperPlannerNode::clipLineToGridRectangle(
  const Eigen::Vector2d & point_body,
  const Eigen::Vector2d & direction_body,
  const GridMapSnapshot & grid,
  double & lambda_min,
  double & lambda_max) const
{
  if (!grid.valid || direction_body.norm() < 1e-12) {
    return false;
  }
  const double cosine = std::cos(grid.origin_yaw_rad);
  const double sine = std::sin(grid.origin_yaw_rad);
  const Eigen::Vector2d relative = point_body - grid.origin_body;
  const Eigen::Vector2d point_local(
    cosine * relative.x() + sine * relative.y(),
    -sine * relative.x() + cosine * relative.y());
  const Eigen::Vector2d direction_local(
    cosine * direction_body.x() + sine * direction_body.y(),
    -sine * direction_body.x() + cosine * direction_body.y());

  lambda_min = -std::numeric_limits<double>::infinity();
  lambda_max = std::numeric_limits<double>::infinity();
  auto clip_slab = [&](double point, double direction, double lower, double upper) {
      if (std::abs(direction) < 1e-12) {
        return point >= lower && point <= upper;
      }
      double first = (lower - point) / direction;
      double second = (upper - point) / direction;
      if (first > second) {std::swap(first, second);}
      lambda_min = std::max(lambda_min, first);
      lambda_max = std::min(lambda_max, second);
      return lambda_min <= lambda_max;
    };
  const double width_m = static_cast<double>(grid.width) * grid.resolution;
  const double height_m = static_cast<double>(grid.height) * grid.resolution;
  return clip_slab(point_local.x(), direction_local.x(), 0.0, width_m) &&
         clip_slab(point_local.y(), direction_local.y(), 0.0, height_m) &&
         std::isfinite(lambda_min) && std::isfinite(lambda_max) &&
         lambda_max >= lambda_min;
}

bool SdMapUpperPlannerNode::isDrivable(int8_t value) const
{
  return egocentric_planner_geometry::gridValueIsDrivable(
    static_cast<int>(value), grid_positive_is_drivable_, grid_value_threshold_);
}

SdMapUpperPlannerNode::PreviewConstraintData
SdMapUpperPlannerNode::extractPreviewIntervals(
  const PreviewReferenceData & reference,
  const GridMapSnapshot * grid,
  const IntervalCenteringState & interval_state) const
{
  PreviewConstraintData output;
  output.interval_update.state = interval_state;
  if (!reference.valid || reference.points_body.size() < 2 ||
    reference.heading_rad.size() != reference.points_body.size())
  {
    return output;
  }

  const int requested_horizon = static_cast<int>(reference.points_body.size()) - 1;
  output.N_pred = requested_horizon;
  const std::size_t size = static_cast<std::size_t>(requested_horizon + 1);
  output.pmk.resize(size);
  output.pMk.resize(size);
  output.raw_lengths.resize(size);
  output.processed_lengths.resize(size);
  output.treatment_modes.resize(size);
  output.source_modes.resize(size);
  output.fallback_reason_codes.assign(
    size, static_cast<int>(IntervalFallbackReason::None));
  output.Nck.assign(size, 0);
  output.interval_info.resize(size);
  output.prk = reference.points_body;
  output.psirk = reference.heading_rad;
  output.position_targets = reference.points_body;
  output.road_segment_heading_rad = reference.road_segment_heading_rad;
  output.delta_psi_road_rad = reference.delta_psi_road_rad;
  output.stable_heading_rad = reference.stable_heading_rad;
  output.waypoint_bearing_rad = reference.waypoint_bearing_rad;
  output.terminal_buffer_turn_rad = reference.terminal_buffer_turn_rad;
  output.used_previous_solution = reference.used_previous_solution;

  std::vector<std::vector<IntervalObservation>> observations(size);
  // Extract every requested stage before updating B, including observed stages
  // beyond a missing interval that shortens the MIQP horizon.

  for (int k = 0; k <= requested_horizon; ++k) {
    const std::size_t step = static_cast<std::size_t>(k);
    const Eigen::Vector2d reference_point = output.prk[step];
    const double heading = output.psirk[step];
    const Eigen::Vector2d lateral_direction(-std::sin(heading), std::cos(heading));

    std::vector<Eigen::Vector2d> raw_starts;
    std::vector<Eigen::Vector2d> raw_ends;
    std::vector<int> raw_source_modes;
    std::vector<std::array<bool, 2>> raw_observed_boundaries;
    IntervalFallbackReason fallback_reason = IntervalFallbackReason::None;
    std::size_t sampled_free = 0;
    std::size_t sampled_blocked = 0;
    std::size_t sampled_unknown = 0;
    std::size_t sampled_outside = 0;
    std::size_t longest_free_run = 0;

    if (grid != nullptr && grid->valid) {
      double lambda_min = 0.0;
      double lambda_max = 0.0;
      if (clipLineToGridRectangle(
          reference_point, lateral_direction, *grid, lambda_min, lambda_max))
      {
        const double span = std::max(0.0, lambda_max - lambda_min);
        std::vector<double> lambdas;
        const int full_steps = static_cast<int>(std::floor(span / preview_line_sample_m_));
        lambdas.reserve(static_cast<std::size_t>(full_steps + 2));
        for (int sample = 0; sample <= full_steps; ++sample) {
          lambdas.push_back(lambda_min + static_cast<double>(sample) * preview_line_sample_m_);
        }
        if (lambdas.empty() || lambdas.back() < lambda_max - 1e-10) {
          lambdas.push_back(lambda_max);
        }

        if (static_cast<int>(lambdas.size()) >= preview_min_segment_samples_) {
          std::vector<Eigen::Vector2d> run;
          bool previous_known_blocked = false;
          bool run_start_observed = false;
          auto flush_run = [&](bool end_observed) {
              if (static_cast<int>(run.size()) >= preview_min_segment_samples_) {
                raw_starts.push_back(run.front());
                raw_ends.push_back(run.back());
                raw_observed_boundaries.push_back({run_start_observed, end_observed});
                raw_source_modes.push_back(static_cast<int>(IntervalSourceMode::BevObserved));
              }
              run.clear();
            };

          for (const double lambda : lambdas) {
            const Eigen::Vector2d point = reference_point + lambda * lateral_direction;
            int8_t value = -1;
            const bool sampled = sampleGridValue(point, *grid, value);
            const bool drivable = sampled && isDrivable(value);
            if (drivable) {
              ++sampled_free;
              if (run.empty()) {run_start_observed = previous_known_blocked;}
              run.push_back(point);
              longest_free_run = std::max(longest_free_run, run.size());
            } else {
              if (!sampled) {
                ++sampled_outside;
              } else if (value < 0) {
                ++sampled_unknown;
              } else {
                ++sampled_blocked;
              }
              flush_run(sampled && value >= 0);
            }
            // Unknown/outside samples are clipping, not observed road boundaries.
            previous_known_blocked = sampled && value >= 0 && !drivable;
          }
          flush_run(false);
          if (raw_starts.empty()) {
            fallback_reason = IntervalFallbackReason::NoValidDrivableRun;
          }
        } else {
          fallback_reason = IntervalFallbackReason::InsufficientBevSamples;
        }
      } else {
        fallback_reason = IntervalFallbackReason::LineOutsideBev;
      }
    } else {
      fallback_reason = IntervalFallbackReason::GridUnavailable;
    }

    output.fallback_reason_codes[step] = static_cast<int>(fallback_reason);

    // main8.m uses a bounded virtual road only when the reference point is
    // unknown or outside the BEV. A cell positively observed as blocked must
    // not be made feasible by the fallback.
    if (raw_starts.empty() && preview_interval_enable_nominal_fallback_) {
      bool reference_sample_available = false;
      int8_t reference_value = -1;
      if (grid != nullptr && grid->valid) {
        reference_sample_available =
          sampleGridValue(reference_point, *grid, reference_value);
      }

      if (egocentric_planner_geometry::referenceAllowsNominalFallback(
          reference_sample_available, static_cast<int>(reference_value),
          grid_positive_is_drivable_, grid_value_threshold_))
      {
        const double half_width = 0.5 * preview_interval_nominal_fallback_width_m_;
        raw_starts.push_back(reference_point - half_width * lateral_direction);
        raw_ends.push_back(reference_point + half_width * lateral_direction);
        raw_source_modes.push_back(static_cast<int>(IntervalSourceMode::NominalFallback));
        raw_observed_boundaries.push_back({false, false});
      }
    }

    for (std::size_t candidate = 0; candidate < raw_starts.size(); ++candidate) {
      const Eigen::Vector2d p0 = raw_starts[candidate];
      const Eigen::Vector2d p1 = raw_ends[candidate];
      const double raw_length = (p1 - p0).norm();
      if (!finitePoint(p0) || !finitePoint(p1) || !std::isfinite(raw_length) ||
        raw_length <= 1e-8)
      {
        continue;
      }

      const double reference_distance = (reference_point - p0).dot((p1 - p0) / raw_length);
      IntervalObservation observation;
      observation.length = raw_length;
      observation.start_boundary_observed = raw_observed_boundaries[candidate][0];
      observation.end_boundary_observed = raw_observed_boundaries[candidate][1];
      observation.reference_inside = reference_distance >= -1e-8 &&
        reference_distance <= raw_length + 1e-8;
      observation.nominal_fallback = raw_source_modes[candidate] ==
        static_cast<int>(IntervalSourceMode::NominalFallback);
      observations[step].push_back(observation);
      output.interval_info[step].push_back(IntervalInfo{observation, {}, 0.0});
      output.pmk[step].push_back(p0);
      output.pMk[step].push_back(p1);
      output.raw_lengths[step].push_back(raw_length);
      output.source_modes[step].push_back(raw_source_modes[candidate]);
    }

    output.Nck[step] = static_cast<int>(output.pmk[step].size());
    // Only k=1,...,N are constrained by the MIQP. A missing k=0 interval
    // means the measured body origin is outside the known drivable cells; it
    // does not make the future preview intervals unusable.
    if (output.Nck[step] == 0) {
      // Report the actual failed lookup without relaxing road constraints.
      // Keep this enabled independently of verbose per-candidate debug output.
      if (k > 0 && grid != nullptr && grid->valid) {
        std::size_t grid_zero = 0;
        std::size_t grid_positive = 0;
        std::size_t grid_unknown = 0;
        std::size_t grid_free = 0;
        for (const int8_t value : grid->data) {
          if (value < 0) {
            ++grid_unknown;
          } else if (value == 0) {
            ++grid_zero;
          } else {
            ++grid_positive;
          }
          if (isDrivable(value)) {++grid_free;}
        }
        int center_gx = -1;
        int center_gy = -1;
        int8_t center_value = -1;
        const bool center_sampled = sampleGridValue(reference_point, *grid, center_value);
        bodyPointToGrid(reference_point, *grid, center_gx, center_gy);
        static rclcpp::Clock diagnostic_clock{RCL_STEADY_TIME};
        RCLCPP_WARN_THROTTLE(
          get_logger(), diagnostic_clock, 2000,
          "interval_diag step=%d reason=%d ref_body=(%.3f,%.3f) heading_deg=%.2f "
          "center_cell=(%d,%d) center_value=%d "
          "line_free=%zu blocked=%zu unknown=%zu outside=%zu longest_run=%zu min_run=%d "
          "grid_free=%zu zero=%zu positive=%zu unknown=%zu "
          "topic=%s frame=%s origin=(%.3f,%.3f) origin_yaw_deg=%.2f "
          "size=%dx%d res=%.4f positive_is_drivable=%d threshold=%d "
          "waypoint_bias=%d previous=%d",
          k, static_cast<int>(fallback_reason), reference_point.x(), reference_point.y(),
          radToDeg(heading), center_gx, center_gy,
          center_sampled ? static_cast<int>(center_value) : -999,
          sampled_free, sampled_blocked, sampled_unknown, sampled_outside,
          longest_free_run, preview_min_segment_samples_,
          grid_free, grid_zero, grid_positive, grid_unknown,
          grid_map_topic_.c_str(), grid->frame_id.c_str(),
          grid->origin_body.x(), grid->origin_body.y(), radToDeg(grid->origin_yaw_rad),
          grid->width, grid->height, grid->resolution,
          grid_positive_is_drivable_ ? 1 : 0, grid_value_threshold_,
          enable_waypoint_bias_ ? 1 : 0, reference.used_previous_solution ? 1 : 0);
      }
      const int usable_horizon =
        egocentric_planner_geometry::previewHorizonAfterMissingInterval(k, output.N_pred);
      if (k == 0) {
        RCLCPP_WARN(
          get_logger(),
          "no valid interval at the measured k=0 state; retaining future horizon=%d reason=%d",
          usable_horizon, static_cast<int>(fallback_reason));
        continue;
      }
      output.N_pred = usable_horizon;
      RCLCPP_WARN(
        get_logger(),
        "no valid preview interval at step=%d; usable horizon=%d reason=%d",
        k, usable_horizon, static_cast<int>(fallback_reason));
    }
  }

  observations.resize(static_cast<std::size_t>(output.N_pred + 1));
  output.interval_update = updateIntervalCentering(
    interval_state, observations, interval_centering_settings_);
  const auto & next_state = output.interval_update.state;
  if (preview_interval_debug_) {
    RCLCPP_INFO(
      get_logger(),
      "interval_scale B_before=%.3f candidate_B=%.3f B=%.3f threshold_1_5B=%.3f "
      "candidate_step=%d candidate_interval=%d",
      interval_state.reference_length, output.interval_update.candidate_length,
      next_state.reference_length,
      hasIntervalReference(next_state) ?
      interval_centering_settings_.relative_length_factor * next_state.reference_length :
      std::numeric_limits<double>::quiet_NaN(),
      output.interval_update.candidate_step, output.interval_update.candidate_interval);
  }

  for (int k = 0; k <= requested_horizon; ++k) {
    const std::size_t step = static_cast<std::size_t>(k);
    std::size_t retained = 0;
    for (std::size_t candidate = 0; candidate < output.pmk[step].size(); ++candidate) {
      auto & info = output.interval_info[step][candidate];
      info.treatment = intervalTreatment(
        info.observation, next_state, interval_centering_settings_,
        preview_interval_soft_ratio_, preview_interval_boundary_margin_m_,
        preview_interval_fallback_max_centering_length_m_);
      const Eigen::Vector2d p0 = output.pmk[step][candidate];
      const Eigen::Vector2d p1 = output.pMk[step][candidate];
      const double raw_length = info.observation.length;
      const Eigen::Vector2d direction = (p1 - p0) / raw_length;
      const Eigen::Vector2d processed_start = p0 + info.treatment.inset * direction;
      const Eigen::Vector2d processed_end = p1 - info.treatment.inset * direction;
      info.processed_length = (processed_end - processed_start).norm();
      if (!finitePoint(processed_start) || !finitePoint(processed_end) ||
        !std::isfinite(info.processed_length) || info.processed_length <= 1e-8)
      {
        continue;
      }
      output.pmk[step][retained] = processed_start;
      output.pMk[step][retained] = processed_end;
      output.raw_lengths[step][retained] = raw_length;
      output.source_modes[step][retained] = output.source_modes[step][candidate];
      output.processed_lengths[step].push_back(info.processed_length);
      output.treatment_modes[step].push_back(static_cast<int>(info.treatment.centering_on ?
        IntervalTreatmentMode::CenterContraction : IntervalTreatmentMode::BoundaryMargin));
      if (preview_interval_debug_) {
        RCLCPP_INFO(
          get_logger(),
          "interval step=%d candidate=%zu raw=%.3f centering=%s",
          k, candidate, raw_length, info.treatment.centering_on ? "ON" : "OFF");
      }
      ++retained;
    }
    output.pmk[step].resize(retained);
    output.pMk[step].resize(retained);
    output.raw_lengths[step].resize(retained);
    output.source_modes[step].resize(retained);
    output.Nck[step] = static_cast<int>(retained);
    if (retained == 0U && k > 0) {output.N_pred = std::min(output.N_pred, k - 1);}
  }

  const std::size_t usable_size = static_cast<std::size_t>(output.N_pred + 1);
  output.pmk.resize(usable_size);
  output.pMk.resize(usable_size);
  output.raw_lengths.resize(usable_size);
  output.processed_lengths.resize(usable_size);
  output.treatment_modes.resize(usable_size);
  output.source_modes.resize(usable_size);
  output.fallback_reason_codes.resize(usable_size);
  output.Nck.resize(usable_size);
  output.prk.resize(usable_size);
  output.psirk.resize(usable_size);
  output.position_targets.resize(usable_size);

  return output;
}

void SdMapUpperPlannerNode::applyTerminalPositionTargets(
  PreviewConstraintData & preview,
  const Eigen::Vector2d & wp0_body,
  const Eigen::Vector2d & wp1_body,
  double delta_s) const
{
  const int horizon = preview.N_pred;
  const std::size_t expected_size = static_cast<std::size_t>(horizon + 1);
  if (horizon <= 0 || preview.prk.size() != expected_size ||
    preview.psirk.size() != expected_size || !std::isfinite(delta_s) || delta_s <= 0.0)
  {
    return;
  }

  constexpr double kDirectionEpsilon = 1e-9;
  preview.position_targets = preview.prk;
  preview.terminal_buffer_turn_rad = 0.0;

  const std::size_t terminal_index = static_cast<std::size_t>(horizon);
  const std::size_t terminal_anchor_index = static_cast<std::size_t>(horizon - 1);
  const Eigen::Vector2d terminal_anchor = preview.prk[terminal_anchor_index];
  const Eigen::Vector2d terminal_geometry_segment =
    preview.prk[terminal_index] - terminal_anchor;
  const double terminal_geometry_spacing = terminal_geometry_segment.norm();

  Eigen::Vector2d terminal_geometry_direction(
    std::cos(preview.psirk[terminal_anchor_index]),
    std::sin(preview.psirk[terminal_anchor_index]));
  Eigen::Vector2d terminal_waypoint = wp1_body;
  if (horizon >= 2 && terminal_geometry_spacing >= kDirectionEpsilon) {
    terminal_geometry_direction = terminal_geometry_segment / terminal_geometry_spacing;
    const double distance_to_wp0 =
      (wp0_body - terminal_anchor).dot(terminal_geometry_direction);
    if (distance_to_wp0 > delta_s) {
      terminal_waypoint = wp0_body;
    }
  }

  preview.stable_heading_rad = std::atan2(
    terminal_geometry_direction.y(), terminal_geometry_direction.x());
  const Eigen::Vector2d terminal_direction = terminal_waypoint - terminal_anchor;
  if (terminal_direction.norm() >= kDirectionEpsilon) {
    preview.position_targets[terminal_index] =
      terminal_anchor + delta_s * terminal_direction.normalized();
    preview.waypoint_bearing_rad =
      std::atan2(terminal_direction.y(), terminal_direction.x());
  }

  if (horizon < 2) {
    return;
  }

  const std::size_t n1_index = static_cast<std::size_t>(horizon - 1);
  const std::size_t n1_anchor_index = static_cast<std::size_t>(horizon - 2);
  const Eigen::Vector2d n1_anchor = preview.prk[n1_anchor_index];
  const Eigen::Vector2d n1_stable_segment = preview.prk[n1_index] - n1_anchor;
  const Eigen::Vector2d n1_waypoint_vector = terminal_waypoint - n1_anchor;
  if (n1_stable_segment.norm() < kDirectionEpsilon ||
    n1_waypoint_vector.norm() < kDirectionEpsilon)
  {
    return;
  }

  const double n1_stable_heading =
    std::atan2(n1_stable_segment.y(), n1_stable_segment.x());
  const double n1_waypoint_bearing =
    std::atan2(n1_waypoint_vector.y(), n1_waypoint_vector.x());
  const double bounded_turn = clampd(
    wrapToPi(n1_waypoint_bearing - n1_stable_heading),
    -terminal_buffer_max_turn_rad_, terminal_buffer_max_turn_rad_);
  const double n1_cost_heading = n1_stable_heading + bounded_turn;
  preview.position_targets[n1_index] = n1_anchor + delta_s * Eigen::Vector2d(
    std::cos(n1_cost_heading), std::sin(n1_cost_heading));
  preview.waypoint_bearing_rad = n1_waypoint_bearing;
  preview.terminal_buffer_turn_rad = bounded_turn;
}

SdMapUpperPlannerNode::UpperSolveResult SdMapUpperPlannerNode::solveUpperMiqp(
  const PreviewConstraintData & preview,
  double planning_speed_mps) const
{
  UpperSolveResult result;
  // Sum all actual solver calls, including fixed-corridor QPs and polishing.
  const auto timed_solve = [&result](QuadraticProblem & problem, Vector<double> & argument) {
      if (!std::isfinite(result.solver_only_time_ms)) result.solver_only_time_ms = 0.0;
      const auto started = std::chrono::steady_clock::now();
      try {
        const double objective = problem.solve_problem(argument);
        result.solver_only_time_ms += std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - started).count();
        return objective;
      } catch (...) {
        result.solver_only_time_ms += std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - started).count();
        throw;
      }
    };
  const auto started = std::chrono::steady_clock::now();
  const int horizon = preview.N_pred;
  if (horizon <= 0 ||
    preview.prk.size() != static_cast<std::size_t>(horizon + 1) ||
    preview.psirk.size() != static_cast<std::size_t>(horizon + 1) ||
    preview.position_targets.size() != static_cast<std::size_t>(horizon + 1) ||
    preview.Nck.size() != static_cast<std::size_t>(horizon + 1) ||
    preview.pmk.size() != static_cast<std::size_t>(horizon + 1) ||
    preview.pMk.size() != static_cast<std::size_t>(horizon + 1))
  {
    result.status = "invalid preview dimensions";
    return result;
  }

  std::vector<int> binary_offsets(static_cast<std::size_t>(horizon + 1), 0);
  for (int k = 1; k <= horizon; ++k) {
    const int candidates = preview.Nck[static_cast<std::size_t>(k)];
    if (candidates <= 0 ||
      preview.pmk[static_cast<std::size_t>(k)].size() !=
      static_cast<std::size_t>(candidates) ||
      preview.pMk[static_cast<std::size_t>(k)].size() !=
      static_cast<std::size_t>(candidates))
    {
      result.status = "invalid candidate dimensions";
      return result;
    }
    binary_offsets[static_cast<std::size_t>(k)] =
      binary_offsets[static_cast<std::size_t>(k - 1)] + candidates;
  }

  const int input_count = 2 * horizon;
  const int binary_count = binary_offsets.back();
  const int variable_count = input_count + binary_count;

  Matrix<double> G(0.0, 2 * (horizon + 1), variable_count);
  for (int state_step = 1; state_step <= horizon; ++state_step) {
    for (int input_step = 0; input_step < state_step; ++input_step) {
      const double heading = preview.psirk[static_cast<std::size_t>(input_step)];
      const int row = 2 * state_step;
      const int column = 2 * input_step;
      G[row][column] = upper_prediction_dt_sec_ * std::cos(heading);
      G[row][column + 1] =
        -upper_prediction_dt_sec_ * planning_speed_mps * std::sin(heading);
      G[row + 1][column] = upper_prediction_dt_sec_ * std::sin(heading);
      G[row + 1][column + 1] =
        upper_prediction_dt_sec_ * planning_speed_mps * std::cos(heading);
    }
  }

  Vector<double> nominal_input(0.0, input_count);
  for (int k = 0; k < horizon; ++k) {
    const double previous_heading =
      k == 0 ? 0.0 : preview.psirk[static_cast<std::size_t>(k - 1)];
    nominal_input[2 * k] = 0.0;
    nominal_input[2 * k + 1] = wrapToPi(
      preview.psirk[static_cast<std::size_t>(k)] - previous_heading);
  }

  const double max_dv = a_max_mps2_ * upper_prediction_dt_sec_;
  const double phi =
    std::tan(max_steering_angle_rad_) * upper_prediction_dt_sec_ / wheelbase_;
  const double normalized_dv_weight = normalize_upper_input_cost_ ?
    upper_r_dv_ / (max_dv * max_dv) : upper_r_dv_;
  const double nominal_max_dpsi = std::max(1e-9, phi * planning_speed_mps);
  const double normalized_dpsi_weight = normalize_upper_input_cost_ ?
    upper_r_dpsi_ / (nominal_max_dpsi * nominal_max_dpsi) : upper_r_dpsi_;

  Matrix<double> hessian(0.0, variable_count, variable_count);
  const double position_weight_sum =
    static_cast<double>(horizon) + terminal_position_weight_multiplier_;
  const double position_stage_weight =
    lambda_position_ /
    (position_scale_m_ * position_scale_m_) /
    position_weight_sum;
  std::vector<double> position_weights(
    static_cast<std::size_t>(horizon + 1), position_stage_weight);
  position_weights[static_cast<std::size_t>(horizon)] =
    terminal_position_weight_multiplier_ * position_stage_weight;
  for (int row = 0; row < variable_count; ++row) {
    for (int column = 0; column < variable_count; ++column) {
      double value = 0.0;
      for (int step = 0; step <= horizon; ++step) {
        const double weight = position_weights[static_cast<std::size_t>(step)];
        value += weight * (
          G[2 * step][row] * G[2 * step][column] +
          G[2 * step + 1][row] * G[2 * step + 1][column]);
      }
      hessian[row][column] = value;
    }
  }
  for (int k = 0; k < horizon; ++k) {
    hessian[2 * k][2 * k] += normalized_dv_weight;
    hessian[2 * k + 1][2 * k + 1] += normalized_dpsi_weight;
  }
  Vector<double> linear_term(0.0, variable_count);
  std::vector<Eigen::Vector2d> position_cost_offset(
    static_cast<std::size_t>(horizon + 1), Eigen::Vector2d::Zero());
  for (int step = 0; step <= horizon; ++step) {
    position_cost_offset[static_cast<std::size_t>(step)] =
      preview.prk[static_cast<std::size_t>(step)] -
      preview.position_targets[static_cast<std::size_t>(step)];
    const double weight = position_weights[static_cast<std::size_t>(step)];
    for (int column = 0; column < input_count; ++column) {
      linear_term[column] += weight * (
        position_cost_offset[static_cast<std::size_t>(step)].x() * G[2 * step][column] +
        position_cost_offset[static_cast<std::size_t>(step)].y() *
        G[2 * step + 1][column]);
    }
  }

  for (int k = 1; k <= horizon; ++k) {
    if (preview.Nck[static_cast<std::size_t>(k)] >= 2) {
      result.branch_step = k;
      break;
    }
  }

  if (result.branch_step >= 1 && lambda_relative_turn_ > 0.0) {
    const int turn_start = std::max(1, result.branch_step - turn_preview_steps_);
    const int active_count = result.branch_step - turn_start + 1;
    std::vector<double> unnormalized_weights(static_cast<std::size_t>(active_count), 1.0);
    double weight_sum = 0.0;
    for (int i = 0; i < active_count; ++i) {
      unnormalized_weights[static_cast<std::size_t>(i)] =
        std::pow(turn_weight_growth_, static_cast<double>(i));
      weight_sum += unnormalized_weights[static_cast<std::size_t>(i)];
    }

    const double turn_base_weight =
      lambda_relative_turn_ /
      (relative_turn_scale_rad_ * relative_turn_scale_rad_);
    for (int i = 0; i < active_count; ++i) {
      const int step = turn_start + i;
      const double stage_weight =
        unnormalized_weights[static_cast<std::size_t>(i)] / weight_sum;
      const double target =
        static_cast<double>(i + 1) / static_cast<double>(active_count) *
        preview.delta_psi_road_rad;
      const double q_weight = turn_base_weight * stage_weight;

      result.turn_window_steps.push_back(step);
      result.turn_target_rad.push_back(target);
      result.turn_stage_weights.push_back(stage_weight);

      for (int first = 0; first < step; ++first) {
        const int first_column = 2 * first + 1;
        linear_term[first_column] += -target * q_weight;
        for (int second = 0; second < step; ++second) {
          const int second_column = 2 * second + 1;
          hessian[first_column][second_column] += q_weight;
        }
      }
    }
  }

  for (int row = 0; row < variable_count; ++row) {
    for (int column = row + 1; column < variable_count; ++column) {
      const double symmetric = 0.5 * (hessian[row][column] + hessian[column][row]);
      hessian[row][column] = symmetric;
      hessian[column][row] = symmetric;
    }
    hessian[row][row] += miqp_diagonal_regularization_;
    if (row >= input_count) {
      // The selector constraint below requires at least one corridor.  Since
      // selecting another corridor can only add restrictions, this positive
      // cost makes the optimum use one selector without a one-hot equality.
      hessian[row][row] += miqp_binary_diagonal_regularization_;
    }
  }

  const int lane_rows = 2 * binary_count + horizon;
  const int input_rows = 4 * horizon;
  Matrix<double> inequality(0.0, lane_rows + input_rows, variable_count);
  Vector<double> inequality_bound(0.0, lane_rows + input_rows);
  int row = 0;

  for (int k = 1; k <= horizon; ++k) {
    const double heading = preview.psirk[static_cast<std::size_t>(k)];
    const Eigen::Vector2d axis(-std::sin(heading), std::cos(heading));
    const Eigen::Vector2d reference = preview.prk[static_cast<std::size_t>(k)];
    const double reference_projection = axis.dot(reference);
    for (int candidate = 0;
      candidate < preview.Nck[static_cast<std::size_t>(k)]; ++candidate)
    {
      double lower = axis.dot(
        preview.pmk[static_cast<std::size_t>(k)][static_cast<std::size_t>(candidate)]);
      double upper = axis.dot(
        preview.pMk[static_cast<std::size_t>(k)][static_cast<std::size_t>(candidate)]);
      if (lower > upper) {std::swap(lower, upper);}
      const int binary_index = input_count +
        binary_offsets[static_cast<std::size_t>(k - 1)] + candidate;

      for (int column = 0; column < input_count; ++column) {
        inequality[row][column] =
          axis.x() * G[2 * k][column] + axis.y() * G[2 * k + 1][column];
      }
      inequality[row][binary_index] = miqp_big_m_;
      inequality_bound[row++] = miqp_big_m_ + upper - reference_projection;

      for (int column = 0; column < input_count; ++column) {
        inequality[row][column] = -(
          axis.x() * G[2 * k][column] + axis.y() * G[2 * k + 1][column]);
      }
      inequality[row][binary_index] = miqp_big_m_;
      inequality_bound[row++] = miqp_big_m_ - lower + reference_projection;
    }

    // DAQP's branch-and-bound becomes singular when the one-hot selector is
    // encoded as an immutable equality and a binary bound is fixed at a node.
    // An at-least-one inequality plus the positive binary cost is equivalent
    // for this disjunctive corridor formulation and keeps BnB well-posed.
    for (int candidate = 0;
      candidate < preview.Nck[static_cast<std::size_t>(k)]; ++candidate)
    {
      inequality[row][input_count +
        binary_offsets[static_cast<std::size_t>(k - 1)] + candidate] = -1.0;
    }
    inequality_bound[row++] = -1.0;
  }

  const double speed = std::max(0.0, planning_speed_mps);

  auto finalize_input_row = [&](int current_row, double base_bound) {
      double adjusted = base_bound;
      for (int column = 0; column < input_count; ++column) {
        adjusted -= inequality[current_row][column] * nominal_input[column];
      }
      inequality_bound[current_row] = adjusted;
    };

  for (int k = 0; k < horizon; ++k) {
    inequality[row][2 * k] = 1.0;
    finalize_input_row(row, max_dv);
    ++row;

    inequality[row][2 * k] = -1.0;
    finalize_input_row(row, max_dv);
    ++row;

    for (int i = 0; i <= k; ++i) {
      inequality[row][2 * i] = -phi;
    }
    inequality[row][2 * k + 1] = 1.0;
    finalize_input_row(row, phi * speed);
    ++row;

    for (int i = 0; i <= k; ++i) {
      inequality[row][2 * i] = -phi;
    }
    inequality[row][2 * k + 1] = -1.0;
    finalize_input_row(row, phi * speed);
    ++row;
  }

  if (row != lane_rows + input_rows) {
    result.status = "inequality row mismatch";
    return result;
  }

  Matrix<double> equality(0.0, 0, variable_count);
  Vector<double> equality_bound(0.0, 0);

  constexpr std::size_t kMaxDirectCorridorCombinations = 256U;
  std::size_t combination_count = 1U;
  bool combination_count_supported = true;
  for (int k = 1; k <= horizon; ++k) {
    const std::size_t candidates = static_cast<std::size_t>(
      preview.Nck[static_cast<std::size_t>(k)]);
    if (candidates == 0U ||
      combination_count > kMaxDirectCorridorCombinations / candidates)
    {
      combination_count_supported = false;
      break;
    }
    combination_count *= candidates;
  }

  // The usual short-horizon problem has only a handful of corridor
  // combinations (and exactly one when branch_step == -1). Solve those
  // combinations as ordinary continuous QPs. This avoids sending forced
  // selector variables through DAQP branch-and-bound and also makes an
  // outright MIQP "infeasible" result unable to bypass the direct solver.
  Vector<double> argument;
  double objective = std::numeric_limits<double>::infinity();
  miqp_solution_validation::FeasibilityReport feasibility;

  if (!combination_count_supported) {
    QuadraticProblem solver(false);
    Variable * variable = solver.vector_variable(variable_count, "upper_decision");
    if (!solver.add_variable(variable)) {
      result.status = "DAQP add_variable failed";
      return result;
    }
    const Var decision = solver.get_variable(variable);
    const Var main_variable = solver.get_main_variable();

    Constraint inequality_constraint(main_variable);
    inequality_constraint.set_constraint_variable(decision, inequality);
    inequality_constraint.set_known_term(inequality_bound);
    if (!solver.add_leq_constraint(inequality_constraint)) {
      result.status = "DAQP failed to add inequality constraints";
      return result;
    }

    if (!solver.set_Q_matrix(hessian) || !solver.set_q0_vector(linear_term)) {
      result.status = "DAQP failed to set the objective";
      return result;
    }
    for (int index = input_count; index < variable_count; ++index) {
      solver.set_binary_var(decision[index]);
    }

    objective = timed_solve(solver, argument);
    if (!std::isfinite(objective) ||
      argument.size() < static_cast<unsigned int>(variable_count))
    {
      result.solve_time_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
      result.status = "DAQP MIQP infeasible (direct corridor limit exceeded)";
      return result;
    }
    for (int index = 0; index < variable_count; ++index) {
      if (!std::isfinite(argument[index])) {
        result.status = "DAQP MIQP returned a non-finite decision";
        return result;
      }
    }

    feasibility = miqp_solution_validation::evaluate(
      inequality, inequality_bound, equality, equality_bound, argument,
      static_cast<std::size_t>(input_count), miqp_feasibility_tolerance_);
  }

  // DAQP can return integral selectors while the continuous vector retained
  // from branch-and-bound belongs to a stale relaxation.  Re-solving one QP
  // with those selectors fixed recovers the continuous solution for the
  // chosen corridor combination and provides an independently checked result.
  if (!combination_count_supported && !feasibility.valid && feasibility.finite &&
    feasibility.max_binary_violation <= miqp_feasibility_tolerance_)
  {
    QuadraticProblem polishing_solver(false);
    Variable * polishing_variable =
      polishing_solver.vector_variable(variable_count, "upper_decision_polished");
    if (polishing_solver.add_variable(polishing_variable)) {
      const Var polishing_decision = polishing_solver.get_variable(polishing_variable);
      const Var polishing_main = polishing_solver.get_main_variable();

      Constraint polishing_inequality(polishing_main);
      polishing_inequality.set_constraint_variable(polishing_decision, inequality);
      polishing_inequality.set_known_term(inequality_bound);
      const bool constraint_added =
        polishing_solver.add_leq_constraint(polishing_inequality);
      const bool objective_added = constraint_added &&
        polishing_solver.set_Q_matrix(hessian) &&
        polishing_solver.set_q0_vector(linear_term);

      if (objective_added) {
        for (int index = input_count; index < variable_count; ++index) {
          const double fixed_value = argument[index] >= 0.5 ? 1.0 : 0.0;
          polishing_solver.set_var_bounds(index, fixed_value, fixed_value);
        }

        Vector<double> polished_argument;
        const double polished_objective =
          timed_solve(polishing_solver, polished_argument);
        if (std::isfinite(polished_objective) &&
          polished_argument.size() >= static_cast<unsigned int>(variable_count))
        {
          const auto polished_feasibility = miqp_solution_validation::evaluate(
            inequality, inequality_bound, equality, equality_bound, polished_argument,
            static_cast<std::size_t>(input_count), miqp_feasibility_tolerance_);
          if (polished_feasibility.valid) {
            argument = polished_argument;
            objective = polished_objective;
            feasibility = polished_feasibility;
            result.polished = true;
          }
        }
      }
    }
  }

  // Enumerating the supported Cartesian product removes both the selector
  // variables and Big-M=100 from every subproblem. Each returned solution is
  // independently checked before it is reconstructed in the full model.
  if (combination_count_supported) {
      Matrix<double> direct_hessian(0.0, input_count, input_count);
      Vector<double> direct_linear(0.0, input_count);
      for (int r = 0; r < input_count; ++r) {
        direct_linear[r] = linear_term[r];
        for (int c = 0; c < input_count; ++c) {
          direct_hessian[r][c] = hessian[r][c];
        }
      }

      const int direct_lane_rows = 2 * horizon;
      const int direct_row_count = direct_lane_rows + input_rows;
      Matrix<double> direct_equality(0.0, 0, input_count);
      Vector<double> direct_equality_bound(0.0, 0);
      Vector<double> best_direct_argument;
      std::vector<int> best_candidates(static_cast<std::size_t>(horizon + 1), -1);
      double best_direct_objective = std::numeric_limits<double>::infinity();

      for (std::size_t combination = 0U;
        combination < combination_count; ++combination)
      {
        std::size_t code = combination;
        std::vector<int> selected(static_cast<std::size_t>(horizon + 1), -1);
        for (int k = 1; k <= horizon; ++k) {
          const int candidates = preview.Nck[static_cast<std::size_t>(k)];
          selected[static_cast<std::size_t>(k)] =
            static_cast<int>(code % static_cast<std::size_t>(candidates));
          code /= static_cast<std::size_t>(candidates);
        }

        Matrix<double> direct_inequality(0.0, direct_row_count, input_count);
        Vector<double> direct_bound(0.0, direct_row_count);
        int direct_row = 0;
        for (int k = 1; k <= horizon; ++k) {
          const std::size_t step = static_cast<std::size_t>(k);
          const std::size_t candidate = static_cast<std::size_t>(selected[step]);
          const double heading = preview.psirk[step];
          const Eigen::Vector2d axis(-std::sin(heading), std::cos(heading));
          const double reference_projection = axis.dot(preview.prk[step]);
          double lower = axis.dot(preview.pmk[step][candidate]);
          double upper = axis.dot(preview.pMk[step][candidate]);
          if (lower > upper) {std::swap(lower, upper);}

          for (int column = 0; column < input_count; ++column) {
            const double projection =
              axis.x() * G[2 * k][column] + axis.y() * G[2 * k + 1][column];
            direct_inequality[direct_row][column] = projection;
            direct_inequality[direct_row + 1][column] = -projection;
          }
          direct_bound[direct_row++] = upper - reference_projection;
          direct_bound[direct_row++] = -lower + reference_projection;
        }
        for (int input_row = 0; input_row < input_rows; ++input_row) {
          for (int column = 0; column < input_count; ++column) {
            direct_inequality[direct_row][column] =
              inequality[lane_rows + input_row][column];
          }
          direct_bound[direct_row++] = inequality_bound[lane_rows + input_row];
        }

        QuadraticProblem direct_solver(false);
        Variable * direct_variable =
          direct_solver.vector_variable(input_count, "upper_direct_corridor");
        if (!direct_solver.add_variable(direct_variable)) {continue;}
        const Var direct_decision = direct_solver.get_variable(direct_variable);
        const Var direct_main = direct_solver.get_main_variable();
        Constraint direct_constraint(direct_main);
        direct_constraint.set_constraint_variable(direct_decision, direct_inequality);
        direct_constraint.set_known_term(direct_bound);
        if (!direct_solver.add_leq_constraint(direct_constraint) ||
          !direct_solver.set_Q_matrix(direct_hessian) ||
          !direct_solver.set_q0_vector(direct_linear))
        {
          continue;
        }

        Vector<double> direct_argument;
        const double direct_objective = timed_solve(direct_solver, direct_argument);
        ++result.direct_corridor_combinations;
        if (!std::isfinite(direct_objective) ||
          direct_argument.size() < static_cast<unsigned int>(input_count))
        {
          continue;
        }
        const auto direct_feasibility = miqp_solution_validation::evaluate(
          direct_inequality, direct_bound, direct_equality, direct_equality_bound,
          direct_argument, static_cast<std::size_t>(input_count),
          miqp_feasibility_tolerance_);
        if (!direct_feasibility.valid || direct_objective >= best_direct_objective) {
          continue;
        }
        best_direct_argument = direct_argument;
        best_direct_objective = direct_objective;
        best_candidates = selected;
      }

      if (std::isfinite(best_direct_objective) &&
        best_direct_argument.size() >= static_cast<unsigned int>(input_count))
      {
        Vector<double> reconstructed_argument(0.0, variable_count);
        for (int index = 0; index < input_count; ++index) {
          reconstructed_argument[index] = best_direct_argument[index];
        }
        for (int k = 1; k <= horizon; ++k) {
          const int selected = best_candidates[static_cast<std::size_t>(k)];
          reconstructed_argument[input_count +
            binary_offsets[static_cast<std::size_t>(k - 1)] + selected] = 1.0;
        }

        const auto reconstructed_feasibility = miqp_solution_validation::evaluate(
          inequality, inequality_bound, equality, equality_bound,
          reconstructed_argument, static_cast<std::size_t>(input_count),
          miqp_feasibility_tolerance_);
        if (reconstructed_feasibility.valid) {
          argument = reconstructed_argument;
          feasibility = reconstructed_feasibility;
          objective = 0.0;
          for (int r = 0; r < variable_count; ++r) {
            objective += linear_term[r] * argument[r];
            for (int c = 0; c < variable_count; ++c) {
              objective += 0.5 * argument[r] * hessian[r][c] * argument[c];
            }
          }
          result.polished = true;
        }
      }
  }

  result.solve_time_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count();
  if (!feasibility.valid) {
    std::ostringstream stream;
    if (combination_count_supported) {
      stream << "DAQP direct corridor QP infeasible: tried=" <<
        result.direct_corridor_combinations;
    } else {
      stream << "DAQP MIQP rejected by post-solve feasibility check: ineq=" <<
        feasibility.max_inequality_violation << " row=" <<
        feasibility.max_inequality_row << " lhs=" <<
        feasibility.max_inequality_lhs << " ub=" <<
        feasibility.max_inequality_bound << " eq=" <<
        feasibility.max_equality_violation << " binary=" <<
        feasibility.max_binary_violation << " direct_qp=" <<
        result.direct_corridor_combinations;
    }
    result.status = stream.str();
    return result;
  }

  result.correction_input.resize(static_cast<std::size_t>(input_count), 0.0);
  result.nominal_input.resize(static_cast<std::size_t>(input_count), 0.0);
  result.actual_input.resize(static_cast<std::size_t>(input_count), 0.0);
  for (int index = 0; index < input_count; ++index) {
    result.correction_input[static_cast<std::size_t>(index)] = argument[index];
    result.nominal_input[static_cast<std::size_t>(index)] = nominal_input[index];
    result.actual_input[static_cast<std::size_t>(index)] =
      argument[index] + nominal_input[index];
  }

  result.predicted_body.resize(static_cast<std::size_t>(horizon + 1));
  std::vector<Eigen::Vector2d> position_error(
    static_cast<std::size_t>(horizon + 1), Eigen::Vector2d::Zero());
  for (int k = 0; k <= horizon; ++k) {
    Eigen::Vector2d error = Eigen::Vector2d::Zero();
    for (int column = 0; column < input_count; ++column) {
      error.x() += G[2 * k][column] * argument[column];
      error.y() += G[2 * k + 1][column] * argument[column];
    }
    position_error[static_cast<std::size_t>(k)] =
      error + position_cost_offset[static_cast<std::size_t>(k)];
    result.predicted_body[static_cast<std::size_t>(k)] =
      preview.prk[static_cast<std::size_t>(k)] + error;
    if (!finitePoint(result.predicted_body[static_cast<std::size_t>(k)])) {
      result.status = "DAQP MIQP produced a non-finite predicted position";
      return result;
    }
  }

  result.selected_corridors.assign(static_cast<std::size_t>(horizon + 1), -1);
  for (int k = 1; k <= horizon; ++k) {
    for (int candidate = 0;
      candidate < preview.Nck[static_cast<std::size_t>(k)]; ++candidate)
    {
      const int index = input_count +
        binary_offsets[static_cast<std::size_t>(k - 1)] + candidate;
      if (argument[index] > 0.5) {
        result.selected_corridors[static_cast<std::size_t>(k)] = candidate;
        break;
      }
    }
  }

  result.psi_pred_cost_rad.assign(static_cast<std::size_t>(horizon), 0.0);
  result.psi_pred_model_rad.assign(static_cast<std::size_t>(horizon), 0.0);
  double accumulated_correction_yaw = 0.0;
  double accumulated_model_yaw = 0.0;
  for (int k = 0; k < horizon; ++k) {
    accumulated_correction_yaw += argument[2 * k + 1];
    accumulated_model_yaw += argument[2 * k + 1] + nominal_input[2 * k + 1];
    result.psi_pred_cost_rad[static_cast<std::size_t>(k)] = accumulated_correction_yaw;
    result.psi_pred_model_rad[static_cast<std::size_t>(k)] = accumulated_model_yaw;
  }

  result.position_cost = 0.0;
  for (int k = 0; k <= horizon; ++k) {
    result.position_cost += 0.5 * position_weights[static_cast<std::size_t>(k)] *
      position_error[static_cast<std::size_t>(k)].squaredNorm();
  }
  result.terminal_position_error =
    position_error[static_cast<std::size_t>(horizon)].norm();
  result.input_cost = 0.0;
  for (int k = 0; k < horizon; ++k) {
    const double dv = argument[2 * k];
    const double dpsi = argument[2 * k + 1];
    result.input_cost += 0.5 * (
      normalized_dv_weight * dv * dv + normalized_dpsi_weight * dpsi * dpsi);
  }
  result.turn_cost = 0.0;
  const double turn_base_weight =
    lambda_relative_turn_ /
    (relative_turn_scale_rad_ * relative_turn_scale_rad_);
  for (std::size_t i = 0; i < result.turn_window_steps.size(); ++i) {
    const int step = result.turn_window_steps[i];
    const double prediction = result.psi_pred_cost_rad[static_cast<std::size_t>(step - 1)];
    const double error = wrapToPi(prediction - result.turn_target_rad[i]);
    result.turn_cost += 0.5 * turn_base_weight *
      result.turn_stage_weights[i] * error * error;
  }

  result.objective = objective;
  result.status = "solved";
  result.valid = true;
  return result;
}

double SdMapUpperPlannerNode::remainingPathLength(
  const std::vector<Eigen::Vector2d> & path,
  const Eigen::Vector2d & position) const
{
  if (path.size() < 2) {return std::numeric_limits<double>::infinity();}
  std::size_t best_segment = 0;
  double best_ratio = 0.0;
  double best_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const Eigen::Vector2d segment = path[i + 1] - path[i];
    const double length_squared = segment.squaredNorm();
    if (length_squared <= 1e-12) {continue;}
    const double ratio = clampd(
      (position - path[i]).dot(segment) / length_squared, 0.0, 1.0);
    const double distance =
      (position - (path[i] + ratio * segment)).squaredNorm();
    if (distance < best_distance) {
      best_distance = distance;
      best_segment = i;
      best_ratio = ratio;
    }
  }
  double remaining =
    (1.0 - best_ratio) * (path[best_segment + 1] - path[best_segment]).norm();
  for (std::size_t i = best_segment + 1; i + 1 < path.size(); ++i) {
    remaining += (path[i + 1] - path[i]).norm();
  }
  return remaining;
}

std::vector<Eigen::Vector2d> SdMapUpperPlannerNode::densifyPath(
  const std::vector<Eigen::Vector2d> & sparse_world,
  double current_speed_mps, std::vector<double> & sample_s) const
{
  sample_s.clear();
  if (sparse_world.size() < 2) {
    sample_s.assign(sparse_world.size(), 0.0);
    return sparse_world;
  }
  std::vector<Eigen::Vector2d> filtered_sparse;
  filtered_sparse.reserve(sparse_world.size());
  filtered_sparse.push_back(sparse_world.front());
  for (std::size_t i = 1; i < sparse_world.size(); ++i) {
    if ((sparse_world[i] - filtered_sparse.back()).norm() > 1e-8) {
      filtered_sparse.push_back(sparse_world[i]);
    }
  }
  if (filtered_sparse.size() < 2) {
    sample_s.assign(filtered_sparse.size(), 0.0);
    return filtered_sparse;
  }

  const double spacing = std::max(
    std::max(0.0, current_speed_mps) * lower_prediction_dt_sec_,
    lower_min_path_spacing_m_);
  std::vector<double> arc(filtered_sparse.size(), 0.0);
  for (std::size_t i = 1; i < filtered_sparse.size(); ++i) {
    arc[i] = arc[i - 1] + (filtered_sparse[i] - filtered_sparse[i - 1]).norm();
  }
  if (arc.back() <= 1e-9) {sample_s = arc; return filtered_sparse;}

  std::vector<Eigen::Vector2d> dense;
  std::size_t segment = 0;
  for (double query = 0.0; query < arc.back(); query += spacing) {
    while (segment + 1 < arc.size() && arc[segment + 1] < query) {++segment;}
    const double length = arc[segment + 1] - arc[segment];
    const double ratio = length > 1e-12 ? (query - arc[segment]) / length : 0.0;
    sample_s.push_back(query);
    dense.push_back(
      filtered_sparse[segment] + ratio *
      (filtered_sparse[segment + 1] - filtered_sparse[segment]));
  }
  if (dense.empty() || (dense.back() - filtered_sparse.back()).norm() > 1e-6) {
    dense.push_back(filtered_sparse.back());
    sample_s.push_back(arc.back());
  }
  return dense;
}

void SdMapUpperPlannerNode::publishPath(
  const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr & publisher,
  const std::vector<Eigen::Vector2d> & points,
  const std::string & frame_id, const std::vector<double> * sample_s) const
{
  if (!publisher) {return;}
  nav_msgs::msg::Path message;
  message.header.stamp = now();
  message.header.frame_id = frame_id;
  message.poses.resize(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    message.poses[i].header = message.header;
    message.poses[i].pose.position.x = points[i].x();
    message.poses[i].pose.position.y = points[i].y();
    double yaw = 0.0;
    if (i + 1 < points.size()) {
      const Eigen::Vector2d tangent = points[i + 1] - points[i];
      if (tangent.norm() > 1e-9) {yaw = std::atan2(tangent.y(), tangent.x());}
    } else if (i > 0) {
      const Eigen::Vector2d tangent = points[i] - points[i - 1];
      if (tangent.norm() > 1e-9) {yaw = std::atan2(tangent.y(), tangent.x());}
    }
    message.poses[i].pose.orientation = yawQuaternion(yaw);
  }
  if (sample_s) {
    if (sample_s->size() != points.size()) {
      RCLCPP_ERROR(get_logger(), "dense path/s size mismatch; path not published");
      return;
    }
    imac_interfaces::msg::PathWithArcLength bundled;
    bundled.path = message;
    bundled.s = *sample_s;
    dense_arc_path_pub_->publish(bundled);
  }
  publisher->publish(message);
}

void SdMapUpperPlannerNode::publishGoalReached(bool reached)
{
  std_msgs::msg::Bool message;
  message.data = reached;
  goal_reached_pub_->publish(message);
}

void SdMapUpperPlannerNode::publishIntervalDebug(
  const PreviewConstraintData & preview) const
{
  std_msgs::msg::Float64MultiArray message;
  for (std::size_t step = 0; step < preview.raw_lengths.size(); ++step) {
    for (std::size_t candidate = 0;
      candidate < preview.raw_lengths[step].size(); ++candidate)
    {
      message.data.push_back(static_cast<double>(step));
      message.data.push_back(static_cast<double>(candidate));
      message.data.push_back(preview.raw_lengths[step][candidate]);
      message.data.push_back(preview.processed_lengths[step][candidate]);
      message.data.push_back(static_cast<double>(preview.treatment_modes[step][candidate]));
      message.data.push_back(static_cast<double>(preview.source_modes[step][candidate]));
      message.data.push_back(
        static_cast<double>(preview.fallback_reason_codes[step]));
    }
  }
  interval_debug_pub_->publish(message);

  // Keep the existing seven-column message unchanged for current consumers.
  // Main9 details cover ALL requested stages, including the unusable suffix.
  std_msgs::msg::Float64MultiArray details;
  for (std::size_t step = 0; step < preview.interval_info.size(); ++step) {
    for (std::size_t candidate = 0; candidate < preview.interval_info[step].size(); ++candidate) {
      const auto & info = preview.interval_info[step][candidate];
      const auto & observation = info.observation;
      const std::vector<double> row{
        static_cast<double>(step), static_cast<double>(candidate), observation.length,
        info.processed_length, info.treatment.centering_on ? 1.0 : 0.0,
        observation.start_boundary_observed ? 1.0 : 0.0,
        observation.end_boundary_observed ? 1.0 : 0.0,
        (!observation.nominal_fallback &&
        (!observation.start_boundary_observed || !observation.end_boundary_observed)) ? 1.0 : 0.0,
        observation.nominal_fallback ? 1.0 : 0.0,
        observation.reference_inside ? 1.0 : 0.0,
        preview.interval_update.state.reference_length, info.treatment.threshold};
      details.data.insert(details.data.end(), row.begin(), row.end());
    }
  }
  details.layout.dim.resize(2);
  details.layout.dim[0].label = "intervals";
  details.layout.dim[0].size = static_cast<uint32_t>(details.data.size() / 12U);
  details.layout.dim[0].stride = static_cast<uint32_t>(details.data.size());
  details.layout.dim[1].label =
    "k,candidate,raw,processed,on,start_observed,end_observed,clipped,fallback,inside,B,threshold";
  details.layout.dim[1].size = 12U;
  details.layout.dim[1].stride = 12U;
  interval_info_pub_->publish(details);

  const auto & update = preview.interval_update;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  std_msgs::msg::Float64MultiArray state_message;
  state_message.data = {
    now().seconds(), update.previous_reference_length, update.state.reference_length,
    hasIntervalReference(update.state) ?
    interval_centering_settings_.relative_length_factor * update.state.reference_length : nan,
    update.candidate_length, static_cast<double>(update.candidate_step),
    static_cast<double>(update.candidate_interval)};
  state_message.layout.dim.resize(1);
  state_message.layout.dim[0].label =
    "stamp,B_before,B,threshold,candidate,candidate_step,candidate_interval";
  state_message.layout.dim[0].size = static_cast<uint32_t>(state_message.data.size());
  state_message.layout.dim[0].stride = static_cast<uint32_t>(state_message.data.size());
  interval_state_pub_->publish(state_message);
}

void SdMapUpperPlannerNode::publishBranchEvent(
  double started, std::uint64_t sequence, std::uint64_t revision,
  const PoseSnapshot & pose, int wp0, int wp1,
  const PreviewConstraintData * preview, const UpperSolveResult * solve,
  const std::string & reason, const std::string & status, bool published)
{
  const double emitted = now().seconds();
  const double c = std::cos(pose.yaw_rad), s = std::sin(pose.yaw_rad);
  auto world = [&](const Eigen::Vector2d & p) -> Eigen::Vector2d {
      return pose.position + Eigen::Vector2d(c*p.x()-s*p.y(),s*p.x()+c*p.y());
    };
  std::ostringstream out;
  auto point = [&](const Eigen::Vector2d & p) {
      out << '[' << branchJsonNumber(p.x()) << ',' << branchJsonNumber(p.y()) << ']';
    };
  auto points = [&](const std::vector<Eigen::Vector2d> & values, bool body) {
      out << '['; bool first = true;
      for (const auto & p : values) {
        if (!first) {out << ',';} first = false; point(body ? world(p) : p);
      }
      out << ']';
    };
  int branch_step = -1, max_candidates = 0;
  if (preview) {
    for (std::size_t k = 1; k < preview->Nck.size(); ++k) {
      max_candidates = std::max(max_candidates, preview->Nck[k]);
      if (branch_step < 0 && preview->Nck[k] >= 2) {branch_step = static_cast<int>(k);}
    }
  }
  const bool solver_called = solve && std::isfinite(solve->solver_only_time_ms);
  const bool branch_failure = branch_step >= 1 && solve && !solve->valid &&
    (reason == "solver_rejected" || reason == "solver_setup_failed");
  const bool retained = !published && diagnostic_last_revision_ == revision &&
    !diagnostic_last_path_.empty();
  out << "{\"schema_version\":1,\"t_start\":" << branchJsonNumber(started)
      << ",\"t_emit\":" << branchJsonNumber(emitted)
      << ",\"plan_seq\":" << sequence << ",\"path_revision\":" << revision
      << ",\"frame_id\":" << branchJsonString(pose.frame_id)
      << ",\"x\":" << branchJsonNumber(pose.position.x())
      << ",\"y\":" << branchJsonNumber(pose.position.y())
      << ",\"yaw_rad\":" << branchJsonNumber(pose.yaw_rad)
      << ",\"wp0\":" << wp0 << ",\"wp1\":" << wp1
      << ",\"horizon\":" << (preview ? preview->N_pred : 0)
      << ",\"branch_step\":" << branch_step << ",\"max_candidates\":" << max_candidates
      << ",\"branch_failure\":" << (branch_failure ? "true" : "false")
      << ",\"valid_plan\":" << (published ? "true" : "false")
      << ",\"solver_valid\":" << (solve && solve->valid ? "true" : "false")
      << ",\"solver_called\":" << (solver_called ? "true" : "false")
      << ",\"solver_time_ms\":" << branchJsonNumber(solve ? solve->solver_only_time_ms : NAN)
      << ",\"reason\":" << branchJsonString(reason)
      << ",\"status\":" << branchJsonString(status)
      << ",\"retained_previous_path\":" << (retained ? "true" : "false")
      << ",\"retained_path_age_sec\":" << branchJsonNumber(retained ? emitted-diagnostic_last_path_stamp_ : NAN)
      << ",\"selected_corridors\":[";
  if (solve) {
    for (std::size_t k = 0; k < solve->selected_corridors.size(); ++k) {
      if (k) {out << ',';} out << solve->selected_corridors[k];
    }
  }
  out << "],\"candidate_counts\":[";
  if (preview) {
    for (std::size_t k = 0; k < preview->Nck.size(); ++k) {
      if (k) {out << ',';} out << preview->Nck[k];
    }
  }
  out << "],\"intervals_world\":[";
  bool first = true;
  if (preview) {
    for (std::size_t k = 1; k < preview->pmk.size(); ++k) {
      for (std::size_t j = 0; j < preview->pmk[k].size(); ++j) {
        if (!first) {out << ',';} first = false;
        const Eigen::Vector2d a = world(preview->pmk[k][j]), b = world(preview->pMk[k][j]);
        out << '[' << k << ',' << j << ',' << branchJsonNumber(a.x()) << ','
            << branchJsonNumber(a.y()) << ',' << branchJsonNumber(b.x()) << ','
            << branchJsonNumber(b.y()) << ',' << branchJsonNumber(preview->raw_lengths[k][j])
            << ',' << branchJsonNumber(preview->processed_lengths[k][j]) << ','
            << preview->source_modes[k][j] << ',' << preview->fallback_reason_codes[k] << ']';
      }
    }
  }
  out << "],\"preview_world\":";
  points(preview ? preview->prk : std::vector<Eigen::Vector2d>{}, true);
  out << ",\"planned_world\":";
  points(solve && solve->valid ? solve->predicted_body : std::vector<Eigen::Vector2d>{}, true);
  out << ",\"retained_world\":";
  points(retained ? diagnostic_last_path_ : std::vector<Eigen::Vector2d>{}, false);
  out << '}';
  std_msgs::msg::String msg; msg.data = out.str(); branch_event_pub_->publish(msg);
  if (published && solve) {
    diagnostic_last_path_.clear();
    for (const auto & p : solve->predicted_body) {diagnostic_last_path_.push_back(world(p));}
    diagnostic_last_revision_ = revision; diagnostic_last_path_stamp_ = emitted;
  }
}

void SdMapUpperPlannerNode::publishUpperTrace(
  const PoseSnapshot & pose,
  int wp0_idx,
  int wp1_idx,
  const PreviewConstraintData & preview,
  const UpperSolveResult & solve) const
{
  std_msgs::msg::Float64MultiArray trace;
  trace.data = {
    now().seconds(),
    pose.position.x(),
    pose.position.y(),
    pose.yaw_rad,
    pose.speed_mps,
    static_cast<double>(wp0_idx),
    static_cast<double>(wp1_idx),
    static_cast<double>(preview.N_pred),
    solve.valid ? 1.0 : 0.0,
    solve.objective,
    solve.solve_time_ms,
    preview.road_segment_heading_rad[0],
    preview.road_segment_heading_rad[1],
    preview.delta_psi_road_rad,
    static_cast<double>(solve.branch_step),
    solve.position_cost,
    solve.input_cost,
    solve.turn_cost
  };
  for (double heading : preview.psirk) {
    trace.data.push_back(heading);
  }
  for (int count : preview.Nck) {
    trace.data.push_back(static_cast<double>(count));
  }
  for (int selected : solve.selected_corridors) {
    trace.data.push_back(static_cast<double>(selected));
  }
  for (double value : solve.psi_pred_cost_rad) {
    trace.data.push_back(value);
  }
  for (double value : solve.psi_pred_model_rad) {
    trace.data.push_back(value);
  }
  for (std::size_t i = 0; i < solve.turn_window_steps.size(); ++i) {
    trace.data.push_back(static_cast<double>(solve.turn_window_steps[i]));
    trace.data.push_back(solve.turn_target_rad[i]);
    trace.data.push_back(solve.turn_stage_weights[i]);
  }
  // Append terminal-reference diagnostics so existing trace field offsets stay
  // unchanged for downstream MATLAB parsers.
  trace.data.push_back(solve.terminal_position_error);
  trace.data.push_back(terminal_position_weight_multiplier_);
  trace.data.push_back(preview.terminal_buffer_turn_rad);
  trace.data.push_back(preview.stable_heading_rad);
  trace.data.push_back(preview.waypoint_bearing_rad);
  trace.data.push_back(preview.used_previous_solution ? 1.0 : 0.0);
  upper_trace_pub_->publish(trace);
}

void SdMapUpperPlannerNode::plannerLoop()
{
  const auto cycle_started = std::chrono::steady_clock::now();
  const double cycle_stamp = now().seconds();
  PoseSnapshot pose;
  {
    std::lock_guard<std::mutex> lock(pose_mtx_);
    pose = pose_;
  }
  if (!pose.valid) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "planner waiting for pose");
    return;
  }
  if (!pose.speed_valid) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "planner waiting for a valid finite-difference speed sample");
    return;
  }

  const rclcpp::Time current_time = now();
  if (input_timeout_sec_ > 0.0 &&
    (current_time - pose.received).seconds() > input_timeout_sec_)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "planner pose is stale");
    return;
  }

  GridMapSnapshot grid;
  {
    std::lock_guard<std::mutex> lock(grid_mtx_);
    grid = grid_;
  }
  bool grid_available = grid.valid;
  if (grid_available && input_timeout_sec_ > 0.0 &&
    (current_time - grid.received).seconds() > input_timeout_sec_)
  {
    grid_available = false;
  }
  if (require_grid_map_ && !grid_available) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "planner grid is missing, invalid, or stale; retaining the last published path");
    return;
  }

  std::vector<Eigen::Vector2d> global_path;
  std::string frame_id;
  int wp0_idx = 0;
  int wp1_idx = 1;
  bool already_reached = false;
  std::uint64_t planning_revision = 0;
  {
    std::lock_guard<std::mutex> lock(path_mtx_);
    global_path = global_path_;
    frame_id = global_path_frame_id_;
    wp0_idx = wp0_index_;
    wp1_idx = wp1_index_;
    already_reached = goal_reached_;
    planning_revision = planning_revision_;
  }
  if (global_path.size() < 2) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "planner waiting for global path");
    return;
  }
  if (!normalizedFrameId(pose.frame_id).empty() &&
    !normalizedFrameId(frame_id).empty() &&
    !frameIdsEquivalent(pose.frame_id, frame_id))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "pose/path frame mismatch: pose='%s' path='%s'; TF conversion is not implemented",
      pose.frame_id.c_str(), frame_id.c_str());
    return;
  }

  const double remaining = remainingPathLength(global_path, pose.position);
  const double direct_goal_distance = (global_path.back() - pose.position).norm();
  const bool reached = already_reached ||
    (goal_stop_distance_m_ >= 0.0 &&
    (remaining <= goal_stop_distance_m_ || direct_goal_distance <= goal_stop_distance_m_));
  if (reached) {
    {
      std::lock_guard<std::mutex> lock(path_mtx_);
      goal_reached_ = true;
    }
    publishGoalReached(true);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "goal reached: remaining=%.3f direct=%.3f", remaining, direct_goal_distance);
    return;
  }
  publishGoalReached(false);

  // main8.m uses vr=3.0 as a fixed upper-model speed. Keeping delta_s and G
  // independent of measured SIH speed also prevents N_pred from oscillating
  // as the vehicle accelerates through BEV extent thresholds.
  const double planning_speed_mps = target_speed_mps_;

  // find_wp.m uses the commanded moving direction even at the initialized
  // route pose. Fall back to the nominal speed so a stationary first odometry
  // sample can still expose [wp1, wp2] using the measured yaw direction.
  const double waypoint_progress_speed = pose.speed_mps > 1e-9 ?
    pose.speed_mps : std::max(0.0, target_speed_mps_);
  const Eigen::Vector2d velocity = waypoint_progress_speed *
    Eigen::Vector2d(std::cos(pose.yaw_rad), std::sin(pose.yaw_rad));
  const int previous_wp0_idx = wp0_idx;
  const int previous_wp1_idx = wp1_idx;
  const auto pair = egocentric_planner_geometry::findWaypointPair(
    global_path, wp0_idx, pose.position, velocity, waypoint_switch_eps_m_,
    waypoint_switch_distance_m_);
  wp0_idx = pair[0];
  wp1_idx = pair[1];
  if (wp1_idx <= wp0_idx || wp1_idx >= static_cast<int>(global_path.size())) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "no valid forward waypoint pair remains: wp0=%d wp1=%d size=%zu",
      wp0_idx, wp1_idx, global_path.size());
    return;
  }

  bool committed_pair_change = false;
  {
    std::lock_guard<std::mutex> lock(path_mtx_);
    if (planning_revision_ == planning_revision) {
      wp0_index_ = wp0_idx;
      wp1_index_ = wp1_idx;
      committed_pair_change =
        wp0_idx != previous_wp0_idx || wp1_idx != previous_wp1_idx;
    }
  }
  if (committed_pair_change) {
    // main8.m carries the successful upper optimum continuously across the
    // waypoint-pair boundary. It is re-expressed with the current measured
    // yaw below, so the lower reference does not jump back to a fresh seed.
    RCLCPP_INFO(
      get_logger(), "waypoint pair advanced: [%d,%d] -> [%d,%d]; previous reference retained",
      previous_wp0_idx, previous_wp1_idx, wp0_idx, wp1_idx);
  }

  const bool has_previous_waypoint = wp0_idx > 0;
  const Eigen::Vector2d wp0_world = global_path[static_cast<std::size_t>(wp0_idx)];
  const Eigen::Vector2d wp1_world = global_path[static_cast<std::size_t>(wp1_idx)];
  const Eigen::Vector2d wp_prev_world = has_previous_waypoint ?
    global_path[static_cast<std::size_t>(wp0_idx - 1)] : pose.position;

  const double cosine = std::cos(pose.yaw_rad);
  const double sine = std::sin(pose.yaw_rad);
  Eigen::Matrix2d rotation_body_world;
  rotation_body_world << cosine, sine, -sine, cosine;
  Eigen::Matrix2d rotation_world_body;
  rotation_world_body << cosine, -sine, sine, cosine;

  const Eigen::Vector2d wp0_body = rotation_body_world * (wp0_world - pose.position);
  const Eigen::Vector2d wp1_body = rotation_body_world * (wp1_world - pose.position);
  const Eigen::Vector2d wp_prev_body =
    rotation_body_world * (wp_prev_world - pose.position);

  int requested_horizon = computeRequestedPreviewSteps(
    grid_available ? &grid : nullptr, planning_speed_mps);
  const int extent_limited_horizon = requested_horizon;
  if (requested_horizon <= 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "no preview horizon fits in the current BEV extent");
    return;
  }

  std::vector<Eigen::Vector2d> previous_reference_body;
  {
    std::lock_guard<std::mutex> lock(previous_solution_mtx_);
    if (previous_solution_valid_) {
      const bool same_revision = previous_solution_revision_ == planning_revision;
      if (same_revision && previous_optimal_path_world_.size() >= 3U) {
        // Preserve MATLAB's shortened horizon unless full-horizon recovery is
        // enabled. On growth, the size check below discards the shorter seed
        // and buildPreviewReference constructs a new one from the current pose.
        requested_horizon = egocentric_planner_geometry::retainMatlabPreviewHorizon(
          requested_horizon, previous_optimal_path_world_.size(),
          preview_restore_full_horizon_);
      }
      bool compatible = same_revision &&
        previous_optimal_path_world_.size() ==
        static_cast<std::size_t>(requested_horizon + 1);
      for (const auto & point : previous_optimal_path_world_) {
        compatible = compatible && finitePoint(point);
      }
      if (compatible) {
        compatible = egocentric_planner_geometry::buildMeasuredFrameShiftedPrefix(
          previous_optimal_path_world_, pose.yaw_rad, requested_horizon,
          previous_reference_body);
      }
      if (!compatible) {
        previous_optimal_path_world_.clear();
        previous_solution_valid_ = false;
      } else {
        RCLCPP_DEBUG(
          get_logger(), "previous upper optimum reprojected with measured yaw %.6f rad",
          pose.yaw_rad);
      }
    }
  }

  // Emit one sample per planning attempt, including early failures. Stamp the
  // start so a solve begun before a route transition cannot enter the next log.
  struct TimingRecord
  {
    std::chrono::steady_clock::time_point started;
    double stamp;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher;
    std::uint64_t sequence;
    double solver_ms{std::numeric_limits<double>::quiet_NaN()};
    bool valid{false};
    ~TimingRecord()
    {
      std_msgs::msg::Float64MultiArray msg;
      const double plan_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
      // start ROS stamp, solver sum ms, planner cycle ms, valid plan,
      // attempt sequence, whether any solver was called.
      msg.data = {stamp, solver_ms, plan_ms, valid ? 1.0 : 0.0,
        static_cast<double>(sequence), std::isfinite(solver_ms) ? 1.0 : 0.0};
      publisher->publish(msg);
    }
  } timing{cycle_started, cycle_stamp, upper_timing_pub_, ++timing_attempt_seq_};

  const PreviewReferenceData reference = buildPreviewReference(
    wp_prev_body, wp0_body, wp1_body, has_previous_waypoint, planning_speed_mps,
    requested_horizon, previous_reference_body);
  if (!reference.valid) {
    publishBranchEvent(cycle_stamp, timing.sequence, planning_revision, pose, wp0_idx, wp1_idx,
      nullptr, nullptr, "preview_failed", reference.status, false);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "failed to build V8 preview reference: %s", reference.status.c_str());
    return;
  }

  std::vector<Eigen::Vector2d> preview_world;
  preview_world.reserve(reference.points_body.size());
  for (const auto & point : reference.points_body) {
    preview_world.push_back(pose.position + rotation_world_body * point);
  }
  publishPath(preview_debug_pub_, preview_world, frame_id);

  PreviewConstraintData constraints = extractPreviewIntervals(
    reference, grid_available ? &grid : nullptr, interval_centering_state_);
  interval_centering_state_ = constraints.interval_update.state;
  if (constraints.N_pred != upper_preview_steps_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "horizon_diag configured=%d extent_limit=%d requested=%d usable=%d "
      "reserve=%d restore=%d step_m=%.3f previous=%d",
      upper_preview_steps_, extent_limited_horizon, requested_horizon, constraints.N_pred,
      preview_grid_reserve_steps_, preview_restore_full_horizon_ ? 1 : 0,
      planning_speed_mps * upper_prediction_dt_sec_, reference.used_previous_solution ? 1 : 0);
  }
  applyTerminalPositionTargets(
    constraints, wp0_body, wp1_body, planning_speed_mps * upper_prediction_dt_sec_);
  publishIntervalDebug(constraints);
  if (constraints.N_pred <= 0) {
    UpperSolveResult failed;
    failed.status = "empty valid interval prefix";
    publishBranchEvent(cycle_stamp, timing.sequence, planning_revision, pose, wp0_idx, wp1_idx,
      &constraints, &failed, "no_candidate", failed.status, false);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "empty valid interval prefix; retaining the last published path");
    publishUpperTrace(pose, wp0_idx, wp1_idx, constraints, failed);
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "road turn wp=[%s%d %d] psi_in=%+.2fdeg psi_out=%+.2fdeg "
    "delta=%+.2fdeg N=%d plan_v=%.2fm/s",
    has_previous_waypoint ? "prev " : "ego ", wp0_idx, wp1_idx,
    radToDeg(constraints.road_segment_heading_rad[0]),
    radToDeg(constraints.road_segment_heading_rad[1]),
    radToDeg(constraints.delta_psi_road_rad), constraints.N_pred,
    planning_speed_mps);

  const UpperSolveResult solve = solveUpperMiqp(constraints, planning_speed_mps);
  timing.solver_ms = solve.solver_only_time_ms;
  // Publish actual solver work only, including unsuccessful solves. No sample
  // is emitted when planning returns before invoking a solver.
  if (std::isfinite(solve.solver_only_time_ms)) {
    std_msgs::msg::Float64 solver_time;
    solver_time.data = solve.solver_only_time_ms;
    upper_solver_time_pub_->publish(solver_time);
  }
  publishUpperTrace(pose, wp0_idx, wp1_idx, constraints, solve);
  if (!solve.valid) {
    publishBranchEvent(cycle_stamp, timing.sequence, planning_revision, pose, wp0_idx, wp1_idx,
      &constraints, &solve, std::isfinite(solve.solver_only_time_ms) ?
      "solver_rejected" : "solver_setup_failed", solve.status, false);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "egocentric MIQP failed: status=%s solve_ms=%.3f; retaining last published path",
      solve.status.c_str(), solve.solve_time_ms);
    return;
  }

  std::vector<Eigen::Vector2d> sparse_world;
  sparse_world.reserve(solve.predicted_body.size());
  for (const auto & point : solve.predicted_body) {
    sparse_world.push_back(pose.position + rotation_world_body * point);
  }
  std::vector<double> dense_s;
  const auto dense_world = densifyPath(sparse_world, planning_speed_mps, dense_s);

  {
    std::lock_guard<std::mutex> lock(path_mtx_);
    if (planning_revision_ != planning_revision) {
      publishBranchEvent(cycle_stamp, timing.sequence, planning_revision, pose, wp0_idx, wp1_idx,
        &constraints, &solve, "path_changed", "Global path changed during solve", false);
      RCLCPP_WARN(
        get_logger(),
        "discarding MIQP result because the global waypoint path changed during solve");
      return;
    }
    std::lock_guard<std::mutex> previous_lock(previous_solution_mtx_);
    if (sparse_world.size() == solve.predicted_body.size() && sparse_world.size() >= 3) {
      previous_optimal_path_world_ = sparse_world;
      previous_solution_revision_ = planning_revision;
      previous_solution_valid_ = true;
    } else {
      previous_optimal_path_world_.clear();
      previous_solution_valid_ = false;
    }
  }

  publishPath(sparse_path_pub_, sparse_world, frame_id);
  publishPath(dense_path_pub_, dense_world, frame_id, &dense_s);
  timing.valid = true;
  publishBranchEvent(cycle_stamp, timing.sequence, planning_revision, pose, wp0_idx, wp1_idx,
    &constraints, &solve, "success", solve.status, true);
  RCLCPP_INFO(
    get_logger(),
    "MIQP solved: N=%d branch=%d obj=%.6f Jpos=%.6f Jin=%.6f Jturn=%.6f "
    "solve_ms=%.3f polished=%s direct_qp=%d sparse=%zu dense=%zu",
    constraints.N_pred, solve.branch_step, solve.objective,
    solve.position_cost, solve.input_cost, solve.turn_cost,
    solve.solve_time_ms, solve.polished ? "yes" : "no",
    solve.direct_corridor_combinations,
    sparse_world.size(), dense_world.size());
}

}  // namespace imac_ctrl

// Unit tests link the same planner implementation without starting a ROS node.
#ifndef VIRTUAL_CONTROL_UPPER_PLANNER_NO_MAIN
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<imac_ctrl::SdMapUpperPlannerNode>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
#endif
