#pragma once

#include <rclcpp/rclcpp.hpp>

#include "virtual_control/interval_centering_state.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <Eigen/Dense>

#include <array>
#include <cstdint>
#include <mutex>
#include <limits>
#include <string>
#include <vector>

namespace imac_ctrl
{

// Egocentric OccupancyGrid + MIQP local planner extracted from the former
// monolithic TrackingControllerNode. The node publishes a body-frame-planned,
// world-frame local reference path and never publishes actuator commands.
class SdMapUpperPlannerNode : public rclcpp::Node
{
public:
  explicit SdMapUpperPlannerNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  enum class IntervalTreatmentMode : int
  {
    CenterContraction = 0,
    BoundaryMargin = 1
  };

  enum class IntervalSourceMode : int
  {
    BevObserved = 0,
    NominalFallback = 1
  };

  enum class IntervalFallbackReason : int
  {
    None = 0,
    LineOutsideBev = 1,
    InsufficientBevSamples = 2,
    NoValidDrivableRun = 3,
    GridUnavailable = 4
  };

  struct PoseSnapshot
  {
    Eigen::Vector2d position{Eigen::Vector2d::Zero()};
    double yaw_rad{0.0};
    double speed_mps{0.0};
    std::string frame_id;
    rclcpp::Time received{0, 0, RCL_ROS_TIME};
    bool speed_valid{false};
    bool valid{false};
  };

  struct GridMapSnapshot
  {
    double resolution{0.0};
    int width{0};
    int height{0};
    Eigen::Vector2d origin_body{Eigen::Vector2d::Zero()};
    double origin_yaw_rad{0.0};
    std::string frame_id;
    std::vector<int8_t> data;
    rclcpp::Time received{0, 0, RCL_ROS_TIME};
    bool valid{false};
  };

  struct PreviewReferenceData
  {
    bool valid{false};
    // Stable geometry used by the prediction model and lane constraints.
    std::vector<Eigen::Vector2d> points_body;
    std::vector<double> heading_rad;
    std::array<double, 2> road_segment_heading_rad{{0.0, 0.0}};
    double delta_psi_road_rad{0.0};
    double stable_heading_rad{0.0};
    double waypoint_bearing_rad{0.0};
    double terminal_buffer_turn_rad{0.0};
    bool used_previous_solution{false};
    std::string status;
  };

  struct IntervalInfo
  {
    interval_centering::Observation observation;
    interval_centering::Treatment treatment;
    double processed_length{0.0};
  };

  struct PreviewConstraintData
  {
    std::vector<std::vector<Eigen::Vector2d>> pmk;
    std::vector<std::vector<Eigen::Vector2d>> pMk;
    std::vector<std::vector<double>> raw_lengths;
    std::vector<std::vector<double>> processed_lengths;
    std::vector<std::vector<int>> treatment_modes;
    std::vector<std::vector<int>> source_modes;
    std::vector<int> fallback_reason_codes;
    std::vector<int> Nck;
    // Full requested preview, including observations beyond a missing MIQP stage.
    std::vector<std::vector<IntervalInfo>> interval_info;
    interval_centering::UpdateInfo interval_update;
    std::vector<Eigen::Vector2d> prk;
    std::vector<double> psirk;
    // Cost-only terminal targets; prk/psirk remain shared model/constraint geometry.
    std::vector<Eigen::Vector2d> position_targets;
    std::array<double, 2> road_segment_heading_rad{{0.0, 0.0}};
    double delta_psi_road_rad{0.0};
    double stable_heading_rad{0.0};
    double waypoint_bearing_rad{0.0};
    double terminal_buffer_turn_rad{0.0};
    bool used_previous_solution{false};
    int N_pred{0};
  };

  struct UpperSolveResult
  {
    bool valid{false};
    std::vector<Eigen::Vector2d> predicted_body;
    std::vector<int> selected_corridors;
    std::vector<double> correction_input;
    std::vector<double> nominal_input;
    std::vector<double> actual_input;
    std::vector<double> psi_pred_cost_rad;
    std::vector<double> psi_pred_model_rad;
    std::vector<int> turn_window_steps;
    std::vector<double> turn_target_rad;
    std::vector<double> turn_stage_weights;
    int branch_step{-1};
    double position_cost{0.0};
    double input_cost{0.0};
    double turn_cost{0.0};
    double terminal_position_error{0.0};
    double objective{0.0};
    double solve_time_ms{0.0};
    double solver_only_time_ms{std::numeric_limits<double>::quiet_NaN()};
    bool polished{false};
    int direct_corridor_combinations{0};
    std::string status;
  };

  void declareAndLoadParameters();
  void validateParameters() const;
  void createInterfaces();

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void poseStampedCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void gridMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void plannerLoop();

  bool loadReferencePathCsvIfConfigured();
  void applyReferencePath(
    const std::vector<Eigen::Vector2d> & path,
    const std::string & frame_id,
    const std::string & source_label);
  void invalidatePreviousReference();
  std::vector<Eigen::Vector2d> applyConfiguredWaypointBias(
    const std::vector<Eigen::Vector2d> & path) const;
  int computeRequestedPreviewSteps(
    const GridMapSnapshot * grid,
    double planning_speed_mps) const;
  PreviewReferenceData buildPreviewReference(
    const Eigen::Vector2d & wp_prev_body,
    const Eigen::Vector2d & wp0_body,
    const Eigen::Vector2d & wp1_body,
    bool has_previous_waypoint,
    double planning_speed_mps,
    int requested_horizon,
    const std::vector<Eigen::Vector2d> & previous_reference_body) const;
  PreviewConstraintData extractPreviewIntervals(
    const PreviewReferenceData & reference,
    const GridMapSnapshot * grid,
    const interval_centering::State & interval_state) const;
  void applyTerminalPositionTargets(
    PreviewConstraintData & preview,
    const Eigen::Vector2d & wp0_body,
    const Eigen::Vector2d & wp1_body,
    double delta_s) const;
  bool clipLineToGridRectangle(
    const Eigen::Vector2d & point_body,
    const Eigen::Vector2d & direction_body,
    const GridMapSnapshot & grid,
    double & lambda_min,
    double & lambda_max) const;
  bool bodyPointToGrid(
    const Eigen::Vector2d & point_body,
    const GridMapSnapshot & grid,
    int & gx,
    int & gy) const;
  bool sampleGridValue(
    const Eigen::Vector2d & point_body,
    const GridMapSnapshot & grid,
    int8_t & value) const;
  bool isDrivable(int8_t value) const;

  UpperSolveResult solveUpperMiqp(
    const PreviewConstraintData & preview,
    double planning_speed_mps) const;
  std::vector<Eigen::Vector2d> densifyPath(
    const std::vector<Eigen::Vector2d> & sparse_world,
    double current_speed_mps) const;
  double remainingPathLength(
    const std::vector<Eigen::Vector2d> & path,
    const Eigen::Vector2d & position) const;

  void publishPath(
    const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr & publisher,
    const std::vector<Eigen::Vector2d> & points,
    const std::string & frame_id) const;
  void publishGoalReached(bool reached);
  void publishBranchEvent(
    double started, std::uint64_t sequence, std::uint64_t revision,
    const PoseSnapshot & pose, int wp0, int wp1,
    const PreviewConstraintData * preview, const UpperSolveResult * solve,
    const std::string & reason, const std::string & status, bool published);
  std::vector<Eigen::Vector2d> diagnostic_last_path_;
  std::uint64_t diagnostic_last_revision_{0};
  double diagnostic_last_path_stamp_{0.0};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr branch_event_pub_;

  void publishUpperTrace(
    const PoseSnapshot & pose,
    int wp0_idx,
    int wp1_idx,
    const PreviewConstraintData & preview,
    const UpperSolveResult & solve) const;
  void publishIntervalDebug(const PreviewConstraintData & preview) const;

  // Multi-rate timing and vehicle model.
  double upper_planner_rate_hz_{2.0};
  double upper_prediction_dt_sec_{1.0};
  int upper_preview_steps_{7};
  bool preview_limit_to_grid_extent_{true};
  int preview_grid_reserve_steps_{1};
  bool preview_restore_full_horizon_{false};
  double lower_prediction_dt_sec_{0.1};
  double lower_min_path_spacing_m_{0.05};
  double target_speed_mps_{3.0};
  std::string scale_mode_{"model"};
  double wheelbase_{0.3};
  double vehicle_length_m_{0.42};
  double bev_forward_m_{1.5};
  double max_steering_angle_rad_{0.5235987756};
  double a_max_mps2_{1.0};

  // V8 normalized MIQP objective.
  double miqp_big_m_{100.0};
  double position_scale_m_{10.0};
  double relative_turn_scale_rad_{0.3490658504};
  double lambda_position_{1.0};
  double lambda_relative_turn_{0.9};
  double terminal_buffer_max_turn_rad_{0.3490658504};
  double terminal_position_weight_multiplier_{1000.0};
  double upper_r_dv_{1.0};
  double upper_r_dpsi_{1.0};
  bool normalize_upper_input_cost_{false};
  int turn_preview_steps_{2};
  double turn_weight_growth_{2.0};
  double miqp_diagonal_regularization_{1e-10};
  double miqp_binary_diagonal_regularization_{1e-4};
  double miqp_feasibility_tolerance_{1e-3};

  // Waypoint selection and optional rigid path bias from main8.m.
  double waypoint_switch_eps_m_{10.0};
  double waypoint_switch_distance_m_{10.0};
  double goal_stop_distance_m_{1.0};
  bool enable_waypoint_bias_{false};
  double waypoint_x_bias_m_{0.0};
  double waypoint_y_bias_m_{0.0};
  double waypoint_yaw_bias_rad_{0};

  // Input availability.
  double input_timeout_sec_{0.6};
  bool require_grid_map_{true};

  // Global path source.
  bool use_csv_global_path_{false};
  std::string csv_global_path_file_{"local_map.csv"};
  bool allow_topic_path_override_when_csv_loaded_{false};
  bool reset_waypoint_on_path_update_{false};

  // V8 straight preview and BEV interval extraction.
  double preview_min_target_distance_m_{1e-6};
  double preview_line_sample_m_{0.10};
  int preview_min_segment_samples_{2};
  double preview_interval_soft_ratio_{0.70};
  interval_centering::Settings interval_centering_settings_;
  double preview_interval_boundary_margin_m_{2.0};
  bool preview_interval_enable_nominal_fallback_{true};
  double preview_interval_nominal_fallback_width_m_{4.5};
  double preview_interval_fallback_max_centering_length_m_{9.0};
  bool preview_interval_debug_{false};

  // OccupancyGrid value convention. Unknown cells remain value < 0.
  int grid_value_threshold_{0};
  bool grid_positive_is_drivable_{true};
  std::string expected_grid_frame_id_{"base_link"};
  bool require_grid_frame_match_{true};

  // Vehicle-state input and pose transformation.
  std::string state_input_type_{"odom"};
  std::string odom_topic_{"/px4/sih/odom_map"};
  bool odom_yaw_is_orientation_z_{false};
  std::string pose_stamped_topic_{"/motive/vehicle/pose"};
  bool pose_stamped_yaw_is_orientation_z_{true};
  double pose_x_offset_{0.0};
  double pose_y_offset_{0.0};
  double pose_yaw_offset_rad_{0.0};
  double pose_position_scale_{1.0};
  bool pose_swap_xy_{false};
  bool pose_invert_x_{false};
  bool pose_invert_y_{false};
  double pose_speed_lpf_alpha_{0.4};
  double pose_max_dt_for_speed_{0.5};

  // ROS interfaces.
  std::string global_path_topic_{"/navigation/global_path"};
  std::string grid_map_topic_{"/grid_map"};
  std::string sparse_path_topic_{"/planner/upper_path_sparse"};
  std::string dense_path_topic_{"/planner/lower_reference_path"};
  std::string goal_reached_topic_{"/planner/goal_reached"};
  std::string path_frame_id_{"map"};

  mutable std::mutex pose_mtx_;
  mutable std::mutex path_mtx_;
  mutable std::mutex grid_mtx_;
  mutable std::mutex previous_solution_mtx_;

  PoseSnapshot pose_;
  bool have_previous_pose_measurement_{false};
  Eigen::Vector2d previous_pose_measurement_{Eigen::Vector2d::Zero()};
  rclcpp::Time previous_pose_time_{0, 0, RCL_ROS_TIME};

  std::vector<Eigen::Vector2d> global_path_;
  std::string global_path_frame_id_{"map"};
  int wp0_index_{0};
  int wp1_index_{1};
  bool goal_reached_{false};
  std::uint64_t planning_revision_{0};
  bool csv_path_loaded_{false};

  std::vector<Eigen::Vector2d> previous_optimal_path_world_;
  std::uint64_t previous_solution_revision_{0};
  bool previous_solution_valid_{false};

  // Initialized once per node run; retained across cycles, missing data and solver failures.
  interval_centering::State interval_centering_state_;

  GridMapSnapshot grid_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr sparse_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr dense_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr preview_debug_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr interval_debug_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr interval_info_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr interval_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr upper_trace_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr upper_timing_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr upper_solver_time_pub_;
  std::uint64_t timing_attempt_seq_{0};
  rclcpp::TimerBase::SharedPtr planner_timer_;
};

}  // namespace imac_ctrl
