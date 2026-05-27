#pragma once

#include "QuadraticProblem.h"
#include "matrix_utils.h"

#include <Eigen/Dense>

#include <optional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <mutex>

#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

namespace imac_ctrl
{

struct SdMapEdge
{
  int id{0};
  std::vector<Eigen::Vector2d> centerline;
  std::vector<int> successors;
  double lane_width{0.30};
};

struct SdMap
{
  std::vector<SdMapEdge> edges;
  std::vector<int> route_edge_ids;
  std::optional<int> goal_edge_id;
  std::optional<Eigen::Vector2d> goal_xy;
};

struct SdMapUpperPlannerConfig
{
  double lookahead_min{1.0};
  double lookahead_max{2.5};
  double ds_nominal{0.20};
  double path_resample_ds{0.05};
  int max_branch_depth{8};

  double goal_weight{20.0};
  double hysteresis_weight{8.0};
  double short_path_weight{10.0};

  double upper_qx_x{1.2};       // 0.3 * 4
  double upper_qx_y{0.3};       // 0.3 * 1
  double upper_ru_dv{0.3};
  double upper_ru_dpsi{0.3};

  double lower_wp0_s{0.4};
  double lower_wp1_s{0.8};
  double upper_replan_margin_s{0.4};
  double goal_reached_tol{0.20};
  double goal_zone_tol{0.35};

  // Generic SD-map planner tuning defaults.
  double min_required_lookahead{0.30};
  double heading_weight{1.5};
  double lane_width_default{0.30};
  double corridor_margin{0.03};
  double min_half_width{0.05};
  double parallel_interval_half_len{0.20};
  double max_interval_half_len{0.60};
  double parallel_eps{1e-3};
  double binary_regularization{1e-6};
};

struct SdMapUpperPreviewData
{
  std::vector<std::vector<Eigen::Vector2d>> pmk;
  std::vector<std::vector<Eigen::Vector2d>> pMk;
  std::vector<int> Nck;
  std::vector<double> mk;
  std::vector<double> nk;
  std::vector<double> ck;
  std::vector<Eigen::Vector2d> prk;
  std::vector<double> psirk_rad;
  int N_pred{0};
  std::vector<std::vector<int>> branch_info;
  std::vector<double> branch_cost;
  double ds_eff{0.0};
  std::vector<int> candidate_terminal_edge_ids;
};

struct SdMapUpperMiqpDebug
{
  int selected_branch_idx{-1};          // 0-based
  std::vector<int> selected_interval;   // 0-based within each step
  std::vector<int> y_branch;
  std::vector<double> z;
  std::vector<double> y;
};

struct SdMapUpperPlanResult
{
  bool valid{false};
  std::vector<Eigen::Vector2d> dense_world;
  int selected_branch_idx{-1};         // 0-based
  int selected_terminal_edge_id{-1};
  double ds_eff{0.0};
  double lookahead{0.0};
  SdMapUpperPreviewData preview;
  SdMapUpperMiqpDebug miqp_debug;
};

struct SdMapLowerGuide
{
  Eigen::Vector2d wp0_body{Eigen::Vector2d::Zero()};
  Eigen::Vector2d wp1_body{Eigen::Vector2d::Zero()};
  double s0{0.0};
  double s1{0.0};
  double remaining_length{0.0};
};

class SdMapUpperPlanner
{
public:
  explicit SdMapUpperPlanner(const SdMapUpperPlannerConfig & config = {});

  void setConfig(const SdMapUpperPlannerConfig & config);
  const SdMapUpperPlannerConfig & config() const;

  void setMap(const SdMap & map);
  const SdMap & map() const;
  bool hasMap() const;

  Eigen::Vector2d goalWorld() const;
  bool goalReached(const Eigen::Vector2d & ego_pos_world, double tol) const;

  SdMapUpperPreviewData makePreviewConstraints(
    const Eigen::Vector2d & ego_pos_world,
    double ego_yaw_rad,
    int prev_terminal_edge_id) const;

  SdMapUpperPlanResult plan(
    const Eigen::Vector2d & ego_pos_world,
    double ego_yaw_rad,
    double vr,
    double delta_M_deg,
    double wheelbase,
    double a_M,
    double big_m,
    int prev_terminal_edge_id,
    double v1 = -1.0) const;

  static std::vector<Eigen::Vector2d> extractAheadWorldPath(
    const std::vector<Eigen::Vector2d> & world_path,
    const Eigen::Vector2d & ego_pos_world);

  SdMapLowerGuide makeLowerGuidesFromUpper(
    const std::vector<Eigen::Vector2d> & upper_dense_world,
    const Eigen::Vector2d & ego_pos_world,
    double ego_yaw_rad,
    double lower_wp_min_separation) const;

  static double polylineLength(const std::vector<Eigen::Vector2d> & pts);

private:
  struct Projection
  {
    Eigen::Vector2d point{Eigen::Vector2d::Zero()};
    double dist{0.0};
    double s_on_edge{0.0};
    Eigen::Vector2d tangent{Eigen::Vector2d::UnitX()};
  };

  struct CandidatePath
  {
    std::vector<int> path_idx;
    std::vector<Eigen::Vector2d> path_world;
    double length{0.0};
    int terminal_edge_id{-1};
    double lane_width{0.30};
  };

  static double clampd(double v, double lo, double hi);
  static std::vector<double> buildArclength(const std::vector<Eigen::Vector2d> & pts);
  static std::vector<Eigen::Vector2d> dedupPolyline(const std::vector<Eigen::Vector2d> & pts);
  static Eigen::Vector2d samplePolylineByArclength(
    const std::vector<Eigen::Vector2d> & pts,
    double s_query);
  static Eigen::Vector2d samplePolylineTangent(
    const std::vector<Eigen::Vector2d> & pts,
    double s_query);
  static std::vector<Eigen::Vector2d> resamplePolyline(
    const std::vector<Eigen::Vector2d> & pts,
    double ds);
  static std::vector<Eigen::Vector2d> trimPolylineFromS(
    const std::vector<Eigen::Vector2d> & pts,
    double s0);
  static Projection projectPointToPolyline(
    const std::vector<Eigen::Vector2d> & pts,
    const Eigen::Vector2d & point_world);
  static std::vector<Eigen::Vector2d> worldToBody(
    const std::vector<Eigen::Vector2d> & pts_world,
    const Eigen::Vector2d & ego_pos_world,
    double ego_yaw_rad);
  static Eigen::Vector2d worldToBodyPoint(
    const Eigen::Vector2d & p_world,
    const Eigen::Vector2d & ego_pos_world,
    double ego_yaw_rad);
  static Eigen::Vector2d bodyToWorldPoint(
    const Eigen::Vector2d & p_body,
    const Eigen::Vector2d & ego_pos_world,
    double ego_yaw_rad);
  static Eigen::Vector2d rotateWorldToBodyVector(
    const Eigen::Vector2d & v_world,
    double ego_yaw_rad);
  static std::vector<double> computeHeadingRad(const std::vector<Eigen::Vector2d> & body_pts);
  static int findClosestPointIndex(
    const std::vector<Eigen::Vector2d> & pts_world,
    const Eigen::Vector2d & query_world);

  int edgeIdToIndex(int edge_id) const;
  int findGoalEdgeIndex() const;
  int findCurrentEdge(
    const Eigen::Vector2d & ego_pos_world,
    double ego_yaw_rad,
    Projection * proj_out) const;
  bool searchEdgePath(int i_start, int i_goal, std::vector<int> * path_idx) const;
  bool buildRouteEdgePath(int i_start, std::vector<int> * path_idx) const;
  void enumerateCandidatePaths(
    int i_start,
    int max_depth,
    std::vector<std::vector<int>> * paths_out) const;
  std::vector<Eigen::Vector2d> concatenateEdgePath(
    const std::vector<int> & path_idx,
    int i_start,
    double s_on_start_edge) const;
  CandidatePath buildCandidatePathFromEdgePath(
    const std::vector<int> & idx_seq,
    int i_start,
    double s_on_start_edge) const;
  std::vector<CandidatePath> buildCandidatePaths(
    int i_start,
    double s_on_start_edge) const;

  std::pair<std::vector<Eigen::Vector2d>, SdMapUpperMiqpDebug> solveUpperMiqp(
    const SdMapUpperPreviewData & preview,
    double vr,
    double Ts,
    double a_M,
    double phi_M,
    double big_m,
    double v1) const;

  SdMapUpperPlannerConfig config_;
  SdMap map_;
};

class UpperPlannerNode : public rclcpp::Node
{
public:
  explicit UpperPlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~UpperPlannerNode() override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace imac_ctrl
