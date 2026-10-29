#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace imac_ctrl::comparison
{
using Point = Eigen::Vector2d;
struct Pose {Point p{0, 0}; double yaw{0};};
Point rotate(const Point & p, double yaw);
struct Vehicle
{
  double wheelbase{2.8}, length{4.7}, width{1.8}, reference_to_rear{1.4};
  double max_steer{0.30}, max_steer_rate{1.0471975512};
};
enum class Cell {Free, Occupied, Unknown, Outside};
struct Grid
{
  int width{0}, height{0}, threshold{0};
  double resolution{0}, origin_yaw{0};
  Point origin{0, 0};             // grid origin in grid header frame
  Pose frame_world;              // header frame pose at acquisition time
  bool positive_is_drivable{false};
  std::vector<int8_t> data;
  std::vector<uint32_t> blocked_prefix;
  // Rebuild after editing data. Planners rebuild once per input snapshot.
  void buildIndex();
  bool rectangleKnownFree(const Point & minimum, const Point & maximum) const;
  bool valid() const;
  Point toGrid(const Point & world) const;
  Point cellCenter(int x, int y) const;
  Cell cell(const Point & world) const;
};
struct CurveSample {Point p, tangent; double curvature;};
// Open cubic Hermite curve. Arc lookup is numerically integrated at <= 5 cm.
// "exact" is a source provenance declaration, never inferred from spacing/bias.
class ReferenceCurve
{
public:
  explicit ReferenceCurve(const std::vector<Point> & points);
  CurveSample at(double s) const;
  double project(const Point & p, double lo, double hi) const;
  double length() const {return arc_.empty() ? 0.0 : arc_.back();}
private:
  CurveSample evaluate(double u) const;
  std::vector<Point> points_, tangents_;
  std::vector<double> arc_, parameters_;
};
struct Settings
{
  double period{0.2}, dt{0.5}, prediction_speed{6.0};
  int steps{7}, sqp_iterations{6};
  double lower_dt{0.1}, min_spacing{0.05}, min_length{1.0};
  // 0.9 m half width + 1.1 m extra = existing 2 m lateral allowance.
  double extra_margin{1.1}, corridor_scan{10.0};
  bool allow_initial_unobserved_footprint{true};
  double projection_back{1.0}, projection_slack{2.0};
  double ring_step{1.0}, angular_step{0.01}, arc_half_angle{1.50};
  double attraction{1.0}, repulsion{1.0}, attraction_power{1.0}, repulsion_power{1.0};
  double valley_gradient{0.12}, max_connection{3.0};
  double q_contour{10.0}, q_lag{20.0}, reward_progress{2.0};
  double r_steer{0.3}, r_steer_change{15.0}, max_virtual_speed{9.0};
  double steer_trust{0.12}, progress_trust{2.0};
};
struct Input
{
  Pose vehicle;
  double actual_speed{0}, time{0}; // monotonic planning time in seconds
  std::optional<double> applied_steer;
  Grid grid;
  std::vector<Point> route;
  Vehicle geometry;
  Settings settings;
  std::optional<double> progress; // externally supplied mission seed, if available
};
struct Output
{
  bool success{false}, goal_reached{false};
  std::string reason;
  std::vector<Point> sparse, path;
  std::vector<double> s, steering, virtual_speed, progress;
  double preprocessing_ms{0}, planning_ms{0}, postprocessing_ms{0}, total_ms{0};
  double projected_s{0}, warm_shift_sec{0}, max_constraint_violation{0};
  int qp_status{0}, iterations{0}, candidates{0};
};
// Every connector is sampled, including between rings / nonlinear MPC knots.
bool footprintFree(const Input & in, const Pose & pose);
bool connectionFree(const Input & in, const Pose & a, const Pose & b);
// A single contiguous interval containing the reference; no envelope of branches.
std::optional<Point> corridor(const Input & in, const CurveSample & sample);
bool densify(const std::vector<Point> & sparse, double spacing,
  std::vector<Point> & dense, std::vector<double> & s);
class NvpPlanner
{
public:
  Output plan(const Input & in);
  void reset();
private:
  std::optional<double> previous_s_;
  double previous_time_{0};
  std::vector<Point> previous_route_;
};
class MpccPlanner
{
public:
  Output plan(const Input & in);
  void reset();
private:
  std::optional<double> previous_s_;
  double previous_time_{0};
  std::vector<double> previous_delta_, previous_virtual_, previous_times_;
  std::vector<Point> previous_route_;
};
}  // namespace imac_ctrl::comparison
