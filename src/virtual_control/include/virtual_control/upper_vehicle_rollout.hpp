#pragma once
#include <array>
#include <string>
#include <vector>

namespace imac_ctrl::upper_vehicle_rollout {
using Point = std::array<double,2>;
struct Settings {
  double dt{.5}, integration_dt{.01}, wheelbase{2.8}, offset{1.4};
  double speed{6.0}, max_acceleration{1.0}, max_angle{.3}, max_rate{.5};
  double initial_steering{0.0}, initial_command{0.0}, tau{.74};
  bool actuator_enabled{true};
  int iterations{12};
  double budget_ms{60.0};
};
struct State {double x{0}, y{0}, yaw{0}, steering{0}, speed{0};};
struct Trajectory {
  bool valid{false};std::string status;
  std::vector<State> states;
  std::vector<int> coarse_indices;
  std::vector<double> commands;
  double max_command_rate{0}, max_effective_rate{0};
};
// Inputs are increments [dv0, dcommand0, dv1, dcommand1, ...]. Commands
// interpolate continuously between knots. All positions refer to the SAME
// point offset metres ahead of the rear axle, as in the lower controller.
Trajectory integrate(const Settings & s, const std::vector<double> & inputs);
struct Corridor {Point normal;double lower{0},upper{0};bool constrained{true};};
struct TurnTarget {int step{0};double target{0},weight{0};};
struct Problem {
  Settings settings;
  std::vector<Point> position_targets;
  std::vector<double> position_weights;
  std::vector<double> reference_headings;
  std::vector<Corridor> corridors;
  // Empty preserves the legacy hard-only problem. Otherwise these nested
  // preferred strips may be violated at a finite cost, in metres.
  std::vector<Corridor> preferred_corridors;
  double soft_linear_weight{1.0}, soft_quadratic_weight{10.0};
  std::vector<TurnTarget> turn_targets;
  std::vector<Point> seed_path;
  std::vector<double> seed_speed_increments;
  double speed_weight{1},heading_increment_weight{3};
  // 0.5 * weight * sum(command[k] - command[k-1])^2, radians squared.
  // The first difference is anchored to settings.initial_command.
  double steering_change_weight{0};
};
struct Solution {
  bool valid{false};std::string status;Trajectory trajectory;
  std::vector<double> inputs;
  double position_cost{0},input_cost{0},turn_cost{0},objective{0};
  double steering_change_cost{0};  // Already included in input_cost.
  double max_corridor_violation{0}, initial_course_error_rad{0};
  double preferred_cost{0}, max_preferred_violation{0};
  std::vector<double> preferred_violations;
  double elapsed_ms{0},solver_ms{0};int iterations{0};
};
// Nonlinear shooting refinement of a selected corridor sequence. Internal
// feasibility-restoration slack is never accepted as a relaxed road bound.
// Preferred-strip slack contributes to cost and does not invalidate a path.
Solution refine(const Problem & p);
}  // namespace imac_ctrl::upper_vehicle_rollout
