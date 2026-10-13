#include <Eigen/Core>
#include <array>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <limits>
#include <sstream>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include "QuadraticProblem.h"
#include "matrix_utils.h"
#include "virtual_control/miqp_solution_validation.hpp"
using namespace imac_ctrl;
using nlohmann::json;
int forced_last=-1;
double clampd(double v,double a,double b){return std::max(a,std::min(v,b));}
double wrapToPi(double x){return atan2(sin(x),cos(x));}
bool finitePoint(const Eigen::Vector2d&p){return p.allFinite();}
struct Replay {
struct PreviewConstraintData {
std::vector<std::vector<Eigen::Vector2d>> pmk,pMk;
std::vector<int> Nck;std::vector<bool> k1_lane_constraint_excluded;
std::vector<Eigen::Vector2d> prk,position_targets;
std::vector<double> psirk;
std::array<double,2> road_segment_heading_rad;
double delta_psi_road_rad,stable_heading_rad,waypoint_bearing_rad,terminal_buffer_turn_rad;
int N_pred,side_boundary_tail_start=-1;
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

double a_max_mps2_=1.0;
double branch_split_continuity_m_=6.75;
bool branch_turn_target_absolute_=true;
bool branch_turn_window_at_split_=true;
bool guard_side_boundary_terminal_loss_=true;
double lambda_position_=1.0;
double lambda_relative_turn_=0.6;
double max_steering_angle_rad_=0.3;
double miqp_big_m_=100.0;
double miqp_binary_diagonal_regularization_=0.0001;
double miqp_diagonal_regularization_=1e-10;
double miqp_feasibility_tolerance_=0.001;
bool normalize_upper_input_cost_=false;
double position_scale_m_=10.0;
double relative_turn_scale_rad_=0.3490658504;
bool skip_front_clipped_k1_lane_constraints_=false;
double terminal_buffer_max_turn_rad_=0.3490658504;
double terminal_position_weight_multiplier_=1000.0;
int turn_preview_steps_=2;
double turn_weight_growth_=2.0;
bool upper_cumulative_increment_model_=true;
double upper_prediction_dt_sec_=0.5;
double upper_r_dpsi_=3.0;
double upper_r_dv_=1.0;
double wheelbase_=2.8;
UpperSolveResult solveUpperMiqp(const PreviewConstraintData&,double) const;
void applyTerminalPositionTargets(PreviewConstraintData&,const Eigen::Vector2d&,const Eigen::Vector2d&,double)const;
};
void Replay::applyTerminalPositionTargets(
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

Replay::UpperSolveResult Replay::solveUpperMiqp(
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
      const int row = 2 * state_step;
      const int column = 2 * input_step;
      // Legacy: input j moves only segment j. Cumulative: a speed/heading increment at j persists on
      // every later segment j..k-1, matching the increment-based steering limit and turn cost.
      const int last_segment =
        upper_cumulative_increment_model_ ? state_step - 1 : input_step;
      for (int segment = input_step; segment <= last_segment; ++segment) {
        const double heading = preview.psirk[static_cast<std::size_t>(segment)];
        G[row][column] += upper_prediction_dt_sec_ * std::cos(heading);
        G[row][column + 1] +=
          -upper_prediction_dt_sec_ * planning_speed_mps * std::sin(heading);
        G[row + 1][column] += upper_prediction_dt_sec_ * std::sin(heading);
        G[row + 1][column + 1] +=
          upper_prediction_dt_sec_ * planning_speed_mps * std::cos(heading);
      }
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
  if (branch_turn_window_at_split_) {
    // Anchor the relative-turn window at the first step where one observed arm continues into
    // two or more arms. Candidates that do not continue the previous section (side remnants of an
    // earlier junction) no longer capture the turn window.
    const auto point_segment = [](const Eigen::Vector2d & p, const Eigen::Vector2d & q0,
        const Eigen::Vector2d & q1) {
        const Eigen::Vector2d d = q1 - q0;
        const double l2 = d.squaredNorm();
        const double u = l2 > 1e-12 ? clampd((p - q0).dot(d) / l2, 0.0, 1.0) : 0.0;
        return (p - (q0 + u * d)).norm();
      };
    const auto segment_distance = [&point_segment](const Eigen::Vector2d & a0,
        const Eigen::Vector2d & a1, const Eigen::Vector2d & b0, const Eigen::Vector2d & b1) {
        return std::min(std::min(point_segment(a0, b0, b1), point_segment(a1, b0, b1)),
          std::min(point_segment(b0, a0, a1), point_segment(b1, a0, a1)));
      };
    int split_step = -1;
    for (int k = 1; k <= horizon && split_step < 0; ++k) {
      const std::size_t sk = static_cast<std::size_t>(k);
      const int previous_count = k == 1 ? 1 : preview.Nck[sk - 1];
      for (int i = 0; i < previous_count && split_step < 0; ++i) {
        const Eigen::Vector2d a0 = k == 1 ? Eigen::Vector2d::Zero() :
          preview.pmk[sk - 1][static_cast<std::size_t>(i)];
        const Eigen::Vector2d a1 = k == 1 ? Eigen::Vector2d::Zero() :
          preview.pMk[sk - 1][static_cast<std::size_t>(i)];
        int continuing = 0;
        for (int j = 0; j < preview.Nck[sk]; ++j) {
          if (segment_distance(a0, a1, preview.pmk[sk][static_cast<std::size_t>(j)],
            preview.pMk[sk][static_cast<std::size_t>(j)]) <= branch_split_continuity_m_)
          {
            ++continuing;
          }
        }
        if (continuing >= 2) {split_step = k;}
      }
    }
    if (split_step >= 1) {result.branch_step = split_step;}
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
      double target =
        static_cast<double>(i + 1) / static_cast<double>(active_count) *
        preview.delta_psi_road_rad;
      if (branch_turn_target_absolute_) {
        // Body-frame heading target (incoming road heading + fraction of the road turn), expressed
        // relative to the reference heading of segment step-1, so a reference that has already
        // turned is not asked to turn again.
        target = wrapToPi(preview.road_segment_heading_rad[0] + target -
          preview.psirk[static_cast<std::size_t>(step - 1)]);
      }
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
  const auto skip_lane = [&](int k, int candidate) {
      if (guard_side_boundary_terminal_loss_ && preview.side_boundary_tail_start >= 2 &&
        k >= preview.side_boundary_tail_start) {return true;}
      return skip_front_clipped_k1_lane_constraints_ && k == 1 && candidate >= 0 &&
             static_cast<std::size_t>(candidate) < preview.k1_lane_constraint_excluded.size() &&
             preview.k1_lane_constraint_excluded[static_cast<std::size_t>(candidate)];
    };
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

      if (skip_lane(k, candidate)) {
        // Keep selector indexing and cost unchanged; these bounds are redundant.
        inequality[row][binary_index] = 1.0;
        inequality_bound[row++] = 1.0;
        inequality[row][binary_index] = 1.0;
        inequality_bound[row++] = 1.0;
        continue;
      }
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

        if(forced_last>=0 && selected.back()!=forced_last)continue;
        Matrix<double> direct_inequality(0.0, direct_row_count, input_count);
        Vector<double> direct_bound(0.0, direct_row_count);
        int direct_row = 0;
        for (int k = 1; k <= horizon; ++k) {
          const std::size_t step = static_cast<std::size_t>(k);
          const std::size_t candidate = static_cast<std::size_t>(selected[step]);
          if (skip_lane(k, static_cast<int>(candidate))) {
            // DAQP ignores satisfied zero rows (0 <= 1).
            direct_bound[direct_row++] = 1.0;
            direct_bound[direct_row++] = 1.0;
            continue;
          }
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

int main(int argc,char**argv){
json records; std::ifstream(argv[1])>>records; json outputs=json::array();
for(auto &rec:records){
auto e=rec["event"];auto t=rec["upper_trace"].get<std::vector<double>>(); int N=e["horizon"]; Replay m; Replay::PreviewConstraintData p;
p.N_pred=N;p.Nck=e["candidate_counts"].get<std::vector<int>>();p.k1_lane_constraint_excluded=e["k1_lane_constraint_excluded"].get<std::vector<bool>>();p.side_boundary_tail_start=e["side_boundary_tail_start"];
double x=e["x"],y=e["y"],yaw=e["yaw_rad"];auto body=[&](double wx,double wy){return Eigen::Vector2d(cos(yaw)*(wx-x)+sin(yaw)*(wy-y),-sin(yaw)*(wx-x)+cos(yaw)*(wy-y));};
for(auto w:e["preview_world"])p.prk.push_back(body(w[0],w[1]));p.psirk.assign(t.begin()+18,t.begin()+18+N+1);
p.pmk.resize(N+1);p.pMk.resize(N+1);for(int k=0;k<=N;k++){p.pmk[k].resize(p.Nck[k],Eigen::Vector2d::Zero());p.pMk[k].resize(p.Nck[k],Eigen::Vector2d::Zero());}
for(auto w:e["intervals_world"]){int k=w[0],j=w[1];p.pmk[k][j]=body(w[2],w[3]);p.pMk[k][j]=body(w[4],w[5]);}
p.road_segment_heading_rad={t[11],t[12]};p.delta_psi_road_rad=t[13];
m.applyTerminalPositionTargets(p,body(-133,56),body(-76,67),3.0);
for(int force:{-1,0,1}){if(force>=0 && int(rec["attempt"])!=2)continue;forced_last=force;
for(double lam:{0.0,0.3,0.6,0.9,1.2,1.3,1.4,1.5,1.6,1.7,1.8,1.9,2.0,3.0,6.0,12.0,30.0,100.0}){
m.lambda_relative_turn_=lam;auto s=m.solveUpperMiqp(p,6.0);json pts=json::array();for(auto b:s.predicted_body)pts.push_back({x+cos(yaw)*b.x()-sin(yaw)*b.y(),y+sin(yaw)*b.x()+cos(yaw)*b.y()});
outputs.push_back({{"epoch",e["t_start"]},{"forced_last",force},{"attempt",rec["attempt"]},{"lambda",lam},{"valid",s.valid},{"position_cost",s.position_cost},{"input_cost",s.input_cost},{"turn_cost",s.turn_cost},{"objective",s.objective},{"branch_step",s.branch_step},{"turn_steps",s.turn_window_steps},{"selected",s.selected_corridors},{"predicted_world",pts},{"model_heading",s.psi_pred_model_rad},{"status",s.status},{"terminal_error",s.terminal_position_error}});
}
}
}
std::ofstream(argv[2])<<outputs.dump(2)<<"\n";
}
