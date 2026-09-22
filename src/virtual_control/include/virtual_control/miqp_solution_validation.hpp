#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace imac_ctrl::miqp_solution_validation
{

struct FeasibilityReport
{
  bool finite{false};
  bool valid{false};
  double max_inequality_violation{std::numeric_limits<double>::infinity()};
  std::size_t max_inequality_row{std::numeric_limits<std::size_t>::max()};
  double max_inequality_lhs{std::numeric_limits<double>::quiet_NaN()};
  double max_inequality_bound{std::numeric_limits<double>::quiet_NaN()};
  double max_equality_violation{std::numeric_limits<double>::infinity()};
  double max_binary_violation{std::numeric_limits<double>::infinity()};
};

template<typename MatrixT, typename VectorT>
FeasibilityReport evaluate(
  const MatrixT & inequality,
  const VectorT & inequality_bound,
  const MatrixT & equality,
  const VectorT & equality_bound,
  const VectorT & solution,
  std::size_t binary_begin,
  double tolerance)
{
  FeasibilityReport report;
  const std::size_t variable_count = static_cast<std::size_t>(inequality.ncols());
  if (!std::isfinite(tolerance) || tolerance < 0.0 ||
    static_cast<std::size_t>(equality.ncols()) != variable_count ||
    static_cast<std::size_t>(inequality.nrows()) != inequality_bound.size() ||
    static_cast<std::size_t>(equality.nrows()) != equality_bound.size() ||
    solution.size() < variable_count || binary_begin > variable_count)
  {
    return report;
  }

  for (std::size_t column = 0; column < variable_count; ++column) {
    if (!std::isfinite(solution[static_cast<unsigned int>(column)])) {
      return report;
    }
  }

  report.finite = true;
  report.max_inequality_violation = 0.0;
  for (std::size_t row = 0;
    row < static_cast<std::size_t>(inequality.nrows()); ++row)
  {
    double lhs = 0.0;
    for (std::size_t column = 0; column < variable_count; ++column) {
      lhs += inequality[static_cast<unsigned int>(row)][static_cast<unsigned int>(column)] *
        solution[static_cast<unsigned int>(column)];
    }
    if (!std::isfinite(lhs)) {
      report.finite = false;
      return report;
    }
    const double bound = inequality_bound[static_cast<unsigned int>(row)];
    const double violation = lhs - bound;
    if (violation > report.max_inequality_violation) {
      report.max_inequality_violation = violation;
      report.max_inequality_row = row;
      report.max_inequality_lhs = lhs;
      report.max_inequality_bound = bound;
    }
  }

  report.max_equality_violation = 0.0;
  for (std::size_t row = 0; row < static_cast<std::size_t>(equality.nrows()); ++row) {
    double lhs = 0.0;
    for (std::size_t column = 0; column < variable_count; ++column) {
      lhs += equality[static_cast<unsigned int>(row)][static_cast<unsigned int>(column)] *
        solution[static_cast<unsigned int>(column)];
    }
    if (!std::isfinite(lhs)) {
      report.finite = false;
      return report;
    }
    report.max_equality_violation = std::max(
      report.max_equality_violation,
      std::abs(lhs - equality_bound[static_cast<unsigned int>(row)]));
  }

  report.max_binary_violation = 0.0;
  for (std::size_t column = binary_begin; column < variable_count; ++column) {
    const double value = solution[static_cast<unsigned int>(column)];
    report.max_binary_violation = std::max(
      report.max_binary_violation,
      std::min(std::abs(value), std::abs(value - 1.0)));
  }

  report.valid = report.finite &&
    report.max_inequality_violation <= tolerance &&
    report.max_equality_violation <= tolerance &&
    report.max_binary_violation <= tolerance;
  return report;
}

}  // namespace imac_ctrl::miqp_solution_validation
