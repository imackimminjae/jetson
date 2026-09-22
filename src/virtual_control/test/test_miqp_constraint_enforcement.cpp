#include "virtual_control/miqp_solution_validation.hpp"

#include "QuadraticProblem.h"
#include "matrix_utils.h"

#include <gtest/gtest.h>

#include <cstddef>
#include <cmath>

namespace validation = imac_ctrl::miqp_solution_validation;

TEST(MiqpConstraintEnforcement, RegularizedBinaryBlockHonorsSelectedCorridor)
{
  constexpr int variable_count = 2;
  constexpr int binary_begin = 1;
  constexpr double big_m = 100.0;

  Matrix<double> hessian(0.0, variable_count, variable_count);
  hessian[0][0] = 1.0;
  hessian[1][1] = 1e-4;
  Vector<double> linear(0.0, variable_count);

  Matrix<double> inequality(0.0, 3, variable_count);
  Vector<double> inequality_bound(0.0, 3);
  inequality[0][0] = 1.0;
  inequality[0][1] = big_m;
  inequality_bound[0] = big_m + 2.4;
  inequality[1][0] = -1.0;
  inequality[1][1] = big_m;
  inequality_bound[1] = big_m - 1.5;
  inequality[2][1] = -1.0;
  inequality_bound[2] = -1.0;

  Matrix<double> equality(0.0, 0, variable_count);
  Vector<double> equality_bound(0.0, 0);

  QuadraticProblem solver(false);
  Variable * variable = solver.vector_variable(variable_count, "decision");
  ASSERT_TRUE(solver.add_variable(variable));
  const Var decision = solver.get_variable(variable);
  const Var main = solver.get_main_variable();

  Constraint inequality_constraint(main);
  ASSERT_TRUE(inequality_constraint.set_constraint_variable(decision, inequality));
  ASSERT_TRUE(inequality_constraint.set_known_term(inequality_bound));
  ASSERT_TRUE(solver.add_leq_constraint(inequality_constraint));

  ASSERT_TRUE(solver.set_Q_matrix(hessian));
  ASSERT_TRUE(solver.set_q0_vector(linear));
  solver.set_binary_var(decision[binary_begin]);

  Vector<double> solution;
  ASSERT_TRUE(std::isfinite(solver.solve_problem(solution)));
  ASSERT_EQ(solution.size(), static_cast<unsigned int>(variable_count));

  const auto report = validation::evaluate(
    inequality, inequality_bound, equality, equality_bound, solution,
    static_cast<std::size_t>(binary_begin), 1e-3);
  EXPECT_TRUE(report.valid);
  EXPECT_NEAR(solution[0], 1.5, 1e-3);
  EXPECT_NEAR(solution[1], 1.0, 1e-6);
}

TEST(MiqpConstraintEnforcement, MultipleCorridorChoicesRemainIntegral)
{
  constexpr int variable_count = 3;
  constexpr int binary_begin = 1;
  constexpr double big_m = 100.0;

  Matrix<double> hessian(0.0, variable_count, variable_count);
  hessian[0][0] = 1.0;
  hessian[1][1] = 1e-4;
  hessian[2][2] = 1e-4;
  Vector<double> linear(0.0, variable_count);

  // z0 selects [-2, -1], z1 selects [1, 2].  Their continuous relaxation
  // admits x=0, z0=z1=0.5, while the MIQP optimum must select one interval.
  Matrix<double> inequality(0.0, 5, variable_count);
  Vector<double> inequality_bound(0.0, 5);
  inequality[0][0] = 1.0;
  inequality[0][1] = big_m;
  inequality_bound[0] = big_m - 1.0;
  inequality[1][0] = -1.0;
  inequality[1][1] = big_m;
  inequality_bound[1] = big_m + 2.0;
  inequality[2][0] = 1.0;
  inequality[2][2] = big_m;
  inequality_bound[2] = big_m + 2.0;
  inequality[3][0] = -1.0;
  inequality[3][2] = big_m;
  inequality_bound[3] = big_m - 1.0;
  inequality[4][1] = -1.0;
  inequality[4][2] = -1.0;
  inequality_bound[4] = -1.0;

  Matrix<double> equality(0.0, 0, variable_count);
  Vector<double> equality_bound(0.0, 0);

  QuadraticProblem solver(false);
  Variable * variable = solver.vector_variable(variable_count, "decision");
  ASSERT_TRUE(solver.add_variable(variable));
  const Var decision = solver.get_variable(variable);
  const Var main = solver.get_main_variable();

  Constraint inequality_constraint(main);
  ASSERT_TRUE(inequality_constraint.set_constraint_variable(decision, inequality));
  ASSERT_TRUE(inequality_constraint.set_known_term(inequality_bound));
  ASSERT_TRUE(solver.add_leq_constraint(inequality_constraint));

  ASSERT_TRUE(solver.set_Q_matrix(hessian));
  ASSERT_TRUE(solver.set_q0_vector(linear));
  solver.set_binary_var(decision[binary_begin]);
  solver.set_binary_var(decision[binary_begin + 1]);

  Vector<double> solution;
  ASSERT_TRUE(std::isfinite(solver.solve_problem(solution)));
  ASSERT_EQ(solution.size(), static_cast<unsigned int>(variable_count));

  const auto report = validation::evaluate(
    inequality, inequality_bound, equality, equality_bound, solution,
    static_cast<std::size_t>(binary_begin), 1e-3);
  EXPECT_TRUE(report.valid);
  EXPECT_NEAR(std::abs(solution[0]), 1.0, 1e-3);
  EXPECT_NEAR(solution[1] + solution[2], 1.0, 1e-6);
  EXPECT_NEAR(std::min(solution[1], solution[2]), 0.0, 1e-6);
  EXPECT_NEAR(std::max(solution[1], solution[2]), 1.0, 1e-6);
}

TEST(MiqpConstraintEnforcement, FixedBinaryPolishHonorsCorridorConstraints)
{
  constexpr int variable_count = 3;
  constexpr int binary_begin = 1;
  constexpr double big_m = 100.0;

  Matrix<double> hessian(0.0, variable_count, variable_count);
  hessian[0][0] = 1.0;
  hessian[1][1] = 1e-4;
  hessian[2][2] = 1e-4;
  Vector<double> linear(0.0, variable_count);

  Matrix<double> inequality(0.0, 5, variable_count);
  Vector<double> inequality_bound(0.0, 5);
  inequality[0][0] = 1.0;
  inequality[0][1] = big_m;
  inequality_bound[0] = big_m - 1.0;
  inequality[1][0] = -1.0;
  inequality[1][1] = big_m;
  inequality_bound[1] = big_m + 2.0;
  inequality[2][0] = 1.0;
  inequality[2][2] = big_m;
  inequality_bound[2] = big_m + 2.0;
  inequality[3][0] = -1.0;
  inequality[3][2] = big_m;
  inequality_bound[3] = big_m - 1.0;
  inequality[4][1] = -1.0;
  inequality[4][2] = -1.0;
  inequality_bound[4] = -1.0;

  Matrix<double> equality(0.0, 0, variable_count);
  Vector<double> equality_bound(0.0, 0);

  QuadraticProblem solver(false);
  Variable * variable = solver.vector_variable(variable_count, "decision");
  ASSERT_TRUE(solver.add_variable(variable));
  const Var decision = solver.get_variable(variable);
  const Var main = solver.get_main_variable();

  Constraint inequality_constraint(main);
  ASSERT_TRUE(inequality_constraint.set_constraint_variable(decision, inequality));
  ASSERT_TRUE(inequality_constraint.set_known_term(inequality_bound));
  ASSERT_TRUE(solver.add_leq_constraint(inequality_constraint));
  ASSERT_TRUE(solver.set_Q_matrix(hessian));
  ASSERT_TRUE(solver.set_q0_vector(linear));
  solver.set_var_bounds(binary_begin, 1.0, 1.0);
  solver.set_var_bounds(binary_begin + 1, 0.0, 0.0);

  Vector<double> solution;
  ASSERT_TRUE(std::isfinite(solver.solve_problem(solution)));
  const auto report = validation::evaluate(
    inequality, inequality_bound, equality, equality_bound, solution,
    static_cast<std::size_t>(binary_begin), 1e-3);
  EXPECT_TRUE(report.valid);
  EXPECT_NEAR(solution[0], -1.0, 1e-3);
  EXPECT_NEAR(solution[1], 1.0, 1e-6);
  EXPECT_NEAR(solution[2], 0.0, 1e-6);
}

TEST(MiqpConstraintEnforcement, PostSolveCheckRejectsReportedButInfeasibleResult)
{
  Matrix<double> inequality(0.0, 1, 2);
  Vector<double> inequality_bound(0.0, 1);
  inequality[0][0] = -1.0;
  inequality[0][1] = 100.0;
  inequality_bound[0] = 98.5;

  Matrix<double> equality(0.0, 1, 2);
  Vector<double> equality_bound(1.0, 1);
  equality[0][1] = 1.0;

  Vector<double> infeasible_solution(0.0, 2);
  infeasible_solution[0] = 0.0;
  infeasible_solution[1] = 1.0;

  const auto report = validation::evaluate(
    inequality, inequality_bound, equality, equality_bound, infeasible_solution, 1U, 1e-3);
  EXPECT_FALSE(report.valid);
  EXPECT_NEAR(report.max_inequality_violation, 1.5, 1e-12);
  EXPECT_EQ(report.max_inequality_row, 0U);
  EXPECT_NEAR(report.max_inequality_lhs, 100.0, 1e-12);
  EXPECT_NEAR(report.max_inequality_bound, 98.5, 1e-12);
  EXPECT_NEAR(report.max_equality_violation, 0.0, 1e-12);
  EXPECT_NEAR(report.max_binary_violation, 0.0, 1e-12);
}
