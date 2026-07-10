#ifndef QUADRATICPROBLEM_H
#define QUADRATICPROBLEM_H

#include "Variable.h"
#include "Constraint.h"

#include <map>
#include <string>
#include <vector>

extern "C" {
#include "daqp/include/types.h"
#include "daqp/include/api.h"
#include "daqp/include/constants.h"
#include "daqp/include/utils.h"
}


/**
 * Quadratic optimization problem interface (DAQP backend in .cpp)
 *
 * min  1/2 x^T Q x + q0^T x
 * s.t. Ax >= b
 *      Cx <= d   (internally converted to >=)
 *      Ex = f
 *
 * NOTE:
 * - This header is synchronized with the "ms = n bounds + binary sense" DAQP implementation.
 */
class QuadraticProblem {

protected:
    // main variable x
    Variable *vx;

    // constraints
    Constraint eq;   // equality
    Constraint geq;  // >= (<= converted into >=)

    // cost
    Matrix<double> Q;
    Vector<double> q0;

    bool constraint_phase;
    bool debug;

    // track added variables
    typedef std::map<Variable*, int> VarMap;
    VarMap variables;

    void print_message(const std::string &msg);
    void init_constraint_phase();

    // [NEW] variable bounds for DAQP (ms = n)
    std::vector<double> lbx_;      // size n
    std::vector<double> ubx_;      // size n
    std::vector<int>    sense_bnd_; // size n (e.g., BINARY flag OR-ed)

public:
    explicit QuadraticProblem(bool debug_output = false)
    : vx(new Variable(false, "x")),
      constraint_phase(false),
      debug(debug_output) {}

    virtual ~QuadraticProblem() { delete vx; }

    // ---- variable builders ----
    Variable* vector_variable(int n,
                              const std::string &name = "",
                              const std::string &base_name = "");

    Variable* vector_variable(int n,
                              const Variable *base_variable,
                              const std::string &name = "");

    bool add_variable(Variable *v);
    Var  get_variable(const Variable *v);

    // entering constraint phase
    Var get_main_variable();

    // ---- constraints ----
    bool add_equality_constraint(Constraint &c);
    bool update_equality_constraint(const Constraint &c,
                                    bool update_matrix,
                                    bool update_vector);

    bool add_geq_constraint(Constraint &c);
    bool update_geq_constraint(const Constraint &c,
                               bool update_matrix,
                               bool update_vector);

    bool add_leq_constraint(Constraint &c);
    bool update_leq_constraint(const Constraint &c,
                               bool update_matrix,
                               bool update_vector);

    // ---- cost ----
    bool set_Q_matrix(const Matrix<double> &Q);
    bool set_q0_vector(const Vector<double> &q0);

    // ---- solve ----
    double solve_problem(Vector<double> &arg);

    void write_information();

    // ---- [NEW] DAQP bounds / binary helpers ----
    // idx: position in main variable x (0..n-1)
    void set_var_bounds(int idx, double lb, double ub);

    // sets bounds AND marks as binary (0/1)
    void set_binary_bounds(int idx, double lb, double ub);

    // convenience: mark scalar Var as binary (assumes size==1)
    void set_binary_var(const Var& v);
};

#endif // QUADRATICPROBLEM_H
