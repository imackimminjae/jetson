#ifndef MPCPROBLEM_H
#define MPCPROBLEM_H

#include <string>
#include <vector>
#include <iostream>
#include <cstdio>   
#include <cmath>
#include <algorithm>

#include "Array.hh"
#include "QuadraticProblem.h"

extern "C" {
#include "daqp/include/types.h"
#include "daqp/include/api.h"
#include "daqp/include/constants.h"
#include "daqp/include/utils.h"
}


#ifdef ENABLE_DISCRETIZATION
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_integration.h>
#endif

/**
 * High level interface for generating and solving model predictive control problems
 */
class MPCProblem {

private:
    // prediction horizon in number of steps
    int T;
    // size of the state matrix
    int n;
    // size of the input vector
    int p;
    // size of the output vector
    int q;
    // size of the output constraint vector
    int q2;

    // discrete-time state space matrices
    Matrix<double> A;
    Matrix<double> B;
    Matrix<double> C1;
    Matrix<double> C2;

    // reference vector (size q or q*(T+1))
    Vector<double> r;

    // initial state & control
    Vector<double> init_x;
    Vector<double> init_u;

    QuadraticProblem qp;

    // sampling time
    double ts;

    // options
    bool minimize_du;
    bool terminal_constraint;
    bool output_slack;
    bool control_slack;
    bool control_derivative_slack;
    bool terminal_slack;

    // indicates whether the problem setup is already done
    bool problem_setup;

    // debug output
    bool debug;

    // -----------------------------
    // main variable and decision vars
    // -----------------------------
    Var s;      // main stacked variable
    Var x0;     // initial state variable (decision var)
    Var e;      // error variable (stacked)
    Var u;      // control variable (stacked)
    Var du;     // control derivative variable (optional)

    // slack variables (optional)
    Var eps_y;
    Var eps_u;
    Var eps_du;
    Var eps_t;
    Var eps_v;

// =============================
    // Yield(occupancy) MIQP members
    // =============================
    bool yield_enable_{false};
    double yield_s_stop_{1e9};
    std::vector<int> yield_allowGo_;   // size (T+1), k=0..T

    // big-M and cost weights for yield block
    double yield_M_{1000.0};
    double yield_w_eps_{1e5};  // eps_occ^2 penalty (tune)
    double yield_w_go_{1.0};   // linear reward for y_occ=1 (tune)

    // decision vars
    Var y_occ;     // size (T+1)  binary
    Var eps_occ;   // size (T+1)  continuous slack

    // constraints (for online RHS update)
    std::vector<Constraint> c_y_stop_;   // stopline
    std::vector<Constraint> c_y_allow_;  // y_k <= allowGo_k
    std::vector<Constraint> c_y_mono_;   // y_k <= y_{k+1}
    std::vector<Constraint> c_y_ylb_;    // enable switch (force y=1 when disabled)
    std::vector<Constraint> c_eps_ge0_;  // eps >= 0
    

    // constraints (stored for online updates)
    Constraint init_state;
    Constraint init_control;
    std::vector<Constraint> error;

    std::vector<Constraint> c_u_min, c_u_max, c_du_min, c_du_max, c_y_min, c_y_max;

    // limits
    Vector<double> u_min, u_max, du_min, du_max, y_min, y_max;
    double w_eps_y_{1e3};

    // -------- internal helpers --------
    void setup_variables();
    void print_message(const std::string &msg);

#ifdef ENABLE_DISCRETIZATION
    struct integrate_params {
        const gsl_matrix *A;
        int r;
        int c;
    };
    static double exp_A_t(double t, void *p);
#endif

public:
    // -----------------------------
    // Constructors
    // -----------------------------
    // NOTE: logic params must be known BEFORE setup_variables() runs.
    MPCProblem(int T, int n, int p, int q, int q2, double ts,
            bool minimize_du, bool terminal_constraint, bool output_slack,
            bool control_slack, bool control_derivative_slack,
            bool terminal_slack, bool debug = false,
            double big_M_in = 1000.0, bool yield_enable = false)
    : s(0), x0(0), e(0), u(0), du(0), eps_y(0), eps_u(0), eps_du(0), eps_t(0), eps_v(0), y_occ(0), eps_occ(0)
    {
        this->T = T;
        this->n = n;
        this->p = p;
        this->q = q;
        this->q2 = q2;
        this->ts = ts;
        this->minimize_du = minimize_du;
        this->terminal_constraint = terminal_constraint;
        this->output_slack = output_slack;
        this->control_slack = control_slack;
        this->control_derivative_slack = control_derivative_slack;
        this->terminal_slack = terminal_slack;
        this->debug = debug;
        this->problem_setup = false;
        this->yield_M_ = big_M_in;
        this->yield_enable_ = yield_enable;
        setup_variables();
    }

    // ë§¤ ì£¼ê¸° A,B ì—…ë°ì´íŠ¸ë¥¼ ë…¸ë“œì—ì„œ í˜¸ì¶œí•˜ë¯€ë¡œ í•„ìš”
    bool update_state_space_matrices(const Matrix<double>& Anew,
                                     const Matrix<double>& Bnew);

#ifdef ENABLE_DISCRETIZATION
    static bool discretize_state_space(const Matrix<double> &A,
                                       const Matrix<double> &B,
                                       Matrix<double> &Ad, Matrix<double> &Bd,
                                       double dt);
#endif

    // -----------------------------
    // Setup / state
    // -----------------------------
    void setup_problem();
    bool is_problem_setup();

    // -----------------------------
    // Model matrices / initial conditions / references
    // -----------------------------
    bool set_state_space_matrices(const Matrix<double> &A,
                                  const Matrix<double> &B,
                                  const Matrix<double> &C1,
                                  const Matrix<double> &C2);

    bool set_initial_state(const Vector<double> &x0);
    bool update_initial_state(const Vector<double> &x0);

    bool set_initial_control(const Vector<double> &u0);
    bool update_initial_control(const Vector<double> &u0);

    bool set_reference_vector(const Vector<double> &r);
    bool update_reference_vector(const Vector<double> &r, int index = -1);

    // -----------------------------
    // Cost setters (synced with cpp)
    // -----------------------------
    bool seq_Q_matrix(const Matrix<double> &Q);
    bool seq_q0_vector(const Vector<double> &q0);

    // -----------------------------
    // Constraints
    // -----------------------------
    bool set_u_max(const Vector<double> &u_max);
    bool update_u_max_vector(const Vector<double> &u_max, int index = -1);

    bool set_u_min(const Vector<double> &u_min);
    bool update_u_min_vector(const Vector<double> &u_min, int index = -1);

    bool set_du_max(const Vector<double> &du_max);
    bool update_du_max_vector(const Vector<double> &du_max, int index = -1);

    bool set_du_min(const Vector<double> &du_min);
    bool update_du_min_vector(const Vector<double> &du_min, int index = -1);

    bool set_y_min(const Vector<double> &y_min);
    bool update_y_min_vector(const Vector<double> &y_min, int index = -1);

    bool set_y_max(const Vector<double> &y_max);
    bool update_y_max_vector(const Vector<double> &y_max, int index = -1);

    // output slack weight (used when output_slack == true)
    void set_output_slack_weight(double w) { w_eps_y_ = w; }

    // -----------------------------
    // Solve
    // -----------------------------
    double solve_mpc(Vector<double> &arg);
    bool is_feasible(double result);

    Matrix<double> get_state_evolution(const Matrix<double> &C,
                                       const Vector<double> &solution);

    // -----------------------------
    // Small accessors used by your controller code
    // -----------------------------
    int get_solution_size() const { return s.get_size(); }
    Var get_u_at(int k) const { return u[k]; }

    int u_pos(int k) const;   // k = 0..T
    int u_dim() const;        // returns p

    // Optional raw getters (kept from original)
    Var get_s()  { return s;  }
    Var get_x0() { return x0; }
    Var get_e()  { return e;  }
    Var get_u()  { return u;  }
    Var get_du() { return du; }

};

#endif // MPCPROBLEM_H
