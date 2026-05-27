#include "MPCProblem.h"
#include <cstdio>
#ifdef ENABLE_DISCRETIZATION
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_integration.h>
#endif
#include <limits>
#include <cmath>
#include <algorithm>

void MPCProblem::print_message(const std::string &msg) {
    if (debug)
        std::cerr << msg;
}

#ifdef ENABLE_DISCRETIZATION

double MPCProblem::exp_A_t(double t, void *p) {
    double v;
    struct integrate_params *params = (struct integrate_params *)p;
    gsl_matrix *M = gsl_matrix_alloc(params->A->size1, params->A->size2);
    gsl_matrix *exp_M = gsl_matrix_alloc(params->A->size1, params->A->size2);

    gsl_matrix_memcpy(M, params->A);
    gsl_matrix_scale(M, t);
    gsl_linalg_exponential_ss(M, exp_M, 0);
    v = gsl_matrix_get(exp_M, params->r, params->c);

    gsl_matrix_free(exp_M);
    gsl_matrix_free(M);
    return v;
}

bool MPCProblem::discretize_state_space(const Matrix<double> &A,
                                        const Matrix<double> &B,
                                        Matrix<double> &Ad, Matrix<double> &Bd,
                                        double dt) {

    gsl_integration_workspace *w = gsl_integration_workspace_alloc (10000);
    gsl_function f;
    double iv, err;
    gsl_matrix *gsl_A = to_gsl(A);
    gsl_matrix *gsl_scaled_A = to_gsl(A);
    gsl_matrix *gsl_Ad = gsl_matrix_alloc(gsl_A->size1, gsl_A->size2);
    gsl_matrix *gsl_Ai = gsl_matrix_alloc(gsl_A->size1, gsl_A->size2);
    Matrix<double> Ai;

    gsl_matrix_scale(gsl_scaled_A, dt);
    gsl_linalg_exponential_ss(gsl_scaled_A, gsl_Ad, 0);
    Ad = from_gsl(gsl_Ad);

    f.function = exp_A_t;
    struct integrate_params p;
    f.params = &p;
    p.A = gsl_A;
    for (int r = 0; r < (int)gsl_A->size1; r++) {
        for (int c = 0; c < (int)gsl_A->size2; c++) {
            p.r = r;
            p.c = c;
            gsl_integration_qags(&f, 0, dt, 0, 1e-7, 10000, w, &iv, &err);
            gsl_matrix_set(gsl_Ai, r, c, iv);
        }
    }
    Ai = from_gsl(gsl_Ai);
    Bd = multiply(Ai, B);

    gsl_matrix_free(gsl_Ai);
    gsl_matrix_free(gsl_Ad);
    gsl_matrix_free(gsl_scaled_A);
    gsl_matrix_free(gsl_A);
    gsl_integration_workspace_free(w);

    return true;
}

#endif

void MPCProblem::setup_variables() {
    Variable *x0 = qp.vector_variable(n, "x0");
    qp.add_variable(x0);
    this->x0 = Var(x0);

    Variable *e_k = qp.vector_variable(q, "e_k");
    Variable *e = qp.vector_variable(T+1, e_k, "e");
    qp.add_variable(e);
    delete e_k;
    this->e = Var(e);

    Variable *u_k = qp.vector_variable(p, "u_k");
    Variable *u = qp.vector_variable(T+1, u_k, "u");
    qp.add_variable(u);
    delete u_k;
    this->u = Var(u);

    Variable *epsvv = qp.vector_variable(T+1, "eps_v");
    qp.add_variable(epsvv);
    this->eps_v = Var(epsvv);

    if (minimize_du) {
        Variable *du_k = qp.vector_variable(p, "du_k");
        Variable *du = qp.vector_variable(T+1, du_k, "du");
        qp.add_variable(du);
        delete du_k;
        this->du = Var(du);
    }

    // slack variables (optional)
    if (output_slack) {
        Variable *eps_yv = qp.vector_variable(1, "eps_y");
        qp.add_variable(eps_yv);
        this->eps_y = Var(eps_yv);
    }
    if (control_slack) {
        Variable *eps_uv = qp.vector_variable(1, "eps_u");
        qp.add_variable(eps_uv);
        this->eps_u = Var(eps_uv);
    }
    if (control_derivative_slack) {
        Variable *eps_duv = qp.vector_variable(1, "eps_du");
        qp.add_variable(eps_duv);
        this->eps_du = Var(eps_duv);
    }
    if (terminal_constraint && terminal_slack) {
        Variable *eps_tv = qp.vector_variable(1, "eps_t");
        qp.add_variable(eps_tv);
        this->eps_t = Var(eps_tv);
    }


    // ===== Yield(occupancy) variables =====
    if (yield_enable_) {
        // y_occ: release(1)/hold(0) binary
        Variable *yoccv = qp.vector_variable(T+1, "y_occ");
        qp.add_variable(yoccv);
        this->y_occ = Var(yoccv);

        // eps_occ: slack for stopline
        Variable *epsoccv = qp.vector_variable(T+1, "eps_occ");
        qp.add_variable(epsoccv);
        this->eps_occ = Var(epsoccv);

        // k=0ì€ ì‚¬ìš© ì•ˆ í•˜ë¯€ë¡œ ê³ ì •(ì•ˆì •)
        qp.set_var_bounds(y_occ[0].get_position(), 1.0, 1.0);   // y0=1
        qp.set_var_bounds(eps_occ[0].get_position(), 0.0, 0.0); // eps0=0

        // k=1..T
        for (int k = 1; k <= T; ++k) {
            qp.set_binary_var(y_occ[k]); // í•­ìƒ binary
            // eps_occëŠ” ì—°ì†ë³€ìˆ˜, boundsëŠ” ì•ˆ ê±´ë‹¤(ì•„ëž˜ì—ì„œ eps>=0 ì œì•½ìœ¼ë¡œ ì²˜ë¦¬)
        }

        // allowGo ê¸°ë³¸ê°’
        if ((int)yield_allowGo_.size() != (T+1)) yield_allowGo_.assign(T+1, 1);
    }

    // ë°˜ë“œì‹œ í•­ìƒ ë§ˆì§€ë§‰ì— main var íšë“
    s = qp.get_main_variable();
}

void MPCProblem::setup_problem() {
    // get the needed powers of the A matrix
    std::vector<Matrix<double> > Ak = get_powers(A, T);

    // precompute matrix multiplications to save time
    std::vector<Matrix<double> > C1Ak, C1AkB, nC1Ak, nC1AkB;
    std::vector<Matrix<double> > C2Ak, C2AkB, nC2Ak, nC2AkB;
    for (int i = 0; i <= T; i++) {
        C1Ak.push_back(multiply(C1, Ak[i]));
        nC1Ak.push_back(-C1Ak[C1Ak.size()-1]);
        C1AkB.push_back(multiply(C1Ak[C1Ak.size()-1], B));
        nC1AkB.push_back(-C1AkB[C1AkB.size()-1]);
        if (y_max.size() != 0 || y_min.size() != 0) {
            C2Ak.push_back(multiply(C2, Ak[i]));
            nC2Ak.push_back(-C2Ak[C2Ak.size() - 1]);
            C2AkB.push_back(multiply(C2Ak[C2Ak.size() - 1], B));
            nC2AkB.push_back(-C2AkB[C2AkB.size() - 1]);
        }
    }

    // setup some commonly used matrices and vectors
    Matrix<double> Ip = identity(p);
    Matrix<double> nIp = -Ip;
    Matrix<double> nIpTs = nIp * ts;
    Vector<double> zp = zero_vector(p);
    Matrix<double> Iq = identity(q);
    Vector<double> zq = zero_vector(q);
    Matrix<double> y_slack_M(1, q2, 1);
    Matrix<double> n_y_slack_M = -y_slack_M;
    Matrix<double> u_slack_M(1, p, 1);
    Matrix<double> n_u_slack_M = -u_slack_M;
    Matrix<double> t_slack_M(1, q, 1);
    Matrix<double> n_t_slack_M = -t_slack_M;
    Vector<double> ref(q);

    // add initial state constraint: x0 = x*
    init_state.set_variable(s);
    init_state.set_constraint_variable(x0, identity(n));
    init_state.set_known_term(init_x);
    qp.add_equality_constraint(init_state);

    // add initial control constraint: u0 = u*
    init_control.set_variable(s);
    init_control.set_constraint_variable(u[0], Ip);
    init_control.set_known_term(init_u);
    qp.add_equality_constraint(init_control);

    // error constraints e_k = C1 * A^k * x0 + ...
    for (int k = 0; k <= T; k++) {
        Constraint error(s);
        error.set_constraint_variable(e[k], Iq);
        error.set_constraint_variable(x0, nC1Ak[k]);
        for (int i = 0; i <= k - 1; i++)
            error.set_constraint_variable(u[i], nC1AkB[k-i-1]);
        if (r.size() == q) {
            error.set_known_term(-r);
        }
        else {
            for (int i = 0; i < q; i++)
                ref[i] = -r[k * q + i];
            error.set_known_term(ref);
        }
        qp.add_equality_constraint(error);
        this->error.push_back(error);
    }

    // constraint u[k+1] = u[k] + du[k] * ts
    if (minimize_du) {
        for (int k = 0; k <= T - 1; k++) {
            Constraint control(s);
            control.set_constraint_variable(u[k + 1], Ip);
            control.set_constraint_variable(u[k], nIp);
            control.set_constraint_variable(du[k], nIpTs);
            control.set_known_term(zp);
            qp.add_equality_constraint(control);
        }
    }

    // terminal condition constraint: e_T = 0
    if (terminal_constraint) {
        if (!terminal_slack) {
            Constraint terminal(s);
            terminal.set_constraint_variable(e[T], Iq);
            terminal.set_known_term(zero_vector(q));
            qp.add_equality_constraint(terminal);
        }
        else {
            Constraint terminal1(s);
            terminal1.set_constraint_variable(e[T], Iq);
            terminal1.set_constraint_variable(eps_t, n_t_slack_M);
            terminal1.set_known_term(zero_vector(q));
            qp.add_leq_constraint(terminal1);

            Constraint terminal2(s);
            terminal2.set_constraint_variable(e[T], Iq);
            terminal2.set_constraint_variable(eps_t, t_slack_M);
            terminal2.set_known_term(zero_vector(q));
            qp.add_geq_constraint(terminal2);
        }
    }

    // control constraints
    if (u_min.size() != 0) {
        for (int k = 0; k <= T - 1; k++) {
            Constraint u_min_c(s);
            u_min_c.set_constraint_variable(u[k + 1], Ip);
            if (control_slack)
                u_min_c.set_constraint_variable(eps_u, u_slack_M);
            u_min_c.set_known_term(u_min);
            qp.add_geq_constraint(u_min_c);
            c_u_min.push_back(u_min_c);
        }
    }
    if (u_max.size() != 0) {
        for (int k = 0; k <= T - 1; k++) {
            Constraint u_max_c(s);
            u_max_c.set_constraint_variable(u[k + 1], Ip);
            if (control_slack)
                u_max_c.set_constraint_variable(eps_u, n_u_slack_M);
            u_max_c.set_known_term(u_max);
            qp.add_leq_constraint(u_max_c);
            c_u_max.push_back(u_max_c);
        }
    }

    // du constraints
    if (du_min.size() != 0) {
        if (!minimize_du) {
            Constraint du_min_c(s);
            du_min_c.set_constraint_variable(u[0], Ip);
            if (control_derivative_slack)
                du_min_c.set_constraint_variable(eps_du, u_slack_M);
            du_min_c.set_known_term(du_min + init_u);
            qp.add_geq_constraint(du_min_c);
            c_du_min.push_back(du_min_c);
        }
        for (int k = 0; k <= T - 1; k++) {
            Constraint du_min_c(s);
            du_min_c.set_constraint_variable(u[k + 1], Ip);
            du_min_c.set_constraint_variable(u[k], nIp);
            if (control_derivative_slack)
                du_min_c.set_constraint_variable(eps_du, u_slack_M);
            du_min_c.set_known_term(du_min);
            qp.add_geq_constraint(du_min_c);
            c_du_min.push_back(du_min_c);
        }
    }
    if (du_max.size() != 0) {
        if (!minimize_du) {
            Constraint du_max_c(s);
            du_max_c.set_constraint_variable(u[0], Ip);
            if (control_derivative_slack)
                du_max_c.set_constraint_variable(eps_du, n_u_slack_M);
            du_max_c.set_known_term(du_max + init_u);
            qp.add_leq_constraint(du_max_c);
            c_du_max.push_back(du_max_c);
        }
        for (int k = 0; k <= T - 1; k++) {
            Constraint du_max_c(s);
            du_max_c.set_constraint_variable(u[k + 1], Ip);
            du_max_c.set_constraint_variable(u[k], nIp);
            if (control_derivative_slack)
                du_max_c.set_constraint_variable(eps_du, n_u_slack_M);
            du_max_c.set_known_term(du_max);
            qp.add_leq_constraint(du_max_c);
            c_du_max.push_back(du_max_c);
        }
    }

    // output constraints (existing)
    if (y_max.size() != 0) {
        for (int k = 0; k <= T - 1; k++) {
            Constraint y_max_c(s);
            y_max_c.set_constraint_variable(x0, C2Ak[k+1]);
            for (int i = 0; i <= k; i++) {
                y_max_c.set_constraint_variable(u[i], C2AkB[k-i]);
            }
            if (output_slack) {
                y_max_c.set_constraint_variable(eps_y, n_y_slack_M);
            }
            y_max_c.set_known_term(y_max);
            qp.add_leq_constraint(y_max_c);
            c_y_max.push_back(y_max_c);
        }
    }
    if (y_min.size() != 0) {
        for (int k = 0; k <= T - 1; k++) {
            Constraint y_min_c(s);
            y_min_c.set_constraint_variable(x0, C2Ak[k+1]);
            for (int i = 0; i <= k; i++) {
                y_min_c.set_constraint_variable(u[i], C2AkB[k-i]);
            }
            if (output_slack){
                y_min_c.set_constraint_variable(eps_y, y_slack_M);
            }
            y_min_c.set_known_term(y_min);
            qp.add_geq_constraint(y_min_c);
            c_y_min.push_back(y_min_c);
        }
    }

    // slack vars >= 0
    if (control_slack) {
        Constraint u_slack(s);
        u_slack.set_constraint_variable(eps_u, 1);
        u_slack.set_known_term(0);
        qp.add_geq_constraint(u_slack);
    }
    if (control_derivative_slack) {
        Constraint du_slack(s);
        du_slack.set_constraint_variable(eps_du, 1);
        du_slack.set_known_term(0);
        qp.add_geq_constraint(du_slack);
    }
    if (output_slack) {
        Constraint out_slack(s);
        out_slack.set_constraint_variable(eps_y, 1);
        out_slack.set_known_term(0);
        qp.add_geq_constraint(out_slack);
    }

    if (yield_enable_) {
        if (n < 1) {
            print_message("[MPC] yield MIQP requires state index 0 = s\n");
        } else {
            const double M = yield_M_;

            // A^j B, j=0..T-1
            std::vector<Matrix<double>> AkB;
            AkB.reserve(T);
            for (int j = 0; j <= T-1; ++j) {
                AkB.push_back(multiply(Ak[j], B)); // (n x p)
            }

            auto row_of = [&](const Matrix<double>& Mx, int r)->Matrix<double> {
                Matrix<double> out; out.resize(0.0, 1, Mx.ncols());
                for (int j=0; j<Mx.ncols(); ++j) out[0][j] = Mx[r][j];
                return out; // 1 x n
            };
            auto row_of_np = [&](const Matrix<double>& Mx, int r)->Matrix<double> {
                Matrix<double> out; out.resize(0.0, 1, Mx.ncols());
                for (int j=0; j<Mx.ncols(); ++j) out[0][j] = Mx[r][j];
                return out; // 1 x p
            };

            c_y_stop_.clear();
            c_y_allow_.clear();
            c_y_mono_.clear();
            c_y_ylb_.clear();
            c_eps_ge0_.clear();

            // k=1..T
            for (int k = 1; k <= T; ++k) {
                // s_k = row(A^k, s)*x0 + sum row(A^{k-1-i}B, s)*u[i]
                Matrix<double> s_x0 = row_of(Ak[k], 0);

                // (1) stopline: s_k - M*y_k - eps_k <= s_stop
                Constraint stop(s);
                stop.set_constraint_variable(x0, s_x0);
                for (int i=0; i<=k-1; ++i) {
                    int pow = (k-1-i);
                    Matrix<double> s_ui = row_of_np(AkB[pow], 0);
                    stop.set_constraint_variable(u[i], s_ui);
                }
                stop.set_constraint_variable(y_occ[k], -M);
                stop.set_constraint_variable(eps_occ[k], -1.0);
                stop.set_known_term(yield_s_stop_);
                qp.add_leq_constraint(stop);
                c_y_stop_.push_back(stop);

                // (2) allowGo: y_k <= allowGo_k
                Constraint allow(s);
                allow.set_constraint_variable(y_occ[k], 1.0);
                double rhs_allow = 1.0;
                if ((int)yield_allowGo_.size() == (T+1)) rhs_allow = (double)yield_allowGo_[k];
                allow.set_known_term(rhs_allow);
                qp.add_leq_constraint(allow);
                c_y_allow_.push_back(allow);

                // (3) y lower-bound switch:
                // -y_k <= -y_lb  (enableì´ë©´ y_lb=0, disableì´ë©´ y_lb=1 -> y_k=1 ê³ ì •)
                Constraint ylb(s);
                ylb.set_constraint_variable(y_occ[k], -1.0);
                double y_lb = (yield_enable_ ? 0.0 : 1.0);
                ylb.set_known_term(-y_lb);
                qp.add_leq_constraint(ylb);
                c_y_ylb_.push_back(ylb);

                // (4) eps_k >= 0
                Constraint epsge0(s);
                epsge0.set_constraint_variable(eps_occ[k], 1.0);
                epsge0.set_known_term(0.0);
                qp.add_geq_constraint(epsge0);
                c_eps_ge0_.push_back(epsge0);
            }

            // (5) monotone: y_k - y_{k+1} <= 0, k=1..T-1
            for (int k = 1; k <= T-1; ++k) {
                Constraint mono(s);
                mono.set_constraint_variable(y_occ[k],   1.0);
                mono.set_constraint_variable(y_occ[k+1], -1.0);
                mono.set_known_term(0.0);
                qp.add_leq_constraint(mono);
                c_y_mono_.push_back(mono);
            }
        }
    }


    // --------- ë¹„ìš©(Q, q0) ì„¸íŒ… ---------
    const int S = s.get_size();
    Matrix<double> Qcost; Qcost.resize(0.0, S, S);
    Vector<double> q0cost; q0cost.resize(0.0, S);

    // error cost
    const double w_e0 = 100.0;
    const double w_e1 = 10.0;
    const double w_e2 =  5.0;

    for (int k = 0; k <= T; ++k) {
        const int pe = e[k].get_position();
        Qcost[pe + 0][pe + 0] += 2.0 * w_e0;
        if (q >= 2) Qcost[pe + 1][pe + 1] += 2.0 * w_e1;
        if (q >= 3) Qcost[pe + 2][pe + 2] += 2.0 * w_e2;
    }

    // input cost
    const double w_u0 = 1.0;
    const double w_u1 = 1.0;

    for (int k = 0; k <= T-1; ++k) {
        const int pu = u[k+1].get_position();
        Qcost[pu + 0][pu + 0] += 2.0 * w_u0;
        if (p >= 2) Qcost[pu + 1][pu + 1] += 2.0 * w_u1;
    }

    // small reg
    const double reg = 1e-6;
    for (int i = 0; i < S; ++i) {
        Qcost[i][i] += 2.0 * reg;
    }

    if (output_slack) {
        const int p_eps = eps_y.get_position();
        Qcost[p_eps][p_eps] += 2.0 * w_eps_y_;
    }
    if (false) {
        const double w_vslack = 1e5; // íŠœë‹ íŒŒë¼ë¯¸í„° (ì˜ˆ: 1e3~1e8)
        for (int k = 1; k <= T; ++k) {
            const int pv = eps_v[k].get_position();
            Qcost[pv][pv] += 2.0 * w_vslack; // 1/2 x^T Q x êµ¬ì¡°ë¼ 2ë°°
        }
    }

    // ===== Yield cost: w_eps*eps^2  + (-w_go)*y =====
    if (yield_enable_) {
        const double w_eps = yield_w_eps_;
        const double w_go  = yield_w_go_;

        // eps penalty (k=1..T)
        for (int k = 1; k <= T; ++k) {
            const int peps = eps_occ[k].get_position();
            Qcost[peps][peps] += 2.0 * w_eps;
        }

        // linear reward for y=1 (k=1..T)
        // minimize: q0^T x, so negative pushes y -> 1
        for (int k = 1; k <= T; ++k) {
            const int py = y_occ[k].get_position();
            q0cost[py] += -w_go;
        }
    }

    qp.set_Q_matrix(Qcost);
    qp.set_q0_vector(q0cost);

    problem_setup = true;
}

bool MPCProblem::is_problem_setup() {
    return problem_setup;
}

bool MPCProblem::set_state_space_matrices(const Matrix<double> &A,
                                          const Matrix<double> &B,
                                          const Matrix<double> &C1,
                                          const Matrix<double> &C2) {
    if (A.nrows() != n || A.ncols() != n) {
        print_message("Matrix A dimensions are not compatible with the size "
                      "of the state variable\n");
        return false;
    }
    if (B.nrows() != n || B.ncols() != p) {
        print_message("Matrix B dimensions are not compatible with the size "
                      "of the state variable and of the control vector\n");
        return false;
    }
    if (C1.nrows() != q || C1.ncols() != n) {
        print_message("Matrix C1 dimensions are not compatible with the size "
                      "of the state variable and of the output vector\n");
        return false;
    }
    if (C2.nrows() != q2 || C2.ncols() != n) {
        print_message("Matrix C2 dimensions are not compatible with the size "
                      "of the state variable and of the output constraint "
                      "vector\n");
        return false;
    }
    this->A = A;
    this->B = B;
    this->C1 = C1;
    this->C2 = C2;
    return true;
}

bool MPCProblem::set_initial_state(const Vector<double> &x0) {
    if (x0.size() != n) {
        print_message("x0 length is not compatible with the size of the "
                      "state vector\n");
        return false;
    }
    init_x = x0;
    return true;
}

bool MPCProblem::update_initial_state(const Vector<double> &x0) {
    if (x0.size() != n) {
        print_message("x0 length is not compatible with the size of the "
                      "state vector\n");
        return false;
    }
    init_x = x0;
    init_state.set_known_term(init_x);
    qp.update_equality_constraint(init_state, false, true);
    return true;
}

bool MPCProblem::update_state_space_matrices(const Matrix<double>& Anew,
                                             const Matrix<double>& Bnew)
{
    this->A = Anew;
    this->B = Bnew;

    // ì•„ì§ setup_problem()ì´ ì•ˆ ëŒì•˜ìœ¼ë©´, ë‹¤ìŒ solveì—ì„œ ìƒˆ A,Bë¡œ setupë¨
    if (!problem_setup) return true;

    // recompute powers
    std::vector<Matrix<double>> Ak = get_powers(A, T);

    // precompute C1Ak, C1AkB (for error constraints update)
    std::vector<Matrix<double>> C1Ak, C1AkB;
    C1Ak.reserve(T+1);
    C1AkB.reserve(T+1);
    for (int k=0; k<=T; ++k) {
        C1Ak.push_back(multiply(C1, Ak[k]));           // (q x n)
        C1AkB.push_back(multiply(C1Ak.back(), B));     // (q x p)
    }

    // update error constraints LHS: e[k] = C1*A^k*x0 + sum ...
    Matrix<double> Iq = identity(q);
    for (int k=0; k<=T; ++k) {
        // keep e[k] term
        error[k].set_constraint_variable(e[k], Iq);
        // update x0 term
        error[k].set_constraint_variable(x0, -C1Ak[k]);
        // update u terms
        for (int i=0; i<=k-1; ++i) {
            error[k].set_constraint_variable(u[i], -C1AkB[k-1-i]);
        }
        // NOTE: ì•„ëž˜ update flagsëŠ” ë„¤ qp êµ¬í˜„ì— ë”°ë¼ ë‹¤ë¥¼ ìˆ˜ ìžˆìŒ.
        // ê¸°ì¡´ ì½”ë“œê°€ (false,true)=RHSë§Œ ì—…ë°ì´íŠ¸ë¼ë©´, LHS ì—…ë°ì´íŠ¸ëŠ” (true,false) ë˜ëŠ” (true,true)ì—¬ì•¼ í•¨.
        qp.update_equality_constraint(error[k], true, false);
    }

    // update y_max/y_min constraints LHS if they exist
    if (y_max.size() != 0 || y_min.size() != 0) {
        std::vector<Matrix<double>> AkB;
        AkB.reserve(T+1);
        for (int k=0; k<=T; ++k) AkB.push_back(multiply(Ak[k], B)); // (n x p)

        // y_max constraints are on x_{k+1}
        for (int k=0; k<=T-1; ++k) {
            if (y_max.size() != 0) {
                c_y_max[k].set_constraint_variable(x0, Ak[k+1]); // C2=I
                for (int i=0; i<=k; ++i) {
                    c_y_max[k].set_constraint_variable(u[i], AkB[k-i]);
                }
                qp.update_leq_constraint(c_y_max[k], true, false);
            }
            if (y_min.size() != 0) {
                c_y_min[k].set_constraint_variable(x0, Ak[k+1]); // C2=I
                for (int i=0; i<=k; ++i) {
                    c_y_min[k].set_constraint_variable(u[i], AkB[k-i]);
                }
                qp.update_geq_constraint(c_y_min[k], true, false);
            }
        }
    }
    return true;
}



bool MPCProblem::set_initial_control(const Vector<double> &u0) {
    if (u0.size() != p) {
        print_message("u0 length is not compatible with the size of the "
                      "control vector\n");
        return false;
    }
    init_u = u0;
    return true;
}

bool MPCProblem::update_initial_control(const Vector<double> &u0) {
    if (u0.size() != p) {
        print_message("u0 length is not compatible with the size of the "
                      "state vector\n");
        return false;
    }
    init_u = u0;
    init_control.set_known_term(init_u);
    qp.update_equality_constraint(init_control, false, true);
    return true;
}

bool MPCProblem::set_reference_vector(const Vector<double> &r) {
    const int full = q * (T+1);
    if (!(r.size() == q || r.size() == full)) {
        print_message("r length is not compatible with the size of the output vector\n");
        return false;
    }
    this->r = r;
    return true;
}

bool MPCProblem::update_reference_vector(const Vector<double> &r, int index) {
    if (r.size() != q) {
        print_message("r length is not compatible with the size of the "
                      "output vector\n");
        return false;
    }
    if (index == -1) {
        for (int k = 0; k <= T; k++) {
            error[k].set_known_term(-r);
            qp.update_equality_constraint(error[k], false, true);
        }
    }
    else {
        if (index >= (int)error.size())
            return false;
        error[index].set_known_term(-r);
        qp.update_equality_constraint(error[index], false, true);
    }
    return true;
}

bool MPCProblem::seq_Q_matrix(const Matrix<double> &Q) {
    if (Q.nrows() != s.get_size() || Q.ncols() != s.get_size()) {
        print_message("Matrix Q dimensions are not compatible with the size "
                      "of the state variable\n");
        return false;
    }
    qp.set_Q_matrix(Q);
    return true;
}

bool MPCProblem::seq_q0_vector(const Vector<double>& q0) {
    if (q0.size() != s.get_size()) {
        print_message("q0 length is not compatible with the size of the main variable\n");
        return false;
    }
    qp.set_q0_vector(q0);
    return true;
}

bool MPCProblem::set_u_max(const Vector<double> &u_max) {
    if (u_max.size() != p) {
        print_message("u_max length is not compatible with the size of the "
                      "control vector\n");
        return false;
    }
    this->u_max = u_max;
    return true;
}

bool MPCProblem::set_u_min(const Vector<double> &u_min) {
    if (u_min.size() != p) {
        print_message("u_min length is not compatible with the size of the "
                      "control vector\n");
        return false;
    }
    this->u_min = u_min;
    return true;
}

bool MPCProblem::set_du_max(const Vector<double> &du_max) {
    if (du_max.size() != p) {
        print_message("du_max length is not compatible with the size of the "
                      "control vector\n");
        return false;
    }
    this->du_max = du_max * ts;
    return true;
}

bool MPCProblem::set_du_min(const Vector<double> &du_min) {
    if (du_min.size() != p) {
        print_message("du_min length is not compatible with the size of the "
                      "control vector\n");
        return false;
    }
    this->du_min = du_min * ts;
    return true;
}

bool MPCProblem::set_y_max(const Vector<double> &y_max) {
    if (y_max.size() != q2) {
        print_message("y_max length is not compatible with the size of the "
                      "output constraint vector\n");
        return false;
    }
    this->y_max = y_max;
    return true;
}

bool MPCProblem::set_y_min(const Vector<double> &y_min) {
    if (y_min.size() != q2) {
        print_message("y_min length is not compatible with the size of the "
                      "output constraint vector\n");
        return false;
    }
    this->y_min = y_min;
    return true;
}

bool MPCProblem::update_u_max_vector(const Vector<double> &u_max, int index) {
    if (u_max.size() != p) {
        print_message("u_max length is not compatible with the size of the "
                      "control vector\n");
        return false;
    }
    if (index == -1) {
        for (int k = 0; k <= T-1; k++) {
            c_u_max[k].set_known_term(u_max);
            qp.update_leq_constraint(c_u_max[k], false, true);
        }
    }
    else {
        if (index >= (int)c_u_max.size())
            return false;
        c_u_max[index].set_known_term(u_max);
        qp.update_leq_constraint(c_u_max[index], false, true);
    }
    return true;
}

bool MPCProblem::update_u_min_vector(const Vector<double> &u_min, int index) {
    if (u_min.size() != p) {
        print_message("u_min length is not compatible with the size of the "
                      "control vector\n");
        return false;
    }
    if (index == -1) {
        for (int k = 0; k <= T-1; k++) {
            c_u_min[k].set_known_term(u_min);
            qp.update_geq_constraint(c_u_min[k], false, true);
        }
    }
    else {
        if (index >= (int)c_u_min.size())
            return false;
        c_u_min[index].set_known_term(u_min);
        qp.update_geq_constraint(c_u_min[index], false, true);
    }
    return true;
}

bool MPCProblem::update_du_max_vector(const Vector<double> &du_max, int index) {
    if (du_max.size() != p) {
        print_message("du_max length is not compatible with the size of the "
                      "control vector\n");
        return false;
    }
    Vector<double> du_max_scaled = du_max * ts;
    if (index == -1) {
        const int sz = (int)c_du_max.size();
        for (int k = 0; k < sz; k++) {
            if (!minimize_du && k == 0) {
                c_du_max[k].set_known_term(du_max_scaled + init_u);
            } else {
                c_du_max[k].set_known_term(du_max_scaled);
            }
            qp.update_leq_constraint(c_du_max[k], false, true);
        }
    }
    else {
        if (index >= (int)c_du_max.size())
            return false;
        if (!minimize_du && index == 0) {
            c_du_max[index].set_known_term(du_max_scaled + init_u);
        } else {
            c_du_max[index].set_known_term(du_max_scaled);
        }
        qp.update_leq_constraint(c_du_max[index], false, true);
    }
    return true;
}

bool MPCProblem::update_du_min_vector(const Vector<double> &du_min, int index) {
    if (du_min.size() != p) {
        print_message("du_min length is not compatible with the size of the "
                      "control vector\n");
        return false;
    }
    Vector<double> du_min_scaled = du_min * ts;
    if (index == -1) {
        const int sz = (int)c_du_min.size();
        for (int k = 0; k < sz; k++) {
            if (!minimize_du && k == 0) {
                c_du_min[k].set_known_term(du_min_scaled + init_u);
            } else {
                c_du_min[k].set_known_term(du_min_scaled);
            }
            qp.update_geq_constraint(c_du_min[k], false, true);
        }
    }
    else {
        if (index >= (int)c_du_min.size())
            return false;
        if (!minimize_du && index == 0) {
            c_du_min[index].set_known_term(du_min_scaled + init_u);
        } else {
            c_du_min[index].set_known_term(du_min_scaled);
        }
        qp.update_geq_constraint(c_du_min[index], false, true);
    }
    return true;
}

bool MPCProblem::update_y_min_vector(const Vector<double> &y_min, int index) {
    if (y_min.size() != q2) {
        print_message("y_min length is not compatible with the size of the "
                      "output constraint vector\n");
        return false;
    }
    if (index == -1) {
        for (int k = 0; k <= T-1; k++) {
            c_y_min[k].set_known_term(y_min);
            qp.update_geq_constraint(c_y_min[k], false, true);
        }
    }
    else {
        if (index >= (int)c_y_min.size())
            return false;
        c_y_min[index].set_known_term(y_min);
        qp.update_geq_constraint(c_y_min[index], false, true);
    }
    return true;
}

bool MPCProblem::update_y_max_vector(const Vector<double> &y_max, int index) {
    if (y_max.size() != q2) {
        print_message("y_max length is not compatible with the size of the "
                      "output constraint vector\n");
        return false;
    }
    if (index == -1) {
        for (int k = 0; k <= T-1; k++) {
            c_y_max[k].set_known_term(y_max);
            qp.update_leq_constraint(c_y_max[k], false, true);
        }
    }
    else {
        if (index >= (int)c_y_max.size())
            return false;
        c_y_max[index].set_known_term(y_max);
        qp.update_leq_constraint(c_y_max[index], false, true);
    }
    return true;
}

double MPCProblem::solve_mpc(Vector<double> &arg) {
    if (!problem_setup) {
        setup_problem();
        if (debug)
            qp.write_information();
    }
    const int S = s.get_size();
    if (S <= 0) {
        if (debug) std::cerr << "[MPC] invalid S=" << S << std::endl;
        return 1.0e30;
    }

    if (arg.size() != S) {
        arg.resize(0.0, S);
    }
    return qp.solve_problem(arg);
}

bool MPCProblem::is_feasible(double result) {
    if (std::numeric_limits<double>::has_infinity)
        return !std::isinf(result);
    else
        return result < 1.0E300;
}

Matrix<double> MPCProblem::get_state_evolution(const Matrix<double> &C,
                                               const Vector<double> &solution) {
    int samples = T + 1;
    int states = C.nrows();
    Matrix<double> output(samples, states);
    Matrix<double> x(samples, n);
    Matrix<double> yk, xk;
    Vector<double> uk;

    copy_vector(x[0], init_x);
    yk = multiply(C, x[0]);
    copy_vector(output[0], yk);
    for (int k = 1; k <= T; k++) {
        uk = subvector(solution, u[k-1].get_position(), u[k-1].get_size());
        xk = multiply(A, x[k-1]) + multiply(B, uk);
        copy_vector(x[k], xk);
        yk = multiply(C, x[k]);
        copy_vector(output[k], yk);
    }
    return output;
}


int MPCProblem::u_pos(int k) const { return u[k].get_position(); }  // k=0..T
int MPCProblem::u_dim() const { return p; }

static inline int clamp_int(int v, int lo, int hi)
{
  return std::max(lo, std::min(hi, v));
}
