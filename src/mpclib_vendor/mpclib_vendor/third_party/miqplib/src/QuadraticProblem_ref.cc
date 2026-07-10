// QuadraticProblem.cpp
#include "QuadraticProblem.h"
#include <vector>
#include <cmath>
#include <limits>// QuadraticProblem.cpp
#include "QuadraticProblem.h"
#include <vector>
#include <cmath>
#include <limits>

extern "C" {
#include "daqp/include/types.h"
#include "daqp/include/api.h"
#include "daqp/include/constants.h"
#include "daqp/include/utils.h"
}


Variable *QuadraticProblem::vector_variable(int n, const std::string &name,
                                            const std::string &base_name) {
    Variable *base = new Variable(true, base_name);
    Variable *vec_var = new Variable(false, name);
    vec_var->add_variable(base, n);
    delete base;
    return vec_var;
}

Variable *QuadraticProblem::vector_variable(int n,
                                            const Variable *base_variable,
                                            const std::string &name) {
    Variable *vec_var = new Variable(false, name);
    vec_var->add_variable(base_variable, n);
    return vec_var;
}

void QuadraticProblem::print_message(const std::string &msg) {
    if (debug)
        std::cerr << msg;
}

bool QuadraticProblem::add_variable(Variable *v) {
    VarMap::iterator i;
    if (constraint_phase) {
        print_message("Cannot add variables anymore. User already started "
                      "adding constraints\n");
        return false;
    }
    i = variables.find(v);
    if (i != variables.end()) {
        print_message("The given variable has already been added.\n");
        return false;
    }
    else {
        variables[v] = vx->get_subvariables_count();
    }
    vx->add_variable(v);
    return true;
}

Var QuadraticProblem::get_variable(const Variable *v) {
    return Var(v);
}

void QuadraticProblem::init_constraint_phase() {
    if (constraint_phase)
        return;
    constraint_phase = true;
    eq.set_variable(Var(vx));
    geq.set_variable(Var(vx));
    q0.resize(0, vx->get_size());
}

Var QuadraticProblem::get_main_variable() {
    init_constraint_phase();
    return Var(vx);
}

bool QuadraticProblem::add_equality_constraint(Constraint &c) {
    init_constraint_phase();
    if (c.get_variable().get_variable() != vx) {
        print_message("The given constraint has been built on a different "
                      "variable and cannot be added to the problem.\n");
        return false;
    }
    return eq.add_constraint(c);
}

bool QuadraticProblem::update_equality_constraint(const Constraint &c,
                                                  bool update_matrix,
                                                  bool update_vector) {
    return eq.update_constraint(c, update_matrix, update_vector);
}

bool QuadraticProblem::add_geq_constraint(Constraint &c) {
    init_constraint_phase();
    if (c.get_variable().get_variable() != vx) {
        print_message("The given constraint has been built on a different "
                      "variable and cannot be added to the problem.\n");
        return false;
    }
    return geq.add_constraint(c);
}

bool QuadraticProblem::update_geq_constraint(const Constraint &c,
                                             bool update_matrix,
                                             bool update_vector) {
    return geq.update_constraint(c, update_matrix, update_vector);
}

bool QuadraticProblem::add_leq_constraint(Constraint &c) {
    bool result;
    init_constraint_phase();
    if (c.get_variable().get_variable() != vx) {
        print_message("The given constraint has been built on a different "
                      "variable and cannot be added to the problem.\n");
        return false;
    }
    // transform <= into >=
    c.invert_constraint();
    result = geq.add_constraint(c);
    c.invert_constraint();
    return result;

}

bool QuadraticProblem::update_leq_constraint(const Constraint &c,
                                             bool update_matrix,
                                             bool update_vector) {
    // transform <= into >=
    Constraint inv(c);
    inv.invert_constraint();
    return geq.update_constraint(inv, update_matrix, update_vector);
}

bool QuadraticProblem::set_Q_matrix(const Matrix<double> &Q) {
    if (!constraint_phase) {
        print_message("Before setting the Q matrix you should enter in the "
                      "constraint phase by either adding constraints or by "
                      "obtaining the x variable via get_main_variable()\n");
        return false;
    }
    if (Q.ncols() != vx->get_size() || Q.nrows() != vx->get_size()) {
        print_message("Incompatible Q matrix. Q should be an N by N square "
                      "matrix with N being the size of the x variable\n");
        return false;
    }
    this->Q = Q;
    return true;
}

bool QuadraticProblem::set_q0_vector(const Vector<double> &q0) {
    if (!constraint_phase) {
        print_message("Before setting the q0 vector you should enter in the "
                      "constraint phase by either adding constraints or by "
                      "obtaining the x variable via get_main_variable()\n");
        return false;
    }
    if (q0.size() != vx->get_size()) {
        print_message("Incompatible q0 vector. q0 should be a vector of "
                      "length N with N being the size of the x variable\n");
        return false;
    }
    this->q0 = q0;
    return true;
}


double QuadraticProblem::solve_problem(Vector<double> &arg) {
    const Matrix<double> &EQ   = eq.get_constraint_matrix();
    const Vector<double> &EQb  = eq.get_known_values_vector();
    const Matrix<double> &GEQ  = geq.get_constraint_matrix();
    const Vector<double> &GEQb = geq.get_known_values_vector();

    const int n     = vx->get_size();
    const int n_eq  = (int)EQb.size();
    const int n_geq = (int)GEQb.size();
    const int m_gen = n_eq + n_geq;

    if (n <= 0) return 1.0e30;

    // DAQP: 첫 ms개 제약은 bounds(각 변수별)
    const int ms = n;
    const int m  = ms + m_gen;

    // INF
    const c_float INF = (c_float)DAQP_INF;

    // --------- H, f 준비 (row-major) ----------
    std::vector<c_float> H((size_t)n * n, (c_float)0);
    std::vector<c_float> f((size_t)n,     (c_float)0);

    for (int r = 0; r < n; ++r) {
        f[r] = (c_float)q0[r];
        for (int c = 0; c < n; ++c) {
            H[(size_t)r * n + c] = (c_float)Q[(size_t)r][(size_t)c];
        }
    }

    // --------- bounds + general constraint arrays ----------
    std::vector<c_float> blower((size_t)m, (c_float)0);
    std::vector<c_float> bupper((size_t)m, (c_float)0);
    std::vector<int>     sense ((size_t)m, 0);

    // bounds 디폴트 준비
    if ((int)lbx_.size() != n) {
        lbx_.assign(n, -(double)DAQP_INF);
        ubx_.assign(n, +(double)DAQP_INF);
        sense_bnd_.assign(n, 0);
    }
    std::vector<c_float> A((size_t)m * n, (c_float)0);

    // (1) bounds: i=0..ms-1
    for (int i = 0; i < ms; ++i) {
        blower[i] = (c_float)lbx_[i];
        bupper[i] = (c_float)ubx_[i];
        sense[i]  = sense_bnd_[i]; // binary 표시가 여기 들어감
    }

    if (debug) {
        int nbinary = 0;
        for (int i = 0; i < ms; ++i) {
            if (sense[i] & BINARY) nbinary++;
        }
        fprintf(stderr, "[sense-check] nbinary=%d / ms=%d\n", nbinary, ms);
    }

    // (2) general constraints matrix A is (m-ms) x n, row-major
    int row = 0; // row index for A (0..m_gen-1)

    // EQ: blower=bupper=EQb, stored in indices ms+row
    for (int r = 0; r < n_eq; ++r, ++row) {
        for (int c = 0; c < n; ++c) {
            A[(size_t)row * n + c] = (c_float)EQ[(size_t)r][(size_t)c];
        }
        blower[ms + row] = (c_float)EQb[r];
        bupper[ms + row] = (c_float)EQb[r];
        sense [ms + row] = 0; // equality도 bl==bu로 표현 가능
    }

    // GEQ: A x >= b  => blower=b, bupper=+INF
    for (int r = 0; r < n_geq; ++r, ++row) {
        for (int c = 0; c < n; ++c) {
            A[(size_t)row * n + c] = (c_float)GEQ[(size_t)r][(size_t)c];
        }
        blower[ms + row] = (c_float)GEQb[r];
        bupper[ms + row] = INF;
        sense [ms + row] = 0;
    }

    if (debug) {
        fprintf(stderr, "[EQ]  rows=%d cols=%d  EQb=%zu\n", (int)EQ.nrows(), (int)EQ.ncols(), EQb.size());
        fprintf(stderr, "[GEQ] rows=%d cols=%d GEQb=%zu\n", (int)GEQ.nrows(), (int)GEQ.ncols(), GEQb.size());
    }
    // --------- DAQPProblem 구성 ----------
    DAQPProblem qp{};
    qp.n  = n;
    qp.m  = m;
    qp.ms = ms;

    qp.H = H.data();
    qp.f = f.data();

    qp.A = (m_gen > 0) ? A.data() : NULL;
    qp.blower = blower.data();
    qp.bupper = bupper.data();
    qp.sense  = sense.data();

    qp.nh = 0; qp.break_points = nullptr;

    // --------- 결과 버퍼 ----------
    std::vector<c_float> x((size_t)n, (c_float)0);
    std::vector<c_float> lam((size_t)m, (c_float)0);

    DAQPResult res{};
    res.x = x.data();
    res.lam = lam.data();

    DAQPSettings settings;
    daqp_default_settings(&settings);
    settings.eps_prox = 1e-5;
    settings.iter_limit = 300;
    settings.progress_tol = 1e-8;
    settings.primal_tol = 1e-4;
    settings.dual_tol = 1e-6;
    settings.rel_subopt = 1e-4;
    settings.abs_subopt = 1e-6;
    fprintf(stderr, "[DAQP-check] sizeof(c_float)=%zu sizeof(double)=%zu\n",
        sizeof(c_float), sizeof(double));
    fprintf(stderr, "[DAQP-check] qp.n=%d qp.m=%d qp.ms=%d\n", qp.n, qp.m, qp.ms);
    daqp_quadprog(&res, &qp, &settings);

    fprintf(stderr,
    "[DAQP] exitflag=%d fval=%g  n=%d m=%d ms=%d  (eq=%d geq=%d)\n",
    res.exitflag, res.fval, qp.n, qp.m, qp.ms, n_eq, n_geq);
    fflush(stderr);

    if (res.exitflag > 0) {
        arg.resize(0.0, n);
        for (int i = 0; i < n; ++i) arg[i] = (double)x[i];
        return (double)res.fval;
    }
    return std::numeric_limits<double>::infinity();
}


void QuadraticProblem::write_information() {
}

void QuadraticProblem::set_var_bounds(int idx, double lb, double ub){
  const int n = vx->get_size();
  if ((int)lbx_.size() != n){
    lbx_.assign(n, -(double)DAQP_INF);
    ubx_.assign(n, +(double)DAQP_INF);
    sense_bnd_.assign(n, 0);
  }
  lbx_[idx] = lb;
  ubx_[idx] = ub;
}

void QuadraticProblem::set_binary_bounds(int idx, double lb, double ub){
  set_var_bounds(idx, lb, ub);
  // “bounds 제약 idx”를 binary로 표시
  sense_bnd_[idx] |= BINARY;
}

void QuadraticProblem::set_binary_var(const Var& v){
  // v가 scalar(크기 1)라고 가정
  const int pos = v.get_position();
  // 필요하면 v.get_size()==1 체크
  set_binary_bounds(pos, 0.0, 1.0);
}
