import re
p='tracking_control_variant.cpp'; s=open(p).read()
# globals
s=s.replace('namespace imac_ctrl\n{','''// ---- VERIFICATION-ONLY variant switches (not production) ----
int g_branch_variant = 0;          // 0=production, 1=vanished-branch truncation, 2=1+split-anchored turn window
double g_continuity_m = 6.75;      // segment-to-segment distance for "same arm" between consecutive sections
int g_last_truncated_at = -1;      // diagnostic: last constrained step of chosen truncated combo (-1 none)
int g_truncated_evals = 0;
namespace imac_ctrl
{''',1)
assert 'g_branch_variant' in s
# continuity + branch step
old='''  for (int k = 1; k <= horizon; ++k) {
    if (preview.Nck[static_cast<std::size_t>(k)] >= 2) {
      result.branch_step = k;
      break;
    }
  }
'''
new='''  // ---- continuity between consecutive section intervals (verification variant) ----
  auto segment_distance = [](const Eigen::Vector2d & a0, const Eigen::Vector2d & a1,
      const Eigen::Vector2d & b0, const Eigen::Vector2d & b1) {
      auto point_segment = [](const Eigen::Vector2d & p, const Eigen::Vector2d & q0,
          const Eigen::Vector2d & q1) {
          const Eigen::Vector2d d = q1 - q0; const double l2 = d.squaredNorm();
          const double u = l2 > 1e-12 ? std::clamp((p - q0).dot(d) / l2, 0.0, 1.0) : 0.0;
          return (p - (q0 + u * d)).norm();
        };
      return std::min({point_segment(a0, b0, b1), point_segment(a1, b0, b1),
          point_segment(b0, a0, a1), point_segment(b1, a0, a1)});
    };
  // cont[k][i][j]: candidate i at step k-1 (k-1==0: ego origin) continues into candidate j at step k
  std::vector<std::vector<std::vector<char>>> cont(static_cast<std::size_t>(horizon + 1));
  for (int k = 1; k <= horizon; ++k) {
    const std::size_t sk = static_cast<std::size_t>(k);
    const int prev_count = k == 1 ? 1 : preview.Nck[sk - 1];
    cont[sk].assign(static_cast<std::size_t>(prev_count),
      std::vector<char>(static_cast<std::size_t>(preview.Nck[sk]), 0));
    for (int i = 0; i < prev_count; ++i) {
      const Eigen::Vector2d a0 = k == 1 ? Eigen::Vector2d::Zero() : preview.pmk[sk - 1][static_cast<std::size_t>(i)];
      const Eigen::Vector2d a1 = k == 1 ? Eigen::Vector2d::Zero() : preview.pMk[sk - 1][static_cast<std::size_t>(i)];
      for (int j = 0; j < preview.Nck[sk]; ++j) {
        cont[sk][static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] =
          segment_distance(a0, a1, preview.pmk[sk][static_cast<std::size_t>(j)],
          preview.pMk[sk][static_cast<std::size_t>(j)]) <= g_continuity_m ? 1 : 0;
      }
    }
  }
  for (int k = 1; k <= horizon; ++k) {
    if (preview.Nck[static_cast<std::size_t>(k)] >= 2) {
      result.branch_step = k;
      break;
    }
  }
  if (g_branch_variant >= 2) {
    // Anchor the relative-turn window where one arm actually splits into >=2 continuing arms.
    int split_step = -1;
    for (int k = 1; k <= horizon && split_step < 0; ++k) {
      const auto & table = cont[static_cast<std::size_t>(k)];
      for (const auto & row_flags : table) {
        int continuing = 0;
        for (char flag : row_flags) {continuing += flag ? 1 : 0;}
        if (continuing >= 2) {split_step = k; break;}
      }
    }
    if (split_step >= 1) {result.branch_step = split_step;}
  }
  g_last_truncated_at = -1;
'''
assert old in s; s=s.replace(old,new,1)
# enumeration: add truncated evaluation
old2='''      Vector<double> best_direct_argument;
      std::vector<int> best_candidates(static_cast<std::size_t>(horizon + 1), -1);
      double best_direct_objective = std::numeric_limits<double>::infinity();
'''
new2='''      Vector<double> best_direct_argument;
      std::vector<int> best_candidates(static_cast<std::size_t>(horizon + 1), -1);
      double best_direct_objective = std::numeric_limits<double>::infinity();
      int best_truncated_at = -1;
      miqp_solution_validation::FeasibilityReport best_truncated_feasibility;
'''
assert old2 in s; s=s.replace(old2,new2,1)
old3='''        Matrix<double> direct_inequality(0.0, direct_row_count, input_count);
        Vector<double> direct_bound(0.0, direct_row_count);
        int direct_row = 0;
        for (int k = 1; k <= horizon; ++k) {'''
new3='''        // Verification variant: if the selected arm vanishes at step m (no candidate at m continues it),
        // additionally evaluate the combination constrained only up to m-1 with a straight tail.
        int truncate_at = -1;
        if (g_branch_variant >= 1) {
          for (int k = 2; k <= horizon; ++k) {
            const auto & row_flags = cont[static_cast<std::size_t>(k)][static_cast<std::size_t>(selected[static_cast<std::size_t>(k - 1)])];
            bool any = false;
            for (char flag : row_flags) {any = any || flag;}
            if (!any) {
              bool canonical = true;
              for (int t = k; t <= horizon; ++t) {canonical = canonical && selected[static_cast<std::size_t>(t)] == 0;}
              const int last = k - 1;
              if (canonical && last >= std::max(2, result.branch_step)) {truncate_at = last;}
              break;
            }
            if (!row_flags[static_cast<std::size_t>(selected[static_cast<std::size_t>(k)])]) {break;}
          }
        }
        if (truncate_at > 0) {
          const int tail_inputs = horizon - truncate_at + 1;
          const int rows = 2 * truncate_at + input_rows + 2 * tail_inputs;
          Matrix<double> ti(0.0, rows, input_count);
          Vector<double> tb(0.0, rows);
          int r = 0;
          for (int k = 1; k <= truncate_at; ++k) {
            const std::size_t step = static_cast<std::size_t>(k);
            const std::size_t candidate = static_cast<std::size_t>(selected[step]);
            const double heading = preview.psirk[step];
            const Eigen::Vector2d axis(-std::sin(heading), std::cos(heading));
            const double reference_projection = axis.dot(preview.prk[step]);
            double lower = axis.dot(preview.pmk[step][candidate]);
            double upper = axis.dot(preview.pMk[step][candidate]);
            if (lower > upper) {std::swap(lower, upper);}
            for (int column = 0; column < input_count; ++column) {
              const double projection = axis.x() * G[2 * k][column] + axis.y() * G[2 * k + 1][column];
              ti[r][column] = projection; ti[r + 1][column] = -projection;
            }
            tb[r++] = upper - reference_projection;
            tb[r++] = -lower + reference_projection;
          }
          for (int input_row = 0; input_row < input_rows; ++input_row) {
            for (int column = 0; column < input_count; ++column) {ti[r][column] = inequality[lane_rows + input_row][column];}
            tb[r++] = inequality_bound[lane_rows + input_row];
          }
          for (int k = truncate_at - 1; k < horizon; ++k) {
            // zero absolute heading increment: x = -nominal
            ti[r][2 * k + 1] = 1.0; tb[r++] = -nominal_input[2 * k + 1];
            ti[r][2 * k + 1] = -1.0; tb[r++] = nominal_input[2 * k + 1];
          }
          QuadraticProblem tsolver(false);
          Variable * tv = tsolver.vector_variable(input_count, "upper_truncated_corridor");
          if (tsolver.add_variable(tv)) {
            const Var td = tsolver.get_variable(tv);
            const Var tm = tsolver.get_main_variable();
            Constraint tc(tm);
            tc.set_constraint_variable(td, ti);
            tc.set_known_term(tb);
            if (tsolver.add_leq_constraint(tc) && tsolver.set_Q_matrix(direct_hessian) &&
              tsolver.set_q0_vector(direct_linear))
            {
              Vector<double> targ;
              const double tobj = timed_solve(tsolver, targ);
              ++g_truncated_evals;
              if (std::isfinite(tobj) && targ.size() >= static_cast<unsigned int>(input_count)) {
                const auto tf = miqp_solution_validation::evaluate(ti, tb, direct_equality,
                  direct_equality_bound, targ, static_cast<std::size_t>(input_count), miqp_feasibility_tolerance_);
                if (tf.valid && tobj < best_direct_objective) {
                  best_direct_argument = targ; best_direct_objective = tobj; best_candidates = selected;
                  best_truncated_at = truncate_at; best_truncated_feasibility = tf;
                }
              }
            }
          }
        }

        Matrix<double> direct_inequality(0.0, direct_row_count, input_count);
        Vector<double> direct_bound(0.0, direct_row_count);
        int direct_row = 0;
        for (int k = 1; k <= horizon; ++k) {'''
assert old3 in s; s=s.replace(old3,new3,1)
old4='''        best_direct_argument = direct_argument;
        best_direct_objective = direct_objective;
        best_candidates = selected;
      }
'''
new4='''        best_direct_argument = direct_argument;
        best_direct_objective = direct_objective;
        best_candidates = selected;
        best_truncated_at = -1;
      }
'''
assert old4 in s; s=s.replace(old4,new4,1)
old5='''        const auto reconstructed_feasibility = miqp_solution_validation::evaluate(
          inequality, inequality_bound, equality, equality_bound,
          reconstructed_argument, static_cast<std::size_t>(input_count),
          miqp_feasibility_tolerance_);
        if (reconstructed_feasibility.valid) {'''
new5='''        auto reconstructed_feasibility = miqp_solution_validation::evaluate(
          inequality, inequality_bound, equality, equality_bound,
          reconstructed_argument, static_cast<std::size_t>(input_count),
          miqp_feasibility_tolerance_);
        if (best_truncated_at > 0) {
          // Truncated combination was checked against its own (shorter) constraint set.
          reconstructed_feasibility = best_truncated_feasibility;
          g_last_truncated_at = best_truncated_at;
        }
        if (reconstructed_feasibility.valid) {'''
assert old5 in s; s=s.replace(old5,new5,1)
open(p,'w').write(s); print('patched')
