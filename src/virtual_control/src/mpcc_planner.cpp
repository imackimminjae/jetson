#include "comparison_internal.hpp"
#include <api.h>
#include <Eigen/Dense>

namespace imac_ctrl::comparison
{
namespace
{
using State=Eigen::Vector4d;
using Matrix=Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>;
State step(const State & x, double delta, double vs, double dt, const Input & in)
{
  const double beta=std::atan(in.geometry.reference_to_rear/in.geometry.wheelbase*std::tan(delta));
  const double speed=in.settings.prediction_speed;
  return x+dt*State(speed*std::cos(x[2]+beta),speed*std::sin(x[2]+beta),
    speed*std::cos(beta)*std::tan(delta)/in.geometry.wheelbase,vs);
}
double interpolate(const std::vector<double> & times, const std::vector<double> & values, double t)
{
  if (t<=times.front()) {return values.front();}
  auto it=std::upper_bound(times.begin(),times.end(),t);
  if (it==times.end()) {return values.back();}
  const size_t k=it-times.begin();
  const double ratio=(t-times[k-1])/(times[k]-times[k-1]);
  return values[k-1]+ratio*(values[k]-values[k-1]);
}
}  // namespace
void MpccPlanner::reset()
{
  previous_s_.reset(); previous_route_.clear(); previous_delta_.clear();
  previous_virtual_.clear(); previous_times_.clear(); previous_time_=0;
}
Output MpccPlanner::plan(const Input & request)
{
  const auto start=Clock::now(); auto pre=start, planned=start;
  Input in=request;
  Output out;
  try {
    validate(in);
    in.grid.buildIndex();
    if (!sameRoute(previous_route_,in.route) || in.time<previous_time_) {reset();}
    ReferenceCurve curve(in.route);
    const auto & c=in.settings;
    out.projected_s=projectProgress(in,curve,previous_s_,previous_time_);
    const double remaining=curve.length()-out.projected_s;
    if (remaining<=0.5 && (curve.at(curve.length()).p-in.vehicle.p).norm()<=0.5) {
      out.goal_reached=true; throw std::runtime_error("open path goal reached");
    }
    // A certified short terminal approach must not deadlock just outside the
    // goal radius because the ordinary minimum path length is longer than it.
    if (remaining>0.5 && remaining<c.min_length &&
      (curve.at(curve.length()).p-in.vehicle.p).norm()<=c.min_length) {
      in.settings.min_length=0.5*remaining;
    }
    // Shorten the final time step near the end of an OPEN reference. Never wrap.
    std::vector<double> durations,times;
    double time=0, budget=remaining/c.prediction_speed;
    for (int k=0; k<c.steps && budget>1e-6; ++k) {
      const double dt=std::min(c.dt,budget);
      times.push_back(time); durations.push_back(dt); time+=dt; budget-=dt;
    }
    const int N=static_cast<int>(durations.size()), n=2*N;
    if (N<1 || time*c.prediction_speed<c.min_length) {throw std::runtime_error("MPCC end horizon too short");}
    Eigen::VectorXd u=Eigen::VectorXd::Zero(n);
    const double elapsed=in.time-previous_time_;
    const bool warm=!previous_delta_.empty() && elapsed>=0 &&
      elapsed<=previous_times_.back()+c.dt;
    out.warm_shift_sec=warm ? elapsed : 0;
    const double applied=in.applied_steer.value_or(0.0);
    for (int k=0; k<N; ++k) {
      const double seed_delta=warm ? interpolate(previous_times_,previous_delta_,elapsed+times[k]) : applied;
      const double anchor=k==0 ? applied : u[2*(k-1)];
      const double limit=in.geometry.max_steer_rate*(k==0 ? c.period : durations[k-1]);
      u[2*k]=std::clamp(std::clamp(seed_delta,anchor-limit,anchor+limit),
        -in.geometry.max_steer,in.geometry.max_steer);
      u[2*k+1]=warm ? interpolate(previous_times_,previous_virtual_,elapsed+times[k]) :
        std::min(c.prediction_speed,c.max_virtual_speed);
    }
    const State initial(in.vehicle.p.x(),in.vehicle.p.y(),in.vehicle.yaw,out.projected_s);
    auto rollout=[&](const Eigen::VectorXd & controls) {
        std::vector<State> x{initial};
        for (int k=0; k<N; ++k) {
          x.push_back(step(x.back(),controls[2*k],controls[2*k+1],durations[k],in));
        }
        return x;
      };
    pre=Clock::now();
    std::vector<State> states;
    for (int iteration=0; iteration<c.sqp_iterations; ++iteration) {
      states=rollout(u);
      // Eliminate linearized dynamics and initial-state equalities exactly.
      std::vector<Eigen::MatrixXd> sensitivities{Eigen::MatrixXd::Zero(4,n)};
      for (int k=0; k<N; ++k) {
        Eigen::Matrix4d A;
        Eigen::Matrix<double,4,2> B;
        constexpr double eps=1e-5;
        for (int j=0; j<4; ++j) {
          State a=states[k],b=a; a[j]+=eps; b[j]-=eps;
          A.col(j)=(step(a,u[2*k],u[2*k+1],durations[k],in)-
            step(b,u[2*k],u[2*k+1],durations[k],in))/(2*eps);
        }
        B.col(0)=(step(states[k],u[2*k]+eps,u[2*k+1],durations[k],in)-
          step(states[k],u[2*k]-eps,u[2*k+1],durations[k],in))/(2*eps);
        B.col(1)=State(0,0,0,durations[k]);
        Eigen::MatrixXd S=A*sensitivities.back(); S.block(0,2*k,4,2)+=B;
        sensitivities.push_back(std::move(S));
      }
      Matrix H=1e-6*Matrix::Identity(n,n);
      Eigen::VectorXd f=Eigen::VectorXd::Zero(n);
      std::vector<Eigen::RowVectorXd> rows;
      std::vector<double> lower,upper;
      auto constraint=[&](const Eigen::RowVectorXd & row,double lo,double hi) {
          rows.push_back(row); lower.push_back(lo); upper.push_back(hi);
        };
      auto square=[&](const Eigen::RowVectorXd & row,double constant,double weight) {
          H.noalias()+=2*weight*row.transpose()*row;
          f.noalias()+=2*weight*constant*row.transpose();
        };
      std::vector<double> bu(n),bl(n);
      for (int k=0; k<N; ++k) {
        bl[2*k]=std::max(-in.geometry.max_steer,u[2*k]-c.steer_trust);
        bu[2*k]=std::min(in.geometry.max_steer,u[2*k]+c.steer_trust);
        bl[2*k+1]=0; bu[2*k+1]=c.max_virtual_speed;
        Eigen::RowVectorXd steer=Eigen::RowVectorXd::Zero(n); steer[2*k]=1;
        square(steer,0,c.r_steer);
        Eigen::RowVectorXd rate=steer;
        if (k>0) {rate[2*(k-1)]=-1;}
        const double anchor=k==0 ? applied : 0;
        const double rate_dt=k==0 ? c.period : durations[k-1];
        square(rate,-anchor,c.r_steer_change/(rate_dt*rate_dt));
        constraint(rate,anchor-in.geometry.max_steer_rate*rate_dt,
          anchor+in.geometry.max_steer_rate*rate_dt);
        const State & x=states[k+1]; const auto & S=sensitivities[k+1];
        const auto ref=curve.at(x[3]);
        const Point normal(-ref.tangent.y(),ref.tangent.x()), e=x.head<2>()-ref.p;
        const double ec=normal.dot(e),el=ref.tangent.dot(e);
        Eigen::RowVector4d jc,jl;
        jc<<normal.x(),normal.y(),0,-ref.curvature*el;
        jl<<ref.tangent.x(),ref.tangent.y(),0,ref.curvature*ec-1;
        const Eigen::RowVectorXd gc=jc*S,gl=jl*S;
        square(gc,ec-gc.dot(u),c.q_contour);
        square(gl,el-gl.dot(u),c.q_lag);
        const auto bounds=corridor(in,ref);
        if (!bounds) {throw std::runtime_error("MPCC reference outside contiguous corridor");}
        // Freeze each normal section in this SQP. Progress trust prevents moving
        // to an unrelated section; final nonlinear geometry is independently checked.
        const Eigen::RowVectorXd lateral=normal.transpose()*S.topRows(2);
        const double constant=ec-lateral.dot(u);
        constraint(lateral,(*bounds)[0]-constant,(*bounds)[1]-constant);
        const double s_constant=x[3]-S.row(3).dot(u);
        constraint(S.row(3),std::max(0.0,x[3]-c.progress_trust)-s_constant,
          std::min(curve.length(),x[3]+c.progress_trust)-s_constant);
      }
      f.noalias()-=c.reward_progress*sensitivities.back().row(3).transpose();
      Matrix A(rows.size(),n);
      for (size_t j=0; j<rows.size(); ++j) {
        A.row(j)=rows[j]; bl.push_back(lower[j]); bu.push_back(upper[j]);
      }
      std::vector<int> sense(bl.size(),0);
      Eigen::VectorXd solution(n);
      DAQPProblem problem{};
      problem.n=n; problem.m=static_cast<int>(bl.size()); problem.ms=n;
      problem.H=H.data(); problem.f=f.data(); problem.A=A.data();
      problem.bupper=bu.data(); problem.blower=bl.data(); problem.sense=sense.data();
      DAQPSettings settings{}; daqp_default_settings(&settings);
      settings.iter_limit=2000;
      DAQPResult result{}; result.x=solution.data();
      daqp_quadprog(&result,&problem,&settings);
      out.qp_status=result.exitflag; out.iterations=iteration+1;
      if (result.exitflag!=EXIT_OPTIMAL || !solution.allFinite()) {
        throw std::runtime_error("MPCC DAQP failure: "+std::to_string(result.exitflag));
      }
      // Check all QP constraints independently of the solver status.
      double violation=0;
      for (int j=0; j<problem.m; ++j) {
        const double value=j<n ? solution[j] : A.row(j-n).dot(solution);
        violation=std::max({violation,bl[j]-value,value-bu[j]});
      }
      out.max_constraint_violation=violation;
      if (violation>1e-4) {throw std::runtime_error("MPCC QP constraint residual");}
      const double change=(solution-u).lpNorm<Eigen::Infinity>();
      u=solution;
      if (change<1e-4) {break;}
    }
    states=rollout(u);
    double prior=applied;
    for (int k=0; k<N; ++k) {
      const double rate_dt=k==0 ? c.period : durations[k-1];
      if (std::abs(u[2*k])>in.geometry.max_steer+1e-5 ||
        std::abs(u[2*k]-prior)>in.geometry.max_steer_rate*rate_dt+1e-5 ||
        states[k+1][3]>curve.length()+1e-4 || states[k+1][3]<states[k][3]-1e-5) {
        throw std::runtime_error("MPCC nonlinear steering/progress violation");
      }
      const auto ref=curve.at(states[k+1][3]);
      const auto bounds=corridor(in,ref);
      const Point normal(-ref.tangent.y(),ref.tangent.x());
      const double lateral=normal.dot(states[k+1].head<2>()-ref.p);
      if (!bounds || lateral<(*bounds)[0]-1e-4 || lateral>(*bounds)[1]+1e-4 ||
        !connectionFree(in,{states[k].head<2>(),states[k][2]},
        {states[k+1].head<2>(),states[k+1][2]})) {
        throw std::runtime_error("MPCC nonlinear corridor/collision violation");
      }
      prior=u[2*k];
      out.steering.push_back(u[2*k]); out.virtual_speed.push_back(u[2*k+1]);
    }
    const auto terminal=curve.at(curve.length());
    if (terminal.tangent.dot(states.back().head<2>()-terminal.p)>0.1) {
      throw std::runtime_error("MPCC open path endpoint overshoot");
    }
    for (const auto & state : states) {
      out.sparse.push_back(state.head<2>()); out.progress.push_back(state[3]);
    }
    planned=Clock::now(); finish(out,in);
    previous_s_=out.projected_s; previous_time_=in.time; previous_route_=in.route;
    previous_delta_=out.steering; previous_virtual_=out.virtual_speed; previous_times_=times;
  } catch (const std::exception & error) {
    fail(out,error);
    // Keep measured projection history to prevent cross-branch relocalization;
    // discard failed optimization inputs.
    previous_delta_.clear(); previous_virtual_.clear(); previous_times_.clear();
    if (std::isfinite(out.projected_s)) {
      previous_s_=out.projected_s; previous_time_=in.time; previous_route_=in.route;
    }
  }
  const auto end=Clock::now();
  if (pre==start) {pre=end; planned=end;} else if (planned==start) {planned=end;}
  out.preprocessing_ms=milliseconds(start,pre); out.planning_ms=milliseconds(pre,planned);
  out.postprocessing_ms=milliseconds(planned,end); out.total_ms=milliseconds(start,end);
  return out;
}
}  // namespace imac_ctrl::comparison
