#include "virtual_control/upper_vehicle_rollout.hpp"
#include "QuadraticProblem.h"
#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace imac_ctrl::upper_vehicle_rollout {
namespace {
constexpr double pi=3.14159265358979323846;
double wrap(double a) {return std::atan2(std::sin(a),std::cos(a));}
bool finite(double a) {return std::isfinite(a);}
State add(const State & x,const State & d,double h) {
 return {x.x+h*d.x,x.y+h*d.y,x.yaw+h*d.yaw,x.steering+h*d.steering,x.speed+h*d.speed};
}
State derivative(const Settings & s,const State & x,double command,double acceleration) {
 const double delta=s.actuator_enabled?x.steering:command;
 const double beta=std::atan(s.offset/s.wheelbase*std::tan(delta));
 return {x.speed*std::cos(x.yaw+beta),x.speed*std::sin(x.yaw+beta),
   x.speed*std::cos(beta)*std::tan(delta)/s.wheelbase,
   s.actuator_enabled?(command-x.steering)/s.tau:0.0,acceleration};
}
bool validSettings(const Settings & s) {
 for(double a:{s.dt,s.integration_dt,s.wheelbase,s.offset,s.speed,s.max_acceleration,
   s.max_angle,s.max_rate,s.initial_steering,s.initial_command,s.tau,s.budget_ms}) {if(!finite(a)) {return false;}}
 return s.dt>0 && s.integration_dt>0 && s.integration_dt<=s.dt &&
   s.wheelbase>0 && s.offset>=0 && s.offset<=s.wheelbase && s.speed>0 &&
   s.max_acceleration>0 && s.max_angle>0 && s.max_angle<pi/2 && s.max_rate>0 &&
   std::abs(s.initial_steering)<=s.max_angle+1e-9 && std::abs(s.initial_command)<=s.max_angle+1e-9 &&
   (!s.actuator_enabled || (s.tau>0 && s.integration_dt<=s.tau*.25)) && s.iterations>0 && s.budget_ms>0 &&
   std::ceil(s.dt/s.integration_dt)<=1000;
}
struct Evaluation {Trajectory trajectory;Eigen::VectorXd residual;double position{0},input{0},turn{0},violation{0},preferred{0},steering_change{0};std::vector<double> preferred_violations;};
Evaluation evaluate(const Problem & p,const std::vector<double> & u) {
 Evaluation e;e.trajectory=integrate(p.settings,u);if(!e.trajectory.valid) {return e;}
 const int n=u.size()/2;
 const bool penalize_steering_change=p.steering_change_weight>0;
 e.residual.resize(4*n+2+p.turn_targets.size()+(penalize_steering_change?n:0));int row=0;
 for(int k=0;k<=n;++k) {
  const auto & x=e.trajectory.states[e.trajectory.coarse_indices[k]];
  for(int a=0;a<2;++a) {double r=(a==0?x.x:x.y)-p.position_targets[k][a];r*=std::sqrt(p.position_weights[k]);e.residual[row++]=r;e.position+=.5*r*r;}
 }
 double previous=0;std::vector<double> corrections;
 for(int k=0;k<n;++k) {
  const auto & a=e.trajectory.states[e.trajectory.coarse_indices[k]];
  const auto & b=e.trajectory.states[e.trajectory.coarse_indices[k+1]];
  double heading=std::atan2(b.y-a.y,b.x-a.x);
  double ref_previous=k==0?0:p.reference_headings[k-1];
  double correction=wrap(wrap(heading-previous)-wrap(p.reference_headings[k]-ref_previous));
  corrections.push_back(wrap(heading-p.reference_headings[k]));previous=heading;
  const double rv=std::sqrt(p.speed_weight)*u[2*k],rh=std::sqrt(p.heading_increment_weight)*correction;
  e.residual[row++]=rv;e.residual[row++]=rh;e.input+=.5*(rv*rv+rh*rh);
  if(penalize_steering_change) {
   const double r=std::sqrt(p.steering_change_weight)*u[2*k+1];
   e.residual[row++]=r;e.steering_change+=.5*r*r;e.input+=.5*r*r;
  }
  const auto & c=p.corridors[k];const double projection=c.normal[0]*b.x+c.normal[1]*b.y;
  if(c.constrained) {e.violation=std::max({e.violation,c.lower-projection,projection-c.upper});}
  if(!p.preferred_corridors.empty()) {
   const auto & preferred=p.preferred_corridors[k];
   const double z=preferred.normal[0]*b.x+preferred.normal[1]*b.y;
   const double v=preferred.constrained?std::max({0.0,preferred.lower-z,z-preferred.upper}):0.0;
   e.preferred_violations.push_back(v);
   e.preferred+=p.soft_linear_weight*v+.5*p.soft_quadratic_weight*v*v;
  }
 }
 for(const auto & t:p.turn_targets) {double r=std::sqrt(t.weight)*wrap(corrections[t.step-1]-t.target);e.residual[row++]=r;e.turn+=.5*r*r;}
 return e;
}
}
Trajectory integrate(const Settings & s,const std::vector<double> & inputs) {
 Trajectory out;
 if(!validSettings(s)||inputs.empty()||inputs.size()%2) {out.status="invalid vehicle rollout settings/dimensions";return out;}
 for(double u:inputs) {if(!finite(u)) {out.status="nonfinite rollout input";return out;}}
 const int n=inputs.size()/2,m=static_cast<int>(std::ceil(s.dt/s.integration_dt));const double h=s.dt/m;
 State x{0,0,0,s.initial_steering,s.speed};double command=s.initial_command;
 out.states.push_back(x);out.commands.push_back(command);out.coarse_indices.push_back(0);
 for(int k=0;k<n;++k) {
  const double accel=inputs[2*k]/s.dt,rate=inputs[2*k+1]/s.dt;
  out.max_command_rate=std::max(out.max_command_rate,std::abs(rate));
  for(int j=0;j<m;++j) {
   const double c0=command+rate*(j*h),cm=c0+rate*h*.5,c1=c0+rate*h;
   if(!s.actuator_enabled) {x.steering=c0;}
   const State d1=derivative(s,x,c0,accel);
   const State d2=derivative(s,add(x,d1,h*.5),cm,accel);
   const State d3=derivative(s,add(x,d2,h*.5),cm,accel);
   const State d4=derivative(s,add(x,d3,h),c1,accel);
   for(const auto & a:{d1,d2,d3,d4}) {out.max_effective_rate=std::max(out.max_effective_rate,s.actuator_enabled?std::abs(a.steering):std::abs(rate));}
   x.x+=h/6*(d1.x+2*d2.x+2*d3.x+d4.x);x.y+=h/6*(d1.y+2*d2.y+2*d3.y+d4.y);
   x.yaw+=h/6*(d1.yaw+2*d2.yaw+2*d3.yaw+d4.yaw);x.steering+=h/6*(d1.steering+2*d2.steering+2*d3.steering+d4.steering);
   x.speed+=h*accel;if(!s.actuator_enabled) {x.steering=c1;}
   if(!finite(x.x)||!finite(x.y)||!finite(x.yaw)||!finite(x.steering)||!finite(x.speed)||x.speed<=0) {out.status="invalid vehicle state";return out;}
   out.states.push_back(x);out.commands.push_back(c1);
  }
  command+=inputs[2*k+1];out.coarse_indices.push_back(out.states.size()-1);
 }
 out.valid=true;return out;
}
}  // namespace imac_ctrl::upper_vehicle_rollout

namespace imac_ctrl::upper_vehicle_rollout {
Solution refine(const Problem & p) {
 const auto started=std::chrono::steady_clock::now();Solution result;
 auto elapsed=[&](){return std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();};
 const auto & s=p.settings;const int n=p.corridors.size(),dim=2*n;
 if(!validSettings(s)||n<1||p.position_targets.size()!=static_cast<std::size_t>(n+1)||
   p.position_weights.size()!=static_cast<std::size_t>(n+1)||p.reference_headings.size()!=static_cast<std::size_t>(n)||
   p.seed_path.size()!=static_cast<std::size_t>(n+1)||!finite(p.speed_weight)||p.speed_weight<0||
   !finite(p.heading_increment_weight)||p.heading_increment_weight<0||
   !finite(p.steering_change_weight)||p.steering_change_weight<0) {result.status="invalid refinement problem";return result;}
 for(const auto & t:p.turn_targets) {if(t.step<1||t.step>n||!finite(t.target)||!finite(t.weight)||t.weight<0) {result.status="invalid turn target";return result;}}
 for(double w:p.position_weights) {if(!finite(w)||w<0) {result.status="invalid position weight";return result;}}
 const bool soft=!p.preferred_corridors.empty();
 if((soft&&p.preferred_corridors.size()!=static_cast<std::size_t>(n))||
   !finite(p.soft_linear_weight)||p.soft_linear_weight<0||
   !finite(p.soft_quadratic_weight)||p.soft_quadratic_weight<0) {
  result.status="invalid preferred corridor settings";return result;
 }
 for(const auto * corridors:{&p.corridors,&p.preferred_corridors}) {
  for(const auto & c:*corridors) {
   if(!finite(c.normal[0])||!finite(c.normal[1])||!finite(c.lower)||!finite(c.upper)||
     c.lower>c.upper||std::hypot(c.normal[0],c.normal[1])<1e-9) {
    result.status="invalid corridor bounds";return result;
   }
  }
 }
 if(s.actuator_enabled && std::abs(s.initial_command-s.initial_steering)/s.tau>s.max_rate+1e-6) {
  result.status="initial effective steering already exceeds requested rate";return result;
 }
 if(!s.actuator_enabled && std::abs(s.initial_command-s.initial_steering)>1e-6) {
  result.status="instant actuator initial command/state mismatch";return result;
 }
 std::vector<double> u(dim,0.0);double command=s.initial_command,previous_heading=0;
 for(int k=0;k<n;++k) {
  const double dx=p.seed_path[k+1][0]-p.seed_path[k][0],dy=p.seed_path[k+1][1]-p.seed_path[k][1];
  const double heading=std::atan2(dy,dx),length=std::hypot(dx,dy);
  const double desired=std::atan(s.wheelbase*wrap(heading-previous_heading)/std::max(.01,length));
  const double next=std::clamp(desired,std::max(-s.max_angle,command-s.max_rate*s.dt),std::min(s.max_angle,command+s.max_rate*s.dt));
  u[2*k+1]=next-command;command=next;previous_heading=heading;
  if(p.seed_speed_increments.size()==static_cast<std::size_t>(n)) {u[2*k]=std::clamp(p.seed_speed_increments[k],-s.max_acceleration*s.dt,s.max_acceleration*s.dt);}
 }
 auto controlsValid=[&](const std::vector<double> & v) {
  double cmd=s.initial_command,speed=s.speed;
  for(int k=0;k<n;++k) {cmd+=v[2*k+1];speed+=v[2*k];
   if(!finite(cmd)||!finite(speed)||std::abs(cmd)>s.max_angle+1e-6||speed<.1-1e-6||
    std::abs(v[2*k])>s.max_acceleration*s.dt+1e-6||std::abs(v[2*k+1])>s.max_rate*s.dt+1e-6) {return false;}}
  return true;
 };
 auto remember=[&](const std::vector<double> & v,const Evaluation & e) {
  const double cost=e.position+e.input+e.turn+e.preferred;
  if(!e.trajectory.valid||e.violation>1e-3||!controlsValid(v)||
    e.trajectory.max_effective_rate>s.max_rate+1e-6||e.trajectory.max_command_rate>s.max_rate+1e-6) {return;}
  if(!result.valid||cost<result.objective) {result.valid=true;result.inputs=v;result.trajectory=e.trajectory;
   result.position_cost=e.position;result.input_cost=e.input;result.turn_cost=e.turn;result.objective=cost;result.max_corridor_violation=e.violation;
   result.steering_change_cost=e.steering_change;
   result.preferred_cost=e.preferred;result.preferred_violations=e.preferred_violations;
   result.max_preferred_violation=e.preferred_violations.empty()?0:*std::max_element(e.preferred_violations.begin(),e.preferred_violations.end());}
 };
 auto merit=[&](const Evaluation & e) {
  if(!e.trajectory.valid) {return std::numeric_limits<double>::infinity();}
  double penalty=0;
  for(int k=0;k<n;++k) {if(!p.corridors[k].constrained) {continue;}
   const auto & x=e.trajectory.states[e.trajectory.coarse_indices[k+1]];const auto & c=p.corridors[k];
   const double z=c.normal[0]*x.x+c.normal[1]*x.y,v=std::max({0.0,c.lower-z,z-c.upper});penalty+=1000*v+5000*v*v;}
  return e.position+e.input+e.turn+e.preferred+penalty;
 };
 double damping=1e-4,trust=1.0;int qp_ok=0,qp_failed=0;double closest=std::numeric_limits<double>::infinity();
 for(int iteration=0;iteration<s.iterations;++iteration) {
  result.iterations=iteration+1;
  const auto current=evaluate(p,u);if(!current.trajectory.valid) {result.status=current.trajectory.status;break;}
  closest=std::min(closest,current.violation);remember(u,current);if(elapsed()>s.budget_ms) {break;}
  const int rcount=current.residual.size();Eigen::MatrixXd J(rcount,dim),P(2*n,dim);
  for(int j=0;j<dim;++j) {
   const double epsilon=j%2?1e-5:1e-4;auto up=u;up[j]+=epsilon;
   auto a=evaluate(p,up);const auto & b=current;
   if(!a.trajectory.valid||!b.trajectory.valid) {result.status="invalid finite-difference rollout";result.elapsed_ms=elapsed();return result;}
   J.col(j)=(a.residual-b.residual)/epsilon;
   for(int k=0;k<n;++k) {
    const auto & x=a.trajectory.states[a.trajectory.coarse_indices[k+1]];const auto & y=b.trajectory.states[b.trajectory.coarse_indices[k+1]];
    P(2*k,j)=(x.x-y.x)/epsilon;P(2*k+1,j)=(x.y-y.y)/epsilon;
   }
  }
  if(elapsed()>s.budget_ms) {break;}
  const Eigen::MatrixXd H=J.transpose()*J;const Eigen::VectorXd q=J.transpose()*current.residual;
  const int variables=dim+n+(soft?n:0);Matrix<double> hessian(0.0,variables,variables);Vector<double> linear(0.0,variables);
  for(int i=0;i<dim;++i) {linear[i]=q[i];for(int j=0;j<dim;++j) {hessian[i][j]=H(i,j);}hessian[i][i]+=damping;}
  for(int k=0;k<n;++k) {hessian[dim+k][dim+k]=10000;linear[dim+k]=1000;}
  if(soft) {for(int k=0;k<n;++k) {hessian[dim+n+k][dim+n+k]=p.soft_quadratic_weight+1e-10;linear[dim+n+k]=p.soft_linear_weight;}}
  const int rows=(soft?13:10)*n;
  Matrix<double> A(0.0,rows,variables);Vector<double> b(0.0,rows);int row=0;
  for(int j=0;j<dim;++j) {
   const double maximum=j%2?s.max_rate*s.dt:s.max_acceleration*s.dt,step=j%2?.10*trust:.25*trust;
   A[row][j]=1;b[row++]=std::min(step,maximum-u[j]);A[row][j]=-1;b[row++]=std::min(step,maximum+u[j]);
  }
  double cmd=s.initial_command,speed=s.speed;
  for(int k=0;k<n;++k) {
   cmd+=u[2*k+1];speed+=u[2*k];
   for(int j=0;j<=k;++j) {A[row][2*j+1]=1;}b[row++]=s.max_angle-cmd;
   for(int j=0;j<=k;++j) {A[row][2*j+1]=-1;}b[row++]=s.max_angle+cmd;
   for(int j=0;j<=k;++j) {A[row][2*j]=-1;}b[row++]=speed-.1;
   const auto & c=p.corridors[k];const auto & x=current.trajectory.states[current.trajectory.coarse_indices[k+1]];
   const double projection=c.normal[0]*x.x+c.normal[1]*x.y;
   if(c.constrained) {
    for(int j=0;j<dim;++j) {A[row][j]=c.normal[0]*P(2*k,j)+c.normal[1]*P(2*k+1,j);}
    A[row][dim+k]=-1;b[row++]=c.upper-projection;
    for(int j=0;j<dim;++j) {A[row][j]=-c.normal[0]*P(2*k,j)-c.normal[1]*P(2*k+1,j);}
    A[row][dim+k]=-1;b[row++]=projection-c.lower;
   } else {b[row++]=1;b[row++]=1;}
   A[row][dim+k]=-1;b[row++]=0;
   if(soft) {
    const auto & pref=p.preferred_corridors[k];
    const double z=pref.normal[0]*x.x+pref.normal[1]*x.y;
    if(pref.constrained) {
     for(int j=0;j<dim;++j) {A[row][j]=pref.normal[0]*P(2*k,j)+pref.normal[1]*P(2*k+1,j);}
     A[row][dim+n+k]=-1;b[row++]=pref.upper-z;
     for(int j=0;j<dim;++j) {A[row][j]=-pref.normal[0]*P(2*k,j)-pref.normal[1]*P(2*k+1,j);}
     A[row][dim+n+k]=-1;b[row++]=z-pref.lower;
    } else {b[row++]=1;b[row++]=1;}
    A[row][dim+n+k]=-1;b[row++]=0;
   }
  }
  if(row!=rows) {result.status="refinement row mismatch";break;}
  QuadraticProblem qp(false);Variable * variable=qp.vector_variable(variables,"vehicle_refinement");
  if(!qp.add_variable(variable)) {result.status="refinement add_variable failed";break;}
  Constraint constraint(qp.get_main_variable());constraint.set_constraint_variable(qp.get_variable(variable),A);constraint.set_known_term(b);
  if(!qp.add_leq_constraint(constraint)||!qp.set_Q_matrix(hessian)||!qp.set_q0_vector(linear)) {result.status="refinement QP setup failed";break;}
  Vector<double> step;double cost=NAN;const auto qstart=std::chrono::steady_clock::now();
  try {cost=qp.solve_problem(step);}catch(const std::exception &) {cost=NAN;}
  result.solver_ms+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-qstart).count();
  if(!finite(cost)||step.size()<static_cast<unsigned>(variables)) {++qp_failed;damping*=10;trust*=.5;continue;}
  bool finite_step=true;
  for(int i=0;i<variables;++i) {if(!finite(step[i])) {finite_step=false;break;}}
  for(int i=0;i<row&&finite_step;++i) {double lhs=0;for(int j=0;j<variables;++j) {lhs+=A[i][j]*step[j];}if(lhs>b[i]+1e-5) {finite_step=false;}}
  if(!finite_step) {++qp_failed;damping*=10;trust*=.5;continue;}
  ++qp_ok;bool accepted=false;double norm=0;for(int j=0;j<dim;++j) {norm=std::max(norm,std::abs(step[j]));}
  const double before=merit(current);
  for(double alpha:{1.0,.5,.25,.125,.0625}) {
   auto next=u;for(int j=0;j<dim;++j) {next[j]+=alpha*step[j];}
   auto e=evaluate(p,next);closest=std::min(closest,e.violation);remember(next,e);
   if(merit(e)<before-1e-10) {u=next;accepted=true;damping=std::max(1e-6,damping*.5);break;}
  }
  if(!accepted) {damping*=10;trust*=.5;}
  if(norm<1e-5&&current.violation<=1e-3) {break;}
 }
 result.elapsed_ms=elapsed();
 if(result.valid) {
  const auto & t=result.trajectory;const double initial_course=std::atan(s.offset/s.wheelbase*std::tan(s.initial_steering));
  result.initial_course_error_rad=wrap(std::atan2(t.states[1].y,t.states[1].x)-initial_course);
  result.status="vehicle rollout feasible";
 }else if(result.status.empty()) {result.status=elapsed()>s.budget_ms?"vehicle refinement budget without feasible path":"vehicle refinement has no feasible path";}
 if(!result.valid) {result.max_corridor_violation=closest;result.status+=" closest="+std::to_string(closest)+" qp_ok="+std::to_string(qp_ok)+" qp_failed="+std::to_string(qp_failed);}
 return result;
}
}  // namespace imac_ctrl::upper_vehicle_rollout
