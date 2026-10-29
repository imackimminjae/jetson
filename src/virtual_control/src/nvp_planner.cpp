#include "comparison_internal.hpp"
#include <limits>

namespace imac_ctrl::comparison
{
namespace
{
// Exact separable squared Euclidean distance transform, with an occupied border.
void edt1d(const std::vector<double> & f, std::vector<double> & d)
{
  const int n=static_cast<int>(f.size());
  std::vector<int> v(n); std::vector<double> z(n+1);
  int k=0; v[0]=0; z[0]=-1e30; z[1]=1e30;
  for (int q=1; q<n; ++q) {
    double s;
    do {
      const int p=v[k]; s=((f[q]+q*q)-(f[p]+p*p))/(2.0*(q-p));
      if (s<=z[k]) {--k;} else {break;}
    } while (k>=0);
    ++k; v[k]=q; z[k]=s; z[k+1]=1e30;
  }
  k=0;
  for (int q=0; q<n; ++q) {
    while (z[k+1]<q) {++k;}
    d[q]=(q-v[k])*(q-v[k])+f[v[k]];
  }
}
class DistanceField
{
public:
  explicit DistanceField(const Grid & grid) : grid_(grid), w_(grid.width+2), h_(grid.height+2)
  {
    std::vector<double> temp(w_*h_,0); values_.resize(w_*h_);
    for (int y=1; y<h_-1; ++y) {
      for (int x=1; x<w_-1; ++x) {
        temp[y*w_+x]=grid.cell(grid.cellCenter(x-1,y-1))==Cell::Free ? 1e12 : 0;
      }
    }
    std::vector<double> f(h_),d(h_);
    for (int x=0; x<w_; ++x) {
      for (int y=0; y<h_; ++y) {f[y]=temp[y*w_+x];}
      edt1d(f,d);
      for (int y=0; y<h_; ++y) {values_[y*w_+x]=d[y];}
    }
    f.resize(w_); d.resize(w_);
    for (int y=0; y<h_; ++y) {
      for (int x=0; x<w_; ++x) {f[x]=values_[y*w_+x];}
      edt1d(f,d);
      for (int x=0; x<w_; ++x) {values_[y*w_+x]=d[x];}
    }
  }
  double distance(const Point & world) const
  {
    if (grid_.cell(world)!=Cell::Free) {return 0;}
    const Point q=grid_.toGrid(world);
    const int x=static_cast<int>(q.x())+1,y=static_cast<int>(q.y())+1;
    // Account for both query and occupied cell half diagonals.
    return std::max(0.0, (std::sqrt(values_[y*w_+x])-std::sqrt(2.0))*grid_.resolution);
  }
private:
  const Grid & grid_; int w_,h_; std::vector<double> values_;
};
}  // namespace
void NvpPlanner::reset() {previous_s_.reset(); previous_route_.clear(); previous_time_=0;}
Output NvpPlanner::plan(const Input & request)
{
  const auto start=Clock::now(); auto pre=start, planned=start;
  Input in=request;
  Output out;
  try {
    validate(in);
    in.grid.buildIndex();
    if (!sameRoute(previous_route_, in.route) || in.time<previous_time_) {reset();}
    ReferenceCurve curve(in.route);
    out.projected_s=projectProgress(in,curve,previous_s_,previous_time_);
    if (curve.length()-out.projected_s<=0.5 &&
      (curve.at(curve.length()).p-in.vehicle.p).norm()<=0.5) {
      out.goal_reached=true; throw std::runtime_error("open path goal reached");
    }
    previous_s_=out.projected_s; previous_time_=in.time; previous_route_=in.route;
    const auto & c=in.settings;
    // Select an actual supplied waypoint ahead of ego; sparse links express intent.
    Point goal=in.route.back();
    const double lookahead=c.prediction_speed*c.dt*c.steps;
    double accumulated=0;
    for (size_t i=1; i<in.route.size(); ++i) {
      accumulated+=(in.route[i]-in.route[i-1]).norm();
      if (accumulated>out.projected_s+lookahead) {goal=in.route[i]; break;}
    }
    const double radius=std::min(lookahead, (goal-in.vehicle.p).norm());
    const bool terminal=(goal-in.route.back()).norm()<1e-6 &&
      (goal-in.vehicle.p).norm()<=std::max(c.min_length,2*c.ring_step) && radius>0.5;
    if (terminal) {in.settings.min_length=std::min(c.min_length,0.5*radius);}
    const double ring_step=terminal ? std::min(c.ring_step,radius/2) : c.ring_step;
    const int rings=static_cast<int>(std::floor(radius/ring_step+1e-8));
    if (rings<2) {throw std::runtime_error("NVP insufficient ring horizon");}
    DistanceField distance(in.grid);
    pre=Clock::now();
    std::vector<std::vector<Point>> valleys;
    const int count=std::max(3, static_cast<int>(std::ceil(2*c.arc_half_angle/c.angular_step))+1);
    for (int ring=rings; ring>=1; --ring) {
      const double r=ring*ring_step;
      std::vector<Point> points; std::vector<double> cost;
      for (int i=0; i<count; ++i) {
        const double angle=-c.arc_half_angle+2*c.arc_half_angle*i/(count-1);
        Point p=in.vehicle.p+rotate(Point(r*std::cos(angle),r*std::sin(angle)),in.vehicle.yaw);
        points.push_back(p);
        const double clearance=distance.distance(p);
        cost.push_back(clearance>0 ?
          c.repulsion/std::pow(clearance,c.repulsion_power)-
          c.attraction/std::pow(std::max(in.grid.resolution,(p-goal).norm()),c.attraction_power) :
          std::numeric_limits<double>::infinity());
      }
      std::vector<Point> candidates;
      // Local minima of the one-dimensional ring potential, requiring a small
      // tangential derivative. Flat connected valley bands contribute one minimum.
      int best=-1;
      auto flush=[&]() {if (best>=0) {candidates.push_back(points[best]); best=-1;}};
      for (int i=1; i<count-1; ++i) {
        const double grad=std::abs(cost[i+1]-cost[i-1])/
          (4*r*c.arc_half_angle/(count-1));
        if (std::isfinite(cost[i]) && std::isfinite(grad) && grad<=c.valley_gradient) {
          if (best<0 || cost[i]<cost[best]) {best=i;}
        } else {flush();}
      }
      flush(); out.candidates+=static_cast<int>(candidates.size());
      if (candidates.empty()) {throw std::runtime_error("NVP no valley on ring");}
      valleys.push_back(std::move(candidates));
    }
    std::vector<Point> reverse;
    Point anchor=goal;
    for (const auto & candidates : valleys) {
      auto ordered=candidates;
      std::sort(ordered.begin(),ordered.end(),[&](const Point & a,const Point & b) {
          return (a-anchor).squaredNorm()<(b-anchor).squaredNorm();});
      bool found=false;
      for (const auto & p : ordered) {
        if (reverse.empty()) {reverse.push_back(p); anchor=p; found=true; break;}
        const double yaw=std::atan2(anchor.y()-p.y(),anchor.x()-p.x());
        const Point forward=rotate(anchor-p,-in.vehicle.yaw);
        if ((p-anchor).norm()<=c.max_connection && forward.x()>0 &&
          connectionFree(in,{p,yaw},{anchor,yaw})) {
          reverse.push_back(p); anchor=p; found=true; break;
        }
      }
      if (!found) {throw std::runtime_error("NVP disconnected/colliding ring connector");}
    }
    out.sparse.push_back(in.vehicle.p);
    for (auto it=reverse.rbegin(); it!=reverse.rend(); ++it) {out.sparse.push_back(*it);}
    const double first_yaw=std::atan2(out.sparse[1].y()-in.vehicle.p.y(),
      out.sparse[1].x()-in.vehicle.p.x());
    if (!connectionFree(in,in.vehicle,{out.sparse[1],first_yaw})) {
      throw std::runtime_error("NVP start connector collision");
    }
    planned=Clock::now(); finish(out,in);
  } catch (const std::exception & error) {fail(out,error);}
  const auto end=Clock::now();
  if (pre==start) {pre=end; planned=end;} else if (planned==start) {planned=end;}
  out.preprocessing_ms=milliseconds(start,pre); out.planning_ms=milliseconds(pre,planned);
  out.postprocessing_ms=milliseconds(planned,end); out.total_ms=milliseconds(start,end);
  return out;
}
}  // namespace imac_ctrl::comparison
