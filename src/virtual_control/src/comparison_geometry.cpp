#include "virtual_control/comparison_planner.hpp"
#include "virtual_control/egocentric_planner_geometry.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace imac_ctrl::comparison
{
Point rotate(const Point & p, double yaw)
{return {std::cos(yaw)*p.x()-std::sin(yaw)*p.y(), std::sin(yaw)*p.x()+std::cos(yaw)*p.y()};}
bool Grid::valid() const
{
  return width > 0 && height > 0 && std::isfinite(resolution) && resolution > 0 &&
    data.size() == static_cast<size_t>(width)*static_cast<size_t>(height) &&
    origin.allFinite() && frame_world.p.allFinite() && std::isfinite(origin_yaw) &&
    std::isfinite(frame_world.yaw);
}
Point Grid::toGrid(const Point & p) const
{return rotate(rotate(p-frame_world.p, -frame_world.yaw)-origin, -origin_yaw)/resolution;}
void Grid::buildIndex()
{
  if (!valid()) {blocked_prefix.clear(); return;}
  blocked_prefix.assign(static_cast<size_t>(width+1)*(height+1),0);
  for (int y=0; y<height; ++y) {
    uint32_t row=0;
    for (int x=0; x<width; ++x) {
      const int value=data[static_cast<size_t>(y)*width+x];
      row+=egocentric_planner_geometry::gridValueIsDrivable(value,positive_is_drivable,threshold) ? 0 : 1;
      blocked_prefix[static_cast<size_t>(y+1)*(width+1)+x+1]=
        blocked_prefix[static_cast<size_t>(y)*(width+1)+x+1]+row;
    }
  }
}
bool Grid::rectangleKnownFree(const Point & minimum,const Point & maximum) const
{
  if (!minimum.allFinite() || !maximum.allFinite() || minimum.x()<0 || minimum.y()<0 ||
    maximum.x()>=width || maximum.y()>=height ||
    blocked_prefix.size()!=static_cast<size_t>(width+1)*(height+1)) {return false;}
  const int x0=static_cast<int>(minimum.x()),y0=static_cast<int>(minimum.y());
  const int x1=static_cast<int>(maximum.x())+1,y1=static_cast<int>(maximum.y())+1;
  const auto at=[&](int x,int y) {return blocked_prefix[static_cast<size_t>(y)*(width+1)+x];};
  return at(x1,y1)+at(x0,y0)==at(x0,y1)+at(x1,y0);
}
Point Grid::cellCenter(int x, int y) const
{
  return frame_world.p+rotate(origin+rotate(
    Point((x+0.5)*resolution, (y+0.5)*resolution), origin_yaw), frame_world.yaw);
}
Cell Grid::cell(const Point & p) const
{
  if (!valid() || !p.allFinite()) {return Cell::Outside;}
  const Point q = toGrid(p);
  if (q.x() < 0 || q.y() < 0 || q.x() >= width || q.y() >= height) {return Cell::Outside;}
  const int v = data[static_cast<size_t>(std::floor(q.y()))*width+
    static_cast<size_t>(std::floor(q.x()))];
  if (v < 0) {return Cell::Unknown;}
  return egocentric_planner_geometry::gridValueIsDrivable(v, positive_is_drivable, threshold) ?
    Cell::Free : Cell::Occupied;
}
ReferenceCurve::ReferenceCurve(const std::vector<Point> & input)
{
  for (const auto & p : input) {
    if (!p.allFinite()) {throw std::invalid_argument("nonfinite reference");}
    if (points_.empty() || (p-points_.back()).norm() > 1e-6) {points_.push_back(p);}
  }
  if (points_.size() < 2) {throw std::invalid_argument("reference needs two distinct points");}
  for (size_t i = 0; i < points_.size(); ++i) {
    tangents_.push_back(i == 0 ? points_[1]-points_[0] : i+1 == points_.size() ?
      points_[i]-points_[i-1] : Point(0.5*(points_[i+1]-points_[i-1])));
  }
  arc_.push_back(0); parameters_.push_back(0);
  Point last = points_[0];
  for (size_t i = 0; i+1 < points_.size(); ++i) {
    const int count = std::max(2, static_cast<int>(std::ceil(
      ((points_[i+1]-points_[i]).norm()+tangents_[i].norm()+tangents_[i+1].norm())/0.05)));
    for (int j = 1; j <= count; ++j) {
      const double u = i+static_cast<double>(j)/count;
      const Point p = evaluate(u).p;
      if ((p-last).norm() > 1e-10) {
        arc_.push_back(arc_.back()+(p-last).norm()); parameters_.push_back(u);
      }
      last = p;
    }
  }
}
CurveSample ReferenceCurve::evaluate(double u) const
{
  u = std::clamp(u, 0.0, static_cast<double>(points_.size()-1));
  const size_t i = std::min(static_cast<size_t>(u), points_.size()-2);
  const double t=u-i, t2=t*t, t3=t2*t;
  const Point & a=points_[i], & b=points_[i+1], & m=tangents_[i], & n=tangents_[i+1];
  const Point p=(2*t3-3*t2+1)*a+(t3-2*t2+t)*m+(-2*t3+3*t2)*b+(t3-t2)*n;
  const Point d=(6*t2-6*t)*a+(3*t2-4*t+1)*m+(-6*t2+6*t)*b+(3*t2-2*t)*n;
  const Point dd=(12*t-6)*a+(6*t-4)*m+(-12*t+6)*b+(6*t-2)*n;
  const double norm=std::max(1e-10, d.norm());
  return {p, d/norm, (d.x()*dd.y()-d.y()*dd.x())/(norm*norm*norm)};
}
CurveSample ReferenceCurve::at(double s) const
{
  s=std::clamp(s, 0.0, length());
  auto it=std::upper_bound(arc_.begin(), arc_.end(), s);
  const size_t k=std::clamp<size_t>(it-arc_.begin(), 1, arc_.size()-1);
  const double ratio=(s-arc_[k-1])/(arc_[k]-arc_[k-1]);
  return evaluate(parameters_[k-1]+ratio*(parameters_[k]-parameters_[k-1]));
}
double ReferenceCurve::project(const Point & p, double lo, double hi) const
{
  lo=std::clamp(lo, 0.0, length()); hi=std::clamp(hi, lo, length());
  double best=lo, dist=(at(lo).p-p).squaredNorm();
  auto consider=[&](double s) {
      const double d=(at(s).p-p).squaredNorm();
      if (d < dist) {dist=d; best=s;}
    };
  // Projection remains inside the caller's reachable progress window.
  for (double s=lo; s < hi; s+=0.1) {consider(s);}
  consider(hi);
  double a=std::max(lo, best-0.1), b=std::min(hi, best+0.1);
  for (int i=0; i<24; ++i) {
    const double c=a+(b-a)/3, d=b-(b-a)/3;
    if ((at(c).p-p).squaredNorm() < (at(d).p-p).squaredNorm()) {b=d;} else {a=c;}
  }
  consider(0.5*(a+b)); return best;
}
bool footprintFree(const Input & in, const Pose & pose)
{
  if (!in.grid.valid() || !pose.p.allFinite() || !std::isfinite(pose.yaw)) {return false;}
  // Half-cell diagonal padding makes sampled checking conservative at cell corners.
  const double padding=in.settings.extra_margin+in.grid.resolution/std::sqrt(2.0);
  const double hx=0.5*in.geometry.length+padding, hy=0.5*in.geometry.width+padding;
  Point minimum=Point::Constant(std::numeric_limits<double>::infinity());
  Point maximum=-minimum;
  for (int sx : {-1,1}) {
    for (int sy : {-1,1}) {
      const Point q=in.grid.toGrid(pose.p+rotate(Point(sx*hx,sy*hy),pose.yaw));
      minimum=minimum.cwiseMin(q); maximum=maximum.cwiseMax(q);
    }
  }
  // Constant-time sufficient test for entirely known-free bounding boxes.
  // Mixed / clipped boxes still use the original oriented footprint check.
  if (in.grid.rectangleKnownFree(minimum,maximum)) {return true;}
  const double step=in.grid.resolution*0.5;
  const int nx=std::max(1, static_cast<int>(std::ceil(2*hx/step)));
  const int ny=std::max(1, static_cast<int>(std::ceil(2*hy/step)));
  for (int i=0; i<=nx; ++i) {
    for (int j=0; j<=ny; ++j) {
      const Point p=pose.p+rotate(Point(-hx+2*hx*i/nx, -hy+2*hy*j/ny), pose.yaw);
      const Cell c=in.grid.cell(p);
      if (c == Cell::Free) {continue;}
      // A forward-only BEV cannot see the footprint already occupied by ego.
      // Exemption applies ONLY to unknown/outside samples inside that initial
      // padded footprint. Known occupied samples are never exempted.
      const Point initial=rotate(p-in.vehicle.p, -in.vehicle.yaw);
      if (in.settings.allow_initial_unobserved_footprint && c != Cell::Occupied &&
        std::abs(initial.x()) <= hx+1e-8 && std::abs(initial.y()) <= hy+1e-8) {continue;}
      return false;
    }
  }
  return true;
}
bool connectionFree(const Input & in, const Pose & a, const Pose & b)
{
  const double angle=std::atan2(std::sin(b.yaw-a.yaw), std::cos(b.yaw-a.yaw));
  const double swept=(b.p-a.p).norm()+std::abs(angle)*0.5*
    std::hypot(in.geometry.length, in.geometry.width);
  const int count=std::max(1, static_cast<int>(std::ceil(swept/(in.grid.resolution*0.5))));
  for (int i=0; i<=count; ++i) {
    const double t=static_cast<double>(i)/count;
    if (!footprintFree(in, {a.p+t*(b.p-a.p), a.yaw+t*angle})) {return false;}
  }
  return true;
}
std::optional<Point> corridor(const Input & in, const CurveSample & ref)
{
  const double yaw=std::atan2(ref.tangent.y(), ref.tangent.x());
  if (!footprintFree(in, {ref.p, yaw})) {return std::nullopt;}
  const Point normal(-ref.tangent.y(), ref.tangent.x());
  const double step=std::min(0.1, in.grid.resolution*0.5);
  Point bounds(0, 0);
  for (int side : {-1, 1}) {
    double last=0;
    for (double d=step; d<=in.settings.corridor_scan; d+=step) {
      if (!footprintFree(in, {ref.p+side*d*normal, yaw})) {break;}
      last=d;
    }
    bounds[side == -1 ? 0 : 1]=side*last;
  }
  return bounds;
}
bool densify(const std::vector<Point> & sparse, double spacing,
  std::vector<Point> & dense, std::vector<double> & s)
{
  dense.clear(); s.clear();
  if (!std::isfinite(spacing) || spacing <= 0 || sparse.size()<2) {return false;}
  std::vector<Point> filtered;
  std::vector<double> arc;
  for (const auto & p : sparse) {
    if (!p.allFinite()) {return false;}
    if (filtered.empty() || (p-filtered.back()).norm()>1e-8) {
      arc.push_back(filtered.empty() ? 0 : arc.back()+(p-filtered.back()).norm());
      filtered.push_back(p);
    }
  }
  if (filtered.size()<2) {return false;}
  size_t segment=0;
  for (double q=0; q<arc.back(); q+=spacing) {
    while (segment+1<arc.size()-1 && arc[segment+1]<q) {++segment;}
    dense.push_back(filtered[segment]+(q-arc[segment])/(arc[segment+1]-arc[segment])*
      (filtered[segment+1]-filtered[segment]));
    s.push_back(q);
  }
  if ((dense.back()-filtered.back()).norm()>1e-6) {
    dense.push_back(filtered.back()); s.push_back(arc.back());
  }
  return true;
}
}  // namespace imac_ctrl::comparison
