#pragma once

#include <Eigen/Core>

#include <cmath>
#include <string>
#include <vector>

namespace imac_ctrl
{

struct LowerPathGeometry
{
  std::vector<Eigen::Vector2d> points;
  std::vector<double> s;
  std::vector<double> psi;
  std::vector<double> kappa;
  bool original_s{false};
  bool valid{false};
  std::string status;
};

// A supplied coordinate is never reconstructed from the dense chord lengths.
// nullptr means an external polyline, whose own chord lengths define s.
inline LowerPathGeometry prepareLowerPath(
  const std::vector<Eigen::Vector2d> & points,
  const std::vector<double> * original_s = nullptr)
{
  LowerPathGeometry path;
  path.original_s = original_s != nullptr;
  if (original_s && original_s->size() != points.size()) {
    path.status = "path/s size mismatch";
    return path;
  }
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (!points[i].allFinite() || (original_s && !std::isfinite((*original_s)[i]))) {
      path.status = "non-finite path/s";
      return path;
    }
    // Remove only consecutive duplicates, including the matching s entry.
    if (!path.points.empty() && (points[i] - path.points.back()).norm() <= 1e-9) {
      continue;
    }
    const double s = original_s ? (*original_s)[i] :
      (path.points.empty() ? 0.0 : path.s.back() + (points[i] - path.points.back()).norm());
    if (!std::isfinite(s) || (!path.s.empty() && s <= path.s.back())) {
      path.status = "non-increasing path s";
      return path;
    }
    path.points.push_back(points[i]);
    path.s.push_back(s);
  }
  const std::size_t count = path.points.size();
  if (count < 2) {
    path.status = "fewer than two distinct path points";
    return path;
  }
  path.psi.resize(count);
  path.kappa.resize(count);
  for (std::size_t i = 0; i + 1 < count; ++i) {
    const Eigen::Vector2d segment = path.points[i + 1] - path.points[i];
    double yaw = std::atan2(segment.y(), segment.x());
    if (i > 0) {
      const double difference = yaw - path.psi[i - 1];
      yaw = path.psi[i - 1] + std::atan2(std::sin(difference), std::cos(difference));
    }
    path.psi[i] = yaw;
  }
  path.psi.back() = path.psi[count - 2];
  for (std::size_t i = 0; i < count; ++i) {
    const std::size_t left = i == 0 ? 0 : i - 1;
    const std::size_t right = i + 1 < count ? i + 1 : i;
    path.kappa[i] = (path.psi[right] - path.psi[left]) / (path.s[right] - path.s[left]);
    if (!std::isfinite(path.kappa[i])) {
      path.status = "non-finite path curvature";
      return path;
    }
  }
  path.valid = true;
  path.status = path.original_s ? "original interpolation s" : "external polyline chord s";
  return path;
}

}  // namespace imac_ctrl
