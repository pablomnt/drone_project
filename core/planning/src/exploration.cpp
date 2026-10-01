#include "drone_core/planning/exploration.hpp"

#include <algorithm>
#include <cmath>

namespace drone_core::planning {

std::optional<Eigen::Vector3d> findExitPoint(const std::vector<Eigen::Vector3d>& path,
                                             const PointTest& is_unknown, double step) {
  if (path.empty() || !is_unknown) return std::nullopt;
  if (is_unknown(path.front())) return path.front();
  step = std::max(step, 1e-3);
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const Eigen::Vector3d d = path[i + 1] - path[i];
    const int n = std::max(1, static_cast<int>(std::ceil(d.norm() / step)));
    for (int k = 1; k <= n; ++k) {
      const Eigen::Vector3d p = path[i] + d * (static_cast<double>(k) / n);
      if (is_unknown(p)) return p;
    }
  }
  return std::nullopt;
}

std::vector<Viewpoint> viewpointCandidates(const Eigen::Vector3d& exit,
                                           const Eigen::Vector3d& drone,
                                           const PointTest& valid,
                                           const PointTest& see_through,
                                           const ViewpointParams& p) {
  std::vector<Viewpoint> out;
  if (!valid || !see_through) return out;
  const double tan_elev = std::tan(p.max_elevation_deg * M_PI / 180.0);
  const int steps = std::max(1, p.azimuth_steps);
  // Distances from the ideal outward, then heights from level outward.
  std::vector<double> dists;
  for (double d = p.min_distance; d <= p.max_distance + 1e-9; d += 0.5) dists.push_back(d);
  const double dz_values[] = {0.0, 0.5, -0.5, 1.0, -1.0};

  const auto sightClear = [&](const Eigen::Vector3d& from) {
    const Eigen::Vector3d d = exit - from;
    const int n = std::max(1, static_cast<int>(std::ceil(d.norm() / std::max(p.los_step, 1e-3))));
    // The exit point itself is in unknown space by definition; stop short of it.
    for (int k = 1; k < n; ++k) {
      if (!see_through(from + d * (static_cast<double>(k) / n))) return false;
    }
    return true;
  };

  for (const double dist : dists) {
    for (const double dz : dz_values) {
      const double horiz2 = dist * dist - dz * dz;
      if (horiz2 <= 0.0) continue;
      const double horiz = std::sqrt(horiz2);
      if (std::abs(dz) > tan_elev * horiz) continue;
      for (int a = 0; a < steps; ++a) {
        const double az = 2.0 * M_PI * a / steps;
        const Eigen::Vector3d pos(exit.x() + horiz * std::cos(az), exit.y() + horiz * std::sin(az),
                                  exit.z() + dz);
        if (!valid(pos) || !sightClear(pos)) continue;
        Viewpoint v;
        v.pos = pos;
        v.yaw = std::atan2(exit.y() - pos.y(), exit.x() - pos.x());
        v.score = std::abs(dist - p.distance) + p.drone_weight * (pos - drone).norm();
        out.push_back(v);
      }
    }
  }
  std::stable_sort(out.begin(), out.end(),
                   [](const Viewpoint& a, const Viewpoint& b) { return a.score < b.score; });
  return out;
}

}  // namespace drone_core::planning
