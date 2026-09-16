#include "drone_core/common/trajectory_eval.hpp"

#include <algorithm>
#include <cmath>

namespace drone_core::common {

namespace {

// Evaluate a polynomial (coefficients in ascending power order) or one of its
// derivatives at local time t.
double evalPoly(const Eigen::VectorXd& c, double t, int deriv) {
  double result = 0.0;
  for (int k = deriv; k < c.size(); ++k) {
    double term = c[k];
    for (int d = 0; d < deriv; ++d) {
      term *= (k - d);
    }
    result += term * std::pow(t, k - deriv);
  }
  return result;
}

}  // namespace

Eigen::Vector3d evalTrajectory(const Trajectory& traj, double elapsed, int deriv) {
  if (traj.empty()) return Eigen::Vector3d::Zero();

  // Locate the active segment for the clamped elapsed time.
  const double clamped = std::clamp(elapsed, 0.0, traj.total_duration);
  std::size_t seg = 0;
  double seg_start = 0.0;
  while (seg + 1 < traj.segment_times.size() &&
         seg_start + traj.segment_times[seg] <= clamped) {
    seg_start += traj.segment_times[seg];
    ++seg;
  }
  const double t = clamped - seg_start;

  return Eigen::Vector3d(evalPoly(traj.coeffs_x[seg], t, deriv),
                         evalPoly(traj.coeffs_y[seg], t, deriv),
                         evalPoly(traj.coeffs_z[seg], t, deriv));
}

MotionState sampleMotion(const Trajectory& traj, double now) {
  MotionState s;
  if (traj.empty()) return s;

  const double elapsed = now - traj.t0;
  s.pos = evalTrajectory(traj, elapsed, 0);
  s.vel = evalTrajectory(traj, elapsed, 1);
  s.acc = evalTrajectory(traj, elapsed, 2);
  s.jerk = evalTrajectory(traj, elapsed, 3);
  return s;
}

}  // namespace drone_core::common
