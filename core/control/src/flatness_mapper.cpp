#include "drone_core/control/flatness_mapper.hpp"

#include <algorithm>
#include <cmath>

namespace drone_core::control {

namespace {

// Evaluate a polynomial (coefficients in ascending power order) or one of its
// derivatives at local time t. deriv = 0 gives position, 1 velocity, 2 acceleration.
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

void FlatnessMapper::reset(double initial_yaw) {
  last_yaw_ = initial_yaw;
}

common::Reference FlatnessMapper::sample(const common::Trajectory& traj, double now) {
  common::Reference ref;
  if (traj.empty()) {
    ref.yaw = last_yaw_;
    return ref;
  }

  // Locate the active segment for the clamped elapsed time.
  const double elapsed = std::clamp(now - traj.t0, 0.0, traj.total_duration);
  size_t seg = 0;
  double seg_start = 0.0;
  while (seg + 1 < traj.segment_times.size() &&
         seg_start + traj.segment_times[seg] <= elapsed) {
    seg_start += traj.segment_times[seg];
    ++seg;
  }
  const double t = elapsed - seg_start;

  ref.pos = Eigen::Vector3d(evalPoly(traj.coeffs_x[seg], t, 0),
                            evalPoly(traj.coeffs_y[seg], t, 0),
                            evalPoly(traj.coeffs_z[seg], t, 0));
  ref.vel_ff = Eigen::Vector3d(evalPoly(traj.coeffs_x[seg], t, 1),
                               evalPoly(traj.coeffs_y[seg], t, 1),
                               evalPoly(traj.coeffs_z[seg], t, 1));
  ref.acc_ff = Eigen::Vector3d(evalPoly(traj.coeffs_x[seg], t, 2),
                               evalPoly(traj.coeffs_y[seg], t, 2),
                               evalPoly(traj.coeffs_z[seg], t, 2));

  // Yaw follows the direction of travel so the forward camera leads the motion.
  // The heading turn rate is available analytically from velocity and
  // acceleration, so the "spinning while slow" guard needs no stored history.
  const double vx = ref.vel_ff.x();
  const double vy = ref.vel_ff.y();
  const double speed_xy = std::hypot(vx, vy);

  if (speed_xy < params_.speed_eps) {
    // Standstill: heading is undefined, hold what we had.
    ref.yaw = last_yaw_;
  } else {
    const double heading = std::atan2(vy, vx);
    const double ax = ref.acc_ff.x();
    const double ay = ref.acc_ff.y();
    const double heading_rate = (vx * ay - vy * ax) / (speed_xy * speed_xy);

    const bool low_speed = speed_xy < params_.speed_min;
    const bool spinning = std::abs(heading_rate) > params_.heading_rate_max;
    if (low_speed && spinning) {
      // Decelerating into a waypoint where the heading whips around: hold yaw
      // rather than chase it.
      ref.yaw = last_yaw_;
    } else {
      ref.yaw = heading;
      last_yaw_ = heading;
    }
  }

  return ref;
}

}  // namespace drone_core::control
