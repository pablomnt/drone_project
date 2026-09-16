#include "drone_core/common/rigid_transform.hpp"

#include <algorithm>
#include <cmath>

namespace drone_core::common {

State transformState(const Eigen::Isometry3d& b_from_a, const State& s) {
  State out = s;
  const Eigen::Matrix3d R = b_from_a.rotation();
  out.pos = b_from_a * s.pos;
  out.vel = R * s.vel;
  const Eigen::Vector3d heading = R * Eigen::Vector3d(std::cos(s.yaw), std::sin(s.yaw), 0.0);
  out.yaw = std::atan2(heading.y(), heading.x());
  return out;
}

MotionState transformMotion(const Eigen::Isometry3d& b_from_a, const MotionState& m) {
  const Eigen::Matrix3d R = b_from_a.rotation();
  MotionState out;
  out.pos = b_from_a * m.pos;
  out.vel = R * m.vel;
  out.acc = R * m.acc;
  out.jerk = R * m.jerk;
  return out;
}

Trajectory transformTrajectory(const Eigen::Isometry3d& b_from_a, const Trajectory& traj) {
  Trajectory out = traj;
  const Eigen::Matrix3d R = b_from_a.rotation();
  const Eigen::Vector3d t = b_from_a.translation();
  for (std::size_t s = 0; s < traj.segment_times.size(); ++s) {
    // The axes are mixed by the rotation, so all three need the same length; a
    // shorter one is a polynomial of lower degree, i.e. zero high-order terms.
    const Eigen::Index n = std::max({traj.coeffs_x[s].size(), traj.coeffs_y[s].size(),
                                     traj.coeffs_z[s].size()});
    const auto coeff = [](const Eigen::VectorXd& c, Eigen::Index k) {
      return k < c.size() ? c[k] : 0.0;
    };
    out.coeffs_x[s] = Eigen::VectorXd::Zero(n);
    out.coeffs_y[s] = Eigen::VectorXd::Zero(n);
    out.coeffs_z[s] = Eigen::VectorXd::Zero(n);
    for (Eigen::Index k = 0; k < n; ++k) {
      Eigen::Vector3d v(coeff(traj.coeffs_x[s], k), coeff(traj.coeffs_y[s], k),
                        coeff(traj.coeffs_z[s], k));
      v = R * v;
      if (k == 0) v += t;
      out.coeffs_x[s][k] = v.x();
      out.coeffs_y[s][k] = v.y();
      out.coeffs_z[s][k] = v.z();
    }
  }
  return out;
}

}  // namespace drone_core::common
