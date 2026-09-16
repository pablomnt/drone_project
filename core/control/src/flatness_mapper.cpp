#include "drone_core/control/flatness_mapper.hpp"

#include <cmath>

#include "drone_core/common/trajectory_eval.hpp"

namespace drone_core::control {

void FlatnessMapper::reset(double initial_yaw) {
  last_yaw_ = initial_yaw;
}

common::Reference FlatnessMapper::sample(const common::Trajectory& traj, double now) {
  common::Reference ref;
  if (traj.empty()) {
    ref.yaw = last_yaw_;
    return ref;
  }

  const common::MotionState m = common::sampleMotion(traj, now);
  ref.pos = m.pos;
  ref.vel_ff = m.vel;
  ref.acc_ff = m.acc;

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
