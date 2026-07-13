#include "drone_core/control/trajectory_tracker.hpp"

namespace drone_core::control {

void TrajectoryTracker::setPositionGains(const Eigen::Vector3d& P) {
  controller_.setPositionGains(P);
}

void TrajectoryTracker::setVelocityGains(const Eigen::Vector3d& P, const Eigen::Vector3d& I, const Eigen::Vector3d& D) {
  controller_.setVelocityGains(P, I, D);
}

void TrajectoryTracker::setHoverThrust(double hover_thrust) {
  controller_.setHoverThrust(hover_thrust);
}

void TrajectoryTracker::enableFeedforward(bool enabled) {
  feedforward_ = enabled;
}

void TrajectoryTracker::reset() {
  controller_.reset();
  mode_ = Mode::kHoverHold;
}

void TrajectoryTracker::setDirectSetpoint(const Eigen::Vector3d& pos, double yaw) {
  direct_pos_ = pos;
  direct_yaw_ = yaw;
  has_direct_ = true;
}

void TrajectoryTracker::setTrajectory(const common::Trajectory& traj, double arrival_time) {
  traj_ = traj;
  has_traj_ = true;
  last_arrival_ = arrival_time;
  // Seed the held yaw to the vehicle's heading on the next update so the
  // commanded yaw does not jump when a new trajectory is engaged.
  mapper_needs_reset_ = true;
}

common::Command TrajectoryTracker::update(const common::State& state, double now, double dt) {
  controller_.setState(state.pos, state.vel, state.yaw);
  controller_.setCurrentAcceleration(state.acc);

  const bool has_fresh_traj =
      has_traj_ && !traj_.empty() && (now - last_arrival_ <= stale_timeout_);

  if (has_fresh_traj) {
    // A planner trajectory is available and fresh: track it.
    mode_ = Mode::kTracking;
    if (mapper_needs_reset_) {
      mapper_.reset(state.yaw);
      mapper_needs_reset_ = false;
    }
    const common::Reference ref = mapper_.sample(traj_, now);
    controller_.enableFeedforward(feedforward_);
    controller_.setReference(ref);
  } else if (has_traj_) {
    // We were tracking but guidance went stale (planner stalled or dead):
    // latch the current position and hold.
    if (mode_ != Mode::kHoverHold) {
      hold_pos_ = state.pos;
      hold_yaw_ = state.yaw;
      mode_ = Mode::kHoverHold;
    }
    common::Reference hold;
    hold.pos = hold_pos_;
    hold.yaw = hold_yaw_;
    controller_.enableFeedforward(false);
    controller_.setReference(hold);
  } else if (has_direct_) {
    // No trajectory yet: follow the explicit commanded setpoint (takeoff,
    // manual hover).
    mode_ = Mode::kDirect;
    common::Reference direct;
    direct.pos = direct_pos_;
    direct.yaw = direct_yaw_;
    controller_.enableFeedforward(false);
    controller_.setReference(direct);
  } else {
    // Nothing commanded at all: hold position.
    if (mode_ != Mode::kHoverHold) {
      hold_pos_ = state.pos;
      hold_yaw_ = state.yaw;
      mode_ = Mode::kHoverHold;
    }
    common::Reference hold;
    hold.pos = hold_pos_;
    hold.yaw = hold_yaw_;
    controller_.enableFeedforward(false);
    controller_.setReference(hold);
  }

  controller_.update(dt);

  common::Command cmd;
  cmd.attitude = controller_.getAttitudeSetpoint();
  cmd.thrust = controller_.getThrustSetpoint();
  return cmd;
}

}  // namespace drone_core::control
