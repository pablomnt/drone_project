#include "drone_core/control/trajectory_tracker.hpp"

#include "drone_core/common/logging.hpp"
#include "drone_core/common/trajectory_eval.hpp"

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

void TrajectoryTracker::setDerivativeTau(double tau) {
  controller_.setDerivativeTau(tau);
}

void TrajectoryTracker::setIntegratorErrorLimit(double max_pos_err) {
  controller_.setIntegratorErrorLimit(max_pos_err);
}

void TrajectoryTracker::enableFeedforward(bool enabled) {
  feedforward_ = enabled;
}

void TrajectoryTracker::reset() {
  controller_.reset();
  mode_ = Mode::kHoverHold;
  // Drop any trajectory across a re-engage. Both are anchored to a wall-clock
  // t0 and were spliced onto a reference that no longer exists, so re-engaging
  // onto one would jump the reference to wherever that curve had got to. A
  // fresh engage must start from the direct setpoint (takeoff) and wait for the
  // planner to solve against the vehicle's actual state.
  has_traj_ = false;
  has_next_ = false;
  traj_ = common::Trajectory{};
  next_ = common::Trajectory{};
  diverged_ = false;
  divergence_event_ = false;
}

void TrajectoryTracker::setDirectSetpoint(const Eigen::Vector3d& pos, double yaw) {
  direct_pos_ = pos;
  direct_yaw_ = yaw;
  has_direct_ = true;
}

void TrajectoryTracker::setTrajectory(const common::Trajectory& traj, double arrival_time) {
  // Staged, not engaged: the outgoing trajectory keeps driving the reference
  // until wall-clock reaches this one's t0 (see the header). Freshness is
  // stamped here, on arrival, so a planner that has stopped producing still
  // trips the stale timeout even while a staged trajectory waits to start.
  next_ = traj;
  has_next_ = true;
  last_arrival_ = arrival_time;
}

void TrajectoryTracker::clearTrajectory() {
  traj_ = common::Trajectory{};
  next_ = common::Trajectory{};
  has_traj_ = false;
  has_next_ = false;
  diverged_ = false;
  divergence_event_ = false;
}

common::Command TrajectoryTracker::update(const common::State& state, double now, double dt) {
  controller_.setState(state.pos, state.vel, state.yaw);
  controller_.setThrustAccel(state.thrust_accel);
  // Lets the D term difference on the estimator's clock instead of the control
  // clock — see PositionControl::setStateStamp.
  controller_.setStateStamp(state.stamp);

  // Promote a staged trajectory once its start instant has arrived. The planner
  // matched position, velocity, acceleration and jerk to the outgoing reference
  // at exactly this instant, so the swap is continuous.
  if (has_next_ && now >= next_.t0) {
    traj_ = next_;
    has_traj_ = true;
    has_next_ = false;
    // A new plan gets a fresh chance; it is checked against the vehicle below
    // before a single reference from it is used.
    diverged_ = false;
    // Seed the held yaw to the vehicle's heading so the commanded yaw does not
    // jump when the new trajectory engages.
    mapper_needs_reset_ = true;
  }

  const bool has_fresh_traj =
      has_traj_ && !traj_.empty() && (now - last_arrival_ <= stale_timeout_);

  // A trajectory can be fresh by the clock and still be the wrong thing to
  // fly: stale_timeout only catches a planner that stopped producing, not a
  // vehicle that has ended up far from where an on-time reference says it
  // should be (a gust, a snag, a bad state estimate). Chasing that reference
  // commands a large correction toward a point the vehicle is not near, so
  // abandon the trajectory instead. Checked before any reference is set, so a
  // diverged trajectory never produces a command. Sampled raw rather than
  // through the flatness mapper, which latches yaw as a side effect.
  if (has_fresh_traj && !diverged_ && max_tracking_error_ > 0.0) {
    const double error = (common::sampleMotion(traj_, now).pos - state.pos).norm();
    if (error > max_tracking_error_) {
      diverged_ = true;
      divergence_event_ = true;
      // Staged onto the reference that just proved wrong: do not engage it.
      has_next_ = false;
      next_ = common::Trajectory{};
      // Hold HERE. Latched explicitly rather than by the hover-hold branch's
      // mode check, which would keep an older hold point if the tracker was
      // already holding when this trajectory was promoted.
      hold_pos_ = state.pos;
      hold_yaw_ = state.yaw;
      mode_ = Mode::kHoverHold;
      DRONE_LOG_ERROR("[track] vehicle " << error << " m from the trajectory reference (limit "
                      << max_tracking_error_ << " m): abandoning the trajectory and holding "
                      "position until a replan from here arrives");
    }
  }
  const bool trust_traj = has_fresh_traj && !diverged_;

  if (trust_traj) {
    // A planner trajectory is available, fresh and being tracked: track it.
    mode_ = Mode::kTracking;
    if (mapper_needs_reset_) {
      mapper_.reset(state.yaw);
      mapper_needs_reset_ = false;
    }
    const common::Reference ref = mapper_.sample(traj_, now);
    controller_.enableFeedforward(feedforward_);
    controller_.setReference(ref);
  } else if (has_traj_) {
    // We were tracking but guidance went stale (planner stalled or dead) or
    // the reference has diverged too far from the measured position: latch
    // the current position and hold rather than chase a reference that is
    // either out of date or physically wrong.
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
