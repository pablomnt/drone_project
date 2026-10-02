#pragma once

#include "drone_core/common/types.hpp"
#include "drone_core/control/flatness_mapper.hpp"
#include "drone_core/control/position_control.hpp"

namespace drone_core::control {

// Drives the position controller from a trajectory and supervises it. While a
// healthy trajectory is available it samples the flatness reference and tracks
// it; if health signals stop arriving (a stalled or dead planner) it latches the
// current position and holds, so loss of guidance degrades to a safe hover
// rather than tracking an unchecked path or losing the control stream.
class TrajectoryTracker {
public:
  // kDirect    - hold an explicit commanded setpoint (manual hover, takeoff).
  // kTracking  - follow a planner trajectory.
  // kHoverHold - failsafe: latch the current position when guidance is lost.
  enum class Mode { kDirect, kTracking, kHoverHold };

  TrajectoryTracker() = default;

  // Forward controller configuration.
  void setPositionGains(const Eigen::Vector3d& P);
  void setVelocityGains(const Eigen::Vector3d& P, const Eigen::Vector3d& I, const Eigen::Vector3d& D);
  void setHoverThrust(double hover_thrust);
  void setDerivativeTau(double tau);
  void setIntegratorErrorLimit(double max_pos_err);
  void enableFeedforward(bool enabled);

  // Seconds without a health signal before falling back to hover-hold. A health
  // signal is a newly installed trajectory (setTrajectory) or keepFresh(): the
  // planner's trajectory monitor calls the latter every tick it re-checks the
  // trajectory against the current map and finds it safe, and stops while that
  // trajectory is unsafe or superseded by a new plan.
  void setHealthTimeout(double seconds) { health_timeout_ = seconds; }

  // Distance [m] between the tracked reference and the measured position above
  // which the trajectory is abandoned for a hover-hold, even though it is still
  // healthy by setHealthTimeout — see isDiverged(). <= 0 disables the check.
  void setMaxTrackingError(double meters) { max_tracking_error_ = meters; }

  // Re-arm the controller (and takeoff logic) on (re)engagement.
  void reset();

  // Command an explicit position/yaw setpoint. Used for takeoff and manual
  // hover before any planner goal is active; a fresh planner trajectory takes
  // precedence over it.
  void setDirectSetpoint(const Eigen::Vector3d& pos, double yaw);

  // Install a freshly planned trajectory, stamped with its arrival time.
  //
  // The trajectory does NOT take effect immediately: it is held until wall-clock
  // reaches its own `t0`, and the outgoing trajectory keeps driving the
  // reference until then. The planner solves a replan against the state the
  // vehicle will be in at `t0` (a lead time ahead, covering the solve), so
  // switching at exactly `t0` is what makes the splice continuous — switching
  // early would jump the reference to a point the vehicle has not reached yet.
  // A `t0` already in the past (the solve overran its lead) takes effect on the
  // next update, which is the graceful-degradation case.
  void setTrajectory(const common::Trajectory& traj, double arrival_time);

  // Health signal: re-stamp the installed trajectory's freshness to `now` without
  // re-staging it.
  // The trajectory is evaluated in absolute wall-clock time (mapper.sample uses
  // now vs traj_.t0), so a single long trajectory keeps playing correctly; the
  // only thing the stale timeout would otherwise trip is the planner-death
  // failsafe. This lets a caller that knows the trajectory is still valid (the
  // preset one-shot square, which is deliberately solved once and never
  // replanned) hold kTracking for the whole duration instead of falling to
  // hover-hold after stale_timeout. Does nothing useful unless a trajectory is
  // installed. Control thread only, like setTrajectory/update.
  //
  // Ignored while the tracker is holding, diverged or emergency-stopped: only a
  // newly installed trajectory ends those. A health signal sent just before the
  // tracker stopped following (the monitor and the control thread run apart) would
  // otherwise arrive after it and bring the abandoned trajectory back to life
  // for a tick — a step of the reference (scratch run 2026-09-29: 0.5 m).
  void keepFresh(double now) {
    if (mode_ == Mode::kHoverHold || diverged_ || stopped_) return;
    last_arrival_ = now;
  }

  // Emergency stop: on the next update, stop following the trajectory at once and
  // hold the vehicle's current position, without waiting for the health timeout,
  // and drop any trajectory staged to follow it. The trajectory monitor raises it
  // when the trajectory runs too close to an obstacle within the next couple of
  // seconds — sooner than a replacement could engage. LATCHED like a divergence:
  // only a newly promoted trajectory, clearTrajectory() or reset() releases it.
  // Does nothing while no trajectory is being followed. Control thread only.
  void emergencyStop() { emergency_requested_ = true; }
  bool isEmergencyStopped() const { return stopped_; }

  // Drop any installed/staged trajectory, so the next update() with a direct
  // setpoint present falls straight to kDirect (POS_SP) rather than kHoverHold.
  // The normal precedence keeps POS_SP unreachable for as long as a trajectory
  // is installed; this is the explicit release the preset one-shot uses when its
  // square completes, so control returns to POS_SP without a disarm. Continuous
  // from a trajectory that has run out (it ends at rest) into a POS_SP that the
  // caller has pointed at that same rest position. Control thread only.
  void clearTrajectory();

  // Run one control step and return the attitude/thrust command.
  common::Command update(const common::State& state, double now, double dt);

  Mode mode() const { return mode_; }
  bool hasTrajectory() const { return (has_traj_ && !traj_.empty()) || has_next_; }
  const common::Trajectory& trajectory() const { return traj_; }
  const PositionControl& controller() const { return controller_; }

  // True while a trajectory has been installed but its t0 has not arrived yet.
  bool hasPendingTrajectory() const { return has_next_; }

  // True while the installed trajectory has been abandoned because the vehicle
  // got further than setMaxTrackingError() from its reference. LATCHED: the
  // tracker holds position and never goes back to that trajectory, even if the
  // reference later passes near the vehicle again. Only a newly promoted
  // trajectory, clearTrajectory() or reset() releases it. Any trajectory staged
  // at the moment of divergence is dropped too, since it was spliced onto the
  // reference that has just proved wrong.
  bool isDiverged() const { return diverged_; }

  // True exactly once per health timeout: the update() on which the tracker
  // stopped following a trajectory because no health signal arrived within
  // setHealthTimeout, then false until the next one. The caller uses it like
  // takeDivergence(): anything planned or staged by splicing onto the trajectory
  // that was just abandoned no longer means anything (it assumes the reference
  // kept moving; the tracker is now holding), so it is dropped and the next plan
  // starts at rest. Control thread only.
  bool takeHealthTimeout() {
    const bool edge = health_timeout_event_;
    health_timeout_event_ = false;
    return edge;
  }

  // True exactly once per divergence, on the update() that detected it, then
  // false until the next one. The caller uses it to act once: stop splicing onto
  // the abandoned trajectory and request a replan from the vehicle's position.
  // Acting on isDiverged() every tick instead would also discard the recovery
  // trajectory staged while the latch is still set. Control thread only.
  bool takeDivergence() {
    const bool edge = divergence_event_;
    divergence_event_ = false;
    return edge;
  }

private:
  PositionControl controller_;
  FlatnessMapper mapper_;

  common::Trajectory traj_;
  bool has_traj_{false};
  // Installed but not yet in effect; promoted to traj_ once now >= next_.t0.
  common::Trajectory next_;
  bool has_next_{false};
  bool mapper_needs_reset_{false};
  double last_arrival_{0.0};
  double health_timeout_{0.5};
  double max_tracking_error_{1.0};
  bool diverged_{false};          // latched; see isDiverged()
  bool divergence_event_{false};  // one-shot; see takeDivergence()
  bool health_timeout_event_{false};  // one-shot; see takeHealthTimeout()
  bool emergency_requested_{false};  // see emergencyStop(); consumed by update()
  bool stopped_{false};              // latched emergency stop
  bool feedforward_{false};

  Mode mode_{Mode::kHoverHold};
  Eigen::Vector3d hold_pos_{Eigen::Vector3d::Zero()};
  double hold_yaw_{0.0};

  bool has_direct_{false};
  Eigen::Vector3d direct_pos_{Eigen::Vector3d::Zero()};
  double direct_yaw_{0.0};
};

}  // namespace drone_core::control
