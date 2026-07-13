#pragma once

#include "drone_core/common/types.hpp"
#include "drone_core/control/flatness_mapper.hpp"
#include "drone_core/control/position_control.hpp"

namespace drone_core::control {

// Drives the position controller from a trajectory and supervises it. While a
// fresh trajectory is available it samples the flatness reference and tracks it;
// if trajectories stop arriving (a stalled or dead planner) it latches the
// current position and holds, so loss of guidance degrades to a safe hover
// rather than tracking a stale path or losing the control stream.
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
  void enableFeedforward(bool enabled);

  // Seconds without a new trajectory before falling back to hover-hold.
  void setStaleTimeout(double seconds) { stale_timeout_ = seconds; }

  // Re-arm the controller (and takeoff logic) on (re)engagement.
  void reset();

  // Command an explicit position/yaw setpoint. Used for takeoff and manual
  // hover before any planner goal is active; a fresh planner trajectory takes
  // precedence over it.
  void setDirectSetpoint(const Eigen::Vector3d& pos, double yaw);

  // Install a freshly planned trajectory, stamped with its arrival time.
  void setTrajectory(const common::Trajectory& traj, double arrival_time);

  // Run one control step and return the attitude/thrust command.
  common::Command update(const common::State& state, double now, double dt);

  Mode mode() const { return mode_; }
  bool hasTrajectory() const { return has_traj_ && !traj_.empty(); }
  const common::Trajectory& trajectory() const { return traj_; }
  const PositionControl& controller() const { return controller_; }

private:
  PositionControl controller_;
  FlatnessMapper mapper_;

  common::Trajectory traj_;
  bool has_traj_{false};
  bool mapper_needs_reset_{false};
  double last_arrival_{0.0};
  double stale_timeout_{0.5};
  bool feedforward_{false};

  Mode mode_{Mode::kHoverHold};
  Eigen::Vector3d hold_pos_{Eigen::Vector3d::Zero()};
  double hold_yaw_{0.0};

  bool has_direct_{false};
  Eigen::Vector3d direct_pos_{Eigen::Vector3d::Zero()};
  double direct_yaw_{0.0};
};

}  // namespace drone_core::control
