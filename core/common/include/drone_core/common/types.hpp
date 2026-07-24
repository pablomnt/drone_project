#pragma once

#include <vector>
#include <Eigen/Dense>

namespace drone_core::common {

// Every quantity that crosses an interface inside the autonomy core is
// expressed in a world ENU frame. Conversions to and from the PX4 NED/FRD
// convention happen only at the ROS boundary (see drone_core/common/frames.hpp),
// so nothing below ever has to think about handedness.

// Estimated vehicle state, as delivered by VIO (or PX4 odometry in simulation).
struct State {
  Eigen::Vector3d pos{Eigen::Vector3d::Zero()};
  Eigen::Vector3d vel{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acc{Eigen::Vector3d::Zero()};  // gravity already removed
  double yaw{0.0};
  double stamp{0.0};  // seconds
};

// Position and its first three derivatives at a single instant. This is the
// boundary state a replanned trajectory splices onto: pinning all four at the
// new trajectory's start leaves the reference C3 continuous across the switch,
// which is the same continuity the corridor QP already enforces at its own
// interior junctions. Default-constructed it is "at rest", the correct boundary
// for the first plan of a flight and for a replan off a hover.
struct MotionState {
  Eigen::Vector3d pos{Eigen::Vector3d::Zero()};
  Eigen::Vector3d vel{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acc{Eigen::Vector3d::Zero()};
  Eigen::Vector3d jerk{Eigen::Vector3d::Zero()};
};

// The reference the position controller tracks at a single instant. Velocity
// and acceleration are differential-flatness feed-forward terms; with both at
// zero the controller degrades gracefully to pure position tracking. There is
// deliberately no yaw rate: the stack commands attitude, not body rates, so a
// desired yaw is all the autopilot needs.
struct Reference {
  Eigen::Vector3d pos{Eigen::Vector3d::Zero()};
  Eigen::Vector3d vel_ff{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acc_ff{Eigen::Vector3d::Zero()};
  double yaw{0.0};
};

// What the controller produces: a desired body orientation and a normalised
// collective thrust in [0, 1]. Expressed in the ENU/FLU convention; the wrapper
// rotates it into PX4's NED/FRD frame before publishing.
struct Command {
  Eigen::Quaterniond attitude{Eigen::Quaterniond::Identity()};
  double thrust{0.0};
};

// A geometric waypoint produced by the global planner.
struct Waypoint {
  Eigen::Vector3d pos{Eigen::Vector3d::Zero()};
};

// A navigation goal in the world frame.
struct Goal {
  Eigen::Vector3d pos{Eigen::Vector3d::Zero()};
};

// A minimum-snap trajectory stored as the artifact the optimiser actually
// solves for: per-segment, per-axis 7th-order polynomials plus the duration of
// each segment. Keeping the coefficients (rather than a pre-sampled point list)
// lets the flatness mapper differentiate analytically for velocity and
// acceleration feed-forward. The trajectory is anchored in the world frame at a
// fixed start time t0; the controller evaluates it at (now - t0) so genuine
// tracking error accumulates between replans.
struct Trajectory {
  std::vector<double> segment_times;        // duration of each segment [s]
  std::vector<Eigen::VectorXd> coeffs_x;    // one coefficient vector per segment
  std::vector<Eigen::VectorXd> coeffs_y;
  std::vector<Eigen::VectorXd> coeffs_z;
  double total_duration{0.0};
  double t0{0.0};                           // world-clock anchor [s]

  bool empty() const { return segment_times.empty(); }
};

}  // namespace drone_core::common
