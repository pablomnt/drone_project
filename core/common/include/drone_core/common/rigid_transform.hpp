#pragma once

#include <Eigen/Geometry>

#include "drone_core/common/types.hpp"

namespace drone_core::common {

// Rigid frame changes for the planning boundary.
//
// The planner and trajectory generator work in the MAP frame, the frame the
// occupancy map is built in; RTAB-Map corrects it for drift, in jumps. The
// tracker and controller work in the WORLD frame, the frame the state estimate
// (OKVIS) is expressed in; it is smooth but drifts. The two differ by a rigid
// transform that the host looks up and hands to the core, and every quantity
// that crosses between planning and control goes through one of these.
//
// A transform named `b_from_a` maps a point expressed in frame a into frame b:
// p_b = b_from_a * p_a. Positions take the whole transform. Velocity,
// acceleration and jerk are free vectors, so they take the rotation only.

// Position and velocity transformed; yaw re-derived by rotating the heading
// vector, so a small tilt in the transform cannot corrupt it. thrust_accel is a
// body-frame scalar and stamp a time, so both pass through unchanged.
State transformState(const Eigen::Isometry3d& b_from_a, const State& s);

MotionState transformMotion(const Eigen::Isometry3d& b_from_a, const MotionState& m);

// Exact: a polynomial trajectory under a rigid transform is again a polynomial
// of the same degree. Every coefficient vector is rotated, and the translation
// is added to the constant term only (higher-order terms are derivatives).
// Segment times, duration and t0 are unchanged.
Trajectory transformTrajectory(const Eigen::Isometry3d& b_from_a, const Trajectory& traj);

}  // namespace drone_core::common
