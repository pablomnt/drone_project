#pragma once

#include "drone_core/common/types.hpp"

namespace drone_core::common {

// Evaluation of the polynomial trajectory produced by the trajectory
// optimisers. This lives in `common` rather than in the flatness mapper because
// both sides of the pipeline need it: control samples the trajectory to build
// the tracked reference, and planning samples the trajectory it is about to
// replace to find the state the replan must splice onto.

// The `deriv`-th derivative of the position polynomials, `elapsed` seconds after
// the trajectory's start. deriv = 0 gives position, 1 velocity, 2 acceleration,
// 3 jerk. `elapsed` is clamped to the trajectory span, so sampling past the end
// holds the final state — which the optimisers pin at rest, so a clamped sample
// reports zero for every derivative.
Eigen::Vector3d evalTrajectory(const Trajectory& traj, double elapsed, int deriv);

// Position and its first three derivatives at wall-clock time `now`, resolved
// against the trajectory's own `t0` anchor. Sampling before `t0` (a trajectory
// anchored in the future, which is what a replan lead time produces) clamps to
// its start state rather than extrapolating the polynomial backwards: the
// corridor guarantee only covers the solved span.
MotionState sampleMotion(const Trajectory& traj, double now);

}  // namespace drone_core::common
