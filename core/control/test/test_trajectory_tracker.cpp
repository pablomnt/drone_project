// Verifies how a replanned trajectory takes over from the one it replaces.
//
// A replan is solved against the state the vehicle will be in a lead time from
// now, not the state it is in at solve time, because the solve itself takes
// time. Two properties follow, and this file pins both:
//
//   1. The tracker must not engage a trajectory before its t0. Engaging early
//      would jump the reference to a point the vehicle has not reached yet, and
//      it is also what makes a too-generous lead free — the planner can budget
//      pessimistically without paying for it.
//   2. Engaging AT t0 must be continuous. The planner pinned the new
//      trajectory's position, velocity and acceleration to the outgoing
//      reference at that instant, so the reference the controller consumes —
//      including the flatness feed-forward — must cross the switch with no step.

#include "drone_core/control/flatness_mapper.hpp"
#include "drone_core/control/trajectory_tracker.hpp"

#include <cmath>
#include <iostream>

#include "drone_core/common/trajectory_eval.hpp"

namespace {

using drone_core::common::MotionState;
using drone_core::common::Reference;
using drone_core::common::State;
using drone_core::common::Trajectory;
using drone_core::control::FlatnessMapper;
using drone_core::control::TrajectoryTracker;

constexpr double kTol = 1e-9;

// A single-segment trajectory from explicit per-axis coefficients (ascending
// powers), anchored at t0.
Trajectory makeTraj(double t0, double duration, const Eigen::VectorXd& cx,
                    const Eigen::VectorXd& cy, const Eigen::VectorXd& cz) {
  Trajectory t;
  t.segment_times = {duration};
  t.coeffs_x = {cx};
  t.coeffs_y = {cy};
  t.coeffs_z = {cz};
  t.total_duration = duration;
  t.t0 = t0;
  return t;
}

Eigen::VectorXd poly(std::initializer_list<double> c) {
  Eigen::VectorXd v(static_cast<int>(c.size()));
  int i = 0;
  for (double x : c) v(i++) = x;
  return v;
}

// The degree-2 Taylor expansion of a motion state: the simplest polynomial that
// starts at exactly (pos, vel, acc), which is what the QP's start boundary
// pins. Standing in for a real replan, whose first segment agrees with the
// outgoing trajectory to the same orders at t = 0.
Trajectory spliceOnto(const MotionState& m, double t0, double duration) {
  return makeTraj(t0, duration,
                  poly({m.pos.x(), m.vel.x(), 0.5 * m.acc.x()}),
                  poly({m.pos.y(), m.vel.y(), 0.5 * m.acc.y()}),
                  poly({m.pos.z(), m.vel.z(), 0.5 * m.acc.z()}));
}

int checkSpliceContinuity() {
  int failures = 0;

  // Outgoing trajectory: something with motion on every axis so a missed term
  // shows up rather than cancelling.
  const Trajectory outgoing = makeTraj(100.0, 10.0,
                                       poly({0.0, 0.8, -0.05, 0.004}),
                                       poly({1.0, 0.2, 0.06, -0.003}),
                                       poly({1.5, 0.1, -0.02, 0.001}));

  const double t_switch = 103.0;  // t0 + 3 s: the lead time the planner used
  const MotionState splice = drone_core::common::sampleMotion(outgoing, t_switch);
  const Trajectory incoming = spliceOnto(splice, t_switch, 5.0);

  FlatnessMapper mapper;
  mapper.reset(0.0);
  const Reference before = mapper.sample(outgoing, t_switch);
  const Reference after = mapper.sample(incoming, t_switch);

  if ((before.pos - after.pos).norm() > kTol) {
    std::cerr << "FAIL: position steps by " << (before.pos - after.pos).norm()
              << " m across the splice\n";
    ++failures;
  }
  if ((before.vel_ff - after.vel_ff).norm() > kTol) {
    std::cerr << "FAIL: velocity feed-forward steps by " << (before.vel_ff - after.vel_ff).norm()
              << " m/s across the splice\n";
    ++failures;
  }
  if ((before.acc_ff - after.acc_ff).norm() > kTol) {
    std::cerr << "FAIL: acceleration feed-forward steps by "
              << (before.acc_ff - after.acc_ff).norm() << " m/s^2 across the splice\n";
    ++failures;
  }

  // The old rest-to-rest behaviour, as a contrast: a replan that re-anchored at
  // the vehicle and started from zero velocity. This is the regression the
  // splice exists to prevent, so assert it really would have been a big step.
  MotionState from_rest;
  from_rest.pos = splice.pos;
  const Trajectory rest_anchored = spliceOnto(from_rest, t_switch, 5.0);
  const Reference rest_ref = mapper.sample(rest_anchored, t_switch);
  if ((before.vel_ff - rest_ref.vel_ff).norm() < 0.1) {
    std::cerr << "FAIL: the rest-anchored contrast case is not actually a step ("
              << (before.vel_ff - rest_ref.vel_ff).norm() << " m/s) — test is not proving anything\n";
    ++failures;
  }

  return failures;
}

int checkDeferredEngage() {
  int failures = 0;

  TrajectoryTracker tracker;
  tracker.setStaleTimeout(5.0);
  tracker.setHoverThrust(0.35);
  tracker.setPositionGains(Eigen::Vector3d(0.95, 0.95, 1.0));
  tracker.setVelocityGains(Eigen::Vector3d(1.8, 1.8, 2.0), Eigen::Vector3d(0.4, 0.4, 0.5),
                           Eigen::Vector3d(0.2, 0.2, 0.2));

  // The vehicle sits where the staged trajectories start, as a real plan would,
  // while the direct setpoint is 7 m away — so engaging early is still a large,
  // visible step in the reference, and the divergence check stays at its
  // default without tripping.
  State state;
  state.pos = Eigen::Vector3d(5.0, 5.0, 1.5);
  state.yaw = 0.0;

  // Airborne reset so the open-loop takeoff ramp does not engage and mask the
  // reference under a flat-attitude thrust ramp.
  tracker.reset();
  const Eigen::Vector3d direct(0.0, 0.0, 1.5);
  tracker.setDirectSetpoint(direct, 0.0);

  double now = 200.0;
  tracker.update(state, now, 0.02);
  if (tracker.mode() != TrajectoryTracker::Mode::kDirect) {
    std::cerr << "FAIL: expected kDirect before any trajectory\n";
    ++failures;
  }

  // Stage a trajectory that starts 0.2 s from now — a lead the planner has
  // budgeted for its own solve.
  const double t0 = now + 0.2;
  const Trajectory staged = makeTraj(t0, 4.0, poly({5.0, 0.5}), poly({5.0, 0.0}),
                                     poly({1.5, 0.0}));
  tracker.setTrajectory(staged, now);

  if (!tracker.hasPendingTrajectory()) {
    std::cerr << "FAIL: staged trajectory not reported as pending\n";
    ++failures;
  }

  // Every tick before t0 must still follow the direct setpoint, NOT the staged
  // trajectory (whose position is 5 m away — a step this size is exactly what
  // engaging early would inflict).
  for (int i = 0; i < 9; ++i) {
    now += 0.02;
    tracker.update(state, now, 0.02);
    if (tracker.mode() != TrajectoryTracker::Mode::kDirect) {
      std::cerr << "FAIL: engaged at t=" << now << ", before t0=" << t0 << "\n";
      ++failures;
      break;
    }
    if ((tracker.controller().getPositionSetpoint() - direct).norm() > kTol) {
      std::cerr << "FAIL: reference left the direct setpoint before t0\n";
      ++failures;
      break;
    }
  }

  // Crossing t0 engages it.
  now = t0 + 0.001;
  tracker.update(state, now, 0.02);
  if (tracker.mode() != TrajectoryTracker::Mode::kTracking) {
    std::cerr << "FAIL: did not engage at t0\n";
    ++failures;
  }
  if (tracker.hasPendingTrajectory()) {
    std::cerr << "FAIL: still reports a pending trajectory after engaging\n";
    ++failures;
  }
  if (std::abs(tracker.controller().getPositionSetpoint().x() - 5.0) > 0.01) {
    std::cerr << "FAIL: reference did not follow the engaged trajectory (x="
              << tracker.controller().getPositionSetpoint().x() << ", wanted ~5.0)\n";
    ++failures;
  }

  // A trajectory whose t0 is already in the past (the solve overran its lead)
  // engages on the next update rather than being held.
  {
    TrajectoryTracker late;
    late.setStaleTimeout(5.0);
    late.reset();
    late.setDirectSetpoint(direct, 0.0);
    late.update(state, 300.0, 0.02);
    late.setTrajectory(makeTraj(299.5, 4.0, poly({5.0}), poly({5.0}), poly({1.5})), 300.0);
    late.update(state, 300.02, 0.02);
    if (late.mode() != TrajectoryTracker::Mode::kTracking) {
      std::cerr << "FAIL: a trajectory with t0 in the past did not engage immediately\n";
      ++failures;
    }
  }

  // reset() drops both the engaged and the staged trajectory: they are anchored
  // to a wall clock and spliced onto a reference that no longer exists.
  tracker.setTrajectory(makeTraj(now + 10.0, 4.0, poly({9.0}), poly({9.0}), poly({9.0})), now);
  tracker.reset();
  if (tracker.hasTrajectory() || tracker.hasPendingTrajectory()) {
    std::cerr << "FAIL: reset() left a trajectory installed\n";
    ++failures;
  }

  return failures;
}

// A trajectory that is still fresh by the clock must be abandoned once the
// vehicle is further than the limit from its reference — the case the stale
// timeout cannot catch, since nothing has stopped arriving — and abandoned for
// good: hover where the vehicle is, drop anything staged onto that reference,
// and resume only on a newly promoted trajectory.
int checkDivergence() {
  int failures = 0;
  auto fail = [&failures](const char* what) {
    std::cerr << "FAIL: " << what << "\n";
    ++failures;
  };

  TrajectoryTracker tracker;
  tracker.setStaleTimeout(5.0);  // generous: only distance can trip this
  tracker.setMaxTrackingError(1.0);
  tracker.setHoverThrust(0.35);
  tracker.reset();

  // Stationary reference at (0, 0, 1.5), engaged immediately.
  const Trajectory traj = makeTraj(200.0, 10.0, poly({0.0}), poly({0.0}), poly({1.5}));
  tracker.setTrajectory(traj, 200.0);

  State near;
  near.pos = Eigen::Vector3d(0.9, 0.0, 1.5);  // inside the 1.0 m limit
  tracker.update(near, 200.01, 0.02);
  if (tracker.mode() != TrajectoryTracker::Mode::kTracking) fail("0.9 m off should still track");
  if (tracker.isDiverged() || tracker.takeDivergence()) fail("diverged inside the limit");

  // A trajectory staged onto this reference, waiting for its t0.
  tracker.setTrajectory(makeTraj(200.5, 5.0, poly({0.0, 0.2}), poly({0.0}), poly({1.5})), 200.02);

  State far;
  far.pos = Eigen::Vector3d(2.0, 0.0, 1.5);  // 2 m off
  tracker.update(far, 200.03, 0.02);
  if (tracker.mode() != TrajectoryTracker::Mode::kHoverHold) fail("2 m off did not hover-hold");
  if (!tracker.isDiverged()) fail("isDiverged() false past the limit");
  if ((tracker.controller().getPositionSetpoint() - far.pos).norm() > kTol) {
    fail("hover-hold is not at the vehicle's position");
  }
  if (tracker.hasPendingTrajectory()) fail("trajectory staged onto the bad reference kept");
  if (!tracker.takeDivergence()) fail("takeDivergence() missed the divergence");
  if (tracker.takeDivergence()) fail("takeDivergence() fired twice for one divergence");

  // Latched: back within the limit of the abandoned reference, and past the
  // staged trajectory's t0, it still holds where it diverged.
  tracker.update(near, 200.6, 0.02);
  if (tracker.mode() != TrajectoryTracker::Mode::kHoverHold) {
    fail("resumed the abandoned trajectory when the vehicle came back near it");
  }
  if ((tracker.controller().getPositionSetpoint() - far.pos).norm() > kTol) {
    fail("hold point moved after divergence");
  }

  // A replan from the vehicle's position releases it.
  tracker.setTrajectory(makeTraj(200.7, 5.0, poly({0.9}), poly({0.0}), poly({1.5})), 200.65);
  tracker.update(near, 200.71, 0.02);
  if (tracker.mode() != TrajectoryTracker::Mode::kTracking) fail("recovery trajectory not tracked");
  if (tracker.isDiverged()) fail("latch not released by a new trajectory");

  // A new trajectory that is itself far from the vehicle is refused before it
  // produces a single reference, and the hold is where the vehicle is now.
  tracker.setTrajectory(makeTraj(200.8, 5.0, poly({5.0}), poly({0.0}), poly({1.5})), 200.75);
  tracker.update(near, 200.81, 0.02);
  if (tracker.mode() != TrajectoryTracker::Mode::kHoverHold) fail("far new trajectory was tracked");
  if ((tracker.controller().getPositionSetpoint() - near.pos).norm() > kTol) {
    fail("refused trajectory still drove the reference, or held the wrong point");
  }

  // clearTrajectory() releases the latch too (the preset's hand-back to POS_SP).
  tracker.setDirectSetpoint(Eigen::Vector3d(0.0, 0.0, 1.5), 0.0);
  tracker.clearTrajectory();
  tracker.update(near, 200.9, 0.02);
  if (tracker.isDiverged() || tracker.mode() != TrajectoryTracker::Mode::kDirect) {
    fail("clearTrajectory() did not release to the direct setpoint");
  }

  // <= 0 disables the check.
  TrajectoryTracker disabled;
  disabled.setStaleTimeout(5.0);
  disabled.setMaxTrackingError(0.0);
  disabled.setHoverThrust(0.35);
  disabled.reset();
  disabled.setTrajectory(traj, 200.0);
  disabled.update(far, 200.01, 0.02);
  if (disabled.mode() != TrajectoryTracker::Mode::kTracking) fail("limit 0 did not disable the check");

  return failures;
}

}  // namespace

int main() {
  const int failures = checkSpliceContinuity() + checkDeferredEngage() + checkDivergence();
  if (failures != 0) {
    std::cerr << "test_trajectory_tracker: " << failures << " check(s) failed\n";
    return 1;
  }
  std::cout << "test_trajectory_tracker: all checks passed\n";
  return 0;
}
