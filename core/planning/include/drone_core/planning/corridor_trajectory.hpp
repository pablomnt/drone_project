#pragma once

#include <vector>

#include <Eigen/Dense>

#include "drone_core/common/types.hpp"
#include "drone_core/planning/corridor.hpp"

namespace drone_core::planning {

// Per-axis dynamic limits for the corridor QP. Applied as box bounds on the
// Bezier control points of each derivative, so the true speed norm can reach
// sqrt(3)*vmax in the corner case — deliberately conservative (a true
// ||v|| <= vmax bound is an SOCP, deferred).
struct CorridorLimits {
  double vmax = 1.0;  // per-axis velocity bound [m/s]
  double amax = 1.5;  // per-axis acceleration bound [m/s^2]
  double jmax = 3.0;  // per-axis jerk bound [m/s^3]
};

// Corridor-constrained minimum-snap trajectory generation: degree-7 polynomial
// segments whose Bezier control points are confined to a per-segment convex
// free region (so the whole trajectory provably stays inside the corridor —
// the Bezier hull contains the curve) and whose velocity/acceleration/jerk
// control points respect per-axis limits. The decision variables stay in the
// monomial basis (same snap cost block as MinSnapTrajectory); the Bernstein
// map only shapes the linear constraint rows, so the solution drops straight
// into common::Trajectory for the flatness mapper. Solved as ONE coupled QP
// over all three axes with OSQP (C API, confined to the .cpp): a polyhedron
// face row mixes x, y and z, so the per-axis decomposition an axis-aligned box
// allowed no longer exists.
class CorridorTrajectoryOptimizer {
public:
  explicit CorridorTrajectoryOptimizer(const CorridorLimits& limits = CorridorLimits{})
      : limits_(limits) {}

  // Inner solve for a fixed time allocation: one degree-7 segment per region,
  // from `start` to rest at `goal` (truncation endpoint, must satisfy
  // regions.back()), C0-C4 continuous at junctions. Segment i is confined to
  // regions[i]; times[i] is its duration. Returns false when the QP is
  // infeasible (corridor too tight / times too short for the limits) or the
  // solver fails; then `out` is untouched. On success fills `out` (t0 left to
  // the caller) and, if given, `cost_out` with the snap objective value (the
  // outer time search minimises this).
  //
  // `start` pins position AND its first three derivatives, so a replan splices
  // onto the state the vehicle will actually be in when the new trajectory
  // takes over instead of pretending it is stationary. start.pos must satisfy
  // regions[0]. Only the END is at rest, which is what makes an un-replaced
  // trajectory a safety stop; the start is at rest only when the caller says
  // so (first plan of a flight, or a replan off a hover).
  //
  // `pin_waypoints`, when non-null, additionally pins the curve's position at
  // every interior junction to the corresponding entry (see optimizeTrajectory).
  // It must hold regions.size() + 1 points. Null leaves the junctions free,
  // which is the planner's behaviour.
  bool solveQP(const common::MotionState& start, const Eigen::Vector3d& goal,
               const std::vector<double>& times, const std::vector<ConvexRegion>& regions,
               common::Trajectory& out, double* cost_out = nullptr,
               const std::vector<Eigen::Vector3d>* pin_waypoints = nullptr) const;

  // Convenience overload: start from rest at `start`.
  bool solveQP(const Eigen::Vector3d& start, const Eigen::Vector3d& goal,
               const std::vector<double>& times, const std::vector<ConvexRegion>& regions,
               common::Trajectory& out, double* cost_out = nullptr) const {
    common::MotionState s;
    s.pos = start;
    return solveQP(s, goal, times, regions, out, cost_out);
  }

  // Outer time-allocation search: BOBYQA over the per-segment durations,
  // minimising the QP snap cost plus a total-time penalty, with an infeasible
  // solve scored as a large penalty so the search is pushed back toward
  // feasible (longer) times. Seeded velocity-consistent — t_i = segment length
  // / vmax + buffer — then grown geometrically until the QP accepts it, so the
  // search always starts on the feasible side of the flat penalty region. The
  // seed also carries a braking floor per segment so a fast-moving start does
  // not begin the search inside the infeasible region.
  // waypoints are the resampled corridor waypoints
  // (waypoints.size() == regions.size() + 1); the trajectory runs from `start`
  // to rest at waypoints.back(), free within the regions in between.
  // start.pos is used in place of waypoints.front(), which it must coincide
  // with for the corridor to contain it. Returns false when no feasible time
  // allocation was found (the final QP at the best times fails).
  //
  // `pin_waypoints` decides what the waypoints are FOR. False (the planner's
  // case) uses them only to shape the corridor and seed the time allocation:
  // the trajectory is pinned at its two ends and is otherwise free to take any
  // minimum-snap route through the regions, which is the point — it smooths out
  // the geometric search's zig-zag rather than tracking it. True additionally
  // constrains the curve to pass exactly through every interior waypoint, for
  // callers whose waypoints ARE the intent rather than a hint (PRESET_WAYPOINTS,
  // where the shape is the thing under test). Note what the free case does to a
  // CLOSED path: with the last waypoint equal to the first, "start here, end
  // here, stay in the regions" is minimised by barely moving at all, so a preset
  // loop collapses instead of being flown.
  bool optimizeTrajectory(const common::MotionState& start,
                          const std::vector<Eigen::Vector3d>& waypoints,
                          const std::vector<ConvexRegion>& regions,
                          common::Trajectory& out,
                          bool pin_waypoints = false) const;

  // Convenience overload: start from rest at waypoints.front().
  bool optimizeTrajectory(const std::vector<Eigen::Vector3d>& waypoints,
                          const std::vector<ConvexRegion>& regions,
                          common::Trajectory& out,
                          bool pin_waypoints = false) const {
    common::MotionState s;
    if (!waypoints.empty()) s.pos = waypoints.front();
    return optimizeTrajectory(s, waypoints, regions, out, pin_waypoints);
  }

  const CorridorLimits& limits() const { return limits_; }

  // Wall-clock budget for optimizeTrajectory's time search [s]; <= 0 means
  // unlimited. Counted from the start of the call, so seed growth uses it up
  // too. When it runs out BOBYQA stops and the best allocation evaluated so far
  // is used — always feasible, since the search starts from a feasible seed and
  // infeasible points score worse than any feasible one. Seed growth itself is
  // NOT cut short: until a feasible allocation exists there is nothing to fall
  // back on, so a budget spent there skips the search and uses the seed. The
  // budget is checked between QP evaluations, so a call can overrun it by one
  // solve plus the final solve.
  void setTimeBudget(double seconds) { time_budget_ = seconds; }
  double timeBudget() const { return time_budget_; }

  // When on, every optimizeTrajectory call logs one line breaking down where
  // its time went: seed growth (time, QP solves, seed and grown durations),
  // BOBYQA's initial model-building probe (its first 2n+1 evaluations), the
  // rest of the search (evaluations, how many were infeasible, the duration it
  // reached), the final solve, and the trajectory duration.
  void setDebug(bool on) { debug_ = on; }

private:
  CorridorLimits limits_;
  double time_budget_ = 0.0;
  bool debug_ = false;

  // Outer-loop tuning. The time penalty mirrors MinSnapTimeOptimizer's (cost
  // per second of flight time, trading smoothness against duration); the
  // infeasible penalty just has to dwarf any feasible score. The eval budget is
  // bounded because each evaluation is a full (cold) OSQP solve and the whole
  // search runs inside the ~1 Hz trajgen tick.
  static constexpr double kTimePenalty = 500.0;
  static constexpr double kInfeasiblePenalty = 1.0e7;
  static constexpr double kSeedBuffer = 0.5;  // per-segment slack over len/vmax [s]
  static constexpr double kMinSegmentTime = 0.1;  // BOBYQA lower bound [s]
  static constexpr int kMaxEvals = 100;
  static constexpr int kMaxSeedGrowth = 6;  // 1.5x seed stretches before giving up (~11x)
};

}  // namespace drone_core::planning
