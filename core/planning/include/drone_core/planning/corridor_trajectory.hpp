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
  // rest-to-rest from `start` (current drone position, must satisfy
  // regions[0]) to `goal` (truncation endpoint, must satisfy regions.back()),
  // C0-C4 continuous at junctions. Segment i is confined to regions[i];
  // times[i] is its duration. Returns false when the QP is infeasible
  // (corridor too tight / times too short for the limits) or the solver
  // fails; then `out` is untouched. On success fills `out` (t0 left to the
  // caller) and, if given, `cost_out` with the snap objective value (the
  // outer time search minimises this).
  bool solveQP(const Eigen::Vector3d& start, const Eigen::Vector3d& goal,
               const std::vector<double>& times, const std::vector<ConvexRegion>& regions,
               common::Trajectory& out, double* cost_out = nullptr) const;

  // Outer time-allocation search: BOBYQA over the per-segment durations,
  // minimising the QP snap cost plus a total-time penalty, with an infeasible
  // solve scored as a large penalty so the search is pushed back toward
  // feasible (longer) times. Seeded velocity-consistent — t_i = segment length
  // / vmax + buffer — then grown geometrically until the QP accepts it, so the
  // search always starts on the feasible side of the flat penalty region.
  // waypoints are the resampled corridor waypoints
  // (waypoints.size() == regions.size() + 1); the trajectory runs rest-to-rest
  // from waypoints.front() to waypoints.back(), free within the regions in
  // between. Returns false when no feasible time allocation was found (the
  // final QP at the best times fails).
  bool optimizeTrajectory(const std::vector<Eigen::Vector3d>& waypoints,
                          const std::vector<ConvexRegion>& regions,
                          common::Trajectory& out) const;

  const CorridorLimits& limits() const { return limits_; }

private:
  CorridorLimits limits_;

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
