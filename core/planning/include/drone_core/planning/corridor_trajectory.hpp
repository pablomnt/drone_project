#pragma once

#include <atomic>
#include <cmath>
#include <string>
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
  //
  // `path_waypoints` (the same resampled list, size regions.size() + 1) enables
  // the path-following term when a path weight is set — see setPathWeight. It is
  // separate from `pin_waypoints` because the two are independent: pinning is a
  // hard equality at the junctions, this is a soft pull over the whole curve, and
  // the planner wants the second without the first. `cost_out` stays the SNAP
  // cost alone whatever the weight; `path_cost_out` receives the path term, so a
  // caller minimising the QP objective has to add the two.
  bool solveQP(const common::MotionState& start, const Eigen::Vector3d& goal,
               const std::vector<double>& times, const std::vector<ConvexRegion>& regions,
               common::Trajectory& out, double* cost_out = nullptr,
               const std::vector<Eigen::Vector3d>* pin_waypoints = nullptr,
               const std::vector<Eigen::Vector3d>* path_waypoints = nullptr,
               double* path_cost_out = nullptr, std::string* status_out = nullptr) const;

  // Convenience overload: start from rest at `start`.
  bool solveQP(const Eigen::Vector3d& start, const Eigen::Vector3d& goal,
               const std::vector<double>& times, const std::vector<ConvexRegion>& regions,
               common::Trajectory& out, double* cost_out = nullptr) const {
    common::MotionState s;
    s.pos = start;
    return solveQP(s, goal, times, regions, out, cost_out);
  }

  // Outer time-allocation search, aiming for the shortest feasible duration in
  // three stages, each of which only ever keeps an allocation the QP accepted:
  //   1. Seed velocity-consistent — t_i = segment length / vmax + buffer, plus
  //      the time a ramp to or from rest costs on the last segment and, when the
  //      start is below kRestSpeed, the first (otherwise a braking allowance for
  //      the moving start) — and grow every segment x1.5 until the QP accepts it,
  //      or, if the seed is accepted at once, shrink it x1/1.5 until the QP
  //      refuses, keeping the last accepted step.
  //   2. Bisect that last growth step on a single scale factor for all
  //      segments, until the bracket is within kBisectGap.
  //   3. Cut groups of segments that turn alike (straights, arcs; see
  //      setGroupCut): the full cut on every group, then half of it, keeping
  //      each cut the QP accepts.
  // Stages 2 and 3 stop early when the time budget runs out (setTimeBudget).
  // waypoints are the resampled corridor waypoints
  // (waypoints.size() == regions.size() + 1); the trajectory runs from `start`
  // to rest at waypoints.back(), free within the regions in between.
  // start.pos is used in place of waypoints.front(), which it must coincide
  // with for the corridor to contain it. Returns false when no feasible time
  // allocation was found (stage 1 ran out of growth steps).
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
  // unlimited. Counted from the start of the call and shared by all three
  // stages. When it runs out the last accepted allocation is used — feasible,
  // just slower. Seed growth itself is NOT cut short: until a feasible
  // allocation exists there is nothing to fall back on (shrinking a seed that
  // was feasible at once is). The budget is checked
  // before each QP solve, so a call can overrun it by one solve.
  void setTimeBudget(double seconds) { time_budget_ = seconds; }

  // Optional external stop for the same budget checks: while *flag reads true the
  // search behaves as if its budget had run out and returns the last accepted
  // allocation — feasible, just slower. Seed growth still runs to a feasible
  // allocation. The trajectory monitor raises it when the trajectory being flown
  // turns unsafe mid-solve, so the replacement is ready sooner. Null = none.
  void setAbortFlag(const std::atomic<bool>* flag) { abort_flag_ = flag; }
  double timeBudget() const { return time_budget_; }

  // When on, every optimizeTrajectory call logs one line breaking down where
  // its time went: seed growth (time, QP solves, seed and grown durations), the
  // bisection of growth's last step (time, solves, duration, final bracket),
  // the segment groups, each group-cut pass (cuts accepted of tried, each
  // group marked accepted / rejected / not tried, duration before and after,
  // time), whether the budget cut it short, and the
  // trajectory duration.
  void setDebug(bool on) { debug_ = on; }

  // How hard the trajectory is pulled toward the geometric path [cost per m^2 of
  // mean squared control-point deviation, per segment]. 0 (the default) disables
  // it and the QP is pure minimum-snap, which is what makes a corridor-QP round
  // every corner as widely as the regions allow: snap is the ONLY thing scored,
  // and a wide turn is smoother than a direct one. The corridor is then the sole
  // thing holding the curve near the plan, so a roomy corridor buys wide turns.
  //
  // The term is the squared distance from each position control point to the
  // corresponding point on the straight chord between its segment's two
  // waypoints, averaged over the control points and summed over segments. The
  // chord rather than the junctions alone: penalising junctions only still lets
  // the curve bulge between them, which is the corner-cutting itself.
  //
  // Note it does NOT scale with the time allocation while the snap cost falls as
  // 1/T^7, so on a long relaxed trajectory this term dominates and the curve
  // hugs the plan, while in a tight spot with short segments snap takes over and
  // the corridor is used for what it is for. That asymmetry is wanted. It also
  // means the two are not in comparable units, so the weight is a pure tuning
  // number with no physical reading.
  //
  // Soft by construction: it changes the objective only, never the constraints,
  // so it can never make a feasible corridor infeasible the way pinning can.
  void setPathWeight(double weight) { path_weight_ = weight; }
  double pathWeight() const { return path_weight_; }

  // Stage 3 of the time search. Segments are grouped by how the path turns:
  // consecutive segments whose joints are all straight (<= kGroupStraightAngle)
  // form one group, and so do consecutive segments whose joints all turn by the
  // same amount in the same direction (an arc); a joint that breaks the pattern,
  // such as a corner between two straights, starts a new group. Each group then
  // has its segments' times cut by `cut` (a fraction, e.g. 0.25) in its middle
  // and by `edge_factor` * `cut` at its two end segments, so the change of speed
  // into the neighbouring groups is eased rather than stepped. Two passes over
  // all groups, the full cut then half of it. A cut is kept only if the QP
  // accepts the whole trajectory with it. <= 0 disables the stage.
  void setGroupCut(double cut) { group_cut_ = cut; }
  void setGroupEdgeFactor(double edge_factor) { group_edge_factor_ = edge_factor; }

private:
  CorridorLimits limits_;
  double time_budget_ = 0.0;
  const std::atomic<bool>* abort_flag_ = nullptr;
  double path_weight_ = 0.0;
  double group_cut_ = 0.25;
  double group_edge_factor_ = 0.6;
  bool debug_ = false;

  static constexpr double kSeedBuffer = 0.5;  // per-segment slack over len/vmax [s]
  // Start speed [m/s] below which the start counts as at rest for the seed's
  // start-from-rest allowance.
  static constexpr double kRestSpeed = 0.1;
  static constexpr double kMinSegmentTime = 0.1;  // floor under a group cut [s]
  static constexpr int kMaxSeedGrowth = 6;  // 1.5x seed stretches before giving up (~11x)
  // Bisection of the last growth step stops once the bracket's (hi - lo) / lo
  // is within this: from 1.5x that is two midpoint solves in the usual case.
  // 7.5% was tried (2026-09-28): one more solve for 44.1 s -> 43.8 s over four
  // scratch paths, within the noise the group cuts add, so not kept.
  static constexpr double kBisectGap = 0.15;
  // Segment grouping (see setGroupCut). A joint turning at most
  // kGroupStraightAngle is straight and its direction is ignored — the turn
  // axis of a near-zero turn is noise. Two turning joints belong to the same
  // arc when their angles differ by at most kGroupTurnTolerance and their turn
  // axes by at most kGroupAxisTolerance. All in radians.
  static constexpr double kGroupStraightAngle = 10.0 * M_PI / 180.0;
  static constexpr double kGroupTurnTolerance = 10.0 * M_PI / 180.0;
  static constexpr double kGroupAxisTolerance = 30.0 * M_PI / 180.0;
};

}  // namespace drone_core::planning
