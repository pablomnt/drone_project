#pragma once

#include <array>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/Planner.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <octomap/octomap.h>

namespace drone_core::planning {

// Map handle shared between the planner and its host. It is a plain octomap
// pointer (the octomap library, not the ROS octomap_msgs package), so the
// planner carries no middleware dependency.
using MapHandle = std::shared_ptr<octomap::OcTree>;

// Which OMPL optimal planner to run. RRT* is the original asymptotically optimal
// RRT; the rest are the BIT* lineage (heuristic/informed graph search), which
// concentrate the search in the start->goal corridor instead of sampling the
// whole workspace uniformly. All share the same state space, validity checker
// and clearance objective; only construction/parameters differ (see makePlanner).
enum class PlannerType { RRTstar, BITstar, ABITstar, AITstar, EITstar };

// PlannerType <-> string, for ROS params and logging. The names match the enum.
inline const char* toString(PlannerType t) {
  switch (t) {
    case PlannerType::BITstar: return "BITstar";
    case PlannerType::ABITstar: return "ABITstar";
    case PlannerType::AITstar: return "AITstar";
    case PlannerType::EITstar: return "EITstar";
    case PlannerType::RRTstar: break;
  }
  return "RRTstar";
}

// Parse a PlannerType name; returns false (and leaves `out` untouched) if the
// string is not a known planner, so the caller can warn and keep its default.
inline bool fromString(const std::string& s, PlannerType& out) {
  if (s == "RRTstar") { out = PlannerType::RRTstar; return true; }
  if (s == "BITstar") { out = PlannerType::BITstar; return true; }
  if (s == "ABITstar") { out = PlannerType::ABITstar; return true; }
  if (s == "AITstar") { out = PlannerType::AITstar; return true; }
  if (s == "EITstar") { out = PlannerType::EITstar; return true; }
  return false;
}

// All per-planner tunables, with their defaults inline here so this header is the
// single place to tune any planner — edit a field and rebuild the core. Defaults
// mirror OMPL's own, except RRT*'s goal_bias (raised from 0.05). Each field names
// the OMPL setter it drives in makePlanner(). Knobs OMPL exposes but we leave at
// their library default are intentionally omitted.
struct PlannerConfig {
  struct RrtStar {
    double range = 1.0;          // setRange [m]; <=0 => OMPL auto-size
    double goal_bias = 0.15;     // setGoalBias (OMPL default 0.05, raised)
    double rewire_factor = 1.1;  // setRewireFactor
    bool informed = true;        // setInformedSampling (ellipse after 1st soln)
    bool k_nearest = true;       // setKNearest
  } rrtstar;

  struct BitStar {
    unsigned samples_per_batch = 100;  // setSamplesPerBatch
    double rewire_factor = 1.1;        // setRewireFactor
    bool use_k_nearest = true;         // setUseKNearest
    bool pruning = true;               // setPruning
    bool jit_sampling = false;         // setJustInTimeSampling
  } bitstar;

  // ABIT* IS-A BIT*, so it also takes the BitStar fields above; these are the
  // extra inflation/truncation knobs it adds. Defaults are OMPL's.
  struct AbitStar {
    double initial_inflation = 1.0e6;  // setInitialInflationFactor
    double inflation_scaling = 10.0;   // setInflationScalingParameter
    double truncation_scaling = 5.0;   // setTruncationScalingParameter
  } abitstar;

  struct AitStar {
    unsigned batch_size = 100;   // setBatchSize
    double rewire_factor = 1.1;  // setRewireFactor
    bool use_k_nearest = true;   // setUseKNearest
  } aitstar;

  struct EitStar {
    unsigned batch_size = 100;     // setBatchSize
    bool use_k_nearest = true;     // setUseKNearest
    double suboptimality = 5.0;    // setSuboptimalityFactor
    double radius_factor = 1.001;  // setRadiusFactor
  } eitstar;
};

// Geometric global planner: searches an SE(3) state space with a selectable OMPL
// optimal planner (see PlannerType) against an occupancy octree and returns a
// collision-free, clearance-aware waypoint list.
class GeometricPlanner {
public:
  // Returns the clearance (metres to the nearest obstacle) at a world point.
  using ClearanceFn = std::function<double(double x, double y, double z)>;

  // planning_time is the optimisation budget per solve [s]. Every supported
  // planner is anytime — it keeps improving the solution until the budget
  // expires — so a larger value returns a more optimal path at the cost of a
  // longer solve.
  explicit GeometricPlanner(const MapHandle& octree, double planning_time = 3.0);
  ~GeometricPlanner() = default;

  // Enable clearance-aware optimisation. The path cost becomes
  //   integral over the path of ( 1 + weight * max(0, threshold - clearance) ),
  // so the planner rewards both short length and distance from obstacles, and a
  // roomier route can outrank a shorter one that hugs an obstacle. threshold is
  // the clearance beyond which extra room stops helping. With no clearance set
  // (the default) the planner optimises pure path length.
  void setClearance(ClearanceFn clearance, double weight, double threshold);

  // Score the cost with a DIFFERENT distance field from the one the collision
  // check uses. Without this, the single field passed to setClearance answers
  // both questions. The host separates them when the two questions have
  // different answers.
  //
  // The collision check asks "would the vehicle hit something here". It must
  // read unmapped space as free, otherwise the informed planners find no path
  // to a goal beyond the mapped frontier at all. The cost asks "is this
  // somewhere we would rather fly", and there the boundary of what has been
  // mapped is worth being repelled by, because a path that skims it is a path
  // whose committed prefix gets cut short downstream.
  //
  // Passing a conservative (frontier-stamped) field here therefore makes the
  // search prefer routes that stay well inside known-free space, without
  // turning the stamped frontier into a wall the search cannot cross.
  //
  // Order-independent with setClearance: whichever is called last, the cost
  // uses this field when one is set and falls back to the validity field when
  // it is not. Pass an empty function to clear it. weight and threshold stay
  // where setClearance put them; this changes only which field is sampled.
  void setCostClearance(ClearanceFn clearance);

  // Select which OMPL planner planPath builds. The per-planner parameters come
  // from the PlannerConfig defaults in this header. Default is RRT*.
  void setPlannerType(PlannerType type) { planner_type_ = type; }

  // Best-effort goal mode. Off (default): an approximate solution that stops more
  // than kGoalFlexibility short of the goal is rejected (planPath returns false),
  // so a goal in unreachable/unmapped space yields no path. On: that approximate
  // solution is accepted and returned — a path to the reachable point closest to
  // the goal (which, with frontier stamped as an obstacle, sits at the frontier
  // edge pointing at the goal). Combined with continuous replanning this lets the
  // host chase a goal through space as it is mapped, "getting as close as
  // possible" and advancing as the frontier recedes. lastGoalGap() reports how far
  // short the returned path stops.
  void setBestEffort(bool on) { best_effort_ = on; }

  // Straight-line distance [m] from the endpoint of the most recent planPath
  // solution to the goal it was asked for: 0 for an exact solution, positive for
  // an approximate (best-effort) one that stops short, +infinity if the last
  // planPath found no solution at all. Lets the host score best-effort progress
  // (a candidate reaching a smaller gap has advanced toward the goal).
  double lastGoalGap() const { return last_goal_gap_; }

  // How far short of the goal a solution may stop and still count as reaching it
  // [m] (see kGoalFlexibility). Static so the host can decide "arrived" without a
  // planner instance.
  static constexpr double goalFlexibility() { return kGoalFlexibility; }

  // Plan from start to goal (each [x, y, z]); fills result_path with waypoints.
  // Returns false if no solution is found within the planner time budget. In
  // strict mode also returns false for an approximate solution stopping more than
  // kGoalFlexibility from the goal; in best-effort mode (setBestEffort) such a
  // solution is accepted and lastGoalGap() reports the residual.
  bool planPath(const std::vector<double>& start,
                const std::vector<double>& goal,
                std::vector<std::vector<double>>& result_path);

  // True if every waypoint and every segment between consecutive waypoints of
  // the given path is collision-free against the current map. Used to decide
  // whether a previously planned path is still safe or needs replanning.
  bool isPathValid(const std::vector<std::vector<double>>& path) const;

  // Cost of an existing waypoint path under the current optimisation objective
  // (the same metric planPath minimises). Lets a candidate plan be compared
  // against the committed one for a hysteresis-gated replan decision.
  double pathCost(const std::vector<std::vector<double>>& path) const;

  // pathCost split into its two additive terms. The objective integrates
  // (1 + w·max(0, threshold − clearance)) over arc length, so the baseline unit
  // term integrates to the path's geometric length and the remainder is the
  // obstacle-proximity penalty. total == pathCost(path). Purely diagnostic: lets
  // a replan log show whether a path scores high because it is long or because it
  // hugs obstacles. Same +infinity convention as pathCost for <2-point paths.
  struct CostBreakdown {
    double length;     // arc-length term [m] (== pathCost with no clearance field)
    double clearance;  // obstacle-proximity penalty term (total − length)
    double total;      // length + clearance (== pathCost(path))
  };
  CostBreakdown costBreakdown(const std::vector<std::vector<double>>& path) const;

  // Smallest clearance (distance to the nearest obstacle, metres) along the path,
  // sampled finely, considering only the points the validity check enforces —
  // i.e. those outside the start escape sphere (points within kStartEscapeRadius
  // of the first waypoint are exempt and skipped, mirroring isStateValid). So
  // this reports the lowest clearance that could actually flag the path, not the
  // absolute minimum. Requires a clearance field (see setClearance); returns
  // +infinity when none is set, the path has fewer than two points, or every
  // sampled point falls inside the escape sphere. Purely a diagnostic.
  double minClearance(const std::vector<std::vector<double>>& path) const;

  // The hard clearance the validity check enforces away from the start [m].
  double collisionMargin() const { return kCollisionMargin; }

  // Radius [m] the host should leave frontier-free around the drone when stamping
  // frontier as occupied (see kFrontierKeepOutRadius). Static so the ROS wrapper
  // can read it without a planner instance.
  static constexpr double frontierKeepOutRadius() { return kFrontierKeepOutRadius; }

  // Snapshot of the search tree from the most recent planPath, for debug
  // visualisation. `nodes` are the tree vertices in world XYZ; `edges` index
  // into `nodes`. Populated only while tree recording is enabled (see
  // setRecordTree) — otherwise left empty so a flight build never builds it.
  struct SearchTree {
    std::vector<std::array<double, 3>> nodes;
    std::vector<std::pair<int, int>> edges;
  };

  // Enable/disable capturing the search tree on each planPath. Off by default:
  // when off, planPath never calls OMPL's getPlannerData(), so there is no cost.
  // Intended to be driven by a single debug parameter.
  void setRecordTree(bool on) { record_tree_ = on; }

  // The tree captured by the most recent planPath (empty if recording was off
  // or no solve has run). Returned by const reference; copy it if you need it on
  // another thread.
  const SearchTree& searchTree() const { return last_tree_; }

private:
  bool isStateValid(const ompl::base::State* state);
  ompl::base::OptimizationObjectivePtr makeObjective() const;

  // Construct and configure the OMPL planner selected by planner_type_, applying
  // the matching PlannerConfig sub-struct. Called once per planPath.
  ompl::base::PlannerPtr makePlanner() const;

  // Clearance-preserving shortcut pass for the clearance-aware case. Removes
  // interior waypoints whose detour is not "paying for itself": a vertex is
  // dropped only when the straight bypass is collision-free AND doing so does
  // not raise the clearance-aware path cost (so a corner-cut that creeps toward
  // a wall, raising the proximity penalty more than it saves in length, is
  // rejected). This straightens the jagged planner result without the wall-hugging
  // that plain length-only simplifyMax would cause.
  void shortcutClearanceAware(std::vector<std::vector<double>>& waypoints) const;

  // True when any clearance field is set, i.e. the planner is running in
  // clearance-aware mode rather than optimising pure path length. Either field
  // alone is enough to put it in that mode.
  bool clearanceMode() const {
    return static_cast<bool>(clearance_fn_) || static_cast<bool>(cost_clearance_fn_);
  }

  // The field the cost objective samples: the cost-specific one when the host
  // set it (see setCostClearance), otherwise the validity field. Keeping the
  // choice here rather than at the call sites is what makes the two setters
  // order-independent.
  const ClearanceFn& costClearanceFn() const {
    return cost_clearance_fn_ ? cost_clearance_fn_ : clearance_fn_;
  }

  MapHandle octree_ptr_;
  ompl::base::StateSpacePtr space_;
  ompl::base::SpaceInformationPtr si_;

  double planning_time_;  // optimisation budget per solve [s]
  PlannerType planner_type_ = PlannerType::RRTstar;  // which OMPL planner to build
  PlannerConfig params_{};                           // per-planner tunables (header defaults)

  // Centre of the start-state collision exemption (see isStateValid): the start
  // passed to planPath, or the first waypoint in isPathValid. Set transiently
  // before each validity sweep, hence mutable so the const isPathValid can
  // anchor it.
  mutable std::array<double, 3> start_pos_{};

  // The validity field: what isStateValid tests against the margin, and what
  // minClearance reports. Null => optimise pure path length with the fallback
  // octree scan for validity.
  ClearanceFn clearance_fn_;
  // Optional separate field for the cost only (see setCostClearance). Null =>
  // the cost samples clearance_fn_, which is the single-field behaviour.
  ClearanceFn cost_clearance_fn_;
  double clearance_weight_ = 1.0;       // obstacle-proximity penalty weight
  double clearance_threshold_ = 1.0;    // clearance saturation distance [m]

  // Validity margins [m]. A state is free when its clearance exceeds the margin:
  // kCollisionMargin in general, the reduced kStartMargin within kStartEscapeRadius
  // of the start so a parked/lifting drone can root the search (see isStateValid).
  static constexpr double kCollisionMargin = 0.5;
  static constexpr double kStartMargin = 0.0;
  static constexpr double kStartEscapeRadius = 0.5;

  // Radius [m] around the drone the host leaves frontier-free when burning
  // frontier into the occupancy map (consumed by the ROS wrapper's frontier
  // stamping, not the planner itself). Kept here beside kStartEscapeRadius since
  // it serves the same "let a drone boxed in by unknown space still root the
  // search" purpose. Must stay well below kStartEscapeRadius/kCollisionMargin so
  // the surrounding frontier's margin reseals the gap — otherwise the planner
  // could route out through the hole into (free-reading) unknown space.
  static constexpr double kFrontierKeepOutRadius = 0.5;

  // Largest straight bypass [m] the clearance-aware shortcut will create. Caps
  // how much waypoint density it removes so the downstream min-snap optimiser
  // still has intermediate points on long runs instead of one giant segment.
  static constexpr double kMaxShortcutSegment = 4.0;

  bool best_effort_ = false;    // accept approximate solutions (see setBestEffort)
  double last_goal_gap_ = 0.0;  // residual goal distance of the last solve (see lastGoalGap)

  bool record_tree_ = false;  // capture the OMPL tree each solve (debug viz only)
  SearchTree last_tree_;      // tree from the most recent planPath

  // goal_flexibility [m]: how far short of the goal a solution may stop and
  // still be accepted. A planner may return an approximate solution that does not
  // reach the goal (e.g. the goal sits in tight clearance); accept it only within this
  // tolerance, otherwise reject the plan rather than commit to a path that stops
  // short. Raise it to tolerate harder-to-reach goals, lower it to insist on
  // (near-)exact arrival.
  static constexpr double kGoalFlexibility = 0.3;
};

}  // namespace drone_core::planning
