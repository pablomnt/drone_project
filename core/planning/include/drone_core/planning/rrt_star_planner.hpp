#pragma once

#include <array>
#include <functional>
#include <memory>
#include <vector>

#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <octomap/octomap.h>

namespace drone_core::planning {

// Map handle shared between the planner and its host. It is a plain octomap
// pointer (the octomap library, not the ROS octomap_msgs package), so the
// planner carries no middleware dependency.
using MapHandle = std::shared_ptr<octomap::OcTree>;

// Geometric global planner: searches an SE(3) state space with RRT* against an
// occupancy octree and returns a collision-free, simplified waypoint list.
class RrtStarPlanner {
public:
  // Returns the clearance (metres to the nearest obstacle) at a world point.
  using ClearanceFn = std::function<double(double x, double y, double z)>;

  // planning_time is the RRT* optimisation budget per solve [s]. RRT* keeps
  // improving the solution until the budget expires, so a larger value returns a
  // more optimal path at the cost of a longer solve.
  explicit RrtStarPlanner(const MapHandle& octree, double planning_time = 3.0);
  ~RrtStarPlanner() = default;

  // Enable clearance-aware optimisation. The path cost becomes
  //   integral over the path of ( 1 + weight * max(0, threshold - clearance) ),
  // so the planner rewards both short length and distance from obstacles, and a
  // roomier route can outrank a shorter one that hugs an obstacle. threshold is
  // the clearance beyond which extra room stops helping. With no clearance set
  // (the default) the planner optimises pure path length.
  void setClearance(ClearanceFn clearance, double weight, double threshold);

  // Plan from start to goal (each [x, y, z]); fills result_path with waypoints.
  // Returns false if no solution is found within the planner time budget.
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

  // Smallest clearance (distance to the nearest obstacle, metres) along the path,
  // sampled finely. Requires a clearance field (see setClearance); returns
  // +infinity when none is set or the path has fewer than two points. Purely a
  // diagnostic — it is what the validity check flags if it drops below the margin.
  double minClearance(const std::vector<std::vector<double>>& path) const;

  // The hard clearance the validity check enforces away from the start [m].
  double collisionMargin() const { return kCollisionMargin; }

private:
  bool isStateValid(const ompl::base::State* state);
  ompl::base::OptimizationObjectivePtr makeObjective() const;

  MapHandle octree_ptr_;
  ompl::base::StateSpacePtr space_;
  ompl::base::SpaceInformationPtr si_;

  double planning_time_;  // RRT* optimisation budget per solve [s]

  // Centre of the start-state collision exemption (see isStateValid): the start
  // passed to planPath, or the first waypoint in isPathValid. Set transiently
  // before each validity sweep, hence mutable so the const isPathValid can
  // anchor it.
  mutable std::array<double, 3> start_pos_{};

  ClearanceFn clearance_fn_;            // null => optimise pure path length
  double clearance_weight_ = 1.0;       // obstacle-proximity penalty weight
  double clearance_threshold_ = 1.0;    // clearance saturation distance [m]

  // Validity margins [m]. A state is free when its clearance exceeds the margin:
  // kCollisionMargin in general, the reduced kStartMargin within kStartEscapeRadius
  // of the start so a parked/lifting drone can root the search (see isStateValid).
  static constexpr double kCollisionMargin = 0.5;
  static constexpr double kStartMargin = 0.0;
  static constexpr double kStartEscapeRadius = 0.5;

  // goal_flexibility [m]: how far short of the goal an RRT* solution may stop and
  // still be accepted. RRT* may return an approximate solution that does not reach
  // the goal (e.g. the goal sits in tight clearance); accept it only within this
  // tolerance, otherwise reject the plan rather than commit to a path that stops
  // short. Raise it to tolerate harder-to-reach goals, lower it to insist on
  // (near-)exact arrival.
  static constexpr double kGoalFlexibility = 0.3;
};

}  // namespace drone_core::planning
