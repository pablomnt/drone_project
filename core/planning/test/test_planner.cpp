// Exercises the planning stack end to end without ROS: plan a geometric path
// through a small octree, then fit a minimum-snap trajectory to a waypoint list.

#include "drone_core/planning/geometric_planner.hpp"
#include "drone_core/planning/min_snap_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>

int main() {
  using namespace drone_core::planning;

  int failures = 0;

  // Empty octree: unmapped space is treated as free, so a short hop through the
  // valid bounds should always be solvable.
  auto octree = std::make_shared<octomap::OcTree>(0.1);

  const std::vector<double> start = {0.0, 0.0, 1.0};
  const std::vector<double> goal = {2.0, 0.0, 1.0};

  // Every selectable planner must construct, configure and solve through free
  // space with the same interface (proves the factory + shared SI/objective work).
  for (PlannerType type : {PlannerType::RRTstar, PlannerType::BITstar,
                           PlannerType::ABITstar, PlannerType::AITstar,
                           PlannerType::EITstar}) {
    GeometricPlanner planner(octree, /*planning_time=*/1.0);
    planner.setPlannerType(type);
    std::vector<std::vector<double>> path;
    if (!planner.planPath(start, goal, path) || path.empty()) {
      std::cerr << "FAIL: " << toString(type)
                << " did not return a path through free space\n";
      ++failures;
    }
  }

  // Best-effort vs strict goal handling on an unreachable goal. Model a solid block
  // straddling the goal with an analytic clearance field (distance to the box) —
  // the same obstacle-model interface the live system feeds from its EDT, so this
  // exercises the O(1) validity + bounded clearance-aware post-processing path the
  // real planner uses, not the slow standalone octree-scan + simplifyMax fallback.
  // The goal sits inside the block, so no collision-free path reaches it and OMPL
  // returns an approximate solution stopping at the block face. Strict mode
  // (default) must reject that (planPath returns false, caller sees "no path");
  // best-effort must accept it and report a positive goal gap — the shortfall to
  // the closest reachable point. This is what lets the drone head toward a goal
  // beyond the mapped frontier instead of refusing it.
  {
    auto empty = std::make_shared<octomap::OcTree>(0.1);
    // Solid block [1.5,2.5] x [-0.5,0.5] x [0.5,1.5]; clearance is the Euclidean
    // distance from a point to that box (0 on/inside it).
    auto boxClearance = [](double x, double y, double z) {
      const double dx = std::max({1.5 - x, x - 2.5, 0.0});
      const double dy = std::max({-0.5 - y, y - 0.5, 0.0});
      const double dz = std::max({0.5 - z, z - 1.5, 0.0});
      return std::sqrt(dx * dx + dy * dy + dz * dz);
    };
    const std::vector<double> buried_goal = {2.0, 0.0, 1.0};  // centre of the block

    GeometricPlanner strict(empty, /*planning_time=*/0.5);
    strict.setClearance(boxClearance, /*weight=*/4.0, /*threshold=*/1.0);
    std::vector<std::vector<double>> spath;
    if (strict.planPath(start, buried_goal, spath)) {
      std::cerr << "FAIL: strict planPath accepted a path to a walled-off goal\n";
      ++failures;
    }

    GeometricPlanner effort(empty, /*planning_time=*/0.5);
    effort.setClearance(boxClearance, /*weight=*/4.0, /*threshold=*/1.0);
    effort.setBestEffort(true);
    std::vector<std::vector<double>> epath;
    if (!effort.planPath(start, buried_goal, epath) || epath.empty()) {
      std::cerr << "FAIL: best-effort planPath returned no path toward a walled-off goal\n";
      ++failures;
    } else if (effort.lastGoalGap() <= GeometricPlanner::goalFlexibility()) {
      std::cerr << "FAIL: best-effort path unexpectedly reached the buried goal (gap="
                << effort.lastGoalGap() << " m)\n";
      ++failures;
    }
  }

  // Minimum-snap fit over a simple corridor of waypoints.
  const std::vector<std::vector<double>> waypoints = {
      {0.0, 0.0, 1.0}, {1.0, 0.0, 1.0}, {2.0, 0.0, 1.0}};
  MinSnapTimeOptimizer optimizer;
  std::vector<std::vector<double>> trajectory;

  if (!optimizer.generateOptimizedTrajectory(waypoints, trajectory) || trajectory.empty()) {
    std::cerr << "FAIL: minimum-snap optimiser produced no trajectory\n";
    ++failures;
  }

  if (failures == 0) {
    std::cout << "planner: all checks passed\n";
    return 0;
  }
  return 1;
}
