// Exercises the planning stack end to end without ROS: plan a geometric path
// through a small octree, then fit a minimum-snap trajectory to a waypoint list.

#include "drone_core/planning/min_snap_trajectory.hpp"
#include "drone_core/planning/rrt_star_planner.hpp"

#include <iostream>
#include <memory>

int main() {
  using namespace drone_core::planning;

  int failures = 0;

  // Empty octree: unmapped space is treated as free, so a short hop through the
  // valid bounds should always be solvable.
  auto octree = std::make_shared<octomap::OcTree>(0.1);
  RrtStarPlanner planner(octree);

  const std::vector<double> start = {0.0, 0.0, 1.0};
  const std::vector<double> goal = {2.0, 0.0, 1.0};
  std::vector<std::vector<double>> path;

  if (!planner.planPath(start, goal, path) || path.empty()) {
    std::cerr << "FAIL: RRT* did not return a path through free space\n";
    ++failures;
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
