#pragma once

#include <memory>
#include <vector>

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
  explicit RrtStarPlanner(const MapHandle& octree);
  ~RrtStarPlanner() = default;

  // Plan from start to goal (each [x, y, z]); fills result_path with waypoints.
  // Returns false if no solution is found within the planner time budget.
  bool planPath(const std::vector<double>& start,
                const std::vector<double>& goal,
                std::vector<std::vector<double>>& result_path);

private:
  bool isStateValid(const ompl::base::State* state);

  MapHandle octree_ptr_;
  ompl::base::StateSpacePtr space_;
  ompl::base::SpaceInformationPtr si_;

  double robot_radius_ = 0.3;  // safety padding around the vehicle [m]
};

}  // namespace drone_core::planning
