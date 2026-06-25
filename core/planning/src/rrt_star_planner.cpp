#include "drone_core/planning/rrt_star_planner.hpp"

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

namespace drone_core::planning {

RrtStarPlanner::RrtStarPlanner(const MapHandle& octree)
    : octree_ptr_(octree) {
  // SE(3) state space leaves room for orientation should the vehicle footprint
  // ever become asymmetric, even though the current collision check is radial.
  space_ = std::make_shared<ompl::base::SE3StateSpace>();

  // Bounds tuned for the office test environment.
  ompl::base::RealVectorBounds bounds(3);
  bounds.setLow(0, -15.0);
  bounds.setHigh(0, 15.0);
  bounds.setLow(1, -15.0);
  bounds.setHigh(1, 15.0);

  // Keep the search above the floor and below eye level.
  bounds.setLow(2, 0.3);
  bounds.setHigh(2, 2.5);

  space_->as<ompl::base::SE3StateSpace>()->setBounds(bounds);

  si_ = std::make_shared<ompl::base::SpaceInformation>(space_);
  si_->setStateValidityChecker([this](const ompl::base::State* state) {
    return isStateValid(state);
  });
  si_->setStateValidityCheckingResolution(0.01);
  si_->setup();
}

bool RrtStarPlanner::isStateValid(const ompl::base::State* state) {
  if (!octree_ptr_) return false;

  const auto* se3state = state->as<ompl::base::SE3StateSpace::StateType>();
  const auto* pos = se3state->as<ompl::base::RealVectorStateSpace::StateType>(0);

  const double safety_dist = 0.4;  // inflate obstacles to clear tight doorways
  const double res = octree_ptr_->getResolution();

  for (double dx = -safety_dist; dx <= safety_dist; dx += res) {
    for (double dy = -safety_dist; dy <= safety_dist; dy += res) {
      const octomap::point3d query(pos->values[0] + dx, pos->values[1] + dy, pos->values[2]);
      const auto* node = octree_ptr_->search(query);

      // Reject only confirmed obstacles. Unknown space (null node) is treated
      // as free so the planner can route into not-yet-mapped regions.
      if (node != nullptr && octree_ptr_->isNodeOccupied(node)) {
        return false;
      }
    }
  }
  return true;
}

bool RrtStarPlanner::planPath(const std::vector<double>& start_vec,
                              const std::vector<double>& goal_vec,
                              std::vector<std::vector<double>>& result_path) {
  ompl::base::ScopedState<ompl::base::SE3StateSpace> start(space_);
  ompl::base::ScopedState<ompl::base::SE3StateSpace> goal(space_);

  start->setXYZ(start_vec[0], start_vec[1], start_vec[2]);
  start->as<ompl::base::SO3StateSpace::StateType>(1)->setIdentity();

  goal->setXYZ(goal_vec[0], goal_vec[1], goal_vec[2]);
  goal->as<ompl::base::SO3StateSpace::StateType>(1)->setIdentity();

  auto pdef = std::make_shared<ompl::base::ProblemDefinition>(si_);
  pdef->setStartAndGoalStates(start, goal);
  pdef->setOptimizationObjective(
      std::make_shared<ompl::base::PathLengthOptimizationObjective>(si_));

  auto planner = std::make_shared<ompl::geometric::RRTstar>(si_);
  planner->setProblemDefinition(pdef);
  planner->setup();

  const ompl::base::PlannerStatus solved = planner->ompl::base::Planner::solve(3.0);
  if (!solved) {
    return false;
  }

  auto path = std::static_pointer_cast<ompl::geometric::PathGeometric>(pdef->getSolutionPath());

  // Straighten the jagged RRT* result. Deliberately skip B-spline smoothing so
  // the original waypoints survive for the minimum-snap optimiser downstream.
  ompl::geometric::PathSimplifier simplifier(si_);
  simplifier.simplifyMax(*path);

  for (std::size_t i = 0; i < path->getStateCount(); ++i) {
    const auto* state = path->getState(i)->as<ompl::base::SE3StateSpace::StateType>();
    const auto* pos = state->as<ompl::base::RealVectorStateSpace::StateType>(0);
    result_path.push_back({pos->values[0], pos->values[1], pos->values[2]});
  }
  return true;
}

}  // namespace drone_core::planning
