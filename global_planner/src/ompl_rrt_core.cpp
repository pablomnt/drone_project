#include "global_planner/ompl_rrt_core.hpp"
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

namespace global_planner {

OmplPlanner::OmplPlanner(const std::shared_ptr<octomap::OcTree>& octree) 
    : octree_ptr_(octree) {
    
    // We define an SE3 state space (x, y, z + orientation), in case the shape of the drone is not symmetrical.
    space_ = std::make_shared<ompl::base::SE3StateSpace>();

    // Set boundaries for the office map
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(0, -15.0); // X min
    bounds.setHigh(0, 15.0); // X max
    bounds.setLow(1, -15.0); // Y min
    bounds.setHigh(1, 15.0); // Y max
    
    // Constraint to keep the drone from diving into the floor-less void
    bounds.setLow(2, 0.3);   // Z min (Floor safety)
    bounds.setHigh(2, 2.5);  // Z max (Ceiling/Eye level)

    space_->as<ompl::base::SE3StateSpace>()->setBounds(bounds);

    // SpaceInformation handles state validity and checking
    si_ = std::make_shared<ompl::base::SpaceInformation>(space_);
    
    // Link the collision checker callback
    si_->setStateValidityChecker([this](const ompl::base::State* state) {
        return isStateValid(state);
    });

    si_->setStateValidityCheckingResolution(0.01);

    si_->setup();
}

bool OmplPlanner::isStateValid(const ompl::base::State* state) {
    if (!octree_ptr_) return false;

    const auto* se3state = state->as<ompl::base::SE3StateSpace::StateType>();
    const auto* pos = se3state->as<ompl::base::RealVectorStateSpace::StateType>(0);

    // Reduce safety distance slightly to 0.2m to help with tight doors
    double safety_dist = 0.4; 
    double res = octree_ptr_->getResolution();

    for (double dx = -safety_dist; dx <= safety_dist; dx += res) {
        for (double dy = -safety_dist; dy <= safety_dist; dy += res) {
            octomap::point3d query(pos->values[0] + dx, pos->values[1] + dy, pos->values[2]);
            auto node = octree_ptr_->search(query);

            // ONLY return false if it's a CONFIRMED wall.
            // If node is nullptr (unknown), we assume it's free space for now.
            if (node != nullptr && octree_ptr_->isNodeOccupied(node)) {
                return false; 
            }
        }
    }
    return true; 
}


bool OmplPlanner::planPath(const std::vector<double>& start_vec, 
                          const std::vector<double>& goal_vec, 
                          std::vector<std::vector<double>>& result_path) {

    // Define Start and Goal states
    ompl::base::ScopedState<ompl::base::SE3StateSpace> start(space_);
    ompl::base::ScopedState<ompl::base::SE3StateSpace> goal(space_);

    start->setXYZ(start_vec[0], start_vec[1], start_vec[2]);
    start->as<ompl::base::SO3StateSpace::StateType>(1)->setIdentity();

    goal->setXYZ(goal_vec[0], goal_vec[1], goal_vec[2]);
    goal->as<ompl::base::SO3StateSpace::StateType>(1)->setIdentity();

    // Create the problem instance
    auto pdef = std::make_shared<ompl::base::ProblemDefinition>(si_);
    pdef->setStartAndGoalStates(start, goal);

    // Tell OMPL to find the shortest path possible
    pdef->setOptimizationObjective(std::make_shared<ompl::base::PathLengthOptimizationObjective>(si_));
    // Use RRT*
    auto planner = std::make_shared<ompl::geometric::RRTstar>(si_);
    planner->setProblemDefinition(pdef);
    planner->setup();

    // Solve with a 1.0 second timeout
    ompl::base::PlannerStatus solved = planner->ompl::base::Planner::solve(1.0);

    if (solved) {
        auto path = std::static_pointer_cast<ompl::geometric::PathGeometric>(pdef->getSolutionPath());
        
        // Smooth the jagged RRT* lines
        ompl::geometric::PathSimplifier simplifier(si_);
        simplifier.simplifyMax(*path);
        //B-spline is removed to prevent number of waypoints from increasing too much and keeping the original waypoints for minimum snap trajectory generation
        //simplifier.smoothBSpline(*path);

        // Convert OMPL states back to a simple vector for ROS
        for (std::size_t i = 0; i < path->getStateCount(); ++i) {
            const auto* state = path->getState(i)->as<ompl::base::SE3StateSpace::StateType>();
            const auto* pos = state->as<ompl::base::RealVectorStateSpace::StateType>(0);
            result_path.push_back({pos->values[0], pos->values[1], pos->values[2]});
        }
        return true;
    }

    return false;
}

} // namespace global_planner