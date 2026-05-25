#ifndef OMPL_RRT_CORE_HPP
#define OMPL_RRT_CORE_HPP

#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/PathSimplifier.h>

#include <octomap/octomap.h>
#include <vector>
#include <memory>

namespace global_planner {

class OmplPlanner {
public:
    // We pass a shared pointer to the map so we aren't copying massive voxel trees
    OmplPlanner(const std::shared_ptr<octomap::OcTree>& octree);
    ~OmplPlanner() = default;

    // The main entry point to find a path
    bool planPath(const std::vector<double>& start, 
                  const std::vector<double>& goal, 
                  std::vector<std::vector<double>>& result_path);

private:
    // This is the core callback OMPL uses to check for collisions
    bool isStateValid(const ompl::base::State* state);

    std::shared_ptr<octomap::OcTree> octree_ptr_;
    
    // OMPL setup objects
    ompl::base::StateSpacePtr space_;
    ompl::base::SpaceInformationPtr si_;

    // Safety padding for the drone (meters)
    double robot_radius_ = 0.3; 
};

} // namespace global_planner

#endif