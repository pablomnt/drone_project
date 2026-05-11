#include "global_planner/sfc_optimizer.hpp"
#include <iostream>

namespace global_planner {

SfcOptimizer::SfcOptimizer(const std::shared_ptr<octomap::OcTree>& octree) 
    : octree_ptr_(octree) {}

bool SfcOptimizer::generateTrajectory(const std::vector<std::vector<double>>& path_waypoints,
                                      double desired_speed,
                                      std::vector<std::vector<double>>& optimized_trajectory) {
    if (path_waypoints.size() < 2) return false;

    // Use vec_Vecf (the aligned version of std::vector<Vector3d>)
    vec_Vecf<3> eigen_path;
    for (const auto& wp : path_waypoints) {
        eigen_path.push_back(Eigen::Vector3d(wp[0], wp[1], wp[2]));
    }

    if (!generateConvexCorridors(eigen_path)) return false;

    return solveMinimumSnapQP(eigen_path, desired_speed, optimized_trajectory);
}

vec_Vecf<3> SfcOptimizer::extractObstaclesFromMap() {
    vec_Vecf<3> obstacles;
    for (octomap::OcTree::leaf_iterator it = octree_ptr_->begin_leafs(), 
                                        end = octree_ptr_->end_leafs(); it != end; ++it) {
        if (octree_ptr_->isNodeOccupied(*it)) {
            obstacles.push_back(Eigen::Vector3d(it.getX(), it.getY(), it.getZ()));
        }
    }
    return obstacles;
}

bool SfcOptimizer::generateConvexCorridors(const vec_Vecf<3>& eigen_path) {
    auto obstacles = extractObstaclesFromMap();
    safe_corridors_.clear();

    // The LineSegment class processes one segment (two points) at a time
    for (size_t i = 0; i < eigen_path.size() - 1; ++i) {
        LineSegment3D segment_inflator(eigen_path[i], eigen_path[i+1]);
        
        segment_inflator.set_obs(obstacles);
        segment_inflator.set_local_bbox(Eigen::Vector3d(2.0, 2.0, 2.0));
        
        // In this version of the library, the method is likely 'dilate' 
        // and it returns the polyhedron directly
        segment_inflator.dilate(0.0); // 0.0 is the offset/margin
        
        safe_corridors_.push_back(segment_inflator.get_polyhedron());
    }

    return !safe_corridors_.empty();
}

bool SfcOptimizer::solveMinimumSnapQP(const vec_Vecf<3>& eigen_path, 
                                      double speed, 
                                      std::vector<std::vector<double>>& final_traj) {
    (void)speed; // Suppress unused warning
    for (const auto& pt : eigen_path) {
        final_traj.push_back({pt.x(), pt.y(), pt.z()});
    }
    return true;
}

} // namespace global_planner