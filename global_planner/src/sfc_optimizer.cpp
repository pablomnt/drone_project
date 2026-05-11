#include "global_planner/sfc_optimizer.hpp"
#include <iostream>

namespace global_planner {

SfcOptimizer::SfcOptimizer(const std::shared_ptr<octomap::OcTree>& octree) 
    : octree_ptr_(octree) {}

bool SfcOptimizer::generateTrajectory(const std::vector<std::vector<double>>& path_waypoints,
                                      double desired_speed,
                                      std::vector<std::vector<double>>& optimized_trajectory) {
    
    if (path_waypoints.size() < 2) return false;

    // 1. Convert standard vectors to Eigen vectors for DecompUtil
    vec_E<Eigen::Vector3d> eigen_path;
    for (const auto& wp : path_waypoints) {
        eigen_path.push_back(Eigen::Vector3d(wp[0], wp[1], wp[2]));
    }

    // 2. Generate the Safe Flight Corridors
    std::cout << "[SfcOptimizer] Inflating Safe Flight Corridors..." << std::endl;
    if (!generateConvexCorridors(eigen_path)) {
        std::cerr << "[SfcOptimizer] Failed to generate corridors!" << std::endl;
        return false;
    }

    // 3. Run Minimum Snap Optimization inside those corridors
    std::cout << "[SfcOptimizer] Solving Minimum Snap Trajectory..." << std::endl;
    if (!solveMinimumSnapQP(eigen_path, desired_speed, optimized_trajectory)) {
        std::cerr << "[SfcOptimizer] Optimizer failed to find a feasible trajectory!" << std::endl;
        return false;
    }

    return true;
}

std::vector<Eigen::Vector3d> SfcOptimizer::extractObstaclesFromMap() {
    std::vector<Eigen::Vector3d> obstacles;
    // Iterate through the OctoMap and extract centers of occupied voxels
    for (octomap::OcTree::leaf_iterator it = octree_ptr_->begin_leafs(), 
                                        end = octree_ptr_->end_leafs(); it != end; ++it) {
        if (octree_ptr_->isNodeOccupied(*it)) {
            obstacles.push_back(Eigen::Vector3d(it.getX(), it.getY(), it.getZ()));
        }
    }
    return obstacles;
}

bool SfcOptimizer::generateConvexCorridors(const std::vector<Eigen::Vector3d>& eigen_path) {
    // Get all obstacles as a point cloud
    auto obstacles = extractObstaclesFromMap();

    // Initialize the DecompUtil Line Segment tool
    LineSegmentDecomp3D decomp_util(eigen_path);

    // Set bounding box (How far out should it look? e.g., 5 meters)
    decomp_util.set_local_bbox(Eigen::Vector3d(5.0, 5.0, 5.0));

    // Pass the OctoMap obstacles to the algorithm
    decomp_util.set_obs(obstacles);

    // Run the slicing algorithm to inflate the polyhedra
    if (!decomp_util.decompose()) {
        return false;
    }

    // Store the resulting safe rooms (The Ax <= b inequalities)
    safe_corridors_ = decomp_util.get_polyhedrons();
    std::cout << "[SfcOptimizer] Created " << safe_corridors_.size() << " overlapping polyhedra." << std::endl;
    
    return true;
}

bool SfcOptimizer::solveMinimumSnapQP(const std::vector<Eigen::Vector3d>& eigen_path, 
                                      double speed, 
                                      std::vector<std::vector<double>>& final_traj) {
    
    // TODO: Integrate ethz-asl/mav_trajectory_generation here.
    // 1. Allocate time segments based on 'speed'.
    // 2. Set the inequalities from safe_corridors_ as constraints.
    // 3. Minimize the 4th derivative.
    // 4. Sample the resulting polynomial at 100Hz and push to final_traj.
    
    // For now, bypass the optimizer and just return the path to keep the node compiling
    for (const auto& pt : eigen_path) {
        final_traj.push_back({pt.x(), pt.y(), pt.z()});
    }
    return true;
}

} // namespace global_planner