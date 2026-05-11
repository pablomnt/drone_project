#pragma once

#include <memory>
#include <vector>
#include <Eigen/Dense>
#include <octomap/octomap.h>

// DecompUtil headers (assuming standard DecompROS installation)
#include <decomp_util/line_segment_decomp.h>
#include <decomp_geometry/polyhedron.h>

namespace global_planner {

class SfcOptimizer {
public:
    // Initialize with the OctoMap
    SfcOptimizer(const std::shared_ptr<octomap::OcTree>& octree);
    ~SfcOptimizer() = default;

    // The main pipeline function called by your ROS node
    bool generateTrajectory(const std::vector<std::vector<double>>& path_waypoints,
                            double desired_speed,
                            std::vector<std::vector<double>>& optimized_trajectory);

private:
    std::shared_ptr<octomap::OcTree> octree_ptr_;
    
    // Extracted polyhedra (the Ax <= b matrices)
    vec_E<Polyhedron3D> safe_corridors_;

    // Pipeline steps
    std::vector<Eigen::Vector3d> extractObstaclesFromMap();
    bool generateConvexCorridors(const std::vector<Eigen::Vector3d>& eigen_path);
    bool solveMinimumSnapQP(const std::vector<Eigen::Vector3d>& eigen_path, 
                            double speed, 
                            std::vector<std::vector<double>>& final_traj);
};

} // namespace global_planner