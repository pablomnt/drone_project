#pragma once

#include <memory>
#include <vector>
#include <Eigen/Dense>
#include <octomap/octomap.h>

// Use the exact file names we found in your include folder
#include <decomp_util/line_segment.h>
#include <decomp_geometry/polyhedron.h>

namespace global_planner {

// These type aliases ensure Eigen memory alignment
typedef LineSegment<3> LineSegment3D;

class SfcOptimizer {
public:
    SfcOptimizer(const std::shared_ptr<octomap::OcTree>& octree);
    ~SfcOptimizer() = default;

    bool generateTrajectory(const std::vector<std::vector<double>>& path_waypoints,
                            double desired_speed,
                            std::vector<std::vector<double>>& optimized_trajectory);

private:
    std::shared_ptr<octomap::OcTree> octree_ptr_;
    vec_E<Polyhedron3D> safe_corridors_;

    // Use vec_E to satisfy the compiler's alignment requirements
    vec_Vecf<3> extractObstaclesFromMap();
    bool generateConvexCorridors(const vec_Vecf<3>& eigen_path);
    bool solveMinimumSnapQP(const vec_Vecf<3>& eigen_path, 
                            double speed, 
                            std::vector<std::vector<double>>& final_traj);
};

} // namespace global_planner