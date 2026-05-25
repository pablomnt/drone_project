#pragma once
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "osqp/osqp.h"

namespace global_planner {

class OSQPTrajectoryOptimizer {
public:
    OSQPTrajectoryOptimizer() = default;

    // We now pass a corridor_width to define the inequality bounds (e.g., 1.0 meters)
    bool generateTrajectory(const std::vector<std::vector<double>>& waypoints,
                            double average_speed,
                            double corridor_width,
                            std::vector<std::vector<double>>& trajectory_out);

private:
    Eigen::VectorXd solveOSQP(const std::vector<double>& waypoints_1d, 
                              const std::vector<double>& times,
                              double corridor_width);

    // Helper to convert Eigen sparse matrices to OSQP format
    void eigenToCSC(const Eigen::SparseMatrix<double>& mat,
                    std::vector<c_float>& values,
                    std::vector<c_int>& inner_indices,
                    std::vector<c_int>& outer_pointers);
};

} // namespace global_planner