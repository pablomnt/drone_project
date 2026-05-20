#pragma once
#include <vector>
#include <Eigen/Dense>

namespace global_planner {

class TrajectoryOptimizer {
public:
    TrajectoryOptimizer() = default;

    bool generateTrajectory(const std::vector<std::vector<double>>& waypoints,
                            double average_speed,
                            std::vector<std::vector<double>>& trajectory_out);

private:
    Eigen::VectorXd solveKKT(const std::vector<double>& waypoints_1d, 
                             const std::vector<double>& times);
};

}