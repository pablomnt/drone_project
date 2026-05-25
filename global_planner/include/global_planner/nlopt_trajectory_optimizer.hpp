#pragma once
#include <vector>
#include "global_planner/trajectory_optimizer.hpp"

namespace global_planner {

struct OptimizerContext {
    TrajectoryOptimizer* solver;
    std::vector<std::vector<double>> waypoints;
    double time_penalty;
};

class NLOptTrajectoryOptimizer {
public:
    NLOptTrajectoryOptimizer() = default;

    bool generateOptimizedTrajectory(const std::vector<std::vector<double>>& waypoints,
                                     std::vector<std::vector<double>>& trajectory_out);

private:
    static double objectiveFunction(const std::vector<double>& x, std::vector<double>& grad, void* data);
};

}