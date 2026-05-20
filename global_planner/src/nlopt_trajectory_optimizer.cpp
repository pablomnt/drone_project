#include "global_planner/nlopt_trajectory_optimizer.hpp"
#include <nlopt.hpp>
#include <cmath>
#include <sstream>
#include <rclcpp/rclcpp.hpp>

namespace global_planner {

double NLOptTrajectoryOptimizer::objectiveFunction(const std::vector<double>& x, std::vector<double>& grad, void* data) {
    (void)grad;
    OptimizerContext* ctx = static_cast<OptimizerContext*>(data);
    
    double total_snap = ctx->solver->computeTotalSnap(ctx->waypoints, x);
    double total_time = 0.0;
    
    for (double t : x) {
        total_time += t;
    }
    
    return total_snap + (ctx->time_penalty * total_time);
}

bool NLOptTrajectoryOptimizer::generateOptimizedTrajectory(const std::vector<std::vector<double>>& waypoints,
                                                           std::vector<std::vector<double>>& trajectory_out) {
    if (waypoints.size() < 2) return false;
    
    int num_segments = waypoints.size() - 1;
    nlopt::opt optimizer(nlopt::LN_BOBYQA, num_segments);
    
    TrajectoryOptimizer inner_solver;
    OptimizerContext ctx;
    ctx.solver = &inner_solver;
    ctx.waypoints = waypoints;
    ctx.time_penalty = 500.0;
    
    optimizer.set_min_objective(objectiveFunction, &ctx);
    
    std::vector<double> lower_bounds(num_segments, 0.1);
    optimizer.set_lower_bounds(lower_bounds);
    optimizer.set_xtol_rel(1e-3);
    optimizer.set_maxeval(200);
    
    std::vector<double> times(num_segments);
    
    std::stringstream initial_guess_ss;
    for (int i = 0; i < num_segments; ++i) {
        double dx = waypoints[i+1][0] - waypoints[i][0];
        double dy = waypoints[i+1][1] - waypoints[i][1];
        double dz = waypoints[i+1][2] - waypoints[i][2];
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        
        times[i] = 0.5 + (std::sqrt(dist) / 2.0);
        initial_guess_ss << times[i] << " ";
    }
    
    // Use ROS 2 logger instead of std::cout
    auto logger = rclcpp::get_logger("NLOpt");
    RCLCPP_INFO(logger, "Initial guess: %s", initial_guess_ss.str().c_str());
    
    double min_cost;
    
    try {
        optimizer.optimize(times, min_cost);
        
        std::stringstream final_times_ss;
        for (double t : times) {
            final_times_ss << t << " ";
        }
        
        RCLCPP_INFO(logger, "Final times: %s", final_times_ss.str().c_str());
        RCLCPP_INFO(logger, "Cost: %f", min_cost);
        
        return inner_solver.generateTrajectoryWithTimes(waypoints, times, trajectory_out);
    } catch (std::exception &e) {
        RCLCPP_ERROR(logger, "Failed: %s", e.what());
        return false;
    }
}

}