#include "global_planner/trajectory_optimizer.hpp"
#include <cmath>
#include <iostream>

namespace global_planner {

bool TrajectoryOptimizer::generateTrajectory(const std::vector<std::vector<double>>& waypoints,
                                             double average_speed,
                                             std::vector<std::vector<double>>& trajectory_out) {
    if (waypoints.size() < 2) return false;

    int num_segments = waypoints.size() - 1;
    std::vector<double> times(num_segments);
    
    std::vector<double> wp_x(waypoints.size());
    std::vector<double> wp_y(waypoints.size());
    std::vector<double> wp_z(waypoints.size());

    // calculate segment times based on distance and split axes
    for (size_t i = 0; i < waypoints.size(); ++i) {
        wp_x[i] = waypoints[i][0];
        wp_y[i] = waypoints[i][1];
        wp_z[i] = waypoints[i][2];

        if (i < waypoints.size() - 1) {
            double dx = waypoints[i+1][0] - waypoints[i][0];
            double dy = waypoints[i+1][1] - waypoints[i][1];
            double dz = waypoints[i+1][2] - waypoints[i][2];
            double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
            
            // apply a non-linear heuristic to reduce polynomial ringing
            // short segments get a baseline time buffer
            times[i] = 0.5 + (std::sqrt(dist) / average_speed);
        }
    }

    // solve kkt system for each axis independently
    Eigen::VectorXd coeffs_x = solveKKT(wp_x, times);
    Eigen::VectorXd coeffs_y = solveKKT(wp_y, times);
    Eigen::VectorXd coeffs_z = solveKKT(wp_z, times);

    // dynamically sample 20 points per segment instead of using fixed dt
    int samples_per_segment = 50;
    
    for (size_t i = 0; i < times.size(); ++i) {
        double T = times[i];
        int idx = i * 8;
        
        double dt = T / samples_per_segment;
        
        for (int s = 0; s <= samples_per_segment; ++s) {
            double t = s * dt; // Calculate continuous time
            
            std::vector<double> point(3);
            double t2 = t*t, t3 = t2*t, t4 = t3*t, t5 = t4*t, t6 = t5*t, t7 = t6*t;

            point[0] = coeffs_x(idx) + coeffs_x(idx+1)*t + coeffs_x(idx+2)*t2 + coeffs_x(idx+3)*t3 + 
                       coeffs_x(idx+4)*t4 + coeffs_x(idx+5)*t5 + coeffs_x(idx+6)*t6 + coeffs_x(idx+7)*t7;
                       
            point[1] = coeffs_y(idx) + coeffs_y(idx+1)*t + coeffs_y(idx+2)*t2 + coeffs_y(idx+3)*t3 + 
                       coeffs_y(idx+4)*t4 + coeffs_y(idx+5)*t5 + coeffs_y(idx+6)*t6 + coeffs_y(idx+7)*t7;
                       
            point[2] = coeffs_z(idx) + coeffs_z(idx+1)*t + coeffs_z(idx+2)*t2 + coeffs_z(idx+3)*t3 + 
                       coeffs_z(idx+4)*t4 + coeffs_z(idx+5)*t5 + coeffs_z(idx+6)*t6 + coeffs_z(idx+7)*t7;

            trajectory_out.push_back(point);
        }
    }

    return true;
}

double TrajectoryOptimizer::computeTotalSnap(const std::vector<std::vector<double>>& waypoints, const std::vector<double>& times) {
    std::vector<double> wp_x(waypoints.size());
    std::vector<double> wp_y(waypoints.size());
    std::vector<double> wp_z(waypoints.size());

    for (size_t i = 0; i < waypoints.size(); ++i) {
        wp_x[i] = waypoints[i][0];
        wp_y[i] = waypoints[i][1];
        wp_z[i] = waypoints[i][2];
    }

    Eigen::VectorXd cx = solveKKT(wp_x, times);
    Eigen::VectorXd cy = solveKKT(wp_y, times);
    Eigen::VectorXd cz = solveKKT(wp_z, times);

    double cost = 0.0;
    cost += cx.norm() + cy.norm() + cz.norm(); 

    return cost;
}

bool TrajectoryOptimizer::generateTrajectoryWithTimes(const std::vector<std::vector<double>>& waypoints,
                                                      const std::vector<double>& times,
                                                      std::vector<std::vector<double>>& trajectory_out) {
    std::vector<double> wp_x(waypoints.size());
    std::vector<double> wp_y(waypoints.size());
    std::vector<double> wp_z(waypoints.size());

    for (size_t i = 0; i < waypoints.size(); ++i) {
        wp_x[i] = waypoints[i][0];
        wp_y[i] = waypoints[i][1];
        wp_z[i] = waypoints[i][2];
    }

    Eigen::VectorXd coeffs_x = solveKKT(wp_x, times);
    Eigen::VectorXd coeffs_y = solveKKT(wp_y, times);
    Eigen::VectorXd coeffs_z = solveKKT(wp_z, times);

    double dt = 0.1;
    for (size_t i = 0; i < times.size(); ++i) {
        double T = times[i];
        int idx = i * 8;
        
        for (double t = 0; t < T; t += dt) {
            std::vector<double> point(3);
            double t2 = t*t, t3 = t2*t, t4 = t3*t, t5 = t4*t, t6 = t5*t, t7 = t6*t;

            point[0] = coeffs_x(idx) + coeffs_x(idx+1)*t + coeffs_x(idx+2)*t2 + coeffs_x(idx+3)*t3 + 
                       coeffs_x(idx+4)*t4 + coeffs_x(idx+5)*t5 + coeffs_x(idx+6)*t6 + coeffs_x(idx+7)*t7;
                       
            point[1] = coeffs_y(idx) + coeffs_y(idx+1)*t + coeffs_y(idx+2)*t2 + coeffs_y(idx+3)*t3 + 
                       coeffs_y(idx+4)*t4 + coeffs_y(idx+5)*t5 + coeffs_y(idx+6)*t6 + coeffs_y(idx+7)*t7;
                       
            point[2] = coeffs_z(idx) + coeffs_z(idx+1)*t + coeffs_z(idx+2)*t2 + coeffs_z(idx+3)*t3 + 
                       coeffs_z(idx+4)*t4 + coeffs_z(idx+5)*t5 + coeffs_z(idx+6)*t6 + coeffs_z(idx+7)*t7;

            trajectory_out.push_back(point);
        }
    }
    return true;
}

Eigen::VectorXd TrajectoryOptimizer::solveKKT(const std::vector<double>& waypoints_1d, 
                                              const std::vector<double>& times) {
    int num_segments = times.size();
    int vars_per_seg = 8;
    int num_vars = num_segments * vars_per_seg;
    int num_constraints = 8 + 6 * (num_segments - 1);
    
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(num_vars, num_vars);
    Eigen::MatrixXd E = Eigen::MatrixXd::Zero(num_constraints, num_vars);
    Eigen::VectorXd d = Eigen::VectorXd::Zero(num_constraints);

    // populate cost matrix
    for (int i = 0; i < num_segments; ++i) {
        double t = times[i];
        int idx = i * vars_per_seg;
        Eigen::MatrixXd Qi = Eigen::MatrixXd::Zero(8, 8);
        
        Qi(4,4) = 576.0 * t;
        Qi(4,5) = 1440.0 * t*t;
        Qi(4,6) = 2880.0 * t*t*t;
        Qi(4,7) = 5040.0 * t*t*t*t;
        
        Qi(5,4) = Qi(4,5);
        Qi(5,5) = 4800.0 * t*t*t;
        Qi(5,6) = 10800.0 * t*t*t*t;
        Qi(5,7) = 20160.0 * t*t*t*t*t;
        
        Qi(6,4) = Qi(4,6);
        Qi(6,5) = Qi(5,6);
        Qi(6,6) = 25920.0 * t*t*t*t*t;
        Qi(6,7) = 50400.0 * t*t*t*t*t*t;
        
        Qi(7,4) = Qi(4,7);
        Qi(7,5) = Qi(5,7);
        Qi(7,6) = Qi(6,7);
        Qi(7,7) = 100800.0 * t*t*t*t*t*t*t;
        
        Q.block(idx, idx, 8, 8) = Qi;
    }

    int row = 0;

    // set initial boundaries
    E(row, 0) = 1.0; d(row++) = waypoints_1d.front();
    E(row, 1) = 1.0; d(row++) = 0.0;
    E(row, 2) = 2.0; d(row++) = 0.0;
    E(row, 3) = 6.0; d(row++) = 0.0;

    // tie segments together at junctions
    for (int i = 0; i < num_segments - 1; ++i) {
        double t = times[i];
        int curr = i * vars_per_seg;
        int next = (i + 1) * vars_per_seg;
        double wp = waypoints_1d[i + 1];

        double t2 = t*t, t3 = t2*t, t4 = t3*t, t5 = t4*t, t6 = t5*t, t7 = t6*t;

        E(row, curr) = 1.0; E(row, curr+1) = t; E(row, curr+2) = t2; E(row, curr+3) = t3;
        E(row, curr+4) = t4; E(row, curr+5) = t5; E(row, curr+6) = t6; E(row, curr+7) = t7;
        d(row++) = wp;

        E(row, next) = 1.0;
        d(row++) = wp;

        E(row, curr+1) = 1.0; E(row, curr+2) = 2.0*t; E(row, curr+3) = 3.0*t2; E(row, curr+4) = 4.0*t3;
        E(row, curr+5) = 5.0*t4; E(row, curr+6) = 6.0*t5; E(row, curr+7) = 7.0*t6;
        E(row, next+1) = -1.0;
        d(row++) = 0.0;

        E(row, curr+2) = 2.0; E(row, curr+3) = 6.0*t; E(row, curr+4) = 12.0*t2; E(row, curr+5) = 20.0*t3;
        E(row, curr+6) = 30.0*t4; E(row, curr+7) = 42.0*t5;
        E(row, next+2) = -2.0;
        d(row++) = 0.0;

        E(row, curr+3) = 6.0; E(row, curr+4) = 24.0*t; E(row, curr+5) = 60.0*t2; E(row, curr+6) = 120.0*t3;
        E(row, curr+7) = 210.0*t4;
        E(row, next+3) = -6.0;
        d(row++) = 0.0;

        E(row, curr+4) = 24.0; E(row, curr+5) = 120.0*t; E(row, curr+6) = 360.0*t2; E(row, curr+7) = 840.0*t3;
        E(row, next+4) = -24.0;
        d(row++) = 0.0;
    }

    // set final boundaries
    int last = (num_segments - 1) * vars_per_seg;
    double t = times.back();
    double t2 = t*t, t3 = t2*t, t4 = t3*t, t5 = t4*t, t6 = t5*t, t7 = t6*t;

    E(row, last) = 1.0; E(row, last+1) = t; E(row, last+2) = t2; E(row, last+3) = t3;
    E(row, last+4) = t4; E(row, last+5) = t5; E(row, last+6) = t6; E(row, last+7) = t7;
    d(row++) = waypoints_1d.back();

    E(row, last+1) = 1.0; E(row, last+2) = 2.0*t; E(row, last+3) = 3.0*t2; E(row, last+4) = 4.0*t3;
    E(row, last+5) = 5.0*t4; E(row, last+6) = 6.0*t5; E(row, last+7) = 7.0*t6;
    d(row++) = 0.0;

    E(row, last+2) = 2.0; E(row, last+3) = 6.0*t; E(row, last+4) = 12.0*t2; E(row, last+5) = 20.0*t3;
    E(row, last+6) = 30.0*t4; E(row, last+7) = 42.0*t5;
    d(row++) = 0.0;

    E(row, last+3) = 6.0; E(row, last+4) = 24.0*t; E(row, last+5) = 60.0*t2; E(row, last+6) = 120.0*t3;
    E(row, last+7) = 210.0*t4;
    d(row++) = 0.0;

    // solve block matrix system
    int total_size = num_vars + num_constraints;
    Eigen::MatrixXd KKT = Eigen::MatrixXd::Zero(total_size, total_size);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(total_size);

    KKT.block(0, 0, num_vars, num_vars) = Q;
    KKT.block(0, num_vars, num_vars, num_constraints) = E.transpose();
    KKT.block(num_vars, 0, num_constraints, num_vars) = E;
    b.tail(num_constraints) = d;

    Eigen::VectorXd solution = KKT.lu().solve(b);
    return solution.head(num_vars);
}

}