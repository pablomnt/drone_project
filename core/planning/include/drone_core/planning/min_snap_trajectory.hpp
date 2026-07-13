#pragma once

#include <vector>

#include <Eigen/Dense>

#include "drone_core/common/types.hpp"

namespace drone_core::planning {

// Minimum-snap trajectory generation over a fixed waypoint list. Each segment is
// a per-axis 7th-order polynomial; the coefficients are recovered by solving the
// KKT system that minimises snap subject to waypoint and continuity constraints.
class MinSnapTrajectory {
public:
  MinSnapTrajectory() = default;

  // Allocate segment times from a nominal speed, then sample the solution.
  bool generateTrajectory(const std::vector<std::vector<double>>& waypoints,
                          double average_speed,
                          std::vector<std::vector<double>>& trajectory_out);

  // Snap cost for a given waypoint/time allocation (used by the time optimiser).
  double computeTotalSnap(const std::vector<std::vector<double>>& waypoints,
                          const std::vector<double>& times);

  // Solve and sample the polynomials for an externally supplied time allocation.
  bool generateTrajectoryWithTimes(const std::vector<std::vector<double>>& waypoints,
                                   const std::vector<double>& times,
                                   std::vector<std::vector<double>>& trajectory_out);

  // Solve the polynomials for a time allocation and return them as a Trajectory
  // (per-segment coefficients + durations). This is the form the flatness mapper
  // consumes: keeping the coefficients lets it differentiate for velocity and
  // acceleration feed-forward instead of finite-differencing sampled points.
  bool buildTrajectory(const std::vector<std::vector<double>>& waypoints,
                       const std::vector<double>& times,
                       common::Trajectory& out);

private:
  Eigen::VectorXd solveKKT(const std::vector<double>& waypoints_1d,
                           const std::vector<double>& times);
};

// Context handed to the NLopt objective: the inner snap solver plus the problem
// data it needs to evaluate a candidate time allocation.
struct TimeOptimizerContext {
  MinSnapTrajectory* solver;
  std::vector<std::vector<double>> waypoints;
  double time_penalty;
};

// Outer optimisation loop: searches per-segment durations (NLopt BOBYQA) that
// minimise snap plus a total-time penalty, then emits the snap-optimal
// trajectory for the best allocation. Equivalent to the free-space ("mode A")
// formulation of ETH Zurich's mav_trajectory_generation.
class MinSnapTimeOptimizer {
public:
  MinSnapTimeOptimizer() = default;

  bool generateOptimizedTrajectory(const std::vector<std::vector<double>>& waypoints,
                                   std::vector<std::vector<double>>& trajectory_out);

  // As above, but returns the optimal-time solution as a coefficient Trajectory
  // for the controller to track.
  bool optimizeTrajectory(const std::vector<std::vector<double>>& waypoints,
                          common::Trajectory& out);

private:
  static double objectiveFunction(const std::vector<double>& x,
                                  std::vector<double>& grad,
                                  void* data);

  // Run the BOBYQA search and return the optimal per-segment durations.
  bool optimalTimes(const std::vector<std::vector<double>>& waypoints,
                    std::vector<double>& times_out);
};

}  // namespace drone_core::planning
