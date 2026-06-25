#include "drone_core/planning/min_snap_trajectory.hpp"

#include <cmath>
#include <sstream>

#include <nlopt.hpp>

#include "drone_core/common/logging.hpp"

namespace drone_core::planning {

bool MinSnapTrajectory::generateTrajectory(const std::vector<std::vector<double>>& waypoints,
                                           double average_speed,
                                           std::vector<std::vector<double>>& trajectory_out) {
  if (waypoints.size() < 2) return false;

  const int num_segments = waypoints.size() - 1;
  std::vector<double> times(num_segments);

  std::vector<double> wp_x(waypoints.size());
  std::vector<double> wp_y(waypoints.size());
  std::vector<double> wp_z(waypoints.size());

  for (size_t i = 0; i < waypoints.size(); ++i) {
    wp_x[i] = waypoints[i][0];
    wp_y[i] = waypoints[i][1];
    wp_z[i] = waypoints[i][2];

    if (i < waypoints.size() - 1) {
      const double dx = waypoints[i + 1][0] - waypoints[i][0];
      const double dy = waypoints[i + 1][1] - waypoints[i][1];
      const double dz = waypoints[i + 1][2] - waypoints[i][2];
      const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

      // sqrt heuristic plus a baseline buffer keeps short segments from ringing.
      times[i] = 0.5 + (std::sqrt(dist) / average_speed);
    }
  }

  const Eigen::VectorXd coeffs_x = solveKKT(wp_x, times);
  const Eigen::VectorXd coeffs_y = solveKKT(wp_y, times);
  const Eigen::VectorXd coeffs_z = solveKKT(wp_z, times);

  const int samples_per_segment = 50;
  for (size_t i = 0; i < times.size(); ++i) {
    const double T = times[i];
    const int idx = i * 8;
    const double dt = T / samples_per_segment;

    for (int s = 0; s <= samples_per_segment; ++s) {
      const double t = s * dt;
      std::vector<double> point(3);
      const double t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t, t6 = t5 * t, t7 = t6 * t;

      point[0] = coeffs_x(idx) + coeffs_x(idx + 1) * t + coeffs_x(idx + 2) * t2 + coeffs_x(idx + 3) * t3 +
                 coeffs_x(idx + 4) * t4 + coeffs_x(idx + 5) * t5 + coeffs_x(idx + 6) * t6 + coeffs_x(idx + 7) * t7;
      point[1] = coeffs_y(idx) + coeffs_y(idx + 1) * t + coeffs_y(idx + 2) * t2 + coeffs_y(idx + 3) * t3 +
                 coeffs_y(idx + 4) * t4 + coeffs_y(idx + 5) * t5 + coeffs_y(idx + 6) * t6 + coeffs_y(idx + 7) * t7;
      point[2] = coeffs_z(idx) + coeffs_z(idx + 1) * t + coeffs_z(idx + 2) * t2 + coeffs_z(idx + 3) * t3 +
                 coeffs_z(idx + 4) * t4 + coeffs_z(idx + 5) * t5 + coeffs_z(idx + 6) * t6 + coeffs_z(idx + 7) * t7;

      trajectory_out.push_back(point);
    }
  }

  return true;
}

double MinSnapTrajectory::computeTotalSnap(const std::vector<std::vector<double>>& waypoints,
                                           const std::vector<double>& times) {
  std::vector<double> wp_x(waypoints.size());
  std::vector<double> wp_y(waypoints.size());
  std::vector<double> wp_z(waypoints.size());

  for (size_t i = 0; i < waypoints.size(); ++i) {
    wp_x[i] = waypoints[i][0];
    wp_y[i] = waypoints[i][1];
    wp_z[i] = waypoints[i][2];
  }

  const Eigen::VectorXd cx = solveKKT(wp_x, times);
  const Eigen::VectorXd cy = solveKKT(wp_y, times);
  const Eigen::VectorXd cz = solveKKT(wp_z, times);

  return cx.norm() + cy.norm() + cz.norm();
}

bool MinSnapTrajectory::generateTrajectoryWithTimes(const std::vector<std::vector<double>>& waypoints,
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

  const Eigen::VectorXd coeffs_x = solveKKT(wp_x, times);
  const Eigen::VectorXd coeffs_y = solveKKT(wp_y, times);
  const Eigen::VectorXd coeffs_z = solveKKT(wp_z, times);

  const double dt = 0.1;
  for (size_t i = 0; i < times.size(); ++i) {
    const double T = times[i];
    const int idx = i * 8;

    for (double t = 0; t < T; t += dt) {
      std::vector<double> point(3);
      const double t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t, t6 = t5 * t, t7 = t6 * t;

      point[0] = coeffs_x(idx) + coeffs_x(idx + 1) * t + coeffs_x(idx + 2) * t2 + coeffs_x(idx + 3) * t3 +
                 coeffs_x(idx + 4) * t4 + coeffs_x(idx + 5) * t5 + coeffs_x(idx + 6) * t6 + coeffs_x(idx + 7) * t7;
      point[1] = coeffs_y(idx) + coeffs_y(idx + 1) * t + coeffs_y(idx + 2) * t2 + coeffs_y(idx + 3) * t3 +
                 coeffs_y(idx + 4) * t4 + coeffs_y(idx + 5) * t5 + coeffs_y(idx + 6) * t6 + coeffs_y(idx + 7) * t7;
      point[2] = coeffs_z(idx) + coeffs_z(idx + 1) * t + coeffs_z(idx + 2) * t2 + coeffs_z(idx + 3) * t3 +
                 coeffs_z(idx + 4) * t4 + coeffs_z(idx + 5) * t5 + coeffs_z(idx + 6) * t6 + coeffs_z(idx + 7) * t7;

      trajectory_out.push_back(point);
    }
  }
  return true;
}

bool MinSnapTrajectory::buildTrajectory(const std::vector<std::vector<double>>& waypoints,
                                        const std::vector<double>& times,
                                        common::Trajectory& out) {
  if (waypoints.size() < 2 || times.size() != waypoints.size() - 1) return false;

  std::vector<double> wp_x(waypoints.size());
  std::vector<double> wp_y(waypoints.size());
  std::vector<double> wp_z(waypoints.size());
  for (size_t i = 0; i < waypoints.size(); ++i) {
    wp_x[i] = waypoints[i][0];
    wp_y[i] = waypoints[i][1];
    wp_z[i] = waypoints[i][2];
  }

  const Eigen::VectorXd cx = solveKKT(wp_x, times);
  const Eigen::VectorXd cy = solveKKT(wp_y, times);
  const Eigen::VectorXd cz = solveKKT(wp_z, times);

  out = common::Trajectory{};
  out.segment_times = times;
  out.total_duration = 0.0;
  for (size_t i = 0; i < times.size(); ++i) {
    const int idx = static_cast<int>(i) * 8;
    out.coeffs_x.push_back(cx.segment(idx, 8));
    out.coeffs_y.push_back(cy.segment(idx, 8));
    out.coeffs_z.push_back(cz.segment(idx, 8));
    out.total_duration += times[i];
  }
  return true;
}

Eigen::VectorXd MinSnapTrajectory::solveKKT(const std::vector<double>& waypoints_1d,
                                            const std::vector<double>& times) {
  const int num_segments = times.size();
  const int vars_per_seg = 8;
  const int num_vars = num_segments * vars_per_seg;
  const int num_constraints = 8 + 6 * (num_segments - 1);

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(num_vars, num_vars);
  Eigen::MatrixXd E = Eigen::MatrixXd::Zero(num_constraints, num_vars);
  Eigen::VectorXd d = Eigen::VectorXd::Zero(num_constraints);

  // Snap cost block per segment.
  for (int i = 0; i < num_segments; ++i) {
    const double t = times[i];
    const int idx = i * vars_per_seg;
    Eigen::MatrixXd Qi = Eigen::MatrixXd::Zero(8, 8);

    Qi(4, 4) = 576.0 * t;
    Qi(4, 5) = 1440.0 * t * t;
    Qi(4, 6) = 2880.0 * t * t * t;
    Qi(4, 7) = 5040.0 * t * t * t * t;

    Qi(5, 4) = Qi(4, 5);
    Qi(5, 5) = 4800.0 * t * t * t;
    Qi(5, 6) = 10800.0 * t * t * t * t;
    Qi(5, 7) = 20160.0 * t * t * t * t * t;

    Qi(6, 4) = Qi(4, 6);
    Qi(6, 5) = Qi(5, 6);
    Qi(6, 6) = 25920.0 * t * t * t * t * t;
    Qi(6, 7) = 50400.0 * t * t * t * t * t * t;

    Qi(7, 4) = Qi(4, 7);
    Qi(7, 5) = Qi(5, 7);
    Qi(7, 6) = Qi(6, 7);
    Qi(7, 7) = 100800.0 * t * t * t * t * t * t * t;

    Q.block(idx, idx, 8, 8) = Qi;
  }

  int row = 0;

  // Initial boundary conditions: position, zero velocity/acceleration/jerk.
  E(row, 0) = 1.0; d(row++) = waypoints_1d.front();
  E(row, 1) = 1.0; d(row++) = 0.0;
  E(row, 2) = 2.0; d(row++) = 0.0;
  E(row, 3) = 6.0; d(row++) = 0.0;

  // Continuity at the interior junctions.
  for (int i = 0; i < num_segments - 1; ++i) {
    const double t = times[i];
    const int curr = i * vars_per_seg;
    const int next = (i + 1) * vars_per_seg;
    const double wp = waypoints_1d[i + 1];

    const double t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t, t6 = t5 * t, t7 = t6 * t;

    E(row, curr) = 1.0; E(row, curr + 1) = t; E(row, curr + 2) = t2; E(row, curr + 3) = t3;
    E(row, curr + 4) = t4; E(row, curr + 5) = t5; E(row, curr + 6) = t6; E(row, curr + 7) = t7;
    d(row++) = wp;

    E(row, next) = 1.0;
    d(row++) = wp;

    E(row, curr + 1) = 1.0; E(row, curr + 2) = 2.0 * t; E(row, curr + 3) = 3.0 * t2; E(row, curr + 4) = 4.0 * t3;
    E(row, curr + 5) = 5.0 * t4; E(row, curr + 6) = 6.0 * t5; E(row, curr + 7) = 7.0 * t6;
    E(row, next + 1) = -1.0;
    d(row++) = 0.0;

    E(row, curr + 2) = 2.0; E(row, curr + 3) = 6.0 * t; E(row, curr + 4) = 12.0 * t2; E(row, curr + 5) = 20.0 * t3;
    E(row, curr + 6) = 30.0 * t4; E(row, curr + 7) = 42.0 * t5;
    E(row, next + 2) = -2.0;
    d(row++) = 0.0;

    E(row, curr + 3) = 6.0; E(row, curr + 4) = 24.0 * t; E(row, curr + 5) = 60.0 * t2; E(row, curr + 6) = 120.0 * t3;
    E(row, curr + 7) = 210.0 * t4;
    E(row, next + 3) = -6.0;
    d(row++) = 0.0;

    E(row, curr + 4) = 24.0; E(row, curr + 5) = 120.0 * t; E(row, curr + 6) = 360.0 * t2; E(row, curr + 7) = 840.0 * t3;
    E(row, next + 4) = -24.0;
    d(row++) = 0.0;
  }

  // Final boundary conditions: position, zero velocity/acceleration/jerk.
  const int last = (num_segments - 1) * vars_per_seg;
  const double t = times.back();
  const double t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t, t6 = t5 * t, t7 = t6 * t;

  E(row, last) = 1.0; E(row, last + 1) = t; E(row, last + 2) = t2; E(row, last + 3) = t3;
  E(row, last + 4) = t4; E(row, last + 5) = t5; E(row, last + 6) = t6; E(row, last + 7) = t7;
  d(row++) = waypoints_1d.back();

  E(row, last + 1) = 1.0; E(row, last + 2) = 2.0 * t; E(row, last + 3) = 3.0 * t2; E(row, last + 4) = 4.0 * t3;
  E(row, last + 5) = 5.0 * t4; E(row, last + 6) = 6.0 * t5; E(row, last + 7) = 7.0 * t6;
  d(row++) = 0.0;

  E(row, last + 2) = 2.0; E(row, last + 3) = 6.0 * t; E(row, last + 4) = 12.0 * t2; E(row, last + 5) = 20.0 * t3;
  E(row, last + 6) = 30.0 * t4; E(row, last + 7) = 42.0 * t5;
  d(row++) = 0.0;

  E(row, last + 3) = 6.0; E(row, last + 4) = 24.0 * t; E(row, last + 5) = 60.0 * t2; E(row, last + 6) = 120.0 * t3;
  E(row, last + 7) = 210.0 * t4;
  d(row++) = 0.0;

  // Assemble and solve the KKT block system.
  const int total_size = num_vars + num_constraints;
  Eigen::MatrixXd KKT = Eigen::MatrixXd::Zero(total_size, total_size);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(total_size);

  KKT.block(0, 0, num_vars, num_vars) = Q;
  KKT.block(0, num_vars, num_vars, num_constraints) = E.transpose();
  KKT.block(num_vars, 0, num_constraints, num_vars) = E;
  b.tail(num_constraints) = d;

  const Eigen::VectorXd solution = KKT.lu().solve(b);
  return solution.head(num_vars);
}

double MinSnapTimeOptimizer::objectiveFunction(const std::vector<double>& x,
                                               std::vector<double>& grad,
                                               void* data) {
  (void)grad;
  auto* ctx = static_cast<TimeOptimizerContext*>(data);

  const double total_snap = ctx->solver->computeTotalSnap(ctx->waypoints, x);
  double total_time = 0.0;
  for (double t : x) {
    total_time += t;
  }

  return total_snap + (ctx->time_penalty * total_time);
}

bool MinSnapTimeOptimizer::optimalTimes(const std::vector<std::vector<double>>& waypoints,
                                        std::vector<double>& times_out) {
  if (waypoints.size() < 2) return false;

  const int num_segments = waypoints.size() - 1;
  nlopt::opt optimizer(nlopt::LN_BOBYQA, num_segments);

  MinSnapTrajectory inner_solver;
  TimeOptimizerContext ctx;
  ctx.solver = &inner_solver;
  ctx.waypoints = waypoints;
  ctx.time_penalty = 500.0;

  optimizer.set_min_objective(objectiveFunction, &ctx);

  std::vector<double> lower_bounds(num_segments, 0.1);
  optimizer.set_lower_bounds(lower_bounds);
  optimizer.set_xtol_rel(1e-3);
  optimizer.set_maxeval(200);

  std::vector<double> times(num_segments);
  for (int i = 0; i < num_segments; ++i) {
    const double dx = waypoints[i + 1][0] - waypoints[i][0];
    const double dy = waypoints[i + 1][1] - waypoints[i][1];
    const double dz = waypoints[i + 1][2] - waypoints[i][2];
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    times[i] = 0.5 + (std::sqrt(dist) / 2.0);
  }

  double min_cost = 0.0;
  try {
    optimizer.optimize(times, min_cost);
  } catch (std::exception& e) {
    DRONE_LOG_ERROR("MinSnapTimeOptimizer failed: " << e.what());
    return false;
  }

  times_out = times;
  return true;
}

bool MinSnapTimeOptimizer::generateOptimizedTrajectory(const std::vector<std::vector<double>>& waypoints,
                                                       std::vector<std::vector<double>>& trajectory_out) {
  std::vector<double> times;
  if (!optimalTimes(waypoints, times)) return false;

  MinSnapTrajectory inner_solver;
  return inner_solver.generateTrajectoryWithTimes(waypoints, times, trajectory_out);
}

bool MinSnapTimeOptimizer::optimizeTrajectory(const std::vector<std::vector<double>>& waypoints,
                                              common::Trajectory& out) {
  std::vector<double> times;
  if (!optimalTimes(waypoints, times)) return false;

  MinSnapTrajectory inner_solver;
  return inner_solver.buildTrajectory(waypoints, times, out);
}

}  // namespace drone_core::planning
