// Checks the flatness mapper's polynomial sampling, end-of-trajectory clamping,
// segment continuity, and the yaw-from-velocity policy.

#include "drone_core/control/flatness_mapper.hpp"

#include <cmath>
#include <iostream>

namespace {

int g_failures = 0;

void expectNear(double a, double b, double tol, const char* what) {
  if (std::abs(a - b) > tol) {
    std::cerr << "FAIL: " << what << " expected " << b << " got " << a << "\n";
    ++g_failures;
  }
}

Eigen::VectorXd poly(double c0, double c1) {
  Eigen::VectorXd c = Eigen::VectorXd::Zero(8);
  c[0] = c0;
  c[1] = c1;
  return c;
}

}  // namespace

int main() {
  using drone_core::common::Trajectory;
  using drone_core::control::FlatnessMapper;

  // Two continuous segments along +x at unit speed: seg0 x=t over [0,1], seg1
  // x=1+t over [0,1]; z held at 1, y at 0.
  Trajectory traj;
  traj.segment_times = {1.0, 1.0};
  traj.total_duration = 2.0;
  traj.t0 = 0.0;
  traj.coeffs_x = {poly(0.0, 1.0), poly(1.0, 1.0)};
  traj.coeffs_y = {poly(0.0, 0.0), poly(0.0, 0.0)};
  traj.coeffs_z = {poly(1.0, 0.0), poly(1.0, 0.0)};

  FlatnessMapper mapper;
  mapper.reset(0.0);

  // Start of trajectory.
  auto r0 = mapper.sample(traj, 0.0);
  expectNear(r0.pos.x(), 0.0, 1e-9, "pos.x at t0");
  expectNear(r0.pos.z(), 1.0, 1e-9, "pos.z at t0");
  expectNear(r0.vel_ff.x(), 1.0, 1e-9, "vel.x at t0");
  expectNear(r0.yaw, 0.0, 1e-9, "yaw follows +x heading");

  // Segment boundary continuity.
  auto before = mapper.sample(traj, 0.999);
  auto after = mapper.sample(traj, 1.001);
  expectNear((after.pos - before.pos).norm(), 0.0, 5e-3, "position continuous across boundary");

  // Sampling past the end clamps to the final state.
  auto r_end = mapper.sample(traj, 10.0);
  expectNear(r_end.pos.x(), 2.0, 1e-9, "pos.x clamped at trajectory end");

  // Standstill trajectory: zero velocity means the heading is undefined, so the
  // mapper must hold the seeded yaw rather than chase a noisy heading.
  Trajectory still;
  still.segment_times = {1.0};
  still.total_duration = 1.0;
  still.t0 = 0.0;
  still.coeffs_x = {poly(5.0, 0.0)};
  still.coeffs_y = {poly(2.0, 0.0)};
  still.coeffs_z = {poly(1.0, 0.0)};

  FlatnessMapper held;
  held.reset(0.7);
  auto rs = held.sample(still, 0.5);
  expectNear(rs.yaw, 0.7, 1e-9, "yaw held at standstill");

  if (g_failures == 0) {
    std::cout << "flatness_mapper: all checks passed\n";
    return 0;
  }
  std::cerr << "flatness_mapper: " << g_failures << " failure(s)\n";
  return 1;
}
