// The map<->world boundary transforms. The trajectory check is the one that
// matters: sampling a transformed trajectory must give exactly the transformed
// samples of the original, for position and every derivative the tracker and the
// splice use, including a transform with a tilt (RTAB-Map's corrections carry a
// few degrees of it, so a translation-only implementation would pass a
// yaw-free test and still be wrong in flight).

#include "drone_core/common/rigid_transform.hpp"

#include <cmath>
#include <iostream>

#include "drone_core/common/trajectory_eval.hpp"

namespace {

int g_failures = 0;

void check(bool ok, const char* what) {
  if (!ok) {
    std::cerr << "FAIL: " << what << "\n";
    ++g_failures;
  }
}

bool near(const Eigen::Vector3d& a, const Eigen::Vector3d& b, double tol) {
  return (a - b).norm() <= tol;
}

}  // namespace

int main() {
  using namespace drone_core::common;

  // A tilted, rotated, translated transform, as RTAB-Map produces after a
  // correction (exaggerated so any missing rotation shows up clearly).
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.rotate(Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()));
  T.rotate(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitY()));
  T.pretranslate(Eigen::Vector3d(-0.37, 0.8, -0.15));
  const Eigen::Matrix3d R = T.rotation();

  // Three segments of degree-7 polynomials with arbitrary coefficients.
  Trajectory traj;
  traj.segment_times = {0.7, 1.1, 0.9};
  traj.total_duration = 2.7;
  traj.t0 = 42.0;
  for (int s = 0; s < 3; ++s) {
    Eigen::VectorXd cx(8), cy(8), cz(8);
    for (int k = 0; k < 8; ++k) {
      cx[k] = std::sin(1.3 * s + 0.7 * k) / (k + 1);
      cy[k] = std::cos(0.9 * s + 1.1 * k) / (k + 1);
      cz[k] = std::sin(2.1 * s - 0.5 * k) / (k + 2);
    }
    traj.coeffs_x.push_back(cx);
    traj.coeffs_y.push_back(cy);
    traj.coeffs_z.push_back(cz);
  }

  const Trajectory moved = transformTrajectory(T, traj);
  check(moved.t0 == traj.t0, "t0 unchanged");
  check(moved.total_duration == traj.total_duration, "duration unchanged");
  check(moved.segment_times == traj.segment_times, "segment times unchanged");

  bool pos_ok = true, vel_ok = true, acc_ok = true, jerk_ok = true, motion_ok = true;
  for (int i = 0; i <= 270; ++i) {
    const double now = traj.t0 + 0.01 * i;
    const MotionState a = sampleMotion(traj, now);
    const MotionState b = sampleMotion(moved, now);
    pos_ok = pos_ok && near(b.pos, T * a.pos, 1e-9);
    vel_ok = vel_ok && near(b.vel, R * a.vel, 1e-9);
    acc_ok = acc_ok && near(b.acc, R * a.acc, 1e-9);
    jerk_ok = jerk_ok && near(b.jerk, R * a.jerk, 1e-9);
    const MotionState c = transformMotion(T, a);
    motion_ok = motion_ok && near(c.pos, b.pos, 1e-9) && near(c.vel, b.vel, 1e-9) &&
                near(c.acc, b.acc, 1e-9) && near(c.jerk, b.jerk, 1e-9);
  }
  check(pos_ok, "transformed trajectory position == T * original position");
  check(vel_ok, "transformed trajectory velocity == R * original velocity (no translation)");
  check(acc_ok, "transformed trajectory acceleration == R * original acceleration");
  check(jerk_ok, "transformed trajectory jerk == R * original jerk");
  check(motion_ok, "transformMotion agrees with sampling the transformed trajectory");

  // Round trip: there and back again recovers the original coefficients.
  const Trajectory back = transformTrajectory(T.inverse(), moved);
  bool round_ok = true;
  for (int s = 0; s < 3; ++s) {
    round_ok = round_ok && (back.coeffs_x[s] - traj.coeffs_x[s]).norm() < 1e-12 &&
               (back.coeffs_y[s] - traj.coeffs_y[s]).norm() < 1e-12 &&
               (back.coeffs_z[s] - traj.coeffs_z[s]).norm() < 1e-12;
  }
  check(round_ok, "T then T^-1 recovers the original coefficients");

  // State: position takes the full transform, velocity only the rotation, and
  // yaw turns with the rotation about z.
  State s;
  s.pos = Eigen::Vector3d(1.0, -2.0, 1.5);
  s.vel = Eigen::Vector3d(0.3, 0.1, -0.2);
  s.yaw = 3.0;
  s.thrust_accel = 9.7;
  s.stamp = 5.0;
  Eigen::Isometry3d Tz = Eigen::Isometry3d::Identity();
  Tz.rotate(Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()));
  Tz.pretranslate(Eigen::Vector3d(2.0, 0.0, 0.0));
  const State sz = transformState(Tz, s);
  check(near(sz.pos, Tz * s.pos, 1e-12), "state position takes the full transform");
  check(near(sz.vel, Tz.rotation() * s.vel, 1e-12), "state velocity takes the rotation only");
  const double expected_yaw = std::atan2(std::sin(3.5), std::cos(3.5));  // 3.0 + 0.5, wrapped
  check(std::abs(sz.yaw - expected_yaw) < 1e-12, "yaw turns with the rotation about z (wrapped)");
  check(sz.thrust_accel == s.thrust_accel && sz.stamp == s.stamp,
        "thrust_accel and stamp pass through");

  if (g_failures == 0) std::cout << "rigid_transform: all checks passed\n";
  return g_failures == 0 ? 0 : 1;
}
