// Sanity checks for the relocated controller: a hover command must produce a
// bounded thrust and a unit attitude quaternion. No test framework, so a
// non-zero return signals failure to CTest.

#include "drone_core/control/position_control.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>

int main() {
  using drone_core::control::PositionControl;

  PositionControl controller;
  controller.setPositionGains(Eigen::Vector3d(0.95, 0.95, 1.0));
  controller.setVelocityGains(Eigen::Vector3d(1.8, 1.8, 2.0),
                              Eigen::Vector3d(0.4, 0.4, 0.5),
                              Eigen::Vector3d(0.2, 0.2, 0.2));
  controller.setHoverThrust(0.35);

  // Vehicle airborne and asked to hold a point 30 cm above.
  controller.setState(Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d::Zero(), 0.0);
  controller.setThrustAccel(9.81);  // hover: thrust exactly cancels gravity
  controller.reset();
  controller.setSetpoint(Eigen::Vector3d(0.0, 0.0, 1.3), 0.0);

  int failures = 0;
  for (int i = 0; i < 50; ++i) {
    controller.update(0.02);

    const double thrust = controller.getThrustSetpoint();
    if (thrust < 0.0 || thrust > 1.0) {
      std::cerr << "FAIL: thrust out of bounds: " << thrust << "\n";
      ++failures;
      break;
    }

    const double qnorm = controller.getAttitudeSetpoint().norm();
    if (std::abs(qnorm - 1.0) > 1e-6) {
      std::cerr << "FAIL: attitude quaternion not unit norm: " << qnorm << "\n";
      ++failures;
      break;
    }
  }

  // The D term must survive a measurement slower than the control loop. The host
  // re-sends the newest estimate every tick, so at 25 Hz VIO against a 50 Hz loop
  // _vel only changes on alternate ticks. Differencing per control tick used to
  // give exactly 0 on the held ticks and double the true rate on the others -- a
  // train of impulses rather than a derivative. Feeding the sample timestamp must
  // instead yield the true, constant dv/dt on every tick.
  {
    PositionControl d;
    d.setVelocityGains(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                       Eigen::Vector3d(0.0, 0.0, 1.0));  // isolate D on z, Kd = 1
    d.setDerivativeTau(0.0);                             // filter off: check the raw path
    d.reset();

    // Climbing with a constant true acceleration of 1 m/s^2, sampled at 25 Hz.
    double stamp = 0.0;
    double vz = 0.0;
    for (int k = 0; k < 10; ++k) {
      if (k % 2 == 0) {
        vz += 1.0 * 0.04;
        stamp += 0.04;
      }
      d.setState(Eigen::Vector3d(0.0, 0.0, 2.0), Eigen::Vector3d(0.0, 0.0, vz), 0.0);
      d.setStateStamp(stamp);
      d.setSetpoint(Eigen::Vector3d(0.0, 0.0, 2.0), 0.0);
      d.update(0.02);

      // Skip the first two ticks: the first sample after a reset has no
      // predecessor to difference against, by design.
      if (k < 2) continue;

      // Derivative on measurement is negated, so a positive dv/dt reads negative.
      const double d_z = d.getVelocityDTerm().z();
      if (std::abs(d_z + 1.0) > 1e-9) {
        std::cerr << "FAIL: D term on tick " << k << " is " << d_z
                  << ", expected -1.0 (held-sample tick reading zero?)\n";
        ++failures;
        break;
      }
    }
  }

  // Integrator gate: the velocity integrator accumulates only while the position
  // error is within the limit, judged separately for XY (norm) and z, and a
  // large error FREEZES it at its current value rather than resetting it.
  {
    PositionControl g;
    g.setPositionGains(Eigen::Vector3d(1.0, 1.0, 1.0));
    g.setVelocityGains(Eigen::Vector3d::Zero(), Eigen::Vector3d(1.0, 1.0, 1.0),
                       Eigen::Vector3d::Zero());  // isolate I
    g.setIntegratorErrorLimit(0.2);
    g.setState(Eigen::Vector3d(0.0, 0.0, 2.0), Eigen::Vector3d::Zero(), 0.0);
    g.reset();

    // Near: 0.1 m on every axis (XY norm 0.14 m) -> all three integrate.
    g.setSetpoint(Eigen::Vector3d(0.1, 0.1, 2.1), 0.0);
    for (int k = 0; k < 10; ++k) g.update(0.02);
    const Eigen::Vector3d near = g.getVelocityITerm();
    if (!(near.x() > 0.0 && near.y() > 0.0 && near.z() > 0.0)) {
      std::cerr << "FAIL: integrator did not accumulate inside the limit: "
                << near.transpose() << "\n";
      ++failures;
    }

    // Far horizontally (1 m), still near vertically -> XY frozen, z integrates.
    g.setSetpoint(Eigen::Vector3d(1.0, 0.0, 2.1), 0.0);
    for (int k = 0; k < 10; ++k) g.update(0.02);
    const Eigen::Vector3d far = g.getVelocityITerm();
    if (far.x() != near.x() || far.y() != near.y()) {
      std::cerr << "FAIL: XY integrator not held at its value outside the limit: "
                << near.transpose() << " -> " << far.transpose() << "\n";
      ++failures;
    }
    if (!(far.z() > near.z())) {
      std::cerr << "FAIL: z integrator frozen by a horizontal-only error\n";
      ++failures;
    }
  }

  // Hover-thrust estimator under flight-level vibration. The accel in flight reads
  // about ±3.5 m/s² of heavy-tailed noise around 9.81. The estimator must keep the
  // accel on top of the fraction it filters (thrust gain a/T): averaging the
  // per-sample ratio T·g/a puts the noise underneath and is biased high (it read
  // 0.31 against a true 0.29 on the 2026-09-14 flights). Closed loop: with zero
  // gains the command is the estimate itself, and the simulated vehicle's thrust
  // accel follows from it.
  {
    constexpr double kTrueHover = 0.29;
    constexpr double kDt = 0.02;
    PositionControl h;  // gains default to zero: thrust_sp == hover estimate
    h.setState(Eigen::Vector3d(0.0, 0.0, 1.5), Eigen::Vector3d::Zero(), 0.0);
    h.setHoverThrust(0.33);  // seeded wrong, like MPC_HOVER_THRUST was
    h.reset();
    h.setSetpoint(Eigen::Vector3d(0.0, 0.0, 1.5), 0.0);

    std::mt19937 rng(42);
    std::student_t_distribution<double> t3(3.0);  // heavy tails; variance 3
    const double noise_scale = 3.5 / std::sqrt(3.0);

    double old_form = 0.33;  // the previous per-sample ratio estimator, as a contrast
    double sum_new = 0.0, sum_old = 0.0;
    int n = 0;
    double cmd = 0.33;
    const int ticks = static_cast<int>(60.0 / kDt);
    for (int k = 0; k < ticks; ++k) {
      const double a = 9.81 * cmd / kTrueHover + noise_scale * t3(rng);
      h.setThrustAccel(a);
      h.update(kDt);
      old_form += (kDt / 2.5) * (std::clamp(cmd * 9.81 / a, 0.2, 0.5) - old_form);
      cmd = h.getThrustSetpoint();
      if (k * kDt >= 40.0) {  // settled window: last 20 s
        sum_new += h.getHoverThrust();
        sum_old += old_form;
        ++n;
      }
    }
    const double est = sum_new / n;
    const double old_est = sum_old / n;
    if (std::abs(est - kTrueHover) > 0.01) {
      std::cerr << "FAIL: hover estimate " << est << " under vibration, expected " << kTrueHover
                << " +/- 0.01\n";
      ++failures;
    }
    // Guard the test itself: if the noise were too mild to bias the old form, a
    // pass above would prove nothing about which way up the fraction is.
    if (std::abs(old_est - kTrueHover) <= 0.01) {
      std::cerr << "FAIL: test noise too weak to exercise the bias (old form read " << old_est
                << ")\n";
      ++failures;
    }
    std::cout << "hover estimate under vibration: " << est << " (per-sample ratio would read "
              << old_est << ", true " << kTrueHover << ")\n";
  }

  if (failures == 0) {
    std::cout << "position_control: all checks passed\n";
    return 0;
  }
  return 1;
}
