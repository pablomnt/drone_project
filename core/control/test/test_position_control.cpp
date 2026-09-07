// Sanity checks for the relocated controller: a hover command must produce a
// bounded thrust and a unit attitude quaternion. No test framework, so a
// non-zero return signals failure to CTest.

#include "drone_core/control/position_control.hpp"

#include <cmath>
#include <iostream>

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

  if (failures == 0) {
    std::cout << "position_control: all checks passed\n";
    return 0;
  }
  return 1;
}
