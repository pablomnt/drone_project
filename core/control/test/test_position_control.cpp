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
  controller.setCurrentAcceleration(Eigen::Vector3d::Zero());
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

  if (failures == 0) {
    std::cout << "position_control: all checks passed\n";
    return 0;
  }
  return 1;
}
