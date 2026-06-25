// Verifies the headline safety property of the feed-forward extension: with
// feed-forward disabled the controller is bit-for-bit identical to the baseline
// position-only behaviour, and with it enabled the command actually changes.

#include "drone_core/control/position_control.hpp"

#include <cmath>
#include <iostream>

namespace {

using drone_core::control::PositionControl;

void configure(PositionControl& c) {
  c.setPositionGains(Eigen::Vector3d(0.95, 0.95, 1.0));
  c.setVelocityGains(Eigen::Vector3d(1.8, 1.8, 2.0),
                     Eigen::Vector3d(0.4, 0.4, 0.5),
                     Eigen::Vector3d(0.2, 0.2, 0.2));
  c.setHoverThrust(0.35);
}

// Start airborne so the open-loop takeoff override (which would mask the
// feed-forward terms) does not engage.
void primeAirborne(PositionControl& c) {
  c.setState(Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d::Zero(), 0.0);
  c.setCurrentAcceleration(Eigen::Vector3d::Zero());
  c.reset();
}

}  // namespace

int main() {
  const Eigen::Vector3d pos_sp(0.5, 0.0, 1.2);
  const double yaw_sp = 0.0;
  const Eigen::Vector3d vel_ff(1.0, 0.5, 0.0);
  const Eigen::Vector3d acc_ff(0.4, 0.0, 0.2);

  drone_core::common::Reference ref;
  ref.pos = pos_sp;
  ref.yaw = yaw_sp;
  ref.vel_ff = vel_ff;
  ref.acc_ff = acc_ff;

  // Baseline: plain setpoint.
  PositionControl baseline;
  configure(baseline);
  primeAirborne(baseline);

  // Same reference with non-zero feed-forward, but feed-forward disabled.
  PositionControl ff_off;
  configure(ff_off);
  primeAirborne(ff_off);
  ff_off.enableFeedforward(false);

  // Same reference with feed-forward enabled.
  PositionControl ff_on;
  configure(ff_on);
  primeAirborne(ff_on);
  ff_on.enableFeedforward(true);

  int failures = 0;
  for (int i = 0; i < 40; ++i) {
    const Eigen::Vector3d state_pos(0.0, 0.0, 1.0);
    const Eigen::Vector3d state_vel = Eigen::Vector3d::Zero();
    const Eigen::Vector3d state_acc = Eigen::Vector3d::Zero();

    baseline.setState(state_pos, state_vel, 0.0);
    baseline.setCurrentAcceleration(state_acc);
    baseline.setSetpoint(pos_sp, yaw_sp);
    baseline.update(0.02);

    ff_off.setState(state_pos, state_vel, 0.0);
    ff_off.setCurrentAcceleration(state_acc);
    ff_off.setReference(ref);
    ff_off.update(0.02);

    ff_on.setState(state_pos, state_vel, 0.0);
    ff_on.setCurrentAcceleration(state_acc);
    ff_on.setReference(ref);
    ff_on.update(0.02);
  }

  // Disabled feed-forward must reproduce the baseline exactly.
  if (std::abs(baseline.getThrustSetpoint() - ff_off.getThrustSetpoint()) > 1e-12) {
    std::cerr << "FAIL: feed-forward OFF diverged from baseline thrust\n";
    ++failures;
  }
  if (baseline.getAttitudeSetpoint().angularDistance(ff_off.getAttitudeSetpoint()) > 1e-12) {
    std::cerr << "FAIL: feed-forward OFF diverged from baseline attitude\n";
    ++failures;
  }

  // Enabled feed-forward must actually change the command.
  const double thrust_delta = std::abs(baseline.getThrustSetpoint() - ff_on.getThrustSetpoint());
  const double att_delta = baseline.getAttitudeSetpoint().angularDistance(ff_on.getAttitudeSetpoint());
  if (thrust_delta < 1e-9 && att_delta < 1e-9) {
    std::cerr << "FAIL: feed-forward ON did not change the command\n";
    ++failures;
  }

  if (failures == 0) {
    std::cout << "feedforward: all checks passed\n";
    return 0;
  }
  return 1;
}
