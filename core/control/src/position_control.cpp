#include "drone_core/control/position_control.hpp"

#include <algorithm>
#include <cmath>

namespace drone_core::control {

PositionControl::PositionControl() {
  // Default to zeroed gains so a forgotten setGains() call produces a limp
  // controller rather than undefined behaviour.
  _gain_pos_p.setZero();
  _gain_vel_p.setZero();
  _gain_vel_i.setZero();
  _gain_vel_d.setZero();
  _vel_int.setZero();
  _prev_vel_error.setZero();

  _vel_p_term.setZero();
  _vel_d_term.setZero();

  _lim_vel_horz = 10.0;
  _lim_vel_up = 2.0;
  _lim_vel_down = 1.0;
  _lim_tilt = 0.43;  // ~25 degrees
  _hover_thrust = 0.4;
  _filtered_thrust_cmd = 0.5;
  _reset_hover_filter = true;
  _hover_thrust_convergence_time = 2.5;  // time constant for the hover-thrust learning rate [s]
  _learning_rate = 0.02;
}

void PositionControl::setPositionGains(const Eigen::Vector3d& P) { _gain_pos_p = P; }

void PositionControl::setVelocityGains(const Eigen::Vector3d& P, const Eigen::Vector3d& I, const Eigen::Vector3d& D) {
  _gain_vel_p = P;
  _gain_vel_i = I;
  _gain_vel_d = D;
}

void PositionControl::setConstraints(double vel_horizontal, double vel_up, double vel_down, double tilt_max_rad) {
  _lim_vel_horz = vel_horizontal;
  _lim_vel_up = vel_up;
  _lim_vel_down = vel_down;
  _lim_tilt = tilt_max_rad;
}

void PositionControl::setHoverThrust(double hover_thrust) { _hover_thrust = hover_thrust; }

void PositionControl::setState(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, double yaw) {
  _pos = pos;
  _vel = vel;
  _yaw = yaw;
}

void PositionControl::setCurrentAcceleration(const Eigen::Vector3d& acc) {
  _acc = acc;
}

void PositionControl::setSetpoint(const Eigen::Vector3d& pos_sp, double yaw_sp) {
  _pos_sp = pos_sp;
  _yaw_sp = yaw_sp;
}

void PositionControl::setReference(const common::Reference& ref) {
  _pos_sp = ref.pos;
  _yaw_sp = ref.yaw;
  _vel_ff = ref.vel_ff;
  _acc_ff = ref.acc_ff;
}

void PositionControl::enableFeedforward(bool enabled) {
  _feedforward_enabled = enabled;
}

void PositionControl::update(double dt) {
  if (dt <= 0.001) return;

  _positionControl();
  _velocityControl(dt);
  _accelerationControl();

  if (_takeoff_primed && _pos_sp.z() > 0.5) {
    _is_taking_off = true;    // begin the open-loop ramp
    _takeoff_primed = false;  // consume the authorisation; only a ground reset can re-prime it
  }

  // Open-loop takeoff override. Closed-loop control near the ground with noisy
  // VIO causes the vehicle to skid, so until liftoff is confirmed we force a
  // level attitude and ramp thrust open-loop instead of trusting the cascade.
  if (_is_taking_off) {
    // Force a perfectly level attitude (yaw passes through) so the vehicle
    // physically cannot skid sideways during the ramp.
    Eigen::Vector3d y_C(-std::sin(_yaw_sp), std::cos(_yaw_sp), 0.0);
    Eigen::Vector3d body_x = y_C.cross(Eigen::Vector3d(0, 0, 1));
    body_x.normalize();
    Eigen::Vector3d body_y = Eigen::Vector3d(0, 0, 1).cross(body_x);

    Eigen::Matrix3d flat_rot;
    flat_rot.col(0) = body_x;
    flat_rot.col(1) = body_y;
    flat_rot.col(2) = Eigen::Vector3d(0, 0, 1);
    _attitude_sp = Eigen::Quaterniond(flat_rot);

    // Ramp thrust until the vehicle starts climbing, then ease off.
    if (_takeoff_phase == 1 && _vel.z() < 0.6) {
      _takeoff_ramp_thrust += (0.3 * dt);
    } else {
      _takeoff_phase = 2;
      if (_vel.z() > 0.6) {
        _takeoff_ramp_thrust -= (0.1 * dt);
      }
    }

    _takeoff_ramp_thrust = std::min(_takeoff_ramp_thrust, 0.4);

    _thrust_sp = _takeoff_ramp_thrust;

    // Hold the integrators at zero so they do not wind up while overridden.
    _vel_int.setZero();

    // Once clear of the ground, hand back to the PID and seed the hover-thrust
    // filter with the thrust that just achieved liftoff for a seamless handoff.
    if (_pos.z() > 1.0) {
      _is_taking_off = false;
      _takeoff_phase = 1;
      _takeoff_ramp_thrust = 0.15;
      _filtered_thrust_cmd = _thrust_sp;
    }
  }

  _updateHoverThrust(dt);
}

void PositionControl::reset() {
  _vel_int.setZero();
  _pos_sp.setZero();
  _vel_sp.setZero();
  _acc_sp.setZero();
  _first_update = true;

  // Tell the hover-thrust estimator a new flight regime is starting.
  _reset_hover_filter = true;
  _takeoff_ramp_thrust = 0.15;
  _takeoff_phase = 1;
  _is_taking_off = false;

  // Decide, at the instant of reset, whether we are sitting on the ground and
  // therefore authorised for exactly one open-loop takeoff. A mid-air reset
  // (mode switch) must not trigger a takeoff ramp.
  if (_pos.z() < 0.2 && std::abs(_vel.z()) < 0.2) {
    _in_air = false;
    _takeoff_primed = true;
  } else {
    _in_air = true;
    _takeoff_primed = false;
  }
}

void PositionControl::_positionControl() {
  const Eigen::Vector3d error = _pos_sp - _pos;
  Eigen::Vector3d vel_sp = error.cwiseProduct(_gain_pos_p);

  // Add the trajectory velocity as feed-forward before clamping, so the total
  // commanded velocity stays within the configured limits.
  if (_feedforward_enabled && !_is_taking_off) {
    vel_sp += _vel_ff;
  }

  const Eigen::Vector2d vel_sp_xy = _constrainXY(vel_sp.head<2>(), _lim_vel_horz);
  const double vel_sp_z = std::clamp(vel_sp.z(), -_lim_vel_down, _lim_vel_up);

  _vel_sp << vel_sp_xy.x(), vel_sp_xy.y(), vel_sp_z;
}

void PositionControl::_velocityControl(double dt) {
  const Eigen::Vector3d vel_error = _vel_sp - _vel;

  // Seed the derivative memory on the first step so it does not see an
  // artificial spike, and convert the hover-thrust convergence time into the
  // per-step learning rate.
  if (_first_update) {
    _prev_vel_error = vel_error;
    _learning_rate = dt / _hover_thrust_convergence_time;
    _first_update = false;
  }

  const Eigen::Vector3d vel_derivative = (vel_error - _prev_vel_error) / dt;
  _prev_vel_error = vel_error;

  _vel_p_term = vel_error.cwiseProduct(_gain_vel_p);
  _vel_d_term = vel_derivative.cwiseProduct(_gain_vel_d);

  _vel_int += vel_error.cwiseProduct(_gain_vel_i) * dt;

  const double int_limit = 5.0;
  _vel_int = _vel_int.cwiseMin(int_limit).cwiseMax(-int_limit);

  _acc_sp = _vel_p_term + _vel_int + _vel_d_term;

  // Add the trajectory acceleration as feed-forward; the PID terms above are
  // then only correcting the residual tracking error.
  if (_feedforward_enabled && !_is_taking_off) {
    _acc_sp += _acc_ff;
  }
}

void PositionControl::_accelerationControl() {
  // Add gravity compensation: in ENU gravity is -9.81, so the thrust vector
  // must push up by +9.81.
  const Eigen::Vector3d thrust_vector = _acc_sp + Eigen::Vector3d(0, 0, 9.81);

  const double accel_norm = thrust_vector.norm();
  _thrust_sp = (accel_norm / 9.81) * _hover_thrust;
  _thrust_sp = std::clamp(_thrust_sp, 0.0, 1.0);

  // Align the body z-axis with the desired acceleration and resolve the
  // remaining degree of freedom with the yaw setpoint.
  const Eigen::Vector3d body_z = thrust_vector.normalized();
  const Eigen::Vector3d y_C(-std::sin(_yaw_sp), std::cos(_yaw_sp), 0.0);
  Eigen::Vector3d body_x = y_C.cross(body_z);

  if (body_x.norm() < 0.0001) {
    // Pointing straight up is singular; fall back to the yaw direction.
    body_x << std::cos(_yaw_sp), std::sin(_yaw_sp), 0.0;
  }
  body_x.normalize();

  const Eigen::Vector3d body_y = body_z.cross(body_x);

  Eigen::Matrix3d rot;
  rot.col(0) = body_x;
  rot.col(1) = body_y;
  rot.col(2) = body_z;

  _attitude_sp = Eigen::Quaterniond(rot);
}

void PositionControl::_updateHoverThrust(double dt) {
  // Re-seed the filter after a regime change (e.g. a mid-air mode switch) so it
  // does not spool up from a stale state.
  if (_reset_hover_filter) {
    _filtered_thrust_cmd = _thrust_sp;
    _reset_hover_filter = false;
  }

  // Suspend estimation while the motors idle or are disarmed; lock the filter
  // to the command so it cannot diverge.
  if (_thrust_sp < 0.1) {
    _filtered_thrust_cmd = _thrust_sp;
    return;
  }

  // Detect that we have actually left the ground before trusting the estimator.
  if (!_in_air) {
    if (_pos.z() > 0.2 || _vel.z() > 0.2) {
      _in_air = true;
    } else {
      // Still grounded: keep the filter pre-charged so there is no glitch at
      // the moment of takeoff.
      _filtered_thrust_cmd = _thrust_sp;
      return;
    }
  }

  // Turn the flag back off if we land or fall.
  if (_pos.z() < 0.2 && _vel.z() < 0.1 && _thrust_sp < 0.2) {
    _in_air = false;
  }

  // First-order low-pass on the thrust command models the propeller spool-up
  // lag, aligning the commanded signal with the measured IMU dynamics.
  constexpr double tau = 0.05;  // motor time constant [s]
  const double alpha = dt / (tau + dt);
  _filtered_thrust_cmd += alpha * (_thrust_sp - _filtered_thrust_cmd);

  // Invert the idealised vertical dynamics a_z = g (T / T_hover - 1) to back out
  // the hover thrust. _acc.z() is already gravity-compensated (0 at hover).
  const double measured_az = _acc.z();
  double inst_hover_thrust = (_filtered_thrust_cmd * 9.81) / (measured_az + 9.81);
  inst_hover_thrust = std::clamp(inst_hover_thrust, 0.2, 0.5);

  // Trust the IMU less while climbing or descending fast, where unmodelled
  // aerodynamics corrupt the instantaneous estimate.
  const double current_speed = std::abs(_vel.z());
  double trust_factor = 1.0;
  if (current_speed > 0.5) {
    trust_factor = 0.5 / current_speed;
  }

  const double effective_learning_rate = _learning_rate * trust_factor;
  _hover_thrust = (_hover_thrust * (1.0 - effective_learning_rate)) +
                  (inst_hover_thrust * effective_learning_rate);
}

Eigen::Quaterniond PositionControl::getAttitudeSetpoint() const { return _attitude_sp; }

double PositionControl::getThrustSetpoint() const { return _thrust_sp; }

Eigen::Vector2d PositionControl::_constrainXY(const Eigen::Vector2d& v0, double max) {
  if (v0.norm() <= max) {
    return v0;
  }
  return v0.normalized() * max;
}

}  // namespace drone_core::control
