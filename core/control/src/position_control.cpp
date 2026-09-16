#include "drone_core/control/position_control.hpp"

#include <algorithm>
#include <cmath>

namespace {
// Shortest state-sample interval treated as a genuine new measurement [s].
// Position sources here run at tens of Hz, so anything faster than 200 Hz is a
// message-queue artefact rather than a real sample.
constexpr double kMinMeasInterval = 0.005;
}  // namespace

namespace drone_core::control {

PositionControl::PositionControl() {
  // Default to zeroed gains so a forgotten setGains() call produces a limp
  // controller rather than undefined behaviour.
  _gain_pos_p.setZero();
  _gain_vel_p.setZero();
  _gain_vel_i.setZero();
  _gain_vel_d.setZero();
  _vel_int.setZero();

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
  // One VIO sample period (~25 Hz) is a defensible starting point: enough to
  // smooth the stepwise raw derivative without burying the loop in phase lag.
  // Tune via MPC_VEL_D_TAU.
  _deriv_tau = 0.04;
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

void PositionControl::setHoverThrust(double hover_thrust) {
  _hover_thrust = hover_thrust;
  _seedHoverFilters();
}

void PositionControl::setState(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, double yaw) {
  _pos = pos;
  _vel = vel;
  _yaw = yaw;
}

void PositionControl::setThrustAccel(double thrust_accel) {
  _thrust_accel = thrust_accel;
}

void PositionControl::setStateStamp(double stamp) {
  _state_stamp = stamp;
}

void PositionControl::setDerivativeTau(double tau) {
  _deriv_tau = std::max(tau, 0.0);
}

void PositionControl::setIntegratorErrorLimit(double max_pos_err) {
  _int_err_limit = max_pos_err;
}

void PositionControl::setSetpoint(const Eigen::Vector3d& pos_sp, double yaw_sp) {
  _pos_sp = pos_sp;
  _yaw_sp = yaw_sp;
}

void PositionControl::setReference(const common::Reference& ref) {
  _pos_sp = ref.pos;
  _yaw_sp = ref.yaw;
  _pos_ff = ref.pos;
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
      _takeoff_ramp_thrust += (0.05 * dt);
    } else {
      _takeoff_phase = 2;
      if (_vel.z() > 0.4*100) {
        _takeoff_ramp_thrust -= (0.05 * dt);
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

  // Drop the derivative history: after a reset the next measurement is not
  // continuous with the last one, so differencing across the gap is meaningless.
  _prev_state_stamp = -1.0;
  _prev_vel_meas.setZero();
  _vel_deriv_raw.setZero();
  _vel_deriv_filt.setZero();

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
    _prev_vel_meas = _vel;
    _learning_rate = dt / _hover_thrust_convergence_time;
    _first_update = false;
  }

  // Derivative on the MEASUREMENT, taken on the estimator's clock rather than
  // the control clock. _vel is a zero-order hold: the host re-sends the newest
  // VIO sample every control tick whether or not a new one arrived, so
  // differencing per tick produced exactly 0.0 on held ticks and, on the ticks
  // a sample did land, the whole accumulated change divided by the control
  // period instead of the true (longer) measurement interval. At 25 Hz VIO
  // against a 50 Hz loop that is a train of double-height impulses on alternate
  // ticks, not a derivative — it delivered damping half the time and overstated
  // it when it did.
  if (_state_stamp < 0.0) {
    // No timestamps from this host (unit tests, or a host that never calls
    // setStateStamp): fall back to per-tick differencing, which is correct when
    // the estimate really does update every tick.
    _vel_deriv_raw = (_vel - _prev_vel_meas) / dt;
    _prev_vel_meas = _vel;
  } else if (_state_stamp > _prev_state_stamp) {
    if (_prev_state_stamp < 0.0) {
      // First sample after a reset: the sentinel is not a time, so there is
      // nothing to difference against yet. Just seed and wait.
      _prev_vel_meas = _vel;
      _prev_state_stamp = _state_stamp;
    } else {
      const double meas_dt = _state_stamp - _prev_state_stamp;
      // Ignore implausibly short intervals. The stamp is a message ARRIVAL time,
      // so two samples delivered back-to-back out of a queue can land a
      // fraction of a millisecond apart while representing a full sample period
      // of motion; dividing by that inflates the derivative enormously. Leave
      // the previous sample untouched so the next update differences across the
      // real span rather than losing it.
      if (meas_dt >= kMinMeasInterval) {
        _vel_deriv_raw = (_vel - _prev_vel_meas) / meas_dt;
        _prev_vel_meas = _vel;
        _prev_state_stamp = _state_stamp;
      }
    }
  }

  // The low-pass runs every control tick regardless of whether the raw value
  // was refreshed, so the D term stays smooth and current between measurements
  // instead of stepping or dropping to zero.
  const double deriv_alpha = (_deriv_tau > 0.0) ? (dt / (_deriv_tau + dt)) : 1.0;
  _vel_deriv_filt += deriv_alpha * (_vel_deriv_raw - _vel_deriv_filt);

  _vel_p_term = vel_error.cwiseProduct(_gain_vel_p);
  // Damp the vehicle's acceleration relative to what the trajectory asks for,
  // not its raw acceleration. Measured -d(vel)/dt alone resists every speed-up,
  // including the planned ones: in flight (2026-09-16) it cancelled 30-45% of the
  // acceleration feed-forward and the drone ran 0.2-0.4 s behind the reference.
  // The trajectory's acceleration stands in for the setpoint half of the
  // derivative, which is smooth (no setpoint-step kick). Outside tracking acc_ff
  // is not applied, so this is the plain measurement form used in hover.
  Eigen::Vector3d accel_ref = Eigen::Vector3d::Zero();
  if (_feedforward_enabled && !_is_taking_off) {
    accel_ref = _acc_ff;
  }
  _vel_d_term = (accel_ref - _vel_deriv_filt).cwiseProduct(_gain_vel_d);

  // Integrate only near the setpoint. Far from it the velocity error is the
  // transient of a large move, not a steady bias, and integrating it winds up a
  // term that must unwind after arrival as overshoot. Frozen rather than reset:
  // the integrator carries the standing trim (hover-thrust mismatch, wind), and
  // throwing that away on every large error would make the vehicle sag or drift
  // on arrival while it re-learns.
  Eigen::Vector3d int_step = vel_error.cwiseProduct(_gain_vel_i) * dt;
  if (_int_err_limit > 0.0) {
    const Eigen::Vector3d pos_err = _pos_sp - _pos;
    if (pos_err.head<2>().norm() > _int_err_limit) int_step.head<2>().setZero();
    if (std::abs(pos_err.z()) > _int_err_limit) int_step.z() = 0.0;
  }
  _vel_int += int_step;

  const double int_limit = 0.4;  // limit the integrator to avoid windup
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

void PositionControl::_seedHoverFilters() {
  // Park the gain filter on the current estimate, so the hover thrust starts
  // exactly at _hover_thrust and moves only as new flight data arrives.
  _thrust_gain_lpf = 9.81 / std::clamp(_hover_thrust, 0.2, 0.5);
}

void PositionControl::_updateHoverThrust(double dt) {
  // Re-seed the filter after a regime change (e.g. a mid-air mode switch) so it
  // does not spool up from a stale state.
  if (_reset_hover_filter) {
    _filtered_thrust_cmd = _thrust_sp;
    _seedHoverFilters();
    _reset_hover_filter = false;
  }

  // Suspend estimation while the motors idle or are disarmed; lock the filter
  // to the command so it cannot diverge.
  if (_thrust_sp < 0.1) {
    _filtered_thrust_cmd = _thrust_sp;
    _seedHoverFilters();
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
      _seedHoverFilters();
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

  // Trust the IMU less while climbing or descending fast, where unmodelled
  // aerodynamics corrupt the measurement.
  const double current_speed = std::abs(_vel.z());
  double trust_factor = 1.0;
  if (current_speed > 0.5) {
    trust_factor = 0.5 / current_speed;
  }
  const double effective_learning_rate = _learning_rate * trust_factor;

  // Invert the idealised vertical dynamics a_z = g (T / T_hover - 1) to back out
  // the hover thrust. That model assumes thrust acts along body z, and
  // _thrust_accel measures the specific force along exactly that axis, so the
  // two assumptions match and the result is correct at any tilt — which is why
  // no attitude rotation belongs here.
  //
  // The noisy accel must stay on TOP of the fraction. In flight it reads
  // ±3.5 m/s² of vibration around 9.81. The previous form averaged T g / a,
  // putting that noise underneath, where a dip to 4 m/s² inflates the result far
  // more than a spike to 15 deflates it: it read 0.31 against a true hover of
  // 0.29 on both 2026-09-14 flights, pinned the z integrator and held the
  // vehicle ~20 cm above its setpoint. So filter the thrust gain a / T instead.
  // Noise enters it linearly and cancels, every sample weighs the same, and T is
  // a clean command. The final g / gain divides by an already-smoothed value,
  // so it adds no bias.
  //
  // Guard: anything under 0.1 thrust returned above, but the motor-lag filter
  // can trail just below that for a tick after the command rises.
  if (_filtered_thrust_cmd > 0.05) {
    const double thrust_gain = _thrust_accel / _filtered_thrust_cmd;
    _thrust_gain_lpf += effective_learning_rate * (thrust_gain - _thrust_gain_lpf);
    _hover_thrust = std::clamp(9.81 / _thrust_gain_lpf, 0.2, 0.5);
  }
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
