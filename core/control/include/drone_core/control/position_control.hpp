#pragma once

#include <Eigen/Dense>

#include "drone_core/common/types.hpp"

namespace drone_core::control {

// Cascaded position controller: position error drives a velocity setpoint, a
// PID velocity loop drives an acceleration setpoint, and that acceleration is
// mapped (with gravity compensation) into a desired attitude and collective
// thrust. The acceleration-to-attitude stage is the differential-flatness
// output map, which is why feed-forward trajectory tracking can be layered on
// without reworking the controller.
//
// These gains and the takeoff/hover-thrust behaviour have been tuned on the
// real vehicle and proven stable in flight. Treat them as load-bearing: change
// them only with a specific reason and validate in simulation first.
class PositionControl {
public:
  PositionControl();

  // Configuration.
  void setPositionGains(const Eigen::Vector3d& P);
  void setVelocityGains(const Eigen::Vector3d& P, const Eigen::Vector3d& I, const Eigen::Vector3d& D);
  void setConstraints(double vel_horizontal, double vel_up, double vel_down, double tilt_max_rad);
  void setHoverThrust(double hover_thrust);
  void setThrustLearningRate(double learning_rate);

  // Time constant of the low-pass on the D term [s]. The raw derivative is
  // recomputed only when a new state measurement arrives (see setStateStamp),
  // so this filter is what turns that stepwise signal back into something
  // smooth enough to feed the acceleration setpoint every tick. Larger =
  // smoother but more phase lag, which costs real damping; keep it near one
  // measurement period.
  void setDerivativeTau(double tau);

  // Position error [m] above which the velocity integrator is frozen: held at
  // its current value, not reset. Checked separately for the horizontal (XY
  // norm) and vertical axes. A value <= 0 disables the gate (always integrate).
  void setIntegratorErrorLimit(double max_pos_err);

  // Loop inputs.
  void setState(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, double yaw);

  // Measured thrust per unit mass along the vehicle's up axis (9.81 in hover),
  // NOT the vehicle's acceleration and NOT a world-frame quantity — see the
  // note on common::State::thrust_accel. Feeds the hover-thrust estimator only.
  void setThrustAccel(double thrust_accel);

  // Timestamp [s] of the pos/vel sample last passed to setState. The estimate
  // is a zero-order hold — the host re-sends the newest sample every control
  // tick whether or not a new one arrived — so the D term needs to know when
  // the measurement genuinely advanced, and by how much, rather than assuming
  // one control period. Optional: a host that never calls this makes the
  // derivative fall back to per-tick differencing.
  void setStateStamp(double stamp);
  void setSetpoint(const Eigen::Vector3d& pos_sp, double yaw_sp);

  // Set the full tracking reference, including velocity and acceleration
  // feed-forward. When feed-forward is disabled (the default) the velocity and
  // acceleration terms are ignored and the controller behaves exactly as it
  // does under setSetpoint() alone.
  void setReference(const common::Reference& ref);

  // Enable or disable differential-flatness feed-forward. Off by default so the
  // baseline, flight-proven behaviour is the default.
  void enableFeedforward(bool enabled);

  // Run one control step. dt is the time since the previous call.
  void update(double dt);
  void reset();

  // Outputs.
  Eigen::Quaterniond getAttitudeSetpoint() const;
  double getThrustSetpoint() const;

  // Telemetry.
  Eigen::Vector3d getPositionSetpoint() const { return _pos_sp; }
  Eigen::Vector3d getVelocitySetpoint() const { return _vel_sp; }
  Eigen::Vector3d getAccelerationSetpoint() const { return _acc_sp; }
  double getHoverThrust() const { return _hover_thrust; }

  // The raw reference last handed to setReference(), before any PID correction
  // is added (setSetpoint() alone does not update these). Position has no
  // separate feedback term the way velocity/acceleration do (the trajectory's
  // position IS the position setpoint), so getPositionFeedforward() and
  // getPositionSetpoint() read identically while tracking through
  // TrajectoryTracker, which always calls setReference(). vel_ff/acc_ff are
  // zero outside kTracking, matching setReference()'s callers.
  Eigen::Vector3d getPositionFeedforward() const { return _pos_ff; }
  Eigen::Vector3d getVelocityFeedforward() const { return _vel_ff; }
  Eigen::Vector3d getAccelerationFeedforward() const { return _acc_ff; }
  Eigen::Vector3d getVelocityPTerm() const { return _vel_p_term; }
  Eigen::Vector3d getVelocityITerm() const { return _vel_int; }
  Eigen::Vector3d getVelocityDTerm() const { return _vel_d_term; }

private:
  void _positionControl();
  void _velocityControl(double dt);
  void _accelerationControl();
  void _updateHoverThrust(double dt);
  void _seedHoverFilters();
  Eigen::Vector2d _constrainXY(const Eigen::Vector2d& v0, double max);

  // Gains.
  Eigen::Vector3d _gain_pos_p;
  Eigen::Vector3d _gain_vel_p;
  Eigen::Vector3d _gain_vel_i;
  Eigen::Vector3d _gain_vel_d;

  // Limits.
  double _lim_vel_horz;
  double _lim_vel_up;
  double _lim_vel_down;
  double _lim_tilt;

  double _hover_thrust;
  double _filtered_thrust_cmd;
  // Low-passed thrust gain: measured thrust accel per unit of lagged command
  // [m/s^2 per unit thrust], 9.81 / hover thrust. The noisy accel sits on top of
  // this fraction so its noise averages out; see _updateHoverThrust.
  double _thrust_gain_lpf{9.81 / 0.4};
  bool _reset_hover_filter;
  double _learning_rate;
  double _hover_thrust_convergence_time;
  bool _in_air{false};
  bool _is_taking_off{false};
  double _takeoff_ramp_thrust{0.0};
  int _takeoff_phase{1};
  bool _takeoff_primed{false};

  // Estimated state.
  Eigen::Vector3d _pos;
  Eigen::Vector3d _vel;
  double _thrust_accel{9.81};  // defaults to hover so a missed setter reads as neutral
  double _yaw;

  // Commanded setpoint.
  Eigen::Vector3d _pos_sp;
  double _yaw_sp;

  // Differential-flatness feed-forward, applied only when enabled and only once
  // the vehicle is past the open-loop takeoff ramp.
  Eigen::Vector3d _pos_ff{Eigen::Vector3d::Zero()};
  Eigen::Vector3d _vel_ff{Eigen::Vector3d::Zero()};
  Eigen::Vector3d _acc_ff{Eigen::Vector3d::Zero()};
  bool _feedforward_enabled{false};

  // Intermediate targets computed by the cascade.
  Eigen::Vector3d _vel_sp;
  Eigen::Vector3d _acc_sp;
  Eigen::Quaterniond _attitude_sp;
  double _thrust_sp;

  // Integrator memory.
  Eigen::Vector3d _vel_int;
  bool _first_update = true;
  double _int_err_limit{0.0};  // <= 0: integrator gate disabled

  // Derivative-on-measurement memory. The derivative is taken on _vel rather
  // than on the velocity error: the two are identical in steady state, but
  // differentiating the error also differentiates the setpoint, which injects a
  // one-tick kick every time the setpoint steps. Dropping that half is why
  // _vel_d_term carries a minus sign.
  double _state_stamp{-1.0};       // newest stamp handed in; <0 = host supplies none
  double _prev_state_stamp{-1.0};  // stamp the raw derivative was last taken at
  Eigen::Vector3d _prev_vel_meas{Eigen::Vector3d::Zero()};
  Eigen::Vector3d _vel_deriv_raw{Eigen::Vector3d::Zero()};
  Eigen::Vector3d _vel_deriv_filt{Eigen::Vector3d::Zero()};
  double _deriv_tau;

  Eigen::Vector3d _vel_p_term;
  Eigen::Vector3d _vel_d_term;
};

}  // namespace drone_core::control
