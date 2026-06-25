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

  // Loop inputs.
  void setState(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, double yaw);
  void setCurrentAcceleration(const Eigen::Vector3d& acc);
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
  Eigen::Vector3d getVelocityPTerm() const { return _vel_p_term; }
  Eigen::Vector3d getVelocityITerm() const { return _vel_int; }
  Eigen::Vector3d getVelocityDTerm() const { return _vel_d_term; }

private:
  void _positionControl();
  void _velocityControl(double dt);
  void _accelerationControl();
  void _updateHoverThrust(double dt);
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
  Eigen::Vector3d _acc;
  double _yaw;

  // Commanded setpoint.
  Eigen::Vector3d _pos_sp;
  double _yaw_sp;

  // Differential-flatness feed-forward, applied only when enabled and only once
  // the vehicle is past the open-loop takeoff ramp.
  Eigen::Vector3d _vel_ff{Eigen::Vector3d::Zero()};
  Eigen::Vector3d _acc_ff{Eigen::Vector3d::Zero()};
  bool _feedforward_enabled{false};

  // Intermediate targets computed by the cascade.
  Eigen::Vector3d _vel_sp;
  Eigen::Vector3d _acc_sp;
  Eigen::Quaterniond _attitude_sp;
  double _thrust_sp;

  // Integrator and derivative memory.
  Eigen::Vector3d _vel_int;
  Eigen::Vector3d _prev_vel_error;
  bool _first_update = true;

  Eigen::Vector3d _vel_p_term;
  Eigen::Vector3d _vel_d_term;
};

}  // namespace drone_core::control
