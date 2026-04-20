#include "position_controller/PositionControl.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

PositionControl::PositionControl() {
    // Default initialization to prevent crashing if you forget to set gains
    _gain_pos_p.setZero();
    _gain_vel_p.setZero();
    _gain_vel_i.setZero();
    _gain_vel_d.setZero();
    _vel_int.setZero();
    _prev_vel_error.setZero();
    
    // Initialize our new telemetry variables
    _vel_p_term.setZero();
    _vel_d_term.setZero();

    _lim_vel_horz = 10.0;
    _lim_vel_up = 2.0;
    _lim_vel_down = 1.0;
    _lim_tilt = 0.43; // ~25 degrees
    _hover_thrust = 0.5;
    _learning_rate = 0.0005;
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

void PositionControl::setThrustLearningRate(double learning_rate) { _learning_rate = learning_rate; }

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

void PositionControl::update(double dt) {
    if (dt <= 0.001) return; // Safety check

    _positionControl();
    _velocityControl(dt);
    _accelerationControl();
    _updateHoverThrust();
}

void PositionControl::reset() {
    _vel_int.setZero();    
    _pos_sp.setZero(); 
    _vel_sp.setZero();
    _acc_sp.setZero();
    _first_update = true;
}

// ---------------------------------------------------------
// Position Loop -> Outputs Velocity Setpoint
// ---------------------------------------------------------
void PositionControl::_positionControl() {
    // 1. Calculate Error (Target - Current)
    Eigen::Vector3d error = _pos_sp - _pos;
    
    // Print the error for debugging
    // std::cout << "Position error: " << error.transpose() << std::endl;

    // 2. Apply P Gain to get desired velocity
    // vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p);
    Eigen::Vector3d vel_sp = error.cwiseProduct(_gain_pos_p);

    // 3. Constrain Velocity (Logic from ControlMath::constrainXY)
    // _vel_sp.xy() = ControlMath::constrainXY(...)
    Eigen::Vector2d vel_sp_xy = _constrainXY(vel_sp.head<2>(), _lim_vel_horz);
    
    // Constrain Z
    double vel_sp_z = std::clamp(vel_sp.z(), -_lim_vel_down, _lim_vel_up);

    _vel_sp << vel_sp_xy.x(), vel_sp_xy.y(), vel_sp_z;
}

// ---------------------------------------------------------
// Velocity Loop -> Outputs Acceleration Setpoint
// ---------------------------------------------------------
void PositionControl::_velocityControl(double dt) {
    Eigen::Vector3d vel_error = _vel_sp - _vel;

    // --- PREVENT DERIVATIVE KICK ---
    // If this is the very first loop, initialize the previous error 
    // so the derivative doesn't see a massive artificial spike.
    if (_first_update) {
        _prev_vel_error = vel_error; 
        _first_update = false;
    }

    // Calculate the rate of change of the error (Derivative) safely
    Eigen::Vector3d vel_derivative = (vel_error - _prev_vel_error) / dt;
    _prev_vel_error = vel_error;

    // 1. Calculate and store the individual PID terms for telemetry
    _vel_p_term = vel_error.cwiseProduct(_gain_vel_p);
    _vel_d_term = vel_derivative.cwiseProduct(_gain_vel_d);

    // 2. Update Integrator (I-term)
    _vel_int += vel_error.cwiseProduct(_gain_vel_i) * dt;
    
    // Clamp integrator to prevent windup
    double int_limit = 5.0;
    _vel_int = _vel_int.cwiseMin(int_limit).cwiseMax(-int_limit);

    // 3. Sum them all together for the final acceleration command
    _acc_sp = _vel_p_term + _vel_int + _vel_d_term;
}

// ---------------------------------------------------------
// Acceleration -> Attitude & Thrust
// ---------------------------------------------------------
void PositionControl::_accelerationControl() {
    // 1. Add Gravity Compensation
    // In ROS (ENU), Gravity is -9.8, so we need to push UP (+9.81).
    Eigen::Vector3d thrust_vector = _acc_sp + Eigen::Vector3d(0, 0, 9.81);

    // 2. Calculate Thrust (Magnitude)
    double accel_norm = thrust_vector.norm();
    _thrust_sp = (accel_norm / 9.81) * _hover_thrust;
    
    // Clamp thrust [0, 1]
    _thrust_sp = std::clamp(_thrust_sp, 0.0, 1.0);

    // 3. Calculate Orientation (Quaternion)
    // We want the drone's Z-axis to align with our desired acceleration vector.
    Eigen::Vector3d body_z = thrust_vector.normalized();

    // Desired Yaw direction
    Eigen::Vector3d y_C(-std::sin(_yaw_sp), std::cos(_yaw_sp), 0.0);
    Eigen::Vector3d body_x = y_C.cross(body_z);

    if (body_x.norm() < 0.0001) {
        // Singularity: if we are thrusting straight up, assume X is forward
        body_x << std::cos(_yaw_sp), std::sin(_yaw_sp), 0.0;
    }
    body_x.normalize();

    Eigen::Vector3d body_y = body_z.cross(body_x);

    Eigen::Matrix3d rot;
    rot.col(0) = body_x;
    rot.col(1) = body_y;
    rot.col(2) = body_z;

    // SAVE THE OUTPUTS TO MEMBER VARIABLES
    _attitude_sp = Eigen::Quaterniond(rot);
}

void PositionControl::_updateHoverThrust() {
    Eigen::Vector3d acc_error = _acc_sp - _acc;
    double acc_error_z = acc_error.z();
    _hover_thrust += acc_error_z * _learning_rate;
}

Eigen::Quaterniond PositionControl::getAttitudeSetpoint() {
    // Just return the pre-calculated value!
    return _attitude_sp;
}

double PositionControl::getThrustSetpoint() {
    // Just return the pre-calculated value!
    return _thrust_sp;
}

Eigen::Vector2d PositionControl::_constrainXY(const Eigen::Vector2d& v0, double max) {
    if (v0.norm() <= max) {
        return v0;
    } else {
        return v0.normalized() * max;
    }
}