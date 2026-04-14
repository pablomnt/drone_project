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
    
    _lim_vel_horz = 10.0;
    _lim_vel_up = 2.0;
    _lim_vel_down = 1.0;
    _lim_tilt = 0.43; // ~25 degrees
    _hover_thrust = 0.5;
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

void PositionControl::setSetpoint(const Eigen::Vector3d& pos_sp, double yaw_sp) {
    _pos_sp = pos_sp;
    _yaw_sp = yaw_sp;
}

void PositionControl::update(double dt) {
    if (dt <= 0.001) return; // Safety check

    _positionControl();
    _velocityControl(dt);
    _accelerationControl();
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
//Currently DOES NOT use D term
void PositionControl::_velocityControl(double dt) {
    // 1. Calculate Velocity Error
    Eigen::Vector3d vel_error = _vel_sp - _vel;

    // 2. PID Calculation
    // acc_sp_velocity = vel_error.emult(_gain_vel_p) + _vel_int - _vel_dot.emult(_gain_vel_d);
    // Note: We ignore D-term here for simplicity (it requires computing derivative of velocity). 
    // You can add it later if needed.
    Eigen::Vector3d acc_sp = vel_error.cwiseProduct(_gain_vel_p) + _vel_int;

    // 3. Update Integrator (I-term)
    // PX4 uses sophisticated anti-windup. We do simple clamping for now.
    _vel_int += vel_error.cwiseProduct(_gain_vel_i) * dt;
    
    // Clamp integrator to avoid it growing infinite
    double int_limit = 5.0; // 5 m/s^2 limit
    _vel_int = _vel_int.cwiseMin(int_limit).cwiseMax(-int_limit);

    _acc_sp = acc_sp;
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