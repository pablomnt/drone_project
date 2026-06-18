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
    _hover_thrust = 0.4;
    _filtered_thrust_cmd = 0.5;
    _reset_hover_filter = true;
    _hover_thrust_convergence_time = 2.5; // Time constant for hover thrust learning rate convergence (seconds)
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

void PositionControl::update(double dt) {
    if (dt <= 0.001) return;

    _positionControl();
    _velocityControl(dt);
    _accelerationControl();

    if (_takeoff_primed && _pos_sp.z() > 0.5) {
    _is_taking_off = true;     // Start the open-loop ramp
    _takeoff_primed = false;   // CONSUME the authorization. Only way to re-prime is through a reset on the ground
    }
    // TAKEOFF OVERRIDE
    if (_is_taking_off) {
        // 1. Force the drone perfectly level so it physically CANNOT skid.
        // We override the PID's attitude calculation with a flat quaternion (no tilt).
        // Only allow Yaw to pass through.
        Eigen::Vector3d y_C(-std::sin(_yaw_sp), std::cos(_yaw_sp), 0.0);
        Eigen::Vector3d body_x = y_C.cross(Eigen::Vector3d(0, 0, 1)); // Z is perfectly up
        body_x.normalize();
        Eigen::Vector3d body_y = Eigen::Vector3d(0, 0, 1).cross(body_x);
        
        Eigen::Matrix3d flat_rot;
        flat_rot.col(0) = body_x;
        flat_rot.col(1) = body_y;
        flat_rot.col(2) = Eigen::Vector3d(0, 0, 1);
        _attitude_sp = Eigen::Quaterniond(flat_rot);

        // 2. Open-Loop Thrust Ramp
        if(_takeoff_phase == 1 && _vel.z() < 0.6) { // Phase 1, increase throttle until 1 m/s^2 acceleration is detected
            _takeoff_ramp_thrust += (0.3 * dt); 
        }else{
            _takeoff_phase = 2;
            if(_vel.z() > 0.6){
                _takeoff_ramp_thrust -= (0.1 * dt);
            }
        }

        _takeoff_ramp_thrust = std::min(_takeoff_ramp_thrust, 0.4);

        // Override the PID's thrust command
        _thrust_sp = _takeoff_ramp_thrust;

        // 3. Freeze the PID Integrators
        // Keep the I-terms at zero so they don't wind up while we are overriding them.
        _vel_int.setZero();

        // 4. Check for Liftoff (Handoff to PID)
        if (_pos.z() > 1.0) {
            _is_taking_off = false; // We have left the ground
            _takeoff_phase = 1;
            _takeoff_ramp_thrust = 0.15; // Reset for next time
            
            // Seamlessly initialize the hover thrust filter to exactly what 
            // the ramp used to get us off the ground.
            _filtered_thrust_cmd = _thrust_sp; 
        }
    }

    // Finally, run the hover thrust estimator
    _updateHoverThrust(dt);
}

void PositionControl::reset() {
    _vel_int.setZero();    
    _pos_sp.setZero(); 
    _vel_sp.setZero();
    _acc_sp.setZero();
    _first_update = true;

    // Tell the estimator a new flight regime just started
    _reset_hover_filter = true;
    _takeoff_ramp_thrust = 0.15;
    _takeoff_phase = 1;
    _is_taking_off = false;
    
    // Evaluate our physical state at the exact moment of reset
    if (_pos.z() < 0.2 && std::abs(_vel.z()) < 0.2) {
        _in_air = false;
        _takeoff_primed = true; // We are on the ground. Authorized for ONE takeoff.
    } else {
        _in_air = true;
        _takeoff_primed = false; // We switched modes mid-air. DO NOT take off.
    }
}

// Position Loop -> Outputs Velocity Setpoint
void PositionControl::_positionControl() {
    // 1. Calculate Error (Target - Current)
    Eigen::Vector3d error = _pos_sp - _pos;
    

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

// Velocity Loop -> Outputs Acceleration Setpoint
void PositionControl::_velocityControl(double dt) {
    Eigen::Vector3d vel_error = _vel_sp - _vel;

    // PREVENT DERIVATIVE KICK
    // If this is the very first loop, initialize the previous error 
    // so the derivative doesn't see a massive artificial spike.
    if (_first_update) {
        _prev_vel_error = vel_error;
        _learning_rate = dt/_hover_thrust_convergence_time;
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

// Acceleration -> Attitude & Thrust
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


void PositionControl::_updateHoverThrust(double dt) {
    // Handle initialization transients (e.g., mid-air mode switches)
    // to prevent the PT1 filter from spooling up from an invalid state.
    if (_reset_hover_filter) {
        _filtered_thrust_cmd = _thrust_sp;
        _reset_hover_filter = false;
    }

    // Suspend estimation when motors are idling or disarmed.
    // Lock the filter state to the command to prevent divergence.
    if (_thrust_sp < 0.1) {
        _filtered_thrust_cmd = _thrust_sp;
        return;
    }

    // New in-air detection logic
    if (!_in_air) {
        // We consider the drone to be flying if it has climbed 0.5 meters
        // OR if it is actively moving upwards faster than 0.2 m/s.
        if (_pos.z() > 0.2 || _vel.z() > 0.2) {
            _in_air = true;
        } else {
            // We are still on the ground. Keep the filter pre-charged to the 
            // current thrust command so there is no math glitch when we do take off.
            _filtered_thrust_cmd = _thrust_sp;
            return; // DO NOT RUN THE ESTIMATOR MATH
        }
    }

    // Optional Safety: If we land or fall out of the sky, turn the flag off
    if (_pos.z() < 0.2 && _vel.z() < 0.1 && _thrust_sp < 0.2) {
        _in_air = false;
    }

    // Apply first-order low-pass filter to the commanded thrust.
    // This models physical actuator phase lag (propeller spool-up time)
    // to align the software control signal with measured IMU dynamics.
    constexpr double tau = 0.05; // System motor time constant [s]
    const double alpha = dt / (tau + dt);
    
    _filtered_thrust_cmd += alpha * (_thrust_sp - _filtered_thrust_cmd);

    // Back-calculate theoretical hover thrust using idealized physics:
    // a_z = g * (T / T_hover - 1)  =>  T_hover = T * g / (a_z + g)
    // Note: _acc.z() already accounts for gravity (0.0 = static hover).
    const double measured_az = _acc.z();
    double inst_hover_thrust = (_filtered_thrust_cmd * 9.81) / (measured_az + 9.81);

    // Constrain instantaneous estimate to physically plausible boundaries
    // to prevent divergence during external physical disturbances.
    inst_hover_thrust = std::clamp(inst_hover_thrust, 0.2, 0.5);

    // Dynamic measurement covariance scaling.
    // Degrade trust in the IMU measurements proportionally to vertical speed
    // to account for unmodeled aerodynamic drag and prop-wash transients.
    const double current_speed = std::abs(_vel.z());
    double trust_factor = 1.0;
    
    if (current_speed > 0.5) {
        trust_factor = 0.5 / current_speed;
    }

    // Update global estimate via Exponential Moving Average (EMA).
    const double effective_learning_rate = _learning_rate * trust_factor;
    
    _hover_thrust = (_hover_thrust * (1.0 - effective_learning_rate)) + 
                    (inst_hover_thrust * effective_learning_rate);
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