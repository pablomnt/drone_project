#pragma once

#include <Eigen/Dense>

class PositionControl {
public:
    PositionControl();
    
    // -- Configuration --
    // Sets P gains for Position Control (x, y, z)
    void setPositionGains(const Eigen::Vector3d& P);

    // Sets PID gains for Velocity Control (P, I, D)
    void setVelocityGains(const Eigen::Vector3d& P, const Eigen::Vector3d& I, const Eigen::Vector3d& D);

    // Sets limits (max speed in xy, max speed in z, max angle)
    // Matches PX4's setVelocityLimits and setTiltLimit
    void setConstraints(double vel_horizontal, double vel_up, double vel_down, double tilt_max_rad);

    // Sets the thrust needed to hover (0.0 to 1.0). Default is usually 0.5
    void setHoverThrust(double hover_thrust);

    void setThrustLearningRate(double learning_rate);


    // -- Main Loop Inputs --
    // Call this every time you get a new Odometry message
    void setState(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, double yaw);

    void setCurrentAcceleration(const Eigen::Vector3d& acc);

    // Call this to update where you want to go
    void setSetpoint(const Eigen::Vector3d& pos_sp, double yaw_sp);


    // -- The Loop --
    // Call this at a fixed rate (e.g., 50Hz). 'dt' is time since last call.
    void update(double dt);
    void reset();

    // -- Outputs --
    // Send these to your drone via MAVROS
    Eigen::Quaterniond getAttitudeSetpoint();
    double getThrustSetpoint();
    // Telemetry Getters
    Eigen::Vector3d getPositionSetpoint() { return _pos_sp; }
    Eigen::Vector3d getVelocitySetpoint() { return _vel_sp; }
    Eigen::Vector3d getAccelerationSetpoint() { return _acc_sp; }
    double getHoverThrust() { return _hover_thrust; }
    Eigen::Vector3d getVelocityPTerm() { return _vel_p_term; }
    Eigen::Vector3d getVelocityITerm() { return _vel_int; }
    Eigen::Vector3d getVelocityDTerm() { return _vel_d_term; }
    

private:
    // Internal functions (Logic from PX4 PositionControl.cpp)
    void _positionControl();
    void _velocityControl(double dt);
    void _accelerationControl();
    void _updateHoverThrust();


    // Helper to clamp horizontal velocity (Logic from PX4 ControlMath.cpp)
    Eigen::Vector2d _constrainXY(const Eigen::Vector2d& v0, double max);

    // -- Gains --
    Eigen::Vector3d _gain_pos_p; // Position P
    Eigen::Vector3d _gain_vel_p; // Velocity P
    Eigen::Vector3d _gain_vel_i; // Velocity I
    Eigen::Vector3d _gain_vel_d; // Velocity D

    // -- Limits --
    double _lim_vel_horz;
    double _lim_vel_up;
    double _lim_vel_down;
    double _lim_tilt;

    double _hover_thrust;
    double _learning_rate;

    // -- States (Where we are) --
    Eigen::Vector3d _pos;
    Eigen::Vector3d _vel;
    Eigen::Vector3d _acc;
    double _yaw;

    // -- Setpoints (Where we want to be) --
    Eigen::Vector3d _pos_sp;
    double _yaw_sp;

    // -- Internal Targets (Calculated by the controller) --
    Eigen::Vector3d _vel_sp;  // Target Velocity (Calculated by Position Loop)
    Eigen::Vector3d _acc_sp;  // Target Acceleration (Calculated by Velocity Loop)

    Eigen::Quaterniond _attitude_sp; // Final calculated orientation
    double _thrust_sp;               // Final calculated thrust (0 to 1)
    
    // -- Integrator State --
    Eigen::Vector3d _vel_int; // Stores the accumulated error for I-term
    // -- Derivator State --
    Eigen::Vector3d _prev_vel_error;
    bool _first_update = true;

    Eigen::Vector3d _vel_p_term;
    Eigen::Vector3d _vel_d_term;
};