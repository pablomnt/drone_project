#include <iostream>
#include "position_controller/PositionControl.hpp"

int main() {
    // 1. Instantiate the controller
    PositionControl controller;

    // 2. Set Gains (similar to PX4 defaults)
    Eigen::Vector3d pos_p(0.95, 0.95, 1.0);
    Eigen::Vector3d vel_p(1.8, 1.8, 4.0);
    Eigen::Vector3d vel_i(0.4, 0.4, 2.0);
    Eigen::Vector3d vel_d(0.2, 0.2, 0.0);

    controller.setPositionGains(pos_p);
    controller.setVelocityGains(vel_p, vel_i, vel_d);
    controller.setHoverThrust(0.5);

    // 3. Mock Drone State (We are at 0,0,0 and stationary)
    Eigen::Vector3d current_pos(0, 0, 0);
    Eigen::Vector3d current_vel(0, 0, 0);
    double current_yaw = 0.0;
    controller.setState(current_pos, current_vel, current_yaw);

    // 4. Set Setpoint (We want to go to x=1.0, y=0.0, z=0.0)
    // This should produce a pitch forward to generate velocity.
    Eigen::Vector3d target_pos(1.0, 0, 0);
    double target_yaw = 0.0;
    controller.setSetpoint(target_pos, target_yaw);

    // 5. Run Update (simulating 0.02 seconds passing)
    controller.update(0.02);

    // 6. Check Results
    Eigen::Quaterniond q = controller.getAttitudeSetpoint();
    double thrust = controller.getThrustSetpoint();

    // Print outputs
    std::cout << "Test Results:" << std::endl;
    std::cout << "Thrust (0-1): " << thrust << std::endl;
    std::cout << "Attitude Quat (w,x,y,z): " 
              << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << std::endl;

    // Simple verification
    if (thrust >= 0.0 && thrust <= 1.0) {
        std::cout << "SUCCESS: Math seems reasonable." << std::endl;
    } else {
        std::cout << "FAILURE: Thrust out of bounds!" << std::endl;
    }

    return 0;
}