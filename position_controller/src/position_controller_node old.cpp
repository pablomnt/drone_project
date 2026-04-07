#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <chrono>
#include <cmath>
#include <vector>

// Dependencies
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_attitude_setpoint.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "nav_msgs/msg/odometry.hpp"

// Our Library
#include "position_controller/PositionControl.hpp"

using namespace std::chrono_literals;

class PositionControllerNode : public rclcpp::Node {
public:
    PositionControllerNode() : Node("position_controller_node") {
        
        // 1. Declare Parameters (So you can tune PID without recompiling)
        this->declare_parameter("MPC_XY_P", 0.95);
        this->declare_parameter("MPC_Z_P", 1.0);
        this->declare_parameter("MPC_XY_VEL_P", 1.8);
        this->declare_parameter("MPC_Z_VEL_P", 4.0);
        this->declare_parameter("MPC_XY_VEL_I", 0.4);
        this->declare_parameter("MPC_Z_VEL_I", 2.0);
        this->declare_parameter("MPC_XY_VEL_D", 0.2);
        this->declare_parameter("MPC_Z_VEL_D", 0.0);
        this->declare_parameter("MPC_HOVER_THRUST", 0.7);
        this->declare_parameter<std::vector<double>>(
            "POS_SP", {1.0, 2.0, 3.0}
        );


        // Update Library with these params
        updateParams();

        // 2. QoS Profiles (Must match PX4's Best Effort)
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        // 3. Subscribers
        // Receive Position/Velocity from PX4 (NED frame)

        sub_joy = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&PositionControllerNode::joyCallback, this, std::placeholders::_1));

        sub_odom_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos, 
            std::bind(&PositionControllerNode::px4OdomCallback, this, std::placeholders::_1));

        // Receive Desired Setpoint from your high-level planner (ENU frame)
        sub_setpoint_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/position_controller/setpoint", 10,
            std::bind(&PositionControllerNode::setpointCallback, this, std::placeholders::_1));

        // Receive Vehicle Status (for arming state, etc.)
        sub_status_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status_v1", qos,
            std::bind(&PositionControllerNode::statusCallback, this, std::placeholders::_1));

        sub_VIO_odometry = this->create_subscription<nav_msgs::msg::Odometry>(
            "/okvis/okvis_odometry", 10,
            std::bind(&PositionControllerNode::vioOdomCallback, this, std::placeholders::_1));

        // 4. Publishers
        // Send Attitude + Thrust to PX4
        pub_attitude_ = this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
            "/fmu/in/vehicle_attitude_setpoint", 10);

        // Send Offboard Heartbeat
        pub_offboard_mode_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);
        
        pub_vehicle_command_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

        // 5. Main Loop (50Hz)
        timer_ = this->create_wall_timer(20ms, std::bind(&PositionControllerNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Position Controller Node Started (C++)");
    }

private:
    PositionControl controller_;
    
    // Publishers & Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_setpoint_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr sub_status_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_VIO_odometry;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr pub_attitude_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr pub_offboard_mode_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_vehicle_command_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Data Storage
    bool has_px4_odom_ = false;
    bool has_vio_odom_ = false;
    bool has_setpoint_ = false;
    std::vector<int> last_buttons_ = std::vector<int>(12, 0);
    double current_px4_yaw_enu_ = 0.0;
    double current_vio_yaw_enu_ = 0.0;
    sensor_msgs::msg::Joy joy_input_;
    px4_msgs::msg::VehicleStatus vehicle_status_;

    // --------------------------------------------------------------------------------
    // Callbacks
    // --------------------------------------------------------------------------------

    void updateParams() {
        // Pull gains from ROS parameters into the C++ Library
        Eigen::Vector3d pos_p(
            this->get_parameter("MPC_XY_P").as_double(),
            this->get_parameter("MPC_XY_P").as_double(),
            this->get_parameter("MPC_Z_P").as_double()
        );
        controller_.setPositionGains(pos_p);

        // Map XY/Z params to P, I, D vectors
        double p_xy = this->get_parameter("MPC_XY_VEL_P").as_double();
        double p_z = this->get_parameter("MPC_Z_VEL_P").as_double();
        Eigen::Vector3d vel_p(p_xy, p_xy, p_z);
        
        double i_xy = this->get_parameter("MPC_XY_VEL_I").as_double();
        double i_z = this->get_parameter("MPC_Z_VEL_I").as_double();
        Eigen::Vector3d vel_i(i_xy, i_xy, i_z);

        double d_xy = this->get_parameter("MPC_XY_VEL_D").as_double();
        double d_z = this->get_parameter("MPC_Z_VEL_D").as_double();
        Eigen::Vector3d vel_d(d_xy, d_xy, d_z);

        controller_.setVelocityGains(vel_p, vel_i, vel_d);
        controller_.setHoverThrust(this->get_parameter("MPC_HOVER_THRUST").as_double());
    }

    void publishVehicleCommand(uint16_t command, float param1=0, float param2=0, float param3=0, float param4=0, float param5=0, float param6=0, float param7=0) {
        px4_msgs::msg::VehicleCommand cmd_msg;
        cmd_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        cmd_msg.command = command;
        cmd_msg.param1 = param1;
        cmd_msg.param2 = param2;
        cmd_msg.param3 = param3;
        cmd_msg.param4 = param4;
        cmd_msg.param5 = param5;
        cmd_msg.param6 = param6;
        cmd_msg.param7 = param7;
        pub_vehicle_command_->publish(cmd_msg);
    }

    void armCommand(){
        // PX4 Arming Command
        // Command: VEHICLE_CMD_COMPONENT_ARM_DISARM
        // Param1: 1 to arm, 0 to disarm
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1); 
    }

    void disarmCommand(){
        // PX4 Disarming Command
        // Command: VEHICLE_CMD_COMPONENT_ARM_DISARM
        // Param1: 1 to arm, 0 to disarm
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0); 
    }

    void engageOffboardMode(){
        // PX4 Offboard Mode Command
        // Command: VEHICLE_CMD_DO_SET_MODE
        // Param1: 1 to set custom mode, 0 to set main mode
        // Param2: Custom mode (e.g., 6 for Offboard)
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6); 
    }

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        joy_input_ = *msg;        
        if(joy_input_.buttons[9]==1 && last_buttons_[9] == 0){
            if(vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED){
                RCLCPP_INFO(this->get_logger(), "Disarming Command Sent");
                disarmCommand();
            } else if (vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED){
                RCLCPP_INFO(this->get_logger(), "Arming Command Sent");
                armCommand();
            }
        }
        if(joy_input_.buttons[1] == 1 && last_buttons_[1] == 0){
            RCLCPP_INFO(this->get_logger(), "Offboard Mode Engage Command Sent");
            engageOffboardMode();
        }
        last_buttons_ = joy_input_.buttons;
    }

    void statusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg){
        // Store the latest vehicle status for use in control logic (e.g., arming state)
        vehicle_status_ = *msg;
    }

    void px4OdomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        // PX4 sends NED (North-East-Down).
        // Our Library uses ENU (East-North-Up).
        // Conversion:
        // ENU X (East)  = NED Y
        // ENU Y (North) = NED X
        // ENU Z (Up)    = -NED Z

        Eigen::Vector3d pos_enu(msg->position[1], msg->position[0], -msg->position[2]);
        Eigen::Vector3d vel_enu(msg->velocity[1], msg->velocity[0], -msg->velocity[2]);

        // Yaw conversion: 
        // PX4 Quaternion is NED (Body-FRD).
        // We calculate Yaw from the quaternion, then apply offset (-90 degrees) to align with ENU.
        // For simplicity, let's just use the raw Yaw from the msg if available, or compute:
        
        // Manual quaternion-to-yaw (NED frame)
        auto q = msg->q; 
        double siny_cosp = 2.0 * (q[0] * q[3] + q[1] * q[2]);
        double cosy_cosp = 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]);
        double yaw_ned = std::atan2(siny_cosp, cosy_cosp);

        // Convert NED Yaw (0 is North) to ENU Yaw (0 is East)
        // North is +90 deg (PI/2) in ENU. 
        // Formula: Yaw_ENU = -Yaw_NED + PI/2
        double yaw_enu = -yaw_ned + M_PI_2;

        // Store the current yaw for use in VIO correction logic
        current_px4_yaw_enu_ = yaw_enu;

        has_px4_odom_ = true;

        // Print everything in one log line
        RCLCPP_INFO(this->get_logger(),
        "PX4 ENU Pos [m]: (%.2f, %.2f, %.2f) | Vel [m/s]: (%.2f, %.2f, %.2f) | Yaw [deg]: %.2f",
        pos_enu.x(), pos_enu.y(), pos_enu.z(),
        vel_enu.x(), vel_enu.y(), vel_enu.z(),
        yaw_enu * 180.0 / 3.1416
        );
    }

    void vioOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received VIO Odometry: Position (%.2f, %.2f, %.2f)", 
            msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        
        Eigen::Vector3d pos_enu(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        Eigen::Vector3d vel_enu(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
        auto q = msg->pose.pose.orientation;

        // Extract Yaw from the VIO quaternion (Standard ROS ENU)
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        double yaw_enu = std::atan2(siny_cosp, cosy_cosp);

        // Save it to a class variable so the control loop can use it!
        current_vio_yaw_enu_ = yaw_enu;
        controller_.setState(pos_enu, vel_enu, yaw_enu);
        
        has_vio_odom_ = true;

        // Print all at once (position, velocity, yaw in degrees)
        RCLCPP_INFO(this->get_logger(),
        "                         VIO ENU Pos [m]: (%.2f, %.2f, %.2f) | Vel [m/s]: (%.2f, %.2f, %.2f) | Yaw [deg]: %.2f",
        pos_enu.x(), pos_enu.y(), pos_enu.z(),
        vel_enu.x(), vel_enu.y(), vel_enu.z(),
        yaw_enu * 180.0 / 3.1416
        );
    }

    void setpointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // Assume incoming setpoints are already in ENU (standard ROS)
        Eigen::Vector3d pos_sp(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        
        // Extract Yaw from orientation
        double qx = msg->pose.orientation.x;
        double qy = msg->pose.orientation.y;
        double qz = msg->pose.orientation.z;
        double qw = msg->pose.orientation.w;
        double siny_cosp = 2.0 * (qw * qz + qx * qy);
        double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        double yaw_sp = std::atan2(siny_cosp, cosy_cosp);
        
        controller_.setSetpoint(pos_sp, yaw_sp);
        has_setpoint_ = true;
    }

    void controlLoop() {
        // Publish Heartbeat (Required for Offboard Mode)
        px4_msgs::msg::OffboardControlMode hb_msg;
        hb_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        hb_msg.position = false;
        hb_msg.velocity = false;
        hb_msg.acceleration = false;
        hb_msg.attitude = true; // We are controlling attitude!
        hb_msg.body_rate = false;
        pub_offboard_mode_->publish(hb_msg);

        if (!has_px4_odom_ || !has_vio_odom_) return;

        // Only run the controller and publish setpoints if we're in Offboard mode (otherwise PX4 will react weirdly out of offboard mode)
        bool offboard_active = (vehicle_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);
        if (!offboard_active) return;

        // Get parameter as std::vector<double>
        std::vector<double> pos_sp_vec =
        this->get_parameter("POS_SP").as_double_array();    

        Eigen::Vector3d pos_sp(pos_sp_vec[0], pos_sp_vec[1], pos_sp_vec[2]);
        double yaw_sp = 0.0;
        controller_.setSetpoint(pos_sp, yaw_sp);
        has_setpoint_ = true; 

        if(!has_setpoint_) return;

        // Run Control Library
        controller_.update(0.02); // 50Hz = 0.02s

        // Get the initial attitude setpoint from the controller (VIO's ENU frame)
        Eigen::Matrix3d R_enu_vio = controller_.getAttitudeSetpoint().toRotationMatrix();
        double thrust = controller_.getThrustSetpoint();

        // ---------------------------------------------------------
        // HEADING DRIFT CORRECTION (Pure ENU)
        // ---------------------------------------------------------
        // How much is PX4 rotated relative to VIO?
        double yaw_drift_enu = current_px4_yaw_enu_ - current_vio_yaw_enu_;

        // Create a rotation matrix around the ENU Z-axis (Up)
        Eigen::Matrix3d R_yaw_correction;
        R_yaw_correction = Eigen::AngleAxisd(yaw_drift_enu, Eigen::Vector3d::UnitZ());

        // Apply the correction! Now the setpoint matches PX4's reality (in ENU)
        Eigen::Matrix3d R_enu_aligned = R_yaw_correction * R_enu_vio;

        // ---------------------------------------------------------
        // CONVERT FROM ALIGNED ENU TO PX4's NED/FRD
        // ---------------------------------------------------------
        // Transform 1: World Frame (ENU -> NED)
        Eigen::Matrix3d R_world_enu_to_ned;
        R_world_enu_to_ned << 0,  1,  0,
                              1,  0,  0,
                              0,  0, -1;

        // Transform 2: Body Frame (FLU -> FRD)
        Eigen::Matrix3d R_body_flu_to_frd;
        R_body_flu_to_frd << 1,  0,  0,
                             0, -1,  0,
                             0,  0, -1;

        // Apply transformations: R_ned = World * R_enu_aligned * Body
        Eigen::Matrix3d R_ned = R_world_enu_to_ned * R_enu_aligned * R_body_flu_to_frd;
        
        Eigen::Quaterniond q_ned(R_ned);
        q_ned.normalize(); // Ensure it remains a valid quaternion
        
        // ---------------------------------------------------------
        // PUBLISH TO PX4
        // ---------------------------------------------------------
        px4_msgs::msg::VehicleAttitudeSetpoint att_msg;
        att_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        att_msg.q_d[0] = q_ned.w();
        att_msg.q_d[1] = q_ned.x();
        att_msg.q_d[2] = q_ned.y();
        att_msg.q_d[3] = q_ned.z();
        
        // Z is Down in NED, so we apply negative thrust to push Up
        att_msg.thrust_body[0] = 0.0;
        att_msg.thrust_body[1] = 0.0;
        att_msg.thrust_body[2] = -thrust; 

        pub_attitude_->publish(att_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PositionControllerNode>());
    rclcpp::shutdown();
    return 0;
}