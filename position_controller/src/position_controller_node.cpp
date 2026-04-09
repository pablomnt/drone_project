// TO RUN:
// Regular mode (uses VIO odometry): ros2 run position_controller position_controller_node
// Simulation mode (uses px4's odometry): ros2 run position_controller position_controller_node --ros-args -p USE_SIM_MODE:=true

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
        this->declare_parameter<int>("TRAJECTORY_SELECTOR", 0);
        
        // NEW: Simulation Mode Parameter (Default: false -> Uses VIO)
        this->declare_parameter<bool>("USE_SIM_MODE", false);
        use_sim_mode_ = this->get_parameter("USE_SIM_MODE").as_bool();

        if (use_sim_mode_) {
            RCLCPP_WARN(this->get_logger(), "SIMULATION MODE ACTIVE: Using PX4 Odometry for state estimation.");
        } else {
            RCLCPP_INFO(this->get_logger(), "NORMAL MODE ACTIVE: Using VIO (OKVIS2) for state estimation.");
        }

        // Update Library with these params
        updateParams();

        // 2. QoS Profiles (Must match PX4's Best Effort)
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        // 3. Subscribers
        sub_joy = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&PositionControllerNode::joyCallback, this, std::placeholders::_1));

        sub_odom_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos, 
            std::bind(&PositionControllerNode::px4OdomCallback, this, std::placeholders::_1));

        sub_setpoint_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/position_controller/setpoint", 10,
            std::bind(&PositionControllerNode::setpointCallback, this, std::placeholders::_1));

        sub_status_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status_v1", qos,
            std::bind(&PositionControllerNode::statusCallback, this, std::placeholders::_1));

        sub_VIO_odometry = this->create_subscription<nav_msgs::msg::Odometry>(
            "/okvis/okvis_odometry", 10,
            std::bind(&PositionControllerNode::vioOdomCallback, this, std::placeholders::_1));

        // 4. Publishers
        pub_attitude_ = this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
            "/fmu/in/vehicle_attitude_setpoint", 10);

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
    bool use_sim_mode_ = false; // Mode flag
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
        Eigen::Vector3d pos_p(
            this->get_parameter("MPC_XY_P").as_double(),
            this->get_parameter("MPC_XY_P").as_double(),
            this->get_parameter("MPC_Z_P").as_double()
        );
        controller_.setPositionGains(pos_p);

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
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1); 
    }

    void disarmCommand(){
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0); 
    }

    void engageOffboardMode(){
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
        vehicle_status_ = *msg;
    }

    void px4OdomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        Eigen::Vector3d pos_enu(msg->position[1], msg->position[0], -msg->position[2]);
        Eigen::Vector3d vel_enu(msg->velocity[1], msg->velocity[0], -msg->velocity[2]);

        use_sim_mode_ = this->get_parameter("USE_SIM_MODE").as_bool();

        auto q = msg->q; 
        double siny_cosp = 2.0 * (q[0] * q[3] + q[1] * q[2]);
        double cosy_cosp = 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]);
        double yaw_ned = std::atan2(siny_cosp, cosy_cosp);
        double yaw_enu = -yaw_ned + M_PI_2;

        current_px4_yaw_enu_ = yaw_enu;
        has_px4_odom_ = true;

        // If in simulation mode, feed PX4 data directly into the controller
        if (use_sim_mode_) {
            controller_.setState(pos_enu, vel_enu, yaw_enu);
            RCLCPP_INFO(this->get_logger(),
                "PX4 ENU Pos [m]: (%.2f, %.2f, %.2f) | Vel [m/s]: (%.2f, %.2f, %.2f) | Yaw [deg]: %.2f",
                pos_enu.x(), pos_enu.y(), pos_enu.z(),
                vel_enu.x(), vel_enu.y(), vel_enu.z(),
                yaw_enu * 180.0 / M_PI
            );
        }


    }

    void vioOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {

        use_sim_mode_ = this->get_parameter("USE_SIM_MODE").as_bool();
        // 1. Position is World Frame (ENU)
        Eigen::Vector3d pos_enu(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        
        // 2. Velocity is Body Frame. We must rotate it!
        Eigen::Vector3d vel_body(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
        auto q_msg = msg->pose.pose.orientation;
        Eigen::Quaterniond q_sensor_to_world(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
        
        // Multiply the rotation matrix by the body velocity to get World Velocity
        Eigen::Vector3d vel_enu = q_sensor_to_world.toRotationMatrix() * vel_body;

        // 3. Extract Yaw and apply the 90-degree OKVIS2 offset
        double siny_cosp = 2.0 * (q_msg.w * q_msg.z + q_msg.x * q_msg.y);
        double cosy_cosp = 1.0 - 2.0 * (q_msg.y * q_msg.y + q_msg.z * q_msg.z);
        double yaw_enu = std::atan2(siny_cosp, cosy_cosp) + M_PI_2;

        // Wrap the angle between -PI and PI so it doesn't jump out of bounds
        yaw_enu = std::atan2(std::sin(yaw_enu), std::cos(yaw_enu));

        current_vio_yaw_enu_ = yaw_enu;
        has_vio_odom_ = true;

        // If in normal mode, feed VIO data into the controller
        if (!use_sim_mode_) {
            controller_.setState(pos_enu, vel_enu, yaw_enu);
            RCLCPP_INFO(this->get_logger(),
                "VIO ENU Pos [m]: (%.2f, %.2f, %.2f) | Vel [m/s]: (%.2f, %.2f, %.2f) | Yaw [deg]: %.2f",
                pos_enu.x(), pos_enu.y(), pos_enu.z(),
                vel_enu.x(), vel_enu.y(), vel_enu.z(),
                yaw_enu * 180.0 / M_PI
            );
        }
    }

    void setpointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        Eigen::Vector3d pos_sp(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        
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
        px4_msgs::msg::OffboardControlMode hb_msg;
        hb_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        hb_msg.position = false;
        hb_msg.velocity = false;
        hb_msg.acceleration = false;
        hb_msg.attitude = true; 
        hb_msg.body_rate = false;
        pub_offboard_mode_->publish(hb_msg);

        // Check dependencies based on the current mode
        if (use_sim_mode_) {
            if (!has_px4_odom_) return; // Only need PX4 in sim
        } else {
            if (!has_px4_odom_ || !has_vio_odom_) return; // Need both for VIO correction
        }

        bool offboard_active = (vehicle_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);
        if (!offboard_active) return;

        

        // Get the current trajectory selection and set the setpoint
        int trajectory_selector = this->get_parameter("TRAJECTORY_SELECTOR").as_int();
        Eigen::Vector3d pos_sp;

        if (trajectory_selector == 0) {
            // Trajectory 0: Hover at origin area
            pos_sp = Eigen::Vector3d(0.0, 0.0, 0.2);
        } else if (trajectory_selector == 1) {
            // Trajectory 1: Square flight pattern
            pos_sp = Eigen::Vector3d(1.0, 1.0, 0.5);
        } else if (trajectory_selector == 2) {
            // Trajectory 2: Higher altitude hover
            pos_sp = Eigen::Vector3d(0.0, 0.0, 1.0);
        } else if (trajectory_selector == 3) {
            // Trajectory 3: 
            pos_sp = Eigen::Vector3d(-1.0, -1.0, 1.0);
        } else {
            // Default to trajectory 0
            pos_sp = Eigen::Vector3d(0.0, 0.0, 0.2);
        }

        has_setpoint_ = true; // Ensure the controller knows we have a setpoint to track

        if(!has_setpoint_) return;

        double yaw_sp = 0.0;
        controller_.setSetpoint(pos_sp, yaw_sp);

        controller_.update(0.02); 

        Eigen::Matrix3d R_enu_vio = controller_.getAttitudeSetpoint().toRotationMatrix();
        double thrust = controller_.getThrustSetpoint();

        // ---------------------------------------------------------
        // HEADING DRIFT CORRECTION (Pure ENU)
        // ---------------------------------------------------------
        double yaw_drift_enu = 0.0;
        
        // Only calculate and apply drift if we are NOT in sim mode
        if (!use_sim_mode_) {
            yaw_drift_enu = current_px4_yaw_enu_ - current_vio_yaw_enu_;
        }

        Eigen::Matrix3d R_yaw_correction;
        R_yaw_correction = Eigen::AngleAxisd(yaw_drift_enu, Eigen::Vector3d::UnitZ());

        Eigen::Matrix3d R_enu_aligned = R_yaw_correction * R_enu_vio;

        // ---------------------------------------------------------
        // CONVERT FROM ALIGNED ENU TO PX4's NED/FRD
        // ---------------------------------------------------------
        Eigen::Matrix3d R_world_enu_to_ned;
        R_world_enu_to_ned << 0,  1,  0,
                              1,  0,  0,
                              0,  0, -1;

        Eigen::Matrix3d R_body_flu_to_frd;
        R_body_flu_to_frd << 1,  0,  0,
                             0, -1,  0,
                             0,  0, -1;

        Eigen::Matrix3d R_ned = R_world_enu_to_ned * R_enu_aligned * R_body_flu_to_frd;
        
        Eigen::Quaterniond q_ned(R_ned);
        q_ned.normalize(); 
        
        // ---------------------------------------------------------
        // PUBLISH TO PX4
        // ---------------------------------------------------------
        px4_msgs::msg::VehicleAttitudeSetpoint att_msg;
        att_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        
        att_msg.q_d[0] = q_ned.w();
        att_msg.q_d[1] = q_ned.x();
        att_msg.q_d[2] = q_ned.y();
        att_msg.q_d[3] = q_ned.z();
        
        //att_msg.q_d[0] = 1.0;
        //att_msg.q_d[1] = 0.0;
        //att_msg.q_d[2] = 0.0;
        //att_msg.q_d[3] = 0.0;
        
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