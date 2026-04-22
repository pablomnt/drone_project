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
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "px4_msgs/msg/sensor_combined.hpp"
#include <rcl_interfaces/msg/set_parameters_result.hpp>

// Our Library
#include "position_controller/PositionControl.hpp"

using namespace std::chrono_literals;

class PositionControllerNode : public rclcpp::Node {
public:
    PositionControllerNode() : Node("position_controller_node") {
        
        // Note: MPC stands for Multi-copter Position Controller, not Model Predictive Control in this context. 
        // These parameters directly influence the PID controller's behavior in the PositionControl library. 

        // Horizontal (XY) - Keep close to PX4 defaults, VIO handles XY well
        this->declare_parameter("MPC_XY_P", 0.95);      // Position P (Standard)
        this->declare_parameter("MPC_XY_VEL_P", 1.8);   // Velocity P (Standard)
        this->declare_parameter("MPC_XY_VEL_I", 0.4);   // Velocity I (Standard)
        this->declare_parameter("MPC_XY_VEL_D", 0.2);   // Velocity D (Standard damping)

        // Vertical (Z) - Soften the P-gain and add D-gain to absorb camera noise
        this->declare_parameter("MPC_Z_P", 1.0);        // Position P (Standard)
        this->declare_parameter("MPC_Z_VEL_P", 0.3);    // Velocity P (Lowered from 4.0 to stop noise spikes)
        this->declare_parameter("MPC_Z_VEL_I", 0.0);    // Velocity I (Lowered from 2.0 to prevent deep wind-up during wobble)
        this->declare_parameter("MPC_Z_VEL_D", 0.0);    // Velocity D (Raised from 0.0 to act as a shock absorber)
        
        this->declare_parameter("MPC_HOVER_THRUST", 0.65);
        this->declare_parameter<double>("MPC_THRUST_LEARNING_RATE", 0.0005);
        this->declare_parameter<int>("TRAJECTORY_SELECTOR", 0);

        // Bring back the custom position parameter!
        this->declare_parameter<std::vector<double>>("POS_SP", {0.0, 0.0, 0.5});
        
        // NEW: Simulation Mode Parameter (Default: false -> Uses VIO)
        this->declare_parameter<bool>("USE_SIM_MODE", false);
        use_sim_mode_ = this->get_parameter("USE_SIM_MODE").as_bool();

        if (use_sim_mode_) {
            RCLCPP_WARN(this->get_logger(), "SIMULATION MODE ACTIVE: Using PX4 Odometry for state estimation.");
        } else {
            RCLCPP_INFO(this->get_logger(), "NORMAL MODE ACTIVE: Using VIO (OKVIS2) for state estimation.");
        }

        // Initialize state machine timer
        step_start_time_ = this->get_clock()->now();

        // Initialize Library with these params
        updatePIDGains();
        controller_.setHoverThrust(this->get_parameter("MPC_HOVER_THRUST").as_double());

        // Register the callback
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&PositionControllerNode::parametersCallback, this, std::placeholders::_1));

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

        sub_sensor_combined = this->create_subscription<px4_msgs::msg::SensorCombined>(
            "/fmu/out/sensor_combined", qos, 
            std::bind(&PositionControllerNode::sensorCombinedCallback, this, std::placeholders::_1));

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

        pub_vehicle_command_mode_executor_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command_mode_executor", 10);

        pub_target_pos_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/debug/target_position", 10);
        pub_target_vel_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/debug/target_velocity", 10);
        pub_target_acc_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/debug/target_acceleration", 10);
        pub_pid_p_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/debug/pid_p_term", 10);
        pub_pid_i_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/debug/pid_i_term", 10);
        pub_pid_d_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/debug/pid_d_term", 10);
        pub_thrust_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/debug/thrust", 10);

        // 5. Main Loop (50Hz)rclcpp::ParameterTypeException'
        timer_ = this->create_wall_timer(20ms, std::bind(&PositionControllerNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Position Controller Node Started (C++)");
    }

private:
    PositionControl controller_;
    
    // --- PARAMETER CALLBACK VARIABLES ---
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    bool update_pid_gains_ = false;
    bool override_hover_thrust_ = false;
    double new_hover_thrust_ = 0.0;

    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        for (const auto &param : parameters) {
            if (param.get_name() == "MPC_HOVER_THRUST") {
                // Only flag the hover thrust if the user specifically touched this parameter
                override_hover_thrust_ = true;
                new_hover_thrust_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Manual Override: Hover thrust queued to %.2f", new_hover_thrust_);
            } 
            else if (param.get_name() == "POS_SP" || param.get_name() == "TRAJECTORY_SELECTOR" || param.get_name() == "USE_SIM_MODE") {
                // Do nothing. The control loop reads these live anyway.
                RCLCPP_INFO(this->get_logger(), "Setpoint/Mode changed to %s", param.value_to_string().c_str());
            }
            else {
                // If it wasn't thrust or a setpoint, it must be a PID gain.
                update_pid_gains_ = true;
            }
        }

        return result;
    }

    // Publishers & Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_setpoint_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr sub_status_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_VIO_odometry;
    rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr sub_sensor_combined;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr pub_attitude_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr pub_offboard_mode_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_vehicle_command_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_vehicle_command_mode_executor_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_target_pos_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_target_vel_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_target_acc_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_pid_p_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_pid_i_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_pid_d_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_thrust_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Data Storage
    bool use_sim_mode_ = false; // Mode flag
    bool was_armed_ = false;
    bool was_offboard_ = false;
    bool has_px4_odom_ = false;
    bool has_sensor_combined_ = false;
    bool has_vio_odom_ = false;
    bool has_setpoint_ = false;
    std::vector<int> last_buttons_ = std::vector<int>(12, 0);
    std::vector<float> last_axes_ = std::vector<float>(8, 0.0);
    double current_px4_yaw_enu_ = 0.0;
    double current_vio_yaw_enu_ = 0.0;
    sensor_msgs::msg::Joy joy_input_;
    px4_msgs::msg::VehicleStatus vehicle_status_;

    // Sequence Trackers
    int last_trajectory_selector_ = -1;
    int trajectory_step_ = 0;
    rclcpp::Time step_start_time_;

    // --------------------------------------------------------------------------------
    // Callbacks
    // --------------------------------------------------------------------------------

    void updatePIDGains() {
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

    void publishVehicleCommandModeExecutor(uint16_t command, float param1=0, float param2=0, float param3=0, float param4=0, float param5=0, float param6=0, float param7=0) {
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
        pub_vehicle_command_mode_executor_->publish(cmd_msg);
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

    void landCommand() {
        // PX4 Auto-Land Command
        // Command: VEHICLE_CMD_NAV_LAND (Command ID 21)
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND); 
        RCLCPP_INFO(this->get_logger(), "Auto-Land Command Sent! PX4 taking over descent.");
    }

    void takeoffCommand(float altitude) {
        // PX4 Takeoff Command
        // Command: VEHICLE_CMD_NAV_TAKEOFF (Command ID 22)
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, altitude);
        publishVehicleCommandModeExecutor(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, altitude); 
        RCLCPP_INFO(this->get_logger(), "Takeoff Command Sent! Ascending to %.2f meters.", altitude);
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
        if(joy_input_.buttons[8]==1 && last_buttons_[8] == 0){
            RCLCPP_INFO(this->get_logger(), "Land command sent");
            landCommand();
        }
        if(joy_input_.axes[7] == 1 && last_buttons_[7] == 0){
            RCLCPP_INFO(this->get_logger(), "Takeoff command sent");
            takeoffCommand(1.0); // Takeoff to 1.0 meters
        }
        last_buttons_ = joy_input_.buttons;
        last_axes_ = joy_input_.axes;
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
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
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
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "VIO ENU Pos [m]: (%.2f, %.2f, %.2f) | Vel [m/s]: (%.2f, %.2f, %.2f) | Yaw [deg]: %.2f",
                pos_enu.x(), pos_enu.y(), pos_enu.z(),
                vel_enu.x(), vel_enu.y(), vel_enu.z(),
                yaw_enu * 180.0 / M_PI
            );
        }
    }

    void sensorCombinedCallback(const px4_msgs::msg::SensorCombined::SharedPtr msg) {
        Eigen::Vector3d acc_enu(msg->accelerometer_m_s2[1], msg->accelerometer_m_s2[0], -msg->accelerometer_m_s2[2]);
        acc_enu.z() -= 9.81; // Remove gravity to get actual acceleration
        has_sensor_combined_ = true;
        controller_.setCurrentAcceleration(acc_enu);
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
        // --- PROCESS PARAMETER CHANGES SAFELY ---
        if (update_pid_gains_) {
            updatePIDGains();
            update_pid_gains_ = false;
            RCLCPP_INFO(this->get_logger(), "PID Gains updated from parameter server.");
        }
        
        if (override_hover_thrust_) {
            controller_.setHoverThrust(new_hover_thrust_);
            override_hover_thrust_ = false;
            // The estimator will now pick up from this new value!
        }

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
            if (!has_px4_odom_ || !has_sensor_combined_ || !has_vio_odom_) return; // Need all for VIO flight
        }

        // 1. Check current states
        bool is_armed = (vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
        bool offboard_active = (vehicle_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);

        // 2. Edge Detection: Did we just disarm OR leave Offboard mode?
        if ((was_armed_ && !is_armed) || (was_offboard_ && !offboard_active)) {
            RCLCPP_INFO(this->get_logger(), "Interruption detected (Disarmed or Left Offboard). Resetting PID and State.");
            controller_.reset();
            has_px4_odom_ = false;
            has_sensor_combined_ = false;
            has_vio_odom_ = false;
            
            // Reset the state machine
            trajectory_step_ = 0;
            last_trajectory_selector_ = -1; 
        }
        
        // Save states for the next loop
        was_armed_ = is_armed; 
        was_offboard_ = offboard_active;

        // 3. Block the controller from running unless BOTH are true
        if (!is_armed || !offboard_active) {
            step_start_time_ = this->get_clock()->now(); 
            return;
        }

        // Get the current trajectory selection and set the setpoint
        int trajectory_selector = this->get_parameter("TRAJECTORY_SELECTOR").as_int();
        
        // Reset the state machine if the user switches sequences
        if (trajectory_selector != last_trajectory_selector_) {
            trajectory_step_ = 0;
            step_start_time_ = this->get_clock()->now();
            last_trajectory_selector_ = trajectory_selector;
        }

        double elapsed_time = (this->get_clock()->now() - step_start_time_).seconds();
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
        } else if (trajectory_selector == 4) {
            // Trajectory 4: Custom setpoint from ROS parameter
            std::vector<double> pos_sp_vec = this->get_parameter("POS_SP").as_double_array();
            pos_sp = Eigen::Vector3d(pos_sp_vec[0], pos_sp_vec[1], pos_sp_vec[2]);
        } else if (trajectory_selector == 5) {
            // Trajectory 5: Sequence with Land
            std::vector<Eigen::Vector3d> points = {
                {0.0, 0.0, 0.3},
                {0.0, 0.0, 0.5},
                {0.0, 1.0, 0.5},
                {0.0, 0.0, 0.5}
            };
            
            if (trajectory_step_ < (int)points.size()) {
                pos_sp = points[trajectory_step_];
                if (elapsed_time >= 5.0) {
                    trajectory_step_++;
                    step_start_time_ = this->get_clock()->now();
                }
            } else {
                // Sequence finished: hover at last point for 3 seconds, then land
                pos_sp = points.back();
                if (elapsed_time >= 5.0) {
                    RCLCPP_INFO(this->get_logger(), "Trajectory 5 complete. Landing...");
                    landCommand();
                    this->set_parameter(rclcpp::Parameter("TRAJECTORY_SELECTOR", 0)); // Prevent spamming land
                    return;
                }
            }
        } else if (trajectory_selector == 6) {
            // Trajectory 6: Sequence with Land
            std::vector<Eigen::Vector3d> points = {
                {0.0, 0.0, 0.3},
                {0.0, 0.0, 0.5},
                {0.0, 1.0, 0.5},
                {0.5, 1.0, 0.5},
                {0.5, 0.0, 0.5},
                {0.0, 0.0, 0.5},
                {0.0, 0.0, 0.3}
            };
            
            if (trajectory_step_ < (int)points.size()) {
                pos_sp = points[trajectory_step_];
                if (elapsed_time >= 5.0) {
                    trajectory_step_++;
                    step_start_time_ = this->get_clock()->now();
                }
            } else {
                // Sequence finished: hover at last point for 3 seconds, then land
                pos_sp = points.back();
                if (elapsed_time >= 5.0) {
                    RCLCPP_INFO(this->get_logger(), "Trajectory 6 complete. Landing...");
                    landCommand();
                    this->set_parameter(rclcpp::Parameter("TRAJECTORY_SELECTOR", 0)); // Prevent spamming land
                    return;
                }
            }
        } else {
            // Default to trajectory 0
            pos_sp = Eigen::Vector3d(0.0, 0.0, 0.2);
        }

        has_setpoint_ = true; // Ensure the controller knows we have a setpoint to track

        if(!has_setpoint_) return;

        double yaw_sp = 3.1416/2;
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
        // DEBUG TELEMETRY PUBLISHERS
        // ---------------------------------------------------------
        auto now = this->get_clock()->now();

        geometry_msgs::msg::PointStamped pos_msg;
        pos_msg.header.stamp = now;
        pos_msg.point.x = controller_.getPositionSetpoint().x();
        pos_msg.point.y = controller_.getPositionSetpoint().y();
        pos_msg.point.z = controller_.getPositionSetpoint().z();
        pub_target_pos_->publish(pos_msg);

        auto publish_vector3 = [&](rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub, Eigen::Vector3d vec) {
            geometry_msgs::msg::Vector3Stamped msg;
            msg.header.stamp = now;
            msg.vector.x = vec.x();
            msg.vector.y = vec.y();
            msg.vector.z = vec.z();
            pub->publish(msg);
        };

        publish_vector3(pub_target_vel_, controller_.getVelocitySetpoint());
        publish_vector3(pub_pid_p_, controller_.getVelocityPTerm());
        publish_vector3(pub_pid_i_, controller_.getVelocityITerm());
        publish_vector3(pub_pid_d_, controller_.getVelocityDTerm());

        std_msgs::msg::Float64MultiArray thrust_msg;
        thrust_msg.data.push_back(controller_.getHoverThrust());
        thrust_msg.data.push_back(thrust);
        pub_thrust_->publish(thrust_msg);

        geometry_msgs::msg::Vector3Stamped acc_msg;
        acc_msg.header.stamp = this->get_clock()->now();
        
        // Ask the controller for the acceleration!
        Eigen::Vector3d current_acc_sp = controller_.getAccelerationSetpoint();
        
        acc_msg.vector.x = current_acc_sp.x();
        acc_msg.vector.y = current_acc_sp.y();
        acc_msg.vector.z = current_acc_sp.z();
        pub_target_acc_->publish(acc_msg);
        
        // ---------------------------------------------------------
        // PUBLISH TO PX4
        // ---------------------------------------------------------
        px4_msgs::msg::VehicleAttitudeSetpoint att_msg;
        att_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        
        att_msg.q_d[0] = q_ned.w();
        att_msg.q_d[1] = q_ned.x();
        att_msg.q_d[2] = q_ned.y();
        att_msg.q_d[3] = q_ned.z();
        
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