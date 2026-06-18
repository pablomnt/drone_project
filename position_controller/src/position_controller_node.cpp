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
#include "position_controller/msg/controller_debug.hpp"

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
        this->declare_parameter("MPC_Z_VEL_P", 2.0);    // Velocity P (Lowered from 4.0 to stop noise spikes)
        this->declare_parameter("MPC_Z_VEL_I", 0.5);    // Velocity I (Lowered from 2.0 to prevent deep wind-up during wobble)
        this->declare_parameter("MPC_Z_VEL_D", 0.2);    // Velocity D (Raised from 0.0 to act as a shock absorber)
        
        this->declare_parameter("MPC_HOVER_THRUST", 0.35);
        this->declare_parameter<int>("TRAJECTORY_SELECTOR", 4);

        // Bring back the custom position parameter!
        this->declare_parameter<std::vector<double>>("POS_SP", {0.0, 0.0, 1.3});
        
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
            "/fmu/in/vehicle_attitude_setpoint_v1", 10);

        pub_offboard_mode_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);
        
        pub_vehicle_command_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

        pub_vehicle_command_mode_executor_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command_mode_executor", 10);

        pub_debug_ = this->create_publisher<position_controller::msg::ControllerDebug>("/debug/telemetry", 10);

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
    rclcpp::Publisher<position_controller::msg::ControllerDebug>::SharedPtr pub_debug_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Data Storage
    bool use_sim_mode_ = false; // Mode flag
    bool was_armed_ = false;
    bool controller_running = false;
    bool was_offboard_ = false;
    bool has_px4_odom_ = false;
    bool has_sensor_combined_ = false;
    bool has_vio_odom_ = false;
    bool has_setpoint_ = false;
    std::vector<int> last_buttons_ = std::vector<int>(12, 0);
    std::vector<float> last_axes_ = std::vector<float>(8, 0.0);
    sensor_msgs::msg::Joy joy_input_;
    px4_msgs::msg::VehicleStatus vehicle_status_;
    
    Eigen::Vector3d pos_enu{0.0, 0.0, 0.0};
    Eigen::Vector3d vel_enu{0.0, 0.0, 0.0};
    Eigen::Vector3d acc_enu{0.0, 0.0, 0.0};
    double yaw_enu = 0.0;
    double yaw_enu_px4 = 0.0;
    double yaw_enu_vio = 0.0;

    // Sequence Trackers
    int last_trajectory_selector_ = -1;
    int trajectory_step_ = 0;
    rclcpp::Time step_start_time_;

    // Callbacks
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
        Eigen::Vector3d pos_enu_px4(msg->position[1], msg->position[0], -msg->position[2]);
        Eigen::Vector3d vel_enu_px4(msg->velocity[1], msg->velocity[0], -msg->velocity[2]);

        use_sim_mode_ = this->get_parameter("USE_SIM_MODE").as_bool();

        auto q = msg->q; 
        double siny_cosp = 2.0 * (q[0] * q[3] + q[1] * q[2]);
        double cosy_cosp = 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]);
        double yaw_ned = std::atan2(siny_cosp, cosy_cosp);

        yaw_enu_px4 = -yaw_ned + M_PI_2;
        // Wrap the angle between -PI and PI so it doesn't jump out of bounds
        yaw_enu_px4 = std::atan2(std::sin(yaw_enu_px4), std::cos(yaw_enu_px4));
        has_px4_odom_ = true;

        // If in simulation mode, feed PX4 data directly into the controller
        if (use_sim_mode_) {
            pos_enu = pos_enu_px4;
            vel_enu = vel_enu_px4;
            yaw_enu = yaw_enu_px4;
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
        Eigen::Vector3d pos_enu_vio(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        
        // 2. Velocity is Body Frame. We must rotate it!
        Eigen::Vector3d vel_body(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
        auto q_msg = msg->pose.pose.orientation;
        Eigen::Quaterniond q_sensor_to_world(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
        
        // Multiply the rotation matrix by the body velocity to get World Velocity
        Eigen::Vector3d vel_enu_vio = q_sensor_to_world.toRotationMatrix() * vel_body;

        // 3. Extract Yaw and apply the 90-degree OKVIS2 offset
        double siny_cosp = 2.0 * (q_msg.w * q_msg.z + q_msg.x * q_msg.y);
        double cosy_cosp = 1.0 - 2.0 * (q_msg.y * q_msg.y + q_msg.z * q_msg.z);
        yaw_enu_vio = std::atan2(siny_cosp, cosy_cosp) + M_PI_2;

        // Wrap the angle between -PI and PI so it doesn't jump out of bounds
        yaw_enu_vio = std::atan2(std::sin(yaw_enu_vio), std::cos(yaw_enu_vio));

        has_vio_odom_ = true;

        // If in normal mode, feed VIO data into the controller
        if (!use_sim_mode_) {
            pos_enu = pos_enu_vio;
            vel_enu = vel_enu_vio;
            yaw_enu = yaw_enu_vio;
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
        acc_enu << msg->accelerometer_m_s2[1], msg->accelerometer_m_s2[0], -msg->accelerometer_m_s2[2];
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
        // PROCESS PARAMETER CHANGES SAFELY
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
            if (!has_px4_odom_){
                controller_running = false; // Update flag, controller isn't running
                return; // Only need PX4 in sim
            }
        } else {
            if (!has_px4_odom_ || !has_sensor_combined_ || !has_vio_odom_) {
                controller_running = false; // Update flag, controller isn't running
                return; // Need all for VIO flight
            }
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
            controller_running = false; // Update flag, controller isn't running
            return;
        }
        
        // Check if controller started running for the first time to log it and reset the controller
        // Reset is also used to prime the takeoff sequence
        if(controller_running==false){
            RCLCPP_INFO(this->get_logger(), "Controller Activated! Starting trajectory execution.");
            controller_.reset();
        }
        controller_running = true;

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
            pos_sp = Eigen::Vector3d(0.0, 0.0, 0.5);
        } else if (trajectory_selector == 1) {
            // Trajectory 1: Square flight pattern
            pos_sp = Eigen::Vector3d(1.0, 1.0, 1.0);
        } else if (trajectory_selector == 2) {
            // Trajectory 2: Higher altitude hover
            pos_sp = Eigen::Vector3d(0.0, 0.0, 1.5);
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
                {0.0, 0.0, 1.0},
                {0.0, 1.0, 1.0},
                {0.0, 0.0, 1.0}
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
                {0.0, 0.0, 0.8},
                {0.0, 0.0, 1.0},
                {0.0, 1.0, 1.0},
                {0.5, 1.0, 1.0},
                {0.5, 0.0, 1.0},
                {0.0, 0.0, 1.0}
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

        // HEADING DRIFT CORRECTION (Pure ENU)
        double yaw_drift_enu = yaw_enu_px4 - yaw_enu;
        // Wrap the angle between -PI and PI so it doesn't jump out of bounds
        yaw_drift_enu = std::atan2(std::sin(yaw_drift_enu), std::cos(yaw_drift_enu));

        Eigen::Matrix3d R_yaw_correction;
        R_yaw_correction = Eigen::AngleAxisd(yaw_drift_enu, Eigen::Vector3d::UnitZ());

        Eigen::Matrix3d R_enu_aligned = R_yaw_correction * R_enu_vio;

        // CONVERT FROM ALIGNED ENU TO PX4's NED/FRD
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

        // DEBUG TELEMETRY PUBLISHERS
        position_controller::msg::ControllerDebug debug_msg;

        debug_msg.pos.x = pos_enu.x();
        debug_msg.pos.y = pos_enu.y();
        debug_msg.pos.z = pos_enu.z();

        debug_msg.yaw = yaw_enu;
        debug_msg.yaw_px4 = yaw_enu_px4;

        debug_msg.vel.x = vel_enu.x();
        debug_msg.vel.y = vel_enu.y();
        debug_msg.vel.z = vel_enu.z();

        debug_msg.acc.x = acc_enu.x();
        debug_msg.acc.y = acc_enu.y();
        debug_msg.acc.z = acc_enu.z();

        debug_msg.pos_sp.x = controller_.getPositionSetpoint().x();
        debug_msg.pos_sp.y = controller_.getPositionSetpoint().y();
        debug_msg.pos_sp.z = controller_.getPositionSetpoint().z();

        debug_msg.vel_sp.x = controller_.getVelocitySetpoint().x();
        debug_msg.vel_sp.y = controller_.getVelocitySetpoint().y();
        debug_msg.vel_sp.z = controller_.getVelocitySetpoint().z();

        Eigen::Vector3d current_acc_sp = controller_.getAccelerationSetpoint();
        debug_msg.acc_sp.x = current_acc_sp.x();
        debug_msg.acc_sp.y = current_acc_sp.y();
        debug_msg.acc_sp.z = current_acc_sp.z();

        debug_msg.thrust_cmd = thrust;
        debug_msg.hover_thrust = controller_.getHoverThrust();

        debug_msg.pid_vel_p_term.x = controller_.getVelocityPTerm().x();
        debug_msg.pid_vel_p_term.y = controller_.getVelocityPTerm().y();
        debug_msg.pid_vel_p_term.z = controller_.getVelocityPTerm().z();

        debug_msg.pid_vel_i_term.x = controller_.getVelocityITerm().x();
        debug_msg.pid_vel_i_term.y = controller_.getVelocityITerm().y();
        debug_msg.pid_vel_i_term.z = controller_.getVelocityITerm().z();

        debug_msg.pid_vel_d_term.x = controller_.getVelocityDTerm().x();
        debug_msg.pid_vel_d_term.y = controller_.getVelocityDTerm().y();
        debug_msg.pid_vel_d_term.z = controller_.getVelocityDTerm().z();

        pub_debug_->publish(debug_msg);
        
        // PUBLISH TO PX4
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