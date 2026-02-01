#ifndef PRUEBAS_PABLO_PRUEBA3_BC_HPP_
#define PRUEBAS_PABLO_PRUEBA3_BC_HPP_

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace pruebas_pablo {

class Prueba3BC : public rclcpp::Node
{
public:
    Prueba3BC();
    ~Prueba3BC();

private:
    px4_msgs::msg::TrajectorySetpoint sp{};
    uint64_t timestamp;
    struct Waypoint {
        double x, y, z, yaw;
    };
    std::vector<Waypoint> waypoints_;
    bool mode_changed_;
    bool armed_;
    bool pos_valid_;
    int counter_;
    int setpoint_counter_;
    int take_off_counter_;

    double current_x_, current_y_, current_z_;
    int reached_counter_;

   
    size_t current_wp_index_;
    double takeoff_altitude_;

    const double reach_thresh_xy_ = 0.3;
    const double reach_thresh_z_ = 0.2;


    void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    void local_position_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timer_callback();
    void send_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f,
                              float param3 = 0.0f, float param4 = 0.0f,
                              float param5 = 0.0f, float param6 = 0.0f, float param7 = 0.0f);

    void take_off_sequence();
    void seguimiento_waypoints(std::vector<Waypoint> waypoints_);

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr pub_ctrl_mode_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr pub_sp_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_cmd_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr sub_status_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_local_pos_;
    rclcpp::TimerBase::SharedPtr timer_;

};

}  // namespace pruebas_pablo

#endif  // PRUEBAS_PABLO_PRUEBA3_BC_HPP_
