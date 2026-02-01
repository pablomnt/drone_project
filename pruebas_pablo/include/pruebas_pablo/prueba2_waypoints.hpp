#ifndef PRUEBAS_PABLO_PRUEBA2_WAYPOINTS_HPP_
#define PRUEBAS_PABLO_PRUEBA2_WAYPOINTS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

namespace pruebas_pablo {

class Prueba2Waypoints : public rclcpp::Node
{
public:
    Prueba2Waypoints();
    ~Prueba2Waypoints();

private:
    void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    void timer_callback();
    void send_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f,
                              float param3 = 0.0f, float param4 = 0.0f,
                              float param5 = 0.0f, float param6 = 0.0f, float param7 = 0.0f);

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr pub_ctrl_mode_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr pub_sp_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_cmd_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr sub_status_;

    rclcpp::TimerBase::SharedPtr timer_;

    // estado interno
    bool mode_changed_;
    bool armed_;
    int counter_;
    int setpoint_counter_;
    int take_off_counter_;

    // waypoints
    struct Waypoint {
        double x;
        double y;
        double z;
        double yaw;
    };
    std::vector<Waypoint> waypoints_;
    size_t current_wp_index_;

    // configuración
    double takeoff_altitude_;
};

}  // namespace pruebas_pablo

#endif  // PRUEBAS_PABLO_PRUEBA2_WAYPOINTS_HPP_
