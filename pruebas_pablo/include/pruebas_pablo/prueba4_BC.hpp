#ifndef PRUEBAS_PABLO_PRUEBA4_BC_HPP_
#define PRUEBAS_PABLO_PRUEBA4_BC_HPP_

/*
Este script es igual que el de prueba 3, pero toma la salida de la camara(nodo okvis2) y se lo manda a px4 por el nodo VehicleVisualOdometry
NO SE COMO FUNCIONA EL FILTRO DE KALMAN INTERNO DE PX4, NO ENTIENDO COMO ESTA RESPONDIENDO EL DRONE A TODO.
ULTIMA OPCION SERIA PROBARLO CON EL DRONE REAL A VER SI FUNCIONA MEJOR.
REVISAR PARAMETROS DE FILTRO EKF2_EV Y EKF2_HGT_REF Y ESO PARA VER SI HAY ALGO MAL

*/

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>


namespace pruebas_pablo {

class Prueba4BC : public rclcpp::Node
{
public:
    Prueba4BC();
    ~Prueba4BC();

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
    void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void timer_callback();
    void send_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f,
                              float param3 = 0.0f, float param4 = 0.0f,
                              float param5 = 0.0f, float param6 = 0.0f, float param7 = 0.0f);

    void take_off_sequence();
    void visual_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void seguimiento_waypoints(std::vector<Waypoint> waypoints_);

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr pub_ctrl_mode_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr pub_sp_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_cmd_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr pub_VIO_; //enviar odometria de camara a px4

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_visual_pos_; //odometria de camara. VIO
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr sub_status_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_local_pos_; //salida de filtro de Kalman. Posicion del drone
    rclcpp::TimerBase::SharedPtr timer_;

};

}  // namespace pruebas_pablo

#endif  // PRUEBAS_PABLO_PRUEBA4_BC_HPP_
