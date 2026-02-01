/*
    Esta prueba consigue cambiar a OFFBOARDMODE y armar. La diferencia con prueba1 es que escribe en un topic llamado "/fmu/in/offboard_control_mode", que
    necesita que le digas como va a ser la consigna que vas a realizarle, si de posicion, velocidad o aceleracion. Si mandas de varias, prevalecerá la de mayor prioridad,
    en este orden: posición, velocidad, aceleración, attitude, body_rate(?), thrust_and_torque(?), y direct_actuators. Con ese topic y el parámetro px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
    si que se consigue comandar al drone, pero hace un despegue no controlado.
*/
#ifndef PRUEBA2_HPP
#define PRUEBA2_HPP

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

namespace pruebas_pablo {

class Prueba2 : public rclcpp::Node {
public:
    Prueba2();
    ~Prueba2();

private:
    void timer_callback();
    void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    void send_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f);

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr pub_ctrl_mode_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr pub_sp_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_cmd_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr sub_status_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool mode_changed_, armed_;
    int counter_, setpoint_counter_;
};

} // namespace pruebas_pablo

#endif // PRUEBA2_HPP