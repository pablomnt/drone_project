#include "pruebas_pablo/prueba2.hpp"

namespace pruebas_pablo {

Prueba2::Prueba2()
: Node("prueba2"), mode_changed_(false), armed_(false), counter_(0), setpoint_counter_(0)
{
    pub_ctrl_mode_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", 10);
    pub_sp_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", 10);
    pub_cmd_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", 10);
    sub_status_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status_v1",
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
        std::bind(&Prueba2::vehicle_status_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&Prueba2::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Nodo Prueba2 iniciado (OffboardControlMode + TrajectorySetpoint)");
}

Prueba2::~Prueba2()
{
    RCLCPP_INFO(this->get_logger(), "Nodo Prueba2 finalizado");
}

void Prueba2::vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
    if (!mode_changed_ && msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
        mode_changed_ = true;
        RCLCPP_INFO(this->get_logger(), "Modo OFFBOARD activo");
    }

    if (!armed_ && msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
        armed_ = true;
        RCLCPP_INFO(this->get_logger(), "Vehículo armado");
    }
}

void Prueba2::timer_callback()
{
    const uint64_t timestamp = this->now().nanoseconds() / 1000ULL;

    // 1. Publicar mensaje de control
    px4_msgs::msg::OffboardControlMode ctrl_mode{};
    ctrl_mode.timestamp = timestamp;
    ctrl_mode.position = true;
    pub_ctrl_mode_->publish(ctrl_mode);

    // 2. Estabilizar antes de despegar
    double z = 0.0;
    if (setpoint_counter_ < 10) {
        // Fase de estabilización en tierra
        z = 0.0;
    } else if (setpoint_counter_ < 20) {
        z = -0.3;
    } else if (setpoint_counter_ < 30) {
        z = -0.6;
    } else if (setpoint_counter_ < 40) {
        z = -1.0;
    } else {
        z = -1.5;  // Altura final de vuelo
    }

    // 3. Publicar setpoint de trayectoria
    px4_msgs::msg::TrajectorySetpoint sp{};
    sp.timestamp = timestamp;
    sp.position = {0.0, 0.0, z};  // Despegue vertical
    sp.yaw = 0.0;
    pub_sp_->publish(sp);

    // 4. Cambio de modo OFFBOARD
    if (!mode_changed_ && setpoint_counter_ == 11) {
        send_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
                             1.0f, 6.0f);
        RCLCPP_INFO(this->get_logger(), "Cambio a modo OFFBOARD solicitado");
    }

    // 5. Armado después del cambio de modo
    if (mode_changed_ && !armed_ && setpoint_counter_ == 21) {
        send_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
                             1.0f);
        RCLCPP_INFO(this->get_logger(), "Armado solicitado");
    }

    setpoint_counter_++;
}


void Prueba2::send_vehicle_command(uint16_t command, float param1, float param2)
{
    px4_msgs::msg::VehicleCommand cmd{};
    cmd.timestamp = this->now().nanoseconds() / 1000ULL;
    cmd.command = command;
    cmd.param1 = param1;
    cmd.param2 = param2;
    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.from_external = true;
    pub_cmd_->publish(cmd);
}

} // namespace pruebas_pablo

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<pruebas_pablo::Prueba2>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}