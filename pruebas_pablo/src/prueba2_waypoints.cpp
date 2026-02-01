#include "pruebas_pablo/prueba2_waypoints.hpp"

using namespace std::chrono_literals;

namespace pruebas_pablo {

Prueba2Waypoints::Prueba2Waypoints()
: Node("prueba2_waypoints"),
  mode_changed_(false),
  armed_(false),
  counter_(0),
  setpoint_counter_(0),
  take_off_counter_(0),
  current_wp_index_(0),
  takeoff_altitude_(1.5)
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
        std::bind(&Prueba2Waypoints::vehicle_status_callback, this, std::placeholders::_1));

    waypoints_.push_back({0.0, 0.0, -takeoff_altitude_, 0.0});
    waypoints_.push_back({2.0, 0.0, -takeoff_altitude_, 0.0});
    waypoints_.push_back({2.0, 2.0, -takeoff_altitude_, 0.0});

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&Prueba2Waypoints::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Nodo Prueba2Waypoints iniciado con %zu waypoints", waypoints_.size());
}

Prueba2Waypoints::~Prueba2Waypoints()
{
    RCLCPP_INFO(this->get_logger(), "Nodo Prueba2Waypoints finalizado");
}

void Prueba2Waypoints::vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
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

void Prueba2Waypoints::timer_callback()
{
    const uint64_t timestamp = this->now().nanoseconds() / 1000ULL;

    // Publicar modo de control
    px4_msgs::msg::OffboardControlMode ctrl_mode{};
    ctrl_mode.timestamp = timestamp;
    ctrl_mode.position = true;
    pub_ctrl_mode_->publish(ctrl_mode);

    // Preparar setpoint
    px4_msgs::msg::TrajectorySetpoint sp{};
    sp.timestamp = timestamp;

    // Despegue progresivo y estabilización
    if (take_off_counter_ < 10) {
        sp.position = {0.0f, 0.0f, 0.0f};
    } else if (take_off_counter_ < 20) {
        sp.position = {0.0f, 0.0f, -0.3f};
    } else if (take_off_counter_ < 30) {
        sp.position = {0.0f, 0.0f, -0.6f};
    } else if (take_off_counter_ < 40) {
        sp.position = {0.0f, 0.0f, -1.0f};
    } else if (take_off_counter_ < 50) {
        sp.position = {0.0f, 0.0f, -1.5f};
    } else {
        Waypoint &wp = waypoints_[current_wp_index_];
        sp.position = {static_cast<float>(wp.x),
                       static_cast<float>(wp.y),
                       static_cast<float>(wp.z)};
        sp.yaw = static_cast<float>(wp.yaw);
    }

    pub_sp_->publish(sp);

    if (!mode_changed_ && setpoint_counter_ == 11) {
        send_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
                             1.0f, 6.0f);
        RCLCPP_INFO(this->get_logger(), "Modo OFFBOARD solicitado");
    }

    if (mode_changed_ && !armed_ && setpoint_counter_ == 21) {
        send_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
                             1.0f);
        RCLCPP_INFO(this->get_logger(), "Armado solicitado");
    }

    // Avanzar entre waypoints cada 100 ciclos
    if (armed_ && setpoint_counter_ > 150) {
        if (current_wp_index_ + 1 < waypoints_.size()) {
            current_wp_index_++;
            RCLCPP_INFO(this->get_logger(), "Avanzando al waypoint %zu", current_wp_index_);
            setpoint_counter_ = 50;  // mantener altitud después del despegue
        } else {
            RCLCPP_INFO(this->get_logger(), "Misión completada. Aterrizando y desarmando...");
            send_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
            send_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
                                 0.0f);
            timer_->cancel();
        }
    }

    if(armed_)
    {
        take_off_counter_++;
    }

    setpoint_counter_++;
    counter_++;
}

void Prueba2Waypoints::send_vehicle_command(uint16_t command, float param1, float param2,
                                            float param3, float param4, float param5,
                                            float param6, float param7)
{
    px4_msgs::msg::VehicleCommand cmd{};
    cmd.timestamp = this->now().nanoseconds() / 1000ULL;
    cmd.command = command;
    cmd.param1 = param1;
    cmd.param2 = param2;
    cmd.param3 = param3;
    cmd.param4 = param4;
    cmd.param5 = param5;
    cmd.param6 = param6;
    cmd.param7 = param7;
    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.from_external = true;
    pub_cmd_->publish(cmd);
}

}  // namespace pruebas_pablo

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<pruebas_pablo::Prueba2Waypoints>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
