
#include "pruebas_pablo/prueba4_BC.hpp"

using namespace std::chrono_literals;

namespace pruebas_pablo {

Prueba4BC::Prueba4BC()
: Node("prueba4_bc"),
  mode_changed_(false),
  armed_(false),
  pos_valid_(false),
  counter_(0),
  setpoint_counter_(0),
  take_off_counter_(0),
  current_x_(0.0), current_y_(0.0), current_z_(0.0),
  reached_counter_(0),
  current_wp_index_(0),
  takeoff_altitude_(1.0)
{
    pub_ctrl_mode_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    pub_sp_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    pub_cmd_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
    pub_VIO_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry", 10);


    sub_status_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status_v1", rclcpp::QoS(1).best_effort(),
        std::bind(&Prueba4BC::vehicle_status_callback, this, std::placeholders::_1));

    sub_local_pos_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position_v1", rclcpp::QoS(1).best_effort(),
        std::bind(&Prueba4BC::local_position_callback, this, std::placeholders::_1));

    sub_visual_pos_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/okvis/okvis_odometry", rclcpp::QoS(1).best_effort(),
    std::bind(&Prueba4BC::visual_odometry_callback, this, std::placeholders::_1));

    // Definición de waypoints provisional, en futuro se leerá de fichero (cuadrado de 2x2 metros)
    waypoints_.push_back({0.0, 0.0, -takeoff_altitude_, 0.0});
    waypoints_.push_back({0.0, 1.0, -takeoff_altitude_, 0.0});
    waypoints_.push_back({-1.0, 1.0, -takeoff_altitude_, 0.0});
    waypoints_.push_back({1.0, 1.0, -takeoff_altitude_, 0.0});


    timer_ = this->create_wall_timer(100ms, std::bind(&Prueba4BC::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Nodo Prueba4BC (bucle cerrado) iniciado con %zu waypoints", waypoints_.size());
}

Prueba4BC::~Prueba4BC()
{
    RCLCPP_INFO(this->get_logger(), "Nodo Prueba4BC finalizado");
}

void Prueba4BC::vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
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

void Prueba4BC::visual_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    px4_msgs::msg::VehicleOdometry vio_msg{};
    vio_msg.timestamp = this->now().nanoseconds() / 1000ULL;

    // Posición directa (asumimos que está en el marco correcto)
    vio_msg.position[0] = msg->pose.pose.position.x;
    vio_msg.position[1] = msg->pose.pose.position.y;
    vio_msg.position[2] = msg->pose.pose.position.z;

    // ---- TRANSFORMACIÓN DE ORIENTACIÓN (Z-up a Z-down) ----
    tf2::Quaternion q_ros(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    // Aplicar rotación 180° en X → pasa de Z-up (ROS) a Z-down (PX4)
    tf2::Quaternion q_rot;
    q_rot.setRPY(M_PI, 0, 0);  // 180 grados de roll

    tf2::Quaternion q_ned = q_rot * q_ros;
    q_ned.normalize();

    vio_msg.q[0] = q_ned.w();
    vio_msg.q[1] = q_ned.x();
    vio_msg.q[2] = q_ned.y();
    vio_msg.q[3] = q_ned.z();
    // --------------------------------------------------------

    // Velocidad
    vio_msg.velocity[0] = msg->twist.twist.linear.x;
    vio_msg.velocity[1] = msg->twist.twist.linear.y;
    vio_msg.velocity[2] = msg->twist.twist.linear.z;

    pub_VIO_->publish(vio_msg);
}

void Prueba4BC::local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    if (msg->xy_valid && msg->z_valid) {
        pos_valid_ = true;
        current_x_ = msg->x;
        current_y_ = msg->y;
        current_z_ = msg->z;
        //RCLCPP_INFO(this->get_logger(), "Posicion x: %.2f, y: %.2f, z: %.2f", current_x_, current_y_, current_z_);
    } else {
        pos_valid_ = false;
    }
}

void Prueba4BC::take_off_sequence()
{
    sp.timestamp = timestamp;
    //RCLCPP_INFO(this->get_logger(), "Intentando despegar, contador: %d", take_off_counter_);
    if (take_off_counter_ < 10) {
        sp.position = {0.0f, 0.0f, 0.0f};
    } else if (take_off_counter_ < 15) {
        sp.position = {0.0f, 0.0f, -0.2f};
    } else if (take_off_counter_ < 30) {
        sp.position = {0.0f, 0.0f, -0.4f};
    } else if (take_off_counter_ < 45) {
        sp.position = {0.0f, 0.0f, -1.0f};
    } else if (take_off_counter_ < 60) {
        sp.position = {0.0f, 0.0f, -takeoff_altitude_};
    }
}

void Prueba4BC::seguimiento_waypoints(std::vector<Waypoint> waypoints_)
{
    if (armed_ && pos_valid_ && setpoint_counter_ > 60) {
        Waypoint &wp = waypoints_[current_wp_index_];
        double dx = current_x_ - wp.x;
        double dy = current_y_ - wp.y;
        double dz = current_z_ - wp.z;
        double dist_xy = std::sqrt(dx*dx + dy*dy);
        //RCLCPP_INFO(this->get_logger(), "distancia x: %.2f, y: %.2f, z: %.2f", dx, dy, dz);
        // Calcular yaw (orientación)
        float yaw = std::atan2(dy, dx);
        if (yaw > M_PI) yaw -= 2 * M_PI;  // Ajustar el yaw al rango [-pi, pi]
        if (yaw < -M_PI) yaw += 2 * M_PI;

        if (dist_xy < reach_thresh_xy_ && std::fabs(dz) < reach_thresh_z_) {
            reached_counter_++;
            if (reached_counter_ == 1)
                RCLCPP_INFO(this->get_logger(), "Waypoint %zu alcanzado, esperando...", current_wp_index_);
            if (reached_counter_ >= 60) {
                if (current_wp_index_ + 1 < waypoints_.size()) {
                    current_wp_index_++;
                    reached_counter_ = 0;
                    RCLCPP_INFO(this->get_logger(), "Avanzando al waypoint %zu", current_wp_index_);
                } else {
                    RCLCPP_INFO(this->get_logger(), "Misión completada. Aterrizando y desarmando...");
                    send_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
                    send_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
                    timer_->cancel();
                }
            }
        } else {
            reached_counter_ = 0;
        }

        // Establecer la posición y el yaw para la publicación
        sp.position = {static_cast<float>(dx), static_cast<float>(dy), static_cast<float>(dz)};
        sp.yaw = static_cast<float>(yaw);
        RCLCPP_INFO(this->get_logger(), "Publicando setpoint: x: %.2f, y: %.2f, z: %.2f, yaw: %.2f",
                    sp.position[0], sp.position[1], sp.position[2], sp.yaw);
        
}
}

void Prueba4BC::timer_callback()
{
    timestamp = this->now().nanoseconds() / 1000ULL;

    // 1. Publicar modo de control
    px4_msgs::msg::OffboardControlMode ctrl_mode{};
    ctrl_mode.timestamp = timestamp;
    ctrl_mode.position = true;
    pub_ctrl_mode_->publish(ctrl_mode);

    // 2. Publicar setpoint (Llamar a la secuencia de despegue y seguimiento de waypoints)
    if(armed_)
    {
        take_off_counter_++;
    }
    take_off_sequence();  // Despegue controlado
    seguimiento_waypoints(waypoints_);

    // 3. Cambios de modo y armado
    if (!mode_changed_ && setpoint_counter_ == 11) {
        send_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
        RCLCPP_INFO(this->get_logger(), "Modo OFFBOARD solicitado");
    }

    if (mode_changed_ && !armed_ && setpoint_counter_ == 21) {
        send_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
        RCLCPP_INFO(this->get_logger(), "Armado solicitado");
    }

    // 4. Control cerrado: avanzar si alcanzó el waypoint
    pub_sp_->publish(sp);  // Publicar el setpoint calculado

    setpoint_counter_++;
    counter_++;
}


void Prueba4BC::send_vehicle_command(uint16_t command, float param1, float param2,
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
    auto node = std::make_shared<pruebas_pablo::Prueba4BC>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
