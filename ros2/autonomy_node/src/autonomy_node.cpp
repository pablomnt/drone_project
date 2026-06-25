// ROS wrapper around the middleware-free autonomy core. Its only jobs are to
// translate messages to and from the core's plain types (including the
// ENU<->NED/FRD frame conversions), drive the core at the control rate, and
// speak to PX4. All guidance and control logic lives in drone_core.
//
// TO RUN:
//   Normal (VIO state estimate):  ros2 run autonomy_node autonomy_node
//   Simulation (PX4 odometry):    ros2 run autonomy_node autonomy_node --ros-args -p USE_SIM_MODE:=true
//   With trajectory feed-forward: add -p ENABLE_FEEDFORWARD:=true

#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/sensor_combined.hpp"
#include "px4_msgs/msg/vehicle_attitude_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "drone_interfaces/msg/controller_debug.hpp"

#include "drone_core/autonomy/autonomy_core.hpp"
#include "drone_core/common/frames.hpp"

using namespace std::chrono_literals;

namespace {
constexpr double kControlDt = 0.02;       // 50 Hz
constexpr double kDefaultYaw = M_PI_2;    // ENU heading for direct/hover setpoints
}  // namespace

class AutonomyNode : public rclcpp::Node {
public:
  AutonomyNode() : Node("autonomy_node") {
    declareParameters();

    use_sim_mode_ = get_parameter("USE_SIM_MODE").as_bool();
    if (use_sim_mode_) {
      RCLCPP_WARN(get_logger(), "SIMULATION MODE: using PX4 odometry for state estimation.");
    } else {
      RCLCPP_INFO(get_logger(), "NORMAL MODE: using VIO (OKVIS2) for state estimation.");
    }

    core_ = std::make_unique<drone_core::autonomy::AutonomyCore>(configFromParameters());
    core_->setClock([this]() { return this->get_clock()->now().seconds(); });
    core_->startPlanner();

    param_callback_ = add_on_set_parameters_callback(
        std::bind(&AutonomyNode::onParameterChange, this, std::placeholders::_1));

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    sub_joy_ = create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&AutonomyNode::onJoy, this, std::placeholders::_1));
    sub_px4_odom_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", qos, std::bind(&AutonomyNode::onPx4Odom, this, std::placeholders::_1));
    sub_sensor_ = create_subscription<px4_msgs::msg::SensorCombined>(
        "/fmu/out/sensor_combined", qos, std::bind(&AutonomyNode::onSensorCombined, this, std::placeholders::_1));
    sub_status_ = create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status_v1", qos, std::bind(&AutonomyNode::onStatus, this, std::placeholders::_1));
    sub_vio_ = create_subscription<nav_msgs::msg::Odometry>(
        "/okvis/okvis_odometry", 10, std::bind(&AutonomyNode::onVioOdom, this, std::placeholders::_1));
    sub_map_ = create_subscription<octomap_msgs::msg::Octomap>(
        "/octomap_binary", 10, std::bind(&AutonomyNode::onOctomap, this, std::placeholders::_1));
    sub_goal_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/planner/goal", 10, std::bind(&AutonomyNode::onGoal, this, std::placeholders::_1));

    pub_attitude_ = create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
        "/fmu/in/vehicle_attitude_setpoint_v1", 10);
    pub_offboard_ = create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    pub_command_ = create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
    pub_debug_ = create_publisher<drone_interfaces::msg::ControllerDebug>("/debug/telemetry", 10);
    pub_path_ = create_publisher<nav_msgs::msg::Path>("/smooth_trajectory", 10);

    control_timer_ = create_wall_timer(20ms, std::bind(&AutonomyNode::controlLoop, this));
    viz_timer_ = create_wall_timer(500ms, std::bind(&AutonomyNode::publishPlannedPath, this));

    RCLCPP_INFO(get_logger(), "Autonomy node started.");
  }

  ~AutonomyNode() override {
    if (core_) core_->stopPlanner();
  }

private:
  void declareParameters() {
    declare_parameter<bool>("USE_SIM_MODE", false);
    declare_parameter<bool>("ENABLE_FEEDFORWARD", false);

    declare_parameter("MPC_XY_P", 0.95);
    declare_parameter("MPC_Z_P", 1.0);
    declare_parameter("MPC_XY_VEL_P", 1.8);
    declare_parameter("MPC_XY_VEL_I", 0.4);
    declare_parameter("MPC_XY_VEL_D", 0.2);
    declare_parameter("MPC_Z_VEL_P", 2.0);
    declare_parameter("MPC_Z_VEL_I", 0.5);
    declare_parameter("MPC_Z_VEL_D", 0.2);
    declare_parameter("MPC_HOVER_THRUST", 0.35);

    declare_parameter<std::vector<double>>("POS_SP", {0.0, 0.0, 1.3});
    declare_parameter("STALE_TIMEOUT", 0.5);
    declare_parameter("RRT_PERIOD", 5.0);
    declare_parameter("TRAJGEN_PERIOD", 1.0);
  }

  drone_core::autonomy::AutonomyCore::Config configFromParameters() {
    drone_core::autonomy::AutonomyCore::Config cfg;
    const double xy_p = get_parameter("MPC_XY_P").as_double();
    const double z_p = get_parameter("MPC_Z_P").as_double();
    cfg.pos_p = Eigen::Vector3d(xy_p, xy_p, z_p);

    const double xy_vp = get_parameter("MPC_XY_VEL_P").as_double();
    const double z_vp = get_parameter("MPC_Z_VEL_P").as_double();
    cfg.vel_p = Eigen::Vector3d(xy_vp, xy_vp, z_vp);

    const double xy_vi = get_parameter("MPC_XY_VEL_I").as_double();
    const double z_vi = get_parameter("MPC_Z_VEL_I").as_double();
    cfg.vel_i = Eigen::Vector3d(xy_vi, xy_vi, z_vi);

    const double xy_vd = get_parameter("MPC_XY_VEL_D").as_double();
    const double z_vd = get_parameter("MPC_Z_VEL_D").as_double();
    cfg.vel_d = Eigen::Vector3d(xy_vd, xy_vd, z_vd);

    cfg.hover_thrust = get_parameter("MPC_HOVER_THRUST").as_double();
    cfg.enable_feedforward = get_parameter("ENABLE_FEEDFORWARD").as_bool();
    cfg.stale_timeout = get_parameter("STALE_TIMEOUT").as_double();
    cfg.rrt_period = get_parameter("RRT_PERIOD").as_double();
    cfg.trajgen_period = get_parameter("TRAJGEN_PERIOD").as_double();
    return cfg;
  }

  rcl_interfaces::msg::SetParametersResult onParameterChange(const std::vector<rclcpp::Parameter>&) {
    // The control loop reads parameters live, so just push a refreshed config.
    if (core_) core_->applyConfig(configFromParameters());
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  // --- PX4 command helpers -------------------------------------------------

  void publishVehicleCommand(uint16_t command, float p1 = 0, float p2 = 0, float p7 = 0) {
    px4_msgs::msg::VehicleCommand msg;
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    msg.command = command;
    msg.param1 = p1;
    msg.param2 = p2;
    msg.param7 = p7;
    pub_command_->publish(msg);
  }

  void arm() { publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1); }
  void disarm() { publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0); }
  void engageOffboard() { publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6); }
  void land() { publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND); }

  // --- Subscriptions -------------------------------------------------------

  void onJoy(const sensor_msgs::msg::Joy::SharedPtr msg) {
    const auto& b = msg->buttons;
    if (b.size() > 9 && b[9] == 1 && last_buttons_.size() > 9 && last_buttons_[9] == 0) {
      if (vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
        disarm();
      } else {
        arm();
      }
    }
    if (b.size() > 1 && b[1] == 1 && last_buttons_.size() > 1 && last_buttons_[1] == 0) engageOffboard();
    if (b.size() > 8 && b[8] == 1 && last_buttons_.size() > 8 && last_buttons_[8] == 0) land();
    last_buttons_ = b;
  }

  void onStatus(const px4_msgs::msg::VehicleStatus::SharedPtr msg) { vehicle_status_ = *msg; }

  void onPx4Odom(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    const Eigen::Vector3d pos_ned(msg->position[0], msg->position[1], msg->position[2]);
    const Eigen::Vector3d vel_ned(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
    const Eigen::Quaterniond q(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);

    px4_pos_enu_ = drone_core::frames::pxNedToEnu(pos_ned);
    px4_vel_enu_ = drone_core::frames::pxNedToEnu(vel_ned);
    yaw_px4_enu_ = drone_core::frames::pxAttitudeToEnuYaw(q);
    has_px4_odom_ = true;
  }

  void onVioOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    vio_pos_enu_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    const Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                               msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    const Eigen::Vector3d vel_body(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    vio_vel_enu_ = drone_core::frames::bodyVelToWorld(q, vel_body);
    yaw_vio_enu_ = drone_core::frames::okvisAttitudeToEnuYaw(q);
    has_vio_odom_ = true;
  }

  void onSensorCombined(const px4_msgs::msg::SensorCombined::SharedPtr msg) {
    Eigen::Vector3d acc_ned(msg->accelerometer_m_s2[0], msg->accelerometer_m_s2[1], msg->accelerometer_m_s2[2]);
    acc_enu_ = drone_core::frames::pxNedToEnu(acc_ned);
    acc_enu_.z() -= 9.81;  // remove gravity to recover dynamic acceleration
    has_sensor_ = true;
  }

  void onOctomap(const octomap_msgs::msg::Octomap::SharedPtr msg) {
    octomap::AbstractOcTree* tree = octomap_msgs::binaryMsgToMap(*msg);
    if (!tree) return;
    if (auto* octree = dynamic_cast<octomap::OcTree*>(tree)) {
      core_->setMap(std::shared_ptr<octomap::OcTree>(octree));
    } else {
      delete tree;
    }
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    drone_core::common::Goal goal;
    goal.pos = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    core_->setGoal(goal);
    RCLCPP_INFO(get_logger(), "New goal: (%.2f, %.2f, %.2f)", goal.pos.x(), goal.pos.y(), goal.pos.z());
  }

  // --- Control loop --------------------------------------------------------

  bool dependenciesReady() const {
    if (use_sim_mode_) return has_px4_odom_;
    return has_px4_odom_ && has_sensor_ && has_vio_odom_;
  }

  void controlLoop() {
    use_sim_mode_ = get_parameter("USE_SIM_MODE").as_bool();

    px4_msgs::msg::OffboardControlMode hb;
    hb.timestamp = get_clock()->now().nanoseconds() / 1000;
    hb.attitude = true;
    pub_offboard_->publish(hb);

    if (!dependenciesReady()) {
      controller_running_ = false;
      return;
    }

    const bool armed = vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    const bool offboard = vehicle_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;

    // Reset on disarm or leaving offboard so takeoff re-primes on the next arm.
    if ((was_armed_ && !armed) || (was_offboard_ && !offboard)) {
      RCLCPP_INFO(get_logger(), "Interruption (disarmed or left offboard). Resetting controller.");
      core_->reset();
      has_px4_odom_ = has_sensor_ = has_vio_odom_ = false;
    }
    was_armed_ = armed;
    was_offboard_ = offboard;

    if (!armed || !offboard) {
      controller_running_ = false;
      return;
    }

    if (!controller_running_) {
      RCLCPP_INFO(get_logger(), "Controller engaged.");
      core_->reset();
      controller_running_ = true;
    }

    // Assemble the ENU state estimate from the active estimator.
    drone_core::common::State state;
    if (use_sim_mode_) {
      state.pos = px4_pos_enu_;
      state.vel = px4_vel_enu_;
      state.yaw = yaw_px4_enu_;
    } else {
      state.pos = vio_pos_enu_;
      state.vel = vio_vel_enu_;
      state.yaw = yaw_vio_enu_;
    }
    state.acc = acc_enu_;
    state.stamp = get_clock()->now().seconds();
    const double yaw_used = state.yaw;
    core_->setState(state);

    // Default direct setpoint for takeoff / manual hover. A planner goal, once
    // set, supersedes this inside the tracker.
    const auto pos_sp = get_parameter("POS_SP").as_double_array();
    if (pos_sp.size() == 3) {
      core_->setSetpoint(Eigen::Vector3d(pos_sp[0], pos_sp[1], pos_sp[2]), kDefaultYaw);
    }

    const drone_core::common::Command cmd = core_->stepControl(kControlDt);

    // Correct the controller's heading toward PX4's yaw estimate, then convert
    // the ENU attitude into PX4's NED/FRD frame.
    const double yaw_drift = drone_core::frames::wrapPi(yaw_px4_enu_ - yaw_used);
    const Eigen::Quaterniond q_ned =
        drone_core::frames::enuAttitudeToPxNed(cmd.attitude.toRotationMatrix(), yaw_drift);

    px4_msgs::msg::VehicleAttitudeSetpoint att;
    att.timestamp = get_clock()->now().nanoseconds() / 1000;
    att.q_d[0] = q_ned.w();
    att.q_d[1] = q_ned.x();
    att.q_d[2] = q_ned.y();
    att.q_d[3] = q_ned.z();
    att.thrust_body[0] = 0.0;
    att.thrust_body[1] = 0.0;
    att.thrust_body[2] = -cmd.thrust;
    pub_attitude_->publish(att);

    publishDebug(state);
  }

  void publishDebug(const drone_core::common::State& state) {
    const auto& c = core_->controller();
    drone_interfaces::msg::ControllerDebug d;
    d.pos.x = state.pos.x(); d.pos.y = state.pos.y(); d.pos.z = state.pos.z();
    d.yaw = state.yaw;
    d.yaw_px4 = yaw_px4_enu_;
    d.vel.x = state.vel.x(); d.vel.y = state.vel.y(); d.vel.z = state.vel.z();
    d.acc.x = state.acc.x(); d.acc.y = state.acc.y(); d.acc.z = state.acc.z();
    const Eigen::Vector3d psp = c.getPositionSetpoint();
    d.pos_sp.x = psp.x(); d.pos_sp.y = psp.y(); d.pos_sp.z = psp.z();
    const Eigen::Vector3d vsp = c.getVelocitySetpoint();
    d.vel_sp.x = vsp.x(); d.vel_sp.y = vsp.y(); d.vel_sp.z = vsp.z();
    const Eigen::Vector3d asp = c.getAccelerationSetpoint();
    d.acc_sp.x = asp.x(); d.acc_sp.y = asp.y(); d.acc_sp.z = asp.z();
    d.thrust_cmd = c.getThrustSetpoint();
    d.hover_thrust = c.getHoverThrust();
    const Eigen::Vector3d p = c.getVelocityPTerm();
    d.pid_vel_p_term.x = p.x(); d.pid_vel_p_term.y = p.y(); d.pid_vel_p_term.z = p.z();
    const Eigen::Vector3d i = c.getVelocityITerm();
    d.pid_vel_i_term.x = i.x(); d.pid_vel_i_term.y = i.y(); d.pid_vel_i_term.z = i.z();
    const Eigen::Vector3d dd = c.getVelocityDTerm();
    d.pid_vel_d_term.x = dd.x(); d.pid_vel_d_term.y = dd.y(); d.pid_vel_d_term.z = dd.z();
    pub_debug_->publish(d);
  }

  void publishPlannedPath() {
    const auto sampled = core_->sampledPlannedPath();
    if (sampled.empty()) return;
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = "map";
    for (const auto& p : sampled) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = p[0];
      pose.pose.position.y = p[1];
      pose.pose.position.z = p[2];
      path.poses.push_back(pose);
    }
    pub_path_->publish(path);
  }

  // --- Members -------------------------------------------------------------

  std::unique_ptr<drone_core::autonomy::AutonomyCore> core_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_px4_odom_;
  rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr sub_sensor_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr sub_status_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_vio_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr sub_map_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;

  rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr pub_attitude_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr pub_offboard_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_command_;
  rclcpp::Publisher<drone_interfaces::msg::ControllerDebug>::SharedPtr pub_debug_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr viz_timer_;

  bool use_sim_mode_{false};
  px4_msgs::msg::VehicleStatus vehicle_status_;
  std::vector<int> last_buttons_;

  Eigen::Vector3d px4_pos_enu_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d px4_vel_enu_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d vio_pos_enu_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d vio_vel_enu_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acc_enu_{Eigen::Vector3d::Zero()};
  double yaw_px4_enu_{0.0};
  double yaw_vio_enu_{0.0};

  bool has_px4_odom_{false};
  bool has_sensor_{false};
  bool has_vio_odom_{false};
  bool was_armed_{false};
  bool was_offboard_{false};
  bool controller_running_{false};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutonomyNode>());
  rclcpp::shutdown();
  return 0;
}
