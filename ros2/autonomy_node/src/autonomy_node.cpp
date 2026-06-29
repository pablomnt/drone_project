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
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
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

// RTAB-Map publishes its octomap as a ColorOcTree (it stores voxel colour for
// visualisation), but the core's map model — and DynamicEDTOctomap — needs a
// plain octomap::OcTree. Copy occupancy across, dropping colour. Coarse (pruned)
// leaves are expanded to resolution-sized voxels so an occupied region stays
// solid instead of collapsing to a single centre voxel (which would punch holes
// in walls for the planner). Works for any OccupancyOcTreeBase node type.
template <typename TreeT>
std::shared_ptr<octomap::OcTree> toOcTree(const TreeT& in) {
  const double res = in.getResolution();
  auto out = std::make_shared<octomap::OcTree>(res);
  for (auto it = in.begin_leafs(), end = in.end_leafs(); it != end; ++it) {
    const float log_odds = it->getLogOdds();
    const double size = it.getSize();
    if (size <= res * 1.5) {
      out->setNodeValue(it.getCoordinate(), log_odds, /*lazy_eval=*/true);
    } else {
      const double half = (size - res) / 2.0;
      const octomap::point3d c = it.getCoordinate();
      for (double dx = -half; dx <= half + 1e-6; dx += res)
        for (double dy = -half; dy <= half + 1e-6; dy += res)
          for (double dz = -half; dz <= half + 1e-6; dz += res)
            out->setNodeValue(octomap::point3d(c.x() + dx, c.y() + dy, c.z() + dz),
                              log_odds, /*lazy_eval=*/true);
    }
  }
  out->updateInnerOccupancy();
  return out;
}
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
    // RTAB-Map publishes the assembled octomap only on map-graph updates (motion-gated)
    // but latches it TRANSIENT_LOCAL. A VOLATILE subscriber never receives that retained
    // sample, so on a static scene map_ would stay null and the planner would starve.
    // Match the publisher's durability so we pull the last map on connect plus any
    // updates. (Also compatible with octomap_server's latched publisher.)
    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    sub_map_ = create_subscription<octomap_msgs::msg::Octomap>(
        "/octomap_binary", map_qos, std::bind(&AutonomyNode::onOctomap, this, std::placeholders::_1));
    sub_goal_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/planner/goal", 10, std::bind(&AutonomyNode::onGoal, this, std::placeholders::_1));

    pub_attitude_ = create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
        "/fmu/in/vehicle_attitude_setpoint_v1", 10);
    pub_offboard_ = create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    pub_command_ = create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
    pub_debug_ = create_publisher<drone_interfaces::msg::ControllerDebug>("/debug/telemetry", 10);
    pub_path_ = create_publisher<nav_msgs::msg::Path>("/smooth_trajectory", 10);
    pub_geom_path_ = create_publisher<visualization_msgs::msg::MarkerArray>("/planner/geometric_path", 10);
    pub_goal_marker_ = create_publisher<visualization_msgs::msg::Marker>("/planner/goal_marker", 10);
    pub_search_tree_ = create_publisher<visualization_msgs::msg::MarkerArray>("/planner/search_tree", 10);
    pub_clearance_field_ = create_publisher<sensor_msgs::msg::PointCloud2>("/planner/clearance_field", 10);

    control_timer_ = create_wall_timer(20ms, std::bind(&AutonomyNode::controlLoop, this));
    viz_timer_ = create_wall_timer(500ms, std::bind(&AutonomyNode::publishViz, this));

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
    declare_parameter("RRT_MONITOR_PERIOD", 1.0);
    declare_parameter("RRT_IMPROVE_PERIOD", 10.0);
    declare_parameter("RRT_SOLVE_TIME", 5.0);
    // Which OMPL planner to run: RRTstar | BITstar | ABITstar | AITstar | EITstar.
    // Live-reconfigurable so you can A/B them on the bench. Per-planner internal
    // tunables live in geometric_planner.hpp (PlannerConfig).
    declare_parameter<std::string>("PLANNER_TYPE", "RRTstar");
    declare_parameter("REPLAN_IMPROVE_RATIO", 0.85);
    declare_parameter("CLEARANCE_WEIGHT", 4.0);
    declare_parameter("CLEARANCE_THRESHOLD", 1.0);
    declare_parameter("TRAJGEN_PERIOD", 1.0);
    // Geometry-first bring-up: with this false the planner only runs RRT* and
    // publishes the geometric path; it does not generate a trajectory or feed
    // the controller, which keeps following POS_SP. Flip to true to enable the
    // min-snap trajectory + tracking stage.
    declare_parameter("PLAN_TRAJECTORY", false);
    // Single switch for the planner debug visualisation: publishes the RRT*
    // search tree (/planner/search_tree) and the EDT clearance field
    // (/planner/clearance_field). Off by default so regular flights pay nothing;
    // flip true for a debugging/tuning run (live-reconfigurable).
    declare_parameter("DEBUG_PLANNER_VIZ", true);
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
    cfg.rrt_monitor_period = get_parameter("RRT_MONITOR_PERIOD").as_double();
    cfg.rrt_improve_period = get_parameter("RRT_IMPROVE_PERIOD").as_double();
    cfg.rrt_solve_time = get_parameter("RRT_SOLVE_TIME").as_double();
    const std::string planner = get_parameter("PLANNER_TYPE").as_string();
    if (!drone_core::planning::fromString(planner, cfg.planner_type)) {
      RCLCPP_WARN(get_logger(), "Unknown PLANNER_TYPE '%s', using RRTstar.", planner.c_str());
      cfg.planner_type = drone_core::planning::PlannerType::RRTstar;
    }
    cfg.replan_improve_ratio = get_parameter("REPLAN_IMPROVE_RATIO").as_double();
    cfg.clearance_weight = get_parameter("CLEARANCE_WEIGHT").as_double();
    cfg.clearance_threshold = get_parameter("CLEARANCE_THRESHOLD").as_double();
    cfg.trajgen_period = get_parameter("TRAJGEN_PERIOD").as_double();
    cfg.plan_trajectory = get_parameter("PLAN_TRAJECTORY").as_bool();
    cfg.debug_planner_viz = get_parameter("DEBUG_PLANNER_VIZ").as_bool();
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
    if (!tree) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "octomap failed to deserialize (binaryMsgToMap returned null)");
      return;
    }

    std::shared_ptr<octomap::OcTree> map;
    if (auto* octree = dynamic_cast<octomap::OcTree*>(tree)) {
      // Already the type the core wants (e.g. octomap_server) — take ownership.
      map = std::shared_ptr<octomap::OcTree>(octree);
    } else if (auto* color = dynamic_cast<octomap::ColorOcTree*>(tree)) {
      // RTAB-Map's case: convert colour tree to a plain OcTree, then free the original.
      map = toOcTree(*color);
      delete tree;
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "octomap is an unsupported tree type (id='%s'), ignoring",
                           msg->id.c_str());
      delete tree;
      return;
    }

    // One-time confirmation the core is actually being fed a map (and how dense).
    RCLCPP_INFO_ONCE(get_logger(), "First octomap received: %zu nodes", map->size());
    core_->setMap(map);
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    drone_core::common::Goal goal;
    goal.pos = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    core_->setGoal(goal);
    goal_pos_ = goal.pos;
    has_goal_ = true;
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

  void publishViz() {
    publishGoalMarker();
    publishGeometricPath();
    publishPlannedPath();
    publishSearchTree();
    publishClearanceField();
  }

  // The active goal as a single sphere marker, so it renders as a point in RViz
  // regardless of display type (unlike the raw /planner/goal PoseStamped, which
  // RViz draws as an orientation arrow).
  void publishGoalMarker() {
    if (!has_goal_) return;
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = now();
    m.ns = "goal";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = goal_pos_.x();
    m.pose.position.y = goal_pos_.y();
    m.pose.position.z = goal_pos_.z();
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 0.3;  // sphere diameter [m]
    m.color.r = 1.0f; m.color.g = 0.1f; m.color.b = 0.1f; m.color.a = 1.0f;
    pub_goal_marker_->publish(m);
  }

  // Raw RRT* waypoints (drone position -> goal) as an RViz MarkerArray: a line
  // strip through the waypoints plus a sphere at each one. Independent of the
  // min-snap trajectory, so it renders even when PLAN_TRAJECTORY is false.
  void publishGeometricPath() {
    const auto wps = core_->geometricPath();

    visualization_msgs::msg::MarkerArray arr;

    // Clear stale markers first so an empty/failed plan removes the old path.
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = "map";
    clear.header.stamp = now();
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);

    if (wps.size() >= 2) {
      visualization_msgs::msg::Marker line;
      line.header.frame_id = "map";
      line.header.stamp = now();
      line.ns = "geometric_path";
      line.id = 0;
      line.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line.action = visualization_msgs::msg::Marker::ADD;
      line.pose.orientation.w = 1.0;
      line.scale.x = 0.03;  // line width [m]
      line.color.r = 0.1f; line.color.g = 1.0f; line.color.b = 0.2f; line.color.a = 1.0f;

      visualization_msgs::msg::Marker nodes;
      nodes.header = line.header;
      nodes.ns = "geometric_path";
      nodes.id = 1;
      nodes.type = visualization_msgs::msg::Marker::SPHERE_LIST;
      nodes.action = visualization_msgs::msg::Marker::ADD;
      nodes.pose.orientation.w = 1.0;
      nodes.scale.x = nodes.scale.y = nodes.scale.z = 0.12;  // sphere diameter [m]
      nodes.color.r = 0.2f; nodes.color.g = 0.4f; nodes.color.b = 1.0f; nodes.color.a = 1.0f;

      for (const auto& w : wps) {
        geometry_msgs::msg::Point p;
        p.x = w[0]; p.y = w[1]; p.z = w[2];
        line.points.push_back(p);
        nodes.points.push_back(p);
      }
      arr.markers.push_back(line);
      arr.markers.push_back(nodes);
    }

    pub_geom_path_->publish(arr);
  }

  // Debug-only (gated by DEBUG_PLANNER_VIZ): the search tree from the most recent
  // solve as a MarkerArray — a faint LINE_LIST of edges plus small POINTS for the
  // nodes. Lets you watch where the planner explored and A/B PLANNER_TYPE /
  // RRT_SOLVE_TIME by eye. When off this returns before building anything.
  void publishSearchTree() {
    if (!get_parameter("DEBUG_PLANNER_VIZ").as_bool()) return;
    const auto tree = core_->searchTree();

    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = "map";
    clear.header.stamp = now();
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);

    if (!tree.nodes.empty()) {
      visualization_msgs::msg::Marker edges;
      edges.header.frame_id = "map";
      edges.header.stamp = now();
      edges.ns = "search_tree";
      edges.id = 0;
      edges.type = visualization_msgs::msg::Marker::LINE_LIST;
      edges.action = visualization_msgs::msg::Marker::ADD;
      edges.pose.orientation.w = 1.0;
      edges.scale.x = 0.01;  // line width [m]
      edges.color.r = 0.6f; edges.color.g = 0.6f; edges.color.b = 0.6f; edges.color.a = 0.5f;
      for (const auto& e : tree.edges) {
        if (e.first < 0 || e.second < 0) continue;
        const auto& a = tree.nodes[static_cast<std::size_t>(e.first)];
        const auto& b = tree.nodes[static_cast<std::size_t>(e.second)];
        geometry_msgs::msg::Point pa, pb;
        pa.x = a[0]; pa.y = a[1]; pa.z = a[2];
        pb.x = b[0]; pb.y = b[1]; pb.z = b[2];
        edges.points.push_back(pa);
        edges.points.push_back(pb);
      }

      visualization_msgs::msg::Marker nodes;
      nodes.header = edges.header;
      nodes.ns = "search_tree";
      nodes.id = 1;
      nodes.type = visualization_msgs::msg::Marker::POINTS;
      nodes.action = visualization_msgs::msg::Marker::ADD;
      nodes.pose.orientation.w = 1.0;
      nodes.scale.x = nodes.scale.y = 0.04;  // point size [m]
      nodes.color.r = 1.0f; nodes.color.g = 0.7f; nodes.color.b = 0.1f; nodes.color.a = 0.9f;
      for (const auto& n : tree.nodes) {
        geometry_msgs::msg::Point p;
        p.x = n[0]; p.y = n[1]; p.z = n[2];
        nodes.points.push_back(p);
      }
      arr.markers.push_back(edges);
      arr.markers.push_back(nodes);
    }
    pub_search_tree_->publish(arr);
  }

  // Debug-only (gated by DEBUG_PLANNER_VIZ): the EDT clearance field as a
  // PointCloud2 with an `intensity` = distance-to-nearest-obstacle field, so
  // RViz/Foxglove colour it near->far. Shows exactly what the clearance cost
  // sees, for tuning CLEARANCE_WEIGHT / CLEARANCE_THRESHOLD. When off this
  // returns before touching the core.
  void publishClearanceField() {
    if (!get_parameter("DEBUG_PLANNER_VIZ").as_bool()) return;
    const auto samples = core_->clearanceSamples();
    if (samples.empty()) return;

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = "map";
    cloud.header.stamp = now();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2Fields(4,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    mod.resize(samples.size());

    sensor_msgs::PointCloud2Iterator<float> ix(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iy(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iz(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> ii(cloud, "intensity");
    for (const auto& s : samples) {
      *ix = static_cast<float>(s[0]);
      *iy = static_cast<float>(s[1]);
      *iz = static_cast<float>(s[2]);
      *ii = static_cast<float>(s[3]);
      ++ix; ++iy; ++iz; ++ii;
    }
    pub_clearance_field_->publish(cloud);
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
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_geom_path_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_goal_marker_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_search_tree_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_clearance_field_;

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

  Eigen::Vector3d goal_pos_{Eigen::Vector3d::Zero()};
  bool has_goal_{false};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutonomyNode>());
  rclcpp::shutdown();
  return 0;
}
