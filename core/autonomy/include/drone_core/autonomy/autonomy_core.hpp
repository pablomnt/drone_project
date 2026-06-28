#pragma once

#include <atomic>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#include "drone_core/common/types.hpp"
#include "drone_core/control/trajectory_tracker.hpp"
#include "drone_core/planning/min_snap_trajectory.hpp"
#include "drone_core/planning/rrt_star_planner.hpp"

namespace drone_core::autonomy {

// Top-level autonomy object. It owns the whole guidance-to-control pipeline and
// runs it at three cadences: RRT* geometric replanning (slow) and minimum-snap
// trajectory generation (mid) on a background worker thread, and the trajectory
// tracker (fast) on whatever thread calls stepControl(). The fast path never
// blocks on planning; finished trajectories are handed across via an atomic
// swap. The object is deliberately middleware-free so it can be driven by a ROS
// node, a simulation harness, or a unit test interchangeably.
class AutonomyCore {
public:
  struct Config {
    Eigen::Vector3d pos_p{0.95, 0.95, 1.0};
    Eigen::Vector3d vel_p{1.8, 1.8, 2.0};
    Eigen::Vector3d vel_i{0.4, 0.4, 0.5};
    Eigen::Vector3d vel_d{0.2, 0.2, 0.2};
    double hover_thrust{0.35};
    bool enable_feedforward{false};
    double stale_timeout{0.5};        // hover-hold fallback threshold [s]
    double rrt_monitor_period{0.5};   // committed-path validity re-check [s]
    double rrt_improve_period{5.0};   // clearance-aware improvement search [s]
    double rrt_solve_time{3.0};       // RRT* optimisation budget per solve [s]
    double replan_improve_ratio{0.85};  // adopt candidate iff cost <= ratio*committed
    double clearance_weight{4.0};     // obstacle-proximity penalty weight
    double clearance_threshold{1.0};  // clearance saturation distance / EDT maxdist [m]
    double trajgen_period{1.0};       // local trajectory replan period [s]
    // When false the worker stops after RRT*: it stores the geometric path for
    // visualisation but never runs min-snap or hands a trajectory to the
    // tracker, so control keeps following the direct setpoint. Used to bring the
    // planner online geometry-first, decoupled from control.
    bool plan_trajectory{true};
  };

  explicit AutonomyCore(const Config& config);
  ~AutonomyCore();

  AutonomyCore(const AutonomyCore&) = delete;
  AutonomyCore& operator=(const AutonomyCore&) = delete;

  // Replace the time source (e.g. with the ROS clock). Defaults to a monotonic
  // steady clock in seconds.
  void setClock(std::function<double()> clock);

  // Inputs (thread-safe).
  void setState(const common::State& state);
  void setMap(const planning::MapHandle& map);
  void setGoal(const common::Goal& goal);

  // Direct position/yaw setpoint for takeoff and manual hover. A fresh planner
  // trajectory (from setGoal) takes precedence over it.
  void setSetpoint(const Eigen::Vector3d& pos, double yaw);

  // Live reconfiguration of the controller gains, hover thrust and feed-forward
  // flag. Applied on the control thread at the next step.
  void applyConfig(const Config& config);

  // Re-arm the controller on (re)engagement.
  void reset();

  // Fast control step; call at the control rate from a single thread.
  common::Command stepControl(double dt);

  // Background planner lifecycle.
  void startPlanner();
  void stopPlanner();

  // Run one synchronous plan (RRT* + trajectory generation) and stage the
  // result. Returns true if a new trajectory was produced. Useful without the
  // worker thread (tests, manual stepping).
  bool planOnce();

  // Introspection (call from the control thread).
  bool hasTrajectory() const;
  bool inHoverHold() const;
  const control::PositionControl& controller() const { return tracker_.controller(); }

  // Sampled copy of the most recently planned trajectory, for visualisation.
  std::vector<std::vector<double>> sampledPlannedPath(double sample_dt = 0.1) const;

  // Raw waypoints of the most recent RRT* geometric plan (start..goal), for
  // visualisation. Independent of trajectory generation, so it is populated even
  // when plan_trajectory is false.
  std::vector<std::vector<double>> geometricPath() const;

private:
  double now() const { return clock_(); }
  bool runGlobalPlan(const common::State& state, const common::Goal& goal,
                     const planning::MapHandle& map,
                     std::vector<std::vector<double>>& path);
  bool runTrajgen(const std::vector<std::vector<double>>& path, double t0,
                  common::Trajectory& traj);
  void stagePending(const common::Trajectory& traj);
  void plannerLoop();

  Config cfg_;
  std::function<double()> clock_;

  control::TrajectoryTracker tracker_;

  mutable std::mutex io_mutex_;
  common::State state_;
  planning::MapHandle map_;
  common::Goal goal_;
  bool has_goal_{false};
  Eigen::Vector3d direct_pos_{Eigen::Vector3d::Zero()};
  double direct_yaw_{0.0};
  bool has_direct_setpoint_{false};
  Config pending_config_;
  bool config_dirty_{false};

  mutable std::mutex traj_mutex_;
  common::Trajectory pending_;
  bool has_pending_{false};
  common::Trajectory last_planned_;  // retained for visualisation
  std::vector<std::vector<double>> last_geometric_path_;  // raw RRT* result, for viz

  std::thread worker_;
  std::atomic<bool> running_{false};
  std::vector<std::vector<double>> cached_path_;
  double last_monitor_{-1.0e9};
  double last_improve_{-1.0e9};
  double last_trajgen_{-1.0e9};
};

}  // namespace drone_core::autonomy
