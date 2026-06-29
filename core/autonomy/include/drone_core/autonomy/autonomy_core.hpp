#pragma once

#include <array>
#include <atomic>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#include "drone_core/common/types.hpp"
#include "drone_core/control/trajectory_tracker.hpp"
#include "drone_core/planning/min_snap_trajectory.hpp"
#include "drone_core/planning/rrt_star_planner.hpp"

// Forward declaration (from dynamicEDT3D) so the cached distance field can be a
// member without pulling that header into this ROS-free public interface.
template <class TREE>
class DynamicEDTOctomapBase;

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
    double rrt_range{1.0};            // RRT* max tree-extension (step size) [m]; <=0 => OMPL auto
    double replan_improve_ratio{0.85};  // adopt candidate iff cost <= ratio*committed
    double clearance_weight{4.0};     // obstacle-proximity penalty weight
    double clearance_threshold{1.0};  // clearance saturation distance / EDT maxdist [m]
    double trajgen_period{1.0};       // local trajectory replan period [s]
    // When false the worker stops after RRT*: it stores the geometric path for
    // visualisation but never runs min-snap or hands a trajectory to the
    // tracker, so control keeps following the direct setpoint. Used to bring the
    // planner online geometry-first, decoupled from control.
    bool plan_trajectory{true};
    // Single switch for the debug planner visualisation (search-tree capture +
    // EDT clearance-field sampling). Off by default so a regular flight pays
    // nothing: with it false the planner never extracts the OMPL tree and the
    // field is never sampled. Safe to flip live for a debugging run.
    bool debug_planner_viz{false};
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

  // Snapshot of the most recent RRT* search tree (nodes + edges), for debug
  // visualisation. Empty unless cfg.debug_planner_viz is set. Thread-safe copy.
  planning::RrtStarPlanner::SearchTree searchTree() const;

  // Coarse samples of the cached clearance (EDT) field as {x, y, z, distance}
  // (distance clamped at clearance_threshold), for debug visualisation. Empty
  // unless cfg.debug_planner_viz is set. Thread-safe copy.
  std::vector<std::array<double, 4>> clearanceSamples() const;

private:
  double now() const { return clock_(); }
  bool runGlobalPlan(const common::State& state, const common::Goal& goal,
                     const planning::MapHandle& map,
                     std::vector<std::vector<double>>& path);
  bool runTrajgen(const std::vector<std::vector<double>>& path, double t0,
                  common::Trajectory& traj);
  void stagePending(const common::Trajectory& traj);
  void plannerLoop();

  // Configure `planner` with the shared clearance-aware objective used by BOTH
  // the monitor and improve passes (build the EDT over `map`, then setClearance
  // with cfg_.clearance_weight / cfg_.clearance_threshold), so a forced replan
  // and an improvement search minimise exactly the same cost. Returns false and
  // leaves the planner length-optimal when the map has no obstacles (clearance
  // is then uniform and the EDT meaningless).
  bool applyClearanceObjective(planning::RrtStarPlanner& planner,
                               const planning::MapHandle& map);

  // Return the cached Euclidean distance field for `map`, rebuilding it only when
  // the map has actually changed (octomap hands us a fresh tree per message). A
  // static scene therefore reuses one EDT instead of rebuilding it every tick.
  // Returns nullptr (and clears the cache) when the map is empty / has no
  // obstacles. Called only from the planner worker thread.
  std::shared_ptr<DynamicEDTOctomapBase<octomap::OcTree>> clearanceField(
      const planning::MapHandle& map);

  // Sample the cached EDT on a coarse grid over the map's bounding box. Called
  // only from the planner worker thread (it reads edt_ / edt_source_map_, which
  // the worker owns), and only when debug_planner_viz is set.
  std::vector<std::array<double, 4>> sampleClearanceField() const;

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
  planning::RrtStarPlanner::SearchTree last_search_tree_;  // debug viz; empty unless enabled
  std::vector<std::array<double, 4>> last_clearance_samples_;  // debug viz; {x,y,z,dist}

  std::thread worker_;
  std::atomic<bool> running_{false};
  std::vector<std::vector<double>> cached_path_;
  double last_trajgen_{-1.0e9};

  // Cached distance field and the map it was built from — the single obstacle
  // model for both collision validity and the clearance cost. See clearanceField.
  std::shared_ptr<DynamicEDTOctomapBase<octomap::OcTree>> edt_;
  planning::MapHandle edt_source_map_;
  planning::MapHandle viz_sampled_map_;  // map the debug clearance samples were taken from
};

}  // namespace drone_core::autonomy
