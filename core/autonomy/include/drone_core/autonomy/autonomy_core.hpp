#pragma once

#include <array>
#include <atomic>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#include "drone_core/common/types.hpp"
#include "drone_core/control/trajectory_tracker.hpp"
#include "drone_core/planning/corridor.hpp"
#include "drone_core/planning/min_snap_trajectory.hpp"
#include "drone_core/planning/geometric_planner.hpp"

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
    double stale_timeout{1.5};        // hover-hold fallback threshold [s]
    double rrt_monitor_period{0.5};   // committed-path validity re-check [s]
    double rrt_improve_period{5.0};   // clearance-aware improvement search [s]
    double rrt_solve_time{3.0};       // planner optimisation budget per solve [s]
    // Which OMPL planner the worker builds. Per-planner tunables (RRT* range/goal
    // bias, BIT*/AIT*/EIT* batch sizes, etc.) live in PlannerConfig in
    // geometric_planner.hpp, the single place to tune them.
    planning::PlannerType planner_type{planning::PlannerType::RRTstar};
    double replan_improve_ratio{0.85};  // adopt candidate iff cost <= ratio*committed
    // Best-effort goal seeking. When true, a goal in unreachable/unmapped space no
    // longer yields "no path": the planner returns a route to the reachable point
    // closest to the goal (the frontier edge), and the worker commits it and keeps
    // advancing the endpoint as new space is mapped and the frontier recedes — the
    // drone gets as close as it can and ratchets forward. When false the planner
    // insists on (near-)exact arrival and holds if the goal is unreachable.
    bool best_effort_goal{true};
    double clearance_weight{4.0};     // obstacle-proximity penalty weight
    double clearance_threshold{1.0};  // clearance saturation distance / EDT maxdist [m]
    // Flat extra cost charged per metre of path routed through space that has
    // never been observed. This is NOT expressible as a clearance weight: the
    // distance field saturates at clearance_threshold, so every point further
    // than that from a mapped obstacle scores a proximity penalty of exactly
    // zero — including the whole interior of unmapped space and everything
    // outside the field's bounding box. Stamping the frontier buys a gradient
    // near the shell only, and the shell has gaps, so without this term the
    // cheapest route to a distant goal can be to leave mapped space entirely and
    // fly there through the unknown. Read as "how many metres of detour through
    // mapped space is one metre through unmapped space worth". 0 disables it.
    //
    // Only applied when the host supplies a conservative map view — i.e. with
    // TREAT_FRONTIER_AS_OBSTACLE off this term is skipped entirely, since that
    // flag is the host declaring unmapped space is not a hazard here.
    double unknown_weight{10.0};
    double trajgen_period{1.0};       // local trajectory replan period [s]
    // Corridor-QP trajectory generation (Stage 1). When true, runTrajgen
    // replaces plain min-snap with the safe-corridor pipeline: truncate the
    // committed path against the conservative map view (see setMap) so it never
    // commits into unknown space, grow a convex polyhedral free corridor along
    // the safe prefix, and solve the corridor-constrained min-snap QP under the
    // per-axis limits below. Any stage failing stages NOTHING — there is no
    // min-snap fallback here, since min-snap ignores obstacles and the only
    // thing that produces this path is the corridor reporting it cannot certify
    // a safe trajectory. The tracker rides out what it has and hovers at
    // stale_timeout. When false, trajgen is exactly the pre-corridor min-snap.
    bool use_corridor_qp{false};
    double vmax{1.0};              // per-axis velocity limit [m/s]
    double amax{1.5};              // per-axis acceleration limit [m/s^2]
    double jmax{3.0};              // per-axis jerk limit [m/s^3]
    // Required clearance from unknown space for the committed trajectory [m].
    // Enforced by truncation against the conservative (frontier-stamped) map;
    // may exceed the collision margin (unknown is riskier than a mapped wall).
    double frontier_margin{0.5};
    // Clearance the corridor boxes must have from obstacles AND unknown space
    // [m]. This is a strictly harder test than the planner's collision margin:
    // the search only checks its centreline (and exempts a sphere at the
    // start), whereas box growth needs this clearance over a whole 3D region.
    // So a path that only just satisfies the search can leave no room to grow a
    // box, and this must usually sit BELOW the planner's margin for the corridor
    // to be constructible at all — the trajectory is still confined to the
    // resulting boxes, so it stays this far from anything mapped or unknown.
    double corridor_margin{0.4};
    // Distance over which truncation's required clearance ramps from 0 at the
    // drone up to the full frontier_margin [m]. Decoupled from the margin on
    // purpose: ramping over the margin itself makes the requirement rise at
    // 1 m/m, which on a thinly-mapped scene meets the shrinking clearance
    // within centimetres and truncates the committed path to nothing. Longer =
    // commits further before demanding full clearance. <= 0 disables the ramp.
    double escape_ramp_dist{1.0};
    double max_segment_len{2.0};   // corridor resample cap: one region per piece [m]
    // Minimum half-extents of the region-growth window in the SEGMENT-ALIGNED
    // frame (x along the segment, y/z lateral) — not world axes. A floor, not a
    // cap: buildCorridor raises it to scale with the longest segment. Pinning
    // it keeps the window from narrowing when max_segment_len is lowered, which
    // would otherwise trade a region's length for its width.
    Eigen::Vector3d corridor_bbox{1.0, 2.0, 2.0};
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
  // Feed the occupancy map(s) — the dual-map view of the corridor pipeline.
  // `map` is the raw (optimistic) map: unknown space reads as free, so the
  // geometric search can chase a goal beyond the mapped frontier (informed
  // planners refuse an unreachable goal outright). `conservative` is the
  // frontier-stamped copy — unknown space reads as occupied — used for
  // truncation, corridor growth and trajectory safety; pass nullptr when no
  // frontier information is available (the raw map then serves both roles and
  // nothing guards against unknown space, matching the pre-frontier behavior).
  // With use_corridor_qp off and a conservative map present, the search runs on
  // the conservative view instead — the legacy single-map behavior of
  // TREAT_FRONTIER_AS_OBSTACLE.
  void setMap(const planning::MapHandle& map,
              const planning::MapHandle& conservative = nullptr);
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
  planning::GeometricPlanner::SearchTree searchTree() const;

  // Coarse samples of the cached clearance (EDT) field as {x, y, z, distance}
  // (distance clamped at clearance_threshold), for debug visualisation. Empty
  // unless cfg.debug_planner_viz is set. Thread-safe copy.
  std::vector<std::array<double, 4>> clearanceSamples() const;

  // Snapshot of the most recent corridor-QP trajgen, for debug visualisation.
  // `committed` is the TRUNCATED prefix actually handed to the trajectory
  // solver — its last point is the intermediate goal inside known-safe space,
  // which ratchets toward the real goal as the map grows — and `regions` are
  // the convex free polyhedra grown along it (one per resampled segment), i.e.
  // the volume the trajectory is provably confined to. Comparing `committed`
  // against geometricPath() shows exactly where truncation cut the optimistic
  // path. Empty unless cfg.debug_planner_viz AND cfg.use_corridor_qp are set,
  // and cleared whenever a trajgen tick truncates to nothing or fails to build
  // a corridor, so a stale corridor is never drawn as if it were current.
  // Thread-safe copy.
  // Deliberately populated on FAILED ticks too, stage by stage, because a
  // failing corridor is what you actually need to look at: a tick that only
  // cleared the drawing left "viz broken", "flag off" and "failing every
  // cycle" indistinguishable on screen. `committed` is filled as soon as
  // truncation succeeds; `raw` and `shrunk` as soon as the decomposition runs,
  // whether or not the result was accepted. `accepted` says whether this
  // corridor was actually used — when false, `shrunk` is the geometry that was
  // rejected. Drawing `raw` against `shrunk` makes the usual failure obvious
  // at a glance: raw has volume, shrunk has none, so the margin ate it.
  struct CorridorSnapshot {
    std::vector<Eigen::Vector3d> committed;
    std::vector<planning::ConvexRegion> raw;     // as decomposed, before the margin
    std::vector<planning::ConvexRegion> shrunk;  // after the margin pull-in
    bool accepted{false};
  };
  CorridorSnapshot corridorSnapshot() const;

private:
  double now() const { return clock_(); }
  bool runGlobalPlan(const common::State& state, const common::Goal& goal,
                     const planning::MapHandle& map,
                     std::vector<std::vector<double>>& path);
  // Trajectory generation for the committed path. cons_edt is the distance
  // field of the conservative map view and cons_map the octree it was built
  // from (nullptr when unavailable): with use_corridor_qp set the field drives
  // truncation and the octree supplies the windowed obstacle points the
  // polyhedral corridor decomposition consumes; without them (or with the flag
  // off) trajgen is plain min-snap over the waypoints.
  //
  // is_unknown, when non-empty, makes truncation stop at the first point in
  // never-observed space — a check the distance field cannot make, because the
  // stamped shell it measures against has gaps. The CALLER decides whether to
  // supply it, and supplies it only when a conservative map view exists (i.e.
  // TREAT_FRONTIER_AS_OBSTACLE is on). Passed as the predicate rather than as a
  // map handle so that decision is visible at the call site next to the
  // `conservative` handle it depends on, instead of being re-derived here from
  // which map objects happen to be distinct.
  bool runTrajgen(const std::vector<std::vector<double>>& path, double t0,
                  const std::shared_ptr<DynamicEDTOctomapBase<octomap::OcTree>>& cons_edt,
                  const planning::MapHandle& cons_map,
                  const planning::CorridorUnknownFn& is_unknown,
                  common::Trajectory& traj);
  void stagePending(const common::Trajectory& traj);
  void plannerLoop();

  // Configure `planner` with the shared clearance-aware objective used by BOTH
  // the monitor and improve passes, so a forced replan and an improvement
  // search minimise exactly the same cost.
  //
  // `map` is the search view. Its EDT drives the planner's collision check and
  // supplies the weight/threshold from cfg_. `cons` is the conservative
  // (frontier-stamped) view, or null when there is none; when it is a distinct
  // object from `map` its EDT additionally scores the cost, which is what stops
  // the search returning paths that run tangent to the frontier and get cut to
  // almost nothing by truncation. Validity is never scored against `cons` —
  // that would make the stamped frontier impassable.
  //
  // `cons` being non-null is also what enables the unknown-space surcharge
  // (unknown_weight): a null one means the host has TREAT_FRONTIER_AS_OBSTACLE
  // off and does not want unmapped space treated as a hazard, so the cost stays
  // out of it. This gates the cost only — truncation's stop at unobserved space
  // is a safety property and holds regardless.
  //
  // Returns false and leaves the planner length-optimal when the search map has
  // no obstacles (clearance is then uniform and the EDT meaningless).
  bool applyClearanceObjective(planning::GeometricPlanner& planner,
                               const planning::MapHandle& map,
                               const planning::MapHandle& cons);

  // Return the cached Euclidean distance field for `map`, rebuilding it only when
  // the map has actually changed (octomap hands us a fresh tree per message). A
  // static scene therefore reuses one EDT instead of rebuilding it every tick.
  // Returns nullptr (and clears the cache) when the map is empty / has no
  // obstacles. Called only from the planner worker thread.
  std::shared_ptr<DynamicEDTOctomapBase<octomap::OcTree>> clearanceField(
      const planning::MapHandle& map);

  // Cached distance field over the conservative map view, for truncation and
  // corridor growth. Mirrors clearanceField's rebuild-on-map-change caching;
  // when the conservative and search maps are the same object (no frontier
  // information), the search field is reused instead of building a second EDT.
  // Worker thread only.
  std::shared_ptr<DynamicEDTOctomapBase<octomap::OcTree>> conservativeField(
      const planning::MapHandle& map);

  // Sample the cached EDT on a coarse grid over the map's bounding box. Samples
  // whichever field the cost objective is scored against — the conservative one
  // when it is a distinct view, the search one otherwise — so the published
  // cloud shows what the objective actually sees. Called only from the planner
  // worker thread (it reads edt_ / cons_edt_ and their source maps, which the
  // worker owns), and only when debug_planner_viz is set.
  std::vector<std::array<double, 4>> sampleClearanceField() const;

  Config cfg_;
  std::function<double()> clock_;

  control::TrajectoryTracker tracker_;

  mutable std::mutex io_mutex_;
  common::State state_;
  planning::MapHandle map_;               // optimistic view (unknown = free)
  planning::MapHandle conservative_map_;  // frontier-stamped view; may be null
  common::Goal goal_;
  bool has_goal_{false};
  bool new_goal_{false};  // raised by setGoal, consumed by the worker to force a replan
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
  planning::GeometricPlanner::SearchTree last_search_tree_;  // debug viz; empty unless enabled
  std::vector<std::array<double, 4>> last_clearance_samples_;  // debug viz; {x,y,z,dist}
  CorridorSnapshot last_corridor_;  // debug viz; empty unless corridor QP + viz on

  std::thread worker_;
  std::atomic<bool> running_{false};
  std::vector<std::vector<double>> cached_path_;
  double last_trajgen_{-1.0e9};

  // Cached distance field and the map it was built from — the single obstacle
  // model for both collision validity and the clearance cost. See clearanceField.
  std::shared_ptr<DynamicEDTOctomapBase<octomap::OcTree>> edt_;
  planning::MapHandle edt_source_map_;
  // Second cached field over the conservative map view (truncation + corridor).
  // See conservativeField.
  std::shared_ptr<DynamicEDTOctomapBase<octomap::OcTree>> cons_edt_;
  planning::MapHandle cons_edt_source_map_;
  planning::MapHandle viz_sampled_map_;  // map the debug clearance samples were taken from
};

}  // namespace drone_core::autonomy
