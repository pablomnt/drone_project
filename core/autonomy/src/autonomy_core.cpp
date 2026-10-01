#include "drone_core/autonomy/autonomy_core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include <ompl/util/Console.h>

#include "drone_core/common/logging.hpp"
#include "drone_core/common/rigid_transform.hpp"
#include "drone_core/common/trajectory_eval.hpp"
#include "drone_core/control/flatness_mapper.hpp"
#include "drone_core/planning/corridor.hpp"
#include "drone_core/planning/corridor_trajectory.hpp"

namespace drone_core::autonomy {

namespace {

// How often the planner worker re-reports a persistent "cannot plan" state [s].
// Slow enough not to bury the rest of the log, fast enough that a stalled bench
// run explains itself while you are watching it.
constexpr double kIdleLogPeriod = 5.0;
// Fraction of FRONTIER_MARGIN truncation lets a point fall short by. The search
// and truncation check the same points against the same ramp, but not quite
// identically: the root is moved to the splice point, which shifts the first
// segment's samples, and the map may have updated since the search. Without
// slack a path the search accepted by a hair is cut a few millimetres later.
constexpr double kTruncationTolerance = 0.05;

double steadyNowSeconds() {
  const auto t = std::chrono::steady_clock::now().time_since_epoch();
  return std::chrono::duration<double>(t).count();
}

// Build the Euclidean distance field over the occupancy octree's bounding box,
// capped at maxdist (metres). getDistance() then gives O(1) clearance lookups.
// Points outside the box read negative, which makeClearanceFn turns into "no
// obstacle within maxdist", matching the planner's treat-unknown-as-free policy.
// The search box grown by `pad` [m] on every side, MAP frame.
void searchBoxGrown(double pad, Eigen::Vector3d& lo, Eigen::Vector3d& hi) {
  for (int i = 0; i < 3; ++i) {
    lo(i) = planning::GeometricPlanner::kSearchLow[i] - pad;
    hi(i) = planning::GeometricPlanner::kSearchHigh[i] + pad;
  }
}

// Cropped to the search box grown by maxdist plus a voxel: nothing plans outside
// the box, obstacles just outside it still count, and map beyond costs nothing.
// 4 threads: nearly as fast as all 8 on the NUC, leaving room for VIO and SLAM.
std::shared_ptr<const planning::DistanceField> buildEdt(
    const std::shared_ptr<octomap::OcTree>& tree, double maxdist) {
  Eigen::Vector3d lo, hi;
  searchBoxGrown(maxdist + tree->getResolution(), lo, hi);
  return std::make_shared<const planning::DistanceField>(*tree, maxdist, /*threads=*/4, lo, hi);
}

// The conservative grid is already cropped by the host (AutonomyCore::mapCrop).
std::shared_ptr<const planning::DistanceField> buildEdt(const ConsGridHandle& grid,
                                                        double maxdist) {
  return std::make_shared<const planning::DistanceField>(*grid, maxdist, /*threads=*/4);
}

// Wrap a distance field as a clearance oracle. The planner's objective, the
// planner's validity check and the corridor stages all take the same
// std::function signature and all need the same out-of-bounds convention:
// getDistance returns a negative sentinel outside the field's bounding box,
// which means nothing is mapped nearby, so report the saturation distance
// instead. Shared rather than written out at each call site so the convention
// cannot drift apart between them.
// Arc length along a polyline [m] at the point closest to `q`: where on the
// committed path a stop point sits, so two stop points can be compared as
// progress along it.
double arcLengthAlong(const std::vector<std::vector<double>>& path, const Eigen::Vector3d& q) {
  if (path.empty()) return 0.0;
  double best = std::numeric_limits<double>::infinity();
  double best_arc = 0.0;
  double acc = 0.0;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const Eigen::Vector3d a(path[i][0], path[i][1], path[i][2]);
    const Eigen::Vector3d b(path[i + 1][0], path[i + 1][1], path[i + 1][2]);
    const Eigen::Vector3d ab = b - a;
    const double len = ab.norm();
    const double u = len > 1e-9 ? std::clamp((q - a).dot(ab) / (len * len), 0.0, 1.0) : 0.0;
    const double d = (a + u * ab - q).norm();
    if (d < best) {
      best = d;
      best_arc = acc + u * len;
    }
    acc += len;
  }
  return best_arc;
}

// Distance [m] from `q` to the nearest point of a polyline.
double distanceToPath(const std::vector<std::vector<double>>& path, const Eigen::Vector3d& q) {
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const Eigen::Vector3d a(path[i][0], path[i][1], path[i][2]);
    const Eigen::Vector3d ab = Eigen::Vector3d(path[i + 1][0], path[i + 1][1], path[i + 1][2]) - a;
    const double len2 = ab.squaredNorm();
    const double u = len2 > 1e-12 ? std::clamp((q - a).dot(ab) / len2, 0.0, 1.0) : 0.0;
    best = std::min(best, (a + u * ab - q).norm());
  }
  return path.size() == 1
             ? (Eigen::Vector3d(path[0][0], path[0][1], path[0][2]) - q).norm()
             : best;
}

planning::CorridorClearanceFn makeClearanceFn(std::shared_ptr<const planning::DistanceField> edt,
                                              double maxd) {
  return [edt = std::move(edt), maxd](double x, double y, double z) {
    const double d = edt->getDistance(octomap::point3d(
        static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)));
    return d < 0.0 ? maxd : d;
  };
}

// A clearance for a test that wants `margin` from mapped obstacles but only
// `margin - relief` from never-observed space: min(d_obstacles, d_all + relief),
// with d_all the conservative field (obstacles and the unknown shell) and
// d_obstacles the raw map's (`obstacles`, null when nothing is mapped: then as
// far as it could say). Held to `margin`, that is exactly d_obstacles >= margin
// and d_all >= margin - relief. relief <= 0 gives the conservative field as is.
planning::CorridorClearanceFn makeSplitClearanceFn(
    planning::CorridorClearanceFn all, std::shared_ptr<const planning::DistanceField> obstacles,
    double obstacles_maxd, double relief) {
  if (!(relief > 0.0)) return all;
  if (!obstacles) {
    return [all = std::move(all), obstacles_maxd, relief](double x, double y, double z) {
      return std::min(obstacles_maxd, all(x, y, z) + relief);
    };
  }
  return [all = std::move(all), obs = makeClearanceFn(std::move(obstacles), obstacles_maxd),
          relief](double x, double y, double z) {
    return std::min(obs(x, y, z), all(x, y, z) + relief);
  };
}

// "Has this point ever been observed?" as a predicate over the octree. A cell
// with no node has never been touched by a sensor ray, which is exactly the
// question, and search() is an O(log n) descent with no allocation. Correct
// outside the map's bounding box too, where every query returns null.
//
// This exists because a distance field structurally cannot answer it: the EDT
// saturates at its maxdist, so unknown space more than maxdist from a mapped
// obstacle is indistinguishable from wide-open free space. The frontier-stamped
// view narrows the gap but does not close it — the stamped shell is only as
// complete as the sensor's coverage, and the host deliberately leaves an
// unstamped ball around the vehicle. Asking the tree directly has no such holes.
planning::CorridorUnknownFn makeUnknownFn(planning::MapHandle map) {
  return [map = std::move(map)](double x, double y, double z) {
    return map->search(octomap::point3d(static_cast<float>(x), static_cast<float>(y),
                                        static_cast<float>(z))) == nullptr;
  };
}

// "Never observed" for truncation and the trajectory monitor: from the
// conservative grid when there is one (the ball around the drone free; shell,
// unobserved cells and anything outside the grid unknown), so it agrees with the
// corridor, whose obstacles come from the same grid; else from the raw octree.
planning::CorridorUnknownFn makeUnknownFn(const planning::MapHandle& map,
                                          const ConsGridHandle& grid) {
  if (grid) {
    return [grid](double x, double y, double z) { return grid->isUnknown(x, y, z); };
  }
  return makeUnknownFn(map);
}

// Remaining committed path from the drone's current position. Splits the committed
// polyline at the drone by exact projection onto each segment (closed-form point-to-
// segment, clamped to the segment), then returns [drone_pos, wp[i+1], ..., goal]
// where segment i is the nearest one — i.e. the drone's actual position followed by
// every waypoint still ahead of it. Lets an improve candidate (also rooted at the
// drone) be scored against what remains of the committed path from here, not its
// full original cost. With 4-5 sparse waypoints the projection point lands mid-
// segment, so nearest-vertex would misjudge progress; hence the segment projection.
std::vector<std::vector<double>> remainingCommittedSuffix(
    const std::vector<std::vector<double>>& path,
    const std::vector<double>& start) {
  if (path.size() < 2) return path;
  const double px = start[0], py = start[1], pz = start[2];
  double best_d2 = std::numeric_limits<double>::infinity();
  std::size_t best_seg = 0;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const double ax = path[i][0], ay = path[i][1], az = path[i][2];
    const double abx = path[i + 1][0] - ax, aby = path[i + 1][1] - ay, abz = path[i + 1][2] - az;
    const double denom = abx * abx + aby * aby + abz * abz;
    double t = denom > 0.0 ? ((px - ax) * abx + (py - ay) * aby + (pz - az) * abz) / denom : 0.0;
    t = std::clamp(t, 0.0, 1.0);
    const double qx = ax + t * abx, qy = ay + t * aby, qz = az + t * abz;
    const double dx = px - qx, dy = py - qy, dz = pz - qz;
    const double d2 = dx * dx + dy * dy + dz * dz;
    if (d2 < best_d2) {
      best_d2 = d2;
      best_seg = i;
    }
  }
  // Waypoints up to and including best_seg are behind the drone; root the remainder
  // at the drone's actual position.
  std::vector<std::vector<double>> suffix;
  suffix.reserve(path.size() - best_seg);
  suffix.push_back(start);
  for (std::size_t j = best_seg + 1; j < path.size(); ++j) suffix.push_back(path[j]);
  return suffix;
}

}  // namespace

AutonomyCore::AutonomyCore(const Config& config)
    : cfg_(config),
      search_cfg_(config),
      control_cfg_(config),
      clock_(steadyNowSeconds),
      // setMap reads the field saturation distances from here to prebuild the
      // fields, so it must hold the real config from the start rather than
      // defaults — otherwise every prebuilt field would miss and the planner
      // threads would rebuild it themselves, which is the stall prebuilding
      // exists to remove.
      pending_config_(config) {
  monitor_cfg_ = config;
  tracker_.setPositionGains(control_cfg_.pos_p);
  tracker_.setVelocityGains(control_cfg_.vel_p, control_cfg_.vel_i, control_cfg_.vel_d);
  tracker_.setDerivativeTau(control_cfg_.vel_d_tau);
  tracker_.setIntegratorErrorLimit(control_cfg_.int_err_limit);
  tracker_.setHoverThrust(control_cfg_.hover_thrust);
  tracker_.enableFeedforward(control_cfg_.enable_feedforward);
  tracker_.setHealthTimeout(control_cfg_.health_timeout);
  tracker_.setMaxTrackingError(control_cfg_.max_tracking_error);

  // Quiet OMPL's own console (the per-solve "RRTstar: ..." INFO/DEBUG spam) so the
  // terminal shows our planner summary; warnings and errors still come through.
  ompl::msg::setLogLevel(ompl::msg::LOG_WARN);
}

AutonomyCore::~AutonomyCore() {
  stopPlanner();
}

void AutonomyCore::setClock(std::function<double()> clock) {
  clock_ = std::move(clock);
}

void AutonomyCore::setVehicleState(const common::State& state) {
  std::lock_guard<std::mutex> lock(io_mutex_);
  state_ = state;
}

void AutonomyCore::setMap(const planning::MapHandle& map, const ConsGridHandle& conservative) {
  // Fields first, map second: once a planner can see this map, the field built
  // from it is already installed, so no planner tick ever has to build one.
  prebuildFields(map, conservative);
  std::lock_guard<std::mutex> lock(io_mutex_);
  map_ = map;
  conservative_map_ = conservative;
}

namespace {

// Whether `map` is one of the maps a cached field has been superseded for.
// Owner comparison, so an expired entry never matches and a freed map's address
// being reused by a new map cannot either.
template <class T>
bool wasSuperseded(const std::vector<std::weak_ptr<T>>& history,
                   const std::shared_ptr<T>& map) {
  for (const auto& w : history) {
    if (!w.owner_before(map) && !map.owner_before(w)) return true;
  }
  return false;
}

template <class T>
void pushSuperseded(std::vector<std::weak_ptr<T>>& history,
                    const std::shared_ptr<T>& old_source, std::size_t cap) {
  if (!old_source) return;
  history.erase(std::remove_if(history.begin(), history.end(),
                               [](const std::weak_ptr<T>& w) { return w.expired(); }),
                history.end());
  history.push_back(old_source);
  if (history.size() > cap) history.erase(history.begin());
}

}  // namespace

void AutonomyCore::prebuildFields(const planning::MapHandle& map,
                                  const ConsGridHandle& conservative) {
  // The same saturation distances the planners will ask for (clearanceField from
  // the search, conservativeField from truncation/corridor and the search's cost).
  // Read from the newest config; a thread still on an older copy for a tick just
  // takes the fallback build in the accessor, as any maxdist change always has.
  double search_md = 0.0, cons_md = 0.0;
  {
    std::lock_guard<std::mutex> lock(io_mutex_);
    search_md = pending_config_.clearance_threshold;
    cons_md = std::max(pending_config_.clearance_threshold, pending_config_.frontier_margin);
  }

  // Built with NO lock held — the whole point. Installation is a pointer swap.
  if (map && map->size() > 0) {
    bool have = false;
    {
      std::lock_guard<std::mutex> lock(edt_mutex_);
      have = edt_ && edt_source_map_ == map && edt_maxdist_ == search_md;
    }
    if (!have) {
      auto field = buildEdt(map, search_md);
      std::lock_guard<std::mutex> lock(edt_mutex_);
      if (edt_source_map_ != map) pushSuperseded(edt_superseded_, edt_source_map_, kSupersededHistory);
      edt_ = std::move(field);
      edt_source_map_ = map;
      edt_maxdist_ = search_md;
      viz_sampled_map_.reset();  // debug clearance samples are of the old field
    }
  }
  // Only a conservative grid needs its own field; without one (no frontier
  // information) conservativeField hands back the search one.
  if (conservative && !conservative->empty()) {
    bool have = false;
    {
      std::lock_guard<std::mutex> lock(edt_mutex_);
      have = cons_edt_ && cons_edt_source_grid_ == conservative && cons_edt_maxdist_ == cons_md;
    }
    if (!have) {
      auto field = buildEdt(conservative, cons_md);
      std::lock_guard<std::mutex> lock(edt_mutex_);
      if (cons_edt_source_grid_ != conservative) {
        pushSuperseded(cons_grid_superseded_, cons_edt_source_grid_, kSupersededHistory);
      }
      cons_edt_ = std::move(field);
      cons_edt_source_grid_ = conservative;
      cons_edt_source_map_.reset();
      cons_edt_maxdist_ = cons_md;
    }
  }
}

void AutonomyCore::mapCrop(Eigen::Vector3d& lo, Eigen::Vector3d& hi) const {
  double pad = 0.0;
  {
    std::lock_guard<std::mutex> lock(io_mutex_);
    pad = std::max(pending_config_.clearance_threshold, pending_config_.frontier_margin);
  }
  searchBoxGrown(pad + 0.1, lo, hi);  // + a voxel or two at any resolution we use
}

void AutonomyCore::setMapToWorld(const Eigen::Isometry3d& world_from_map) {
  std::lock_guard<std::mutex> lock(io_mutex_);
  world_from_map_ = world_from_map;
  has_map_to_world_ = true;
}

void AutonomyCore::setGoal(const common::Goal& goal) {
  std::lock_guard<std::mutex> lock(io_mutex_);
  goal_ = goal;
  has_goal_ = true;
  // Force a fresh geometric plan toward the new goal from the drone's current
  // position. Only raise a flag here (under io_mutex_) rather than touching
  // cached_path_ directly: that path is owned by the worker thread. Clearing it
  // here races with the worker's adopt() and can be overwritten by an in-flight
  // solve for the previous goal, leaving a stale, still-collision-valid path
  // rooted at the position where the old goal was issued (the monitor keeps it
  // because it only checks for collisions, not the target). The worker consumes
  // the flag and clears the path on its own thread, so a goal that lands
  // mid-tick is honored on the next cycle instead of being lost.
  new_goal_ = true;
}

void AutonomyCore::setSetpoint(const Eigen::Vector3d& pos, double yaw) {
  std::lock_guard<std::mutex> lock(io_mutex_);
  direct_pos_ = pos;
  direct_yaw_ = yaw;
  has_direct_setpoint_ = true;
}

void AutonomyCore::firePreset(const std::vector<Eigen::Vector3d>& waypoints) {
  std::lock_guard<std::mutex> lock(io_mutex_);
  preset_waypoints_ = waypoints;
  preset_pending_ = true;
  // A preset square is an explicit override of planning: drop any active goal so
  // the worker does not resume replanning toward it after the preset finishes and
  // yank the vehicle off the POS_SP hold it just returned to. There is no separate
  // goal-cancel path, so this is where a live goal gets cleared.
  has_goal_ = false;
}

void AutonomyCore::applyConfig(const Config& config) {
  std::lock_guard<std::mutex> lock(io_mutex_);
  pending_config_ = config;
  search_config_dirty_ = true;
  trajgen_config_dirty_ = true;
  monitor_config_dirty_ = true;
  control_config_dirty_ = true;
}

void AutonomyCore::reset() {
  tracker_.reset();
  // Any in-flight preset is abandoned on a re-engage (disarm / leaving offboard):
  // the tracker just dropped its trajectories, so keeping the keep-alive alive
  // would re-stamp freshness on nothing. preset_pending_ is under io_mutex_.
  preset_active_.store(false);
  {
    std::lock_guard<std::mutex> lock(io_mutex_);
    preset_pending_ = false;
  }
  // The tracker just dropped its trajectories, so there is nothing left to
  // splice onto. Clearing this makes the next replan anchor at rest on the
  // measured position, which is what a fresh engage needs; anything solved or
  // waiting against the old ones is thrown away (splice_epoch_).
  splice_epoch_.fetch_add(1);
  std::lock_guard<std::mutex> lock(traj_mutex_);
  has_pending_ = false;
  has_last_planned_ = false;
  last_planned_ = common::Trajectory{};
  records_.clear();
}

common::Command AutonomyCore::stepControl(double dt) {
  const double t = now();

  common::State state;
  Eigen::Vector3d direct_pos;
  double direct_yaw = 0.0;
  bool has_direct = false;
  Config config;
  bool config_dirty = false;
  {
    std::lock_guard<std::mutex> lock(io_mutex_);
    state = state_;
    direct_pos = direct_pos_;
    direct_yaw = direct_yaw_;
    has_direct = has_direct_setpoint_;
    config_dirty = control_config_dirty_;
    if (config_dirty) config = pending_config_;
    control_config_dirty_ = false;
  }

  if (config_dirty) {
    // Capture this BEFORE cfg_ is overwritten: hover thrust is the one field in
    // this block that is not merely a gain but also live estimator state, so
    // re-pushing an unchanged value is not a no-op — it wipes a converged
    // estimate. The node's onParameterChange pushes the WHOLE config on any
    // parameter write, so without this guard, setting POS_SP (or anything else)
    // reset the learned hover thrust to MPC_HOVER_THRUST on the next tick and the
    // estimator had to re-converge over its 2.5 s time constant every time.
    // Comparing keeps MPC_HOVER_THRUST working as a deliberate operator override
    // while leaving the estimator alone the rest of the time.
    const bool hover_thrust_changed = (control_cfg_.hover_thrust != config.hover_thrust);
    control_cfg_ = config;
    tracker_.setPositionGains(control_cfg_.pos_p);
    tracker_.setVelocityGains(control_cfg_.vel_p, control_cfg_.vel_i, control_cfg_.vel_d);
    tracker_.setDerivativeTau(control_cfg_.vel_d_tau);
    tracker_.setIntegratorErrorLimit(control_cfg_.int_err_limit);
    if (hover_thrust_changed) tracker_.setHoverThrust(control_cfg_.hover_thrust);
    tracker_.enableFeedforward(control_cfg_.enable_feedforward);
    tracker_.setHealthTimeout(control_cfg_.health_timeout);
    tracker_.setMaxTrackingError(control_cfg_.max_tracking_error);
  }

  // The tracker is only ever touched from this control thread, so apply the
  // staged direct setpoint and pick up a freshly planned trajectory here.
  if (has_direct) {
    tracker_.setDirectSetpoint(direct_pos, direct_yaw);
  }
  {
    std::lock_guard<std::mutex> lock(traj_mutex_);
    if (has_pending_) {
      tracker_.setTrajectory(pending_, t);
      has_pending_ = false;
    }
  }

  // Health signal from the trajectory monitor: the trajectory being flown was
  // just re-checked against the current map and is safe.
  if (heartbeat_.exchange(false)) tracker_.keepFresh(t);

  // Emergency stop from the monitor: hold here now. The monitor has already
  // dropped everything staged or solved against the abandoned trajectory, so the
  // next one starts at rest from where the vehicle stops.
  if (emergency_request_.exchange(false)) tracker_.emergencyStop();

  // A preset one-shot is solved once and never replanned, so the monitor never
  // checks it and sends no health signals for it. Keep it fresh here for its
  // whole duration so it holds kTracking instead of falling to hover-hold at
  // health_timeout (the trajectory plays in absolute time, so this
  // only defers the planner-death failsafe, which does not apply to a deliberate
  // one-shot). Once it has run its course, release the trajectory so control
  // drops back to the direct setpoint (POS_SP) rather than latching a hover.
  if (preset_active_.load()) {
    if (t < preset_end_.load()) {
      tracker_.keepFresh(t);
    } else {
      tracker_.clearTrajectory();
      preset_active_.store(false);
    }
  }

  const common::Command cmd = tracker_.update(state, t, dt);
  // Hovering over a trajectory it has stopped following (health timeout,
  // emergency, divergence): the next solve must start at rest, not splice.
  tracker_holding_.store(tracker_.mode() == control::TrajectoryTracker::Mode::kHoverHold &&
                         tracker_.hasTrajectory());

  // The tracker has just abandoned its trajectory because the vehicle got too far
  // from the reference (see TrajectoryTracker::isDiverged). Once, on that edge:
  // forget the trajectory as a splice source, so the next plan starts at rest at
  // the vehicle's real position like a fresh engage rather than at wherever the
  // abandoned reference has got to; drop any trajectory the worker already
  // staged against it; and ask the worker for a new geometric path from here,
  // since the committed one was laid out from where the vehicle no longer is.
  // Edge-triggered on purpose: clearing on every held tick would also discard
  // the recovery trajectory staged while the tracker is still holding, and the
  // plan after that would then start from rest while the vehicle is moving.
  // A health timeout is handled the same way: the tracker is now holding where the
  // vehicle is, so a plan spliced onto the trajectory it abandoned would start
  // ahead of the hold point and step the reference when it took over (scratch
  // run 2026-09-29: 0.36 m).
  const bool lost_reference = tracker_.takeDivergence();
  const bool health_lapsed = tracker_.takeHealthTimeout();
  if (lost_reference || health_lapsed) {
    splice_epoch_.fetch_add(1);
    {
      std::lock_guard<std::mutex> lock(traj_mutex_);
      has_pending_ = false;
      has_last_planned_ = false;
      last_planned_ = common::Trajectory{};
      records_.clear();
    }
    search_replan_requested_.store(true);
    trajgen_replan_requested_.store(true);
  }

  return cmd;
}

void AutonomyCore::startPlanner() {
  if (running_.exchange(true)) return;
  // Separate threads on purpose — see searchLoop/trajgenLoop/monitorLoop.
  search_worker_ = std::thread(&AutonomyCore::searchLoop, this);
  trajgen_worker_ = std::thread(&AutonomyCore::trajgenLoop, this);
  monitor_worker_ = std::thread(&AutonomyCore::monitorLoop, this);
}

void AutonomyCore::stopPlanner() {
  if (!running_.exchange(false)) return;
  if (search_worker_.joinable()) search_worker_.join();
  if (trajgen_worker_.joinable()) trajgen_worker_.join();
  if (monitor_worker_.joinable()) monitor_worker_.join();
}

std::vector<std::vector<double>> AutonomyCore::committedPath(std::uint64_t* version) const {
  std::lock_guard<std::mutex> lock(path_mutex_);
  if (version) *version = path_version_;
  return committed_path_;
}

void AutonomyCore::setCommittedPath(std::vector<std::vector<double>> path) {
  std::lock_guard<std::mutex> lock(path_mutex_);
  committed_path_ = std::move(path);
  ++path_version_;  // a new plan: the monitor treats the trajectory built on the old one as invalid
}

planning::MapHandle AutonomyCore::vizSampledMap() const {
  std::lock_guard<std::mutex> lock(edt_mutex_);
  return viz_sampled_map_;
}

void AutonomyCore::setVizSampledMap(const planning::MapHandle& map) {
  std::lock_guard<std::mutex> lock(edt_mutex_);
  viz_sampled_map_ = map;
}

bool AutonomyCore::planOnce() {
  common::State state;
  planning::MapHandle map;
  ConsGridHandle conservative;
  common::Goal goal;
  bool has_goal = false;
  Eigen::Isometry3d world_from_map = Eigen::Isometry3d::Identity();
  bool has_frame = false;
  {
    std::lock_guard<std::mutex> lock(io_mutex_);
    state = state_;
    map = map_;
    conservative = conservative_map_;
    goal = goal_;
    has_goal = has_goal_;
    world_from_map = world_from_map_;
    has_frame = has_map_to_world_;
    // planOnce runs BOTH stages on the calling thread, so it refreshes both
    // planner-side copies (it is documented as not running alongside the two
    // planner threads, which own them otherwise).
    if (search_config_dirty_) {
      search_cfg_ = pending_config_;
      search_config_dirty_ = false;
    }
    if (trajgen_config_dirty_) {
      cfg_ = pending_config_;
      trajgen_config_dirty_ = false;
    }
  }

  if (!has_goal || !map) return false;
  if (cfg_.require_map_to_world && !has_frame) return false;

  // Plan in the map frame (see the frame note on the class), from the vehicle's
  // position expressed there.
  const common::State state_map = common::transformState(world_from_map.inverse(), state);
  std::vector<std::vector<double>> path;
  if (!runGlobalPlan(state_map, goal, map, path)) return false;

  // Same splice anchoring as the worker's trajgen tick — see spliceAnchor.
  const SpliceAnchor anchor = spliceAnchor(state, now(), world_from_map);
  if (!path.empty()) {
    path.front() = {anchor.start.pos.x(), anchor.start.pos.y(), anchor.start.pos.z()};
  }

  common::Trajectory traj;
  // Truncation stops at unobserved space only when the operator asked for it.
  // The predicate reads the conservative grid — see the worker's copy of this.
  TrajgenInfo info;
  if (!runTrajgen(path, anchor.t0, anchor.start,
                  conservativeField(map, conservative,
                                    std::max(cfg_.clearance_threshold, cfg_.frontier_margin)),
                  map, conservative,
                  cfg_.treat_unknown_as_hazard ? makeUnknownFn(map, conservative)
                                               : planning::CorridorUnknownFn{},
                  traj, /*pin_waypoints=*/false, /*root_shift=*/-1.0, &info))
    return false;

  std::uint64_t version = 0;
  committedPath(&version);
  recordStaged(stagePlanned(anchor, traj, world_from_map), info, version, path);
  return true;
}

bool AutonomyCore::hasTrajectory() const {
  return tracker_.hasTrajectory();
}

bool AutonomyCore::inHoverHold() const {
  return tracker_.mode() == control::TrajectoryTracker::Mode::kHoverHold;
}

std::vector<std::vector<double>> AutonomyCore::sampledPlannedPath(double sample_dt) const {
  common::Trajectory traj;
  {
    std::lock_guard<std::mutex> lock(traj_mutex_);
    traj = last_planned_;
  }

  std::vector<std::vector<double>> path;
  if (traj.empty()) return path;

  control::FlatnessMapper mapper;
  for (double t = 0.0; t <= traj.total_duration; t += sample_dt) {
    const common::Reference ref = mapper.sample(traj, traj.t0 + t);
    path.push_back({ref.pos.x(), ref.pos.y(), ref.pos.z()});
  }
  return path;
}

std::vector<std::array<double, 4>> AutonomyCore::clearanceSamples() const {
  std::lock_guard<std::mutex> lock(traj_mutex_);
  return last_clearance_samples_;
}

AutonomyCore::CorridorSnapshot AutonomyCore::corridorSnapshot() const {
  std::lock_guard<std::mutex> lock(traj_mutex_);
  return last_corridor_;
}

std::vector<std::array<double, 4>> AutonomyCore::sampleClearanceField(double maxdist) const {
  std::vector<std::array<double, 4>> out;

  // Sample whichever field the cost is actually scored against, since tuning
  // clearance_weight / clearance_threshold by eye only works if the picture
  // shows what the objective sees. That is the conservative field whenever one
  // exists as a distinct view, and the search field otherwise. Both are current
  // for this tick: applyClearanceObjective runs before this and populates them.
  std::shared_ptr<const planning::DistanceField> field;
  {
    // Pick the field up under the lock, then sample outside it: the walk is slow
    // and the other planner thread must not wait on it.
    std::lock_guard<std::mutex> lock(edt_mutex_);
    field = (cons_edt_ && cons_edt_source_grid_) ? cons_edt_ : edt_;
  }
  if (!field) return out;

  const Eigen::Vector3d box_lo = field->boxMin(), box_hi = field->boxMax();
  const double xmin = box_lo.x(), ymin = box_lo.y(), zmin = box_lo.z();
  const double xmax = box_hi.x(), ymax = box_hi.y(), zmax = box_hi.z();

  constexpr double step = 0.15;  // grid spacing [m] — coarse, debug-only
  const double maxd = maxdist;
  for (double x = xmin; x <= xmax; x += step) {
    for (double y = ymin; y <= ymax; y += step) {
      for (double z = zmin; z <= zmax; z += step) {
        double d = field->getDistance(octomap::point3d(
            static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)));
        if (d < 0.0) continue;       // outside the EDT bounding box
        if (d > maxd) d = maxd;      // clamp to the saturation threshold
        out.push_back({x, y, z, d});
      }
    }
  }
  return out;
}

bool AutonomyCore::runGlobalPlan(const common::State& state, const common::Goal& goal,
                                 const planning::MapHandle& map,
                                 std::vector<std::vector<double>>& path) {
  planning::GeometricPlanner planner(map, search_cfg_.rrt_solve_time);
  planner.setPlannerType(search_cfg_.planner_type);
  planner.setBestEffort(search_cfg_.best_effort_goal);
  planner.setEscapeRamp(search_cfg_.escape_ramp_dist);
  const std::vector<double> start = {state.pos.x(), state.pos.y(), state.pos.z()};
  const std::vector<double> goal_vec = {goal.pos.x(), goal.pos.y(), goal.pos.z()};
  return planner.planPath(start, goal_vec, path);
}

bool AutonomyCore::runTrajgen(const std::vector<std::vector<double>>& path, double t0,
                              const common::MotionState& start,
                              const std::shared_ptr<const planning::DistanceField>& cons_edt,
                              const planning::MapHandle& map, const ConsGridHandle& cons_grid,
                              const planning::CorridorUnknownFn& unknown_fn,
                              common::Trajectory& traj, bool pin_waypoints,
                              double root_shift, TrajgenInfo* info) {
  trajgen_corridor_time_ = 0.0;
  trajgen_qp_time_ = 0.0;
  if (info) *info = TrajgenInfo{};
  if (path.size() < 2) return false;

  if (cfg_.use_corridor_qp && cons_edt && map) {
    const double res = cons_grid ? cons_grid->resolution() : map->getResolution();
    const double t_corridor = now();
    // Corridor pipeline: truncate the (possibly optimistic) path to the prefix
    // that is safely inside known-free space, grow one free box per resampled
    // segment, and solve the corridor-constrained min-snap QP. Every stage runs
    // against the conservative distance field, so unknown space counts as an
    // obstacle throughout.
    std::vector<Eigen::Vector3d> epath;
    epath.reserve(path.size());
    for (const auto& w : path) epath.emplace_back(w[0], w[1], w[2]);

    const double maxd = std::max(cfg_.clearance_threshold, cfg_.frontier_margin);
    const planning::CorridorClearanceFn cons_fn = makeClearanceFn(cons_edt, maxd);
    // Truncation holds mapped obstacles to frontier_margin and never-observed
    // space only to unknown_margin (when that is smaller and there is a grid).
    const planning::CorridorClearanceFn trunc_fn =
        cons_grid ? makeSplitClearanceFn(cons_fn, clearanceField(map, cfg_.clearance_threshold),
                                         cfg_.clearance_threshold,
                                         cfg_.frontier_margin - cfg_.unknown_margin)
                  : cons_fn;

    planning::CorridorParams params;
    params.max_segment_len = cfg_.max_segment_len;
    params.margin = cfg_.corridor_margin;
    params.unknown_margin = cons_grid ? cfg_.unknown_margin : -1.0;
    params.local_bbox = cfg_.corridor_bbox;
    // The trajectory stays inside the box the search plans in (grown by
    // buildCorridor to hold the path's own waypoints).
    for (int i = 0; i < 3; ++i) {
      params.bounds_lo(i) = planning::GeometricPlanner::kSearchLow[i];
      params.bounds_hi(i) = planning::GeometricPlanner::kSearchHigh[i];
    }
    // Same distance the truncation ramp uses, and for the same reason: it is
    // how far from the drone we accept reduced clearance in exchange for being
    // able to move at all. Sharing the parameter keeps the two stages from
    // disagreeing about where the vehicle stops being a special case.
    params.start_relax_dist = cfg_.escape_ramp_dist;
    // A bridge region replaces a joint waypoint with two points off the path,
    // which pinned waypoints (presets) would then force the trajectory through.
    params.bridge_joints = !pin_waypoints;
    // Obstacle points are voxel centres; the corridor must clear the voxel's
    // worst-case corner, so tell it the map's half-diagonal.
    params.voxel_half_diagonal = res * std::sqrt(3.0) / 2.0;

    // Truncation enforces the configured frontier margin (less kTruncationTolerance) — it is NOT
    // floored by the corridor margin, so FRONTIER_MARGIN means what it says and
    // the two stages can be tuned independently. Whatever it is set to, a
    // committed point must still have strictly positive clearance, so the
    // prefix can never reach into an occupied or unknown voxel.
    // When the caller supplied the unknown predicate it is an absolute stop,
    // exempt from the escape ramp: the ramp trades margin for mobility against a
    // hazard whose distance we can measure, and unobserved space is not that.
    // When it did not — TREAT_FRONTIER_AS_OBSTACLE off — truncation is purely
    // the clearance walk it always was, and unmapped space reads as free.
    planning::TruncationCut cut;
    const double trunc_margin = cfg_.frontier_margin * (1.0 - kTruncationTolerance);
    const auto committed =
        planning::truncatePath(trunc_fn, epath, trunc_margin, cfg_.escape_ramp_dist,
                               /*sample_step=*/0.05, unknown_fn, &cut,
                               /*start_floor_slack=*/res,
                               /*start_floor_rel=*/kTruncationTolerance);

    // Why truncation stopped, for both log lines below. The escape ramp is
    // centred on the path root, which the caller has moved to the splice point;
    // the search centred its own ramp on where it started, so a large
    // `root_shift` means the two stages held this point to different margins.
    const auto describeCut = [&]() {
      std::ostringstream os;
      os << "cut at (" << cut.point.x() << ", " << cut.point.y() << ", " << cut.point.z()
         << "), " << cut.from_start << " m from the root in a straight line: ";
      if (cut.unknown) {
        os << "never-observed space";
      } else {
        os << "clearance " << cut.clearance << " m < required " << cut.required
           << " m (FRONTIER_MARGIN " << cfg_.frontier_margin << " m from obstacles, UNKNOWN_MARGIN "
           << cfg_.unknown_margin << " m from unknown, less "
           << kTruncationTolerance * 100.0 << "% tolerance, ramped over ESCAPE_RAMP_DIST "
           << cfg_.escape_ramp_dist << " m, floored at " << cut.floor
           << " m from the root's clearance " << cut.root_clearance << " m)";
      }
      if (root_shift >= 0.0) os << "; root " << root_shift << " m from the search start";
      return os.str();
    };

    // Debug viz snapshot, published by the host. Cleared on every corridor tick
    // and refilled only once a corridor is actually built, so a failed tick
    // erases the drawing instead of leaving a stale corridor on screen. Costs
    // nothing when debug_planner_viz is off.
    // Snapshot each stage as it completes rather than only on success: a
    // failing corridor is precisely what needs looking at, and clearing the
    // drawing made "viz broken", "flag off" and "failing every tick" look
    // identical. Reset here, then fill in as truncation and the decomposition
    // produce geometry.
    const auto resetSnapshot = [this]() {
      if (!cfg_.debug_planner_viz) return;
      std::lock_guard<std::mutex> lock(traj_mutex_);
      last_corridor_ = CorridorSnapshot{};
    };
    const auto snapshotCommitted = [this](const std::vector<Eigen::Vector3d>& path) {
      if (!cfg_.debug_planner_viz) return;
      std::lock_guard<std::mutex> lock(traj_mutex_);
      last_corridor_.committed = path;
    };
    const auto snapshotRegions = [this](const planning::CorridorAttempt& att, bool accepted) {
      if (!cfg_.debug_planner_viz) return;
      std::lock_guard<std::mutex> lock(traj_mutex_);
      last_corridor_.raw = att.raw;
      last_corridor_.shrunk = att.shrunk;
      last_corridor_.accepted = accepted;
    };
    resetSnapshot();
    if (cfg_.debug_planner_viz &&
        (committed.size() < 2 || (committed.back() - epath.back()).norm() > 1e-6)) {
      std::lock_guard<std::mutex> lock(traj_mutex_);
      last_corridor_.untruncated = epath;
    }

    if (committed.size() < 2) {
      // Nothing of the path is safely committable (start hemmed in by frontier
      // or obstacles). Stage nothing: the tracker rides out its current
      // trajectory and falls to the stale hover-hold if this persists.
      //
      // Name which of the two tests stopped it. "The path leaves observed space
      // immediately" and "the path has too little clearance immediately" want
      // opposite responses — map more before moving, versus lower a margin —
      // and on a thinly mapped scene the first is much the more likely, since
      // an unobserved cell one sample ahead of the drone is enough. Costs one
      // predicate call, and only on the tick that already failed.
      DRONE_LOG_INFO("[trajgen] corridor: truncation empty ("
                     << (cut.cut ? describeCut() : std::string("degenerate path"))
                     << ") -> no new trajectory");
      trajgen_corridor_time_ = now() - t_corridor;
      return false;
    }
    snapshotCommitted(committed);
    // Cancellation checkpoint (see SolveJob): nothing past here is worth doing
    // for a solve the monitor has already given up on.
    if (solve_cancel_.load()) {
      trajgen_corridor_time_ = now() - t_corridor;
      return false;
    }

    // Numbers for the logs below. The corridor stage and the QP are separate
    // faults needing opposite fixes, so they report separately and quantified.
    // The tightest CONSERVATIVE clearance is the number that explains a
    // decomposition failure: the search validated its centreline against the
    // optimistic map, while the corridor must clear the frontier-stamped map by
    // CORRIDOR_MARGIN. Computed only on the paths that log.
    const auto polylineLength = [](const std::vector<Eigen::Vector3d>& p) {
      double len = 0.0;
      for (std::size_t i = 0; i + 1 < p.size(); ++i) len += (p[i + 1] - p[i]).norm();
      return len;
    };
    const auto minConservativeClearance = [&cons_fn](const std::vector<Eigen::Vector3d>& p) {
      double lo = std::numeric_limits<double>::infinity();
      for (std::size_t i = 0; i + 1 < p.size(); ++i) {
        const double seg = (p[i + 1] - p[i]).norm();
        const int n = std::max(1, static_cast<int>(std::ceil(seg / 0.05)));
        for (int k = 0; k <= n; ++k) {
          const Eigen::Vector3d q = p[i] + (static_cast<double>(k) / n) * (p[i + 1] - p[i]);
          lo = std::min(lo, cons_fn(q.x(), q.y(), q.z()));
        }
      }
      return lo;
    };

    // Truncation shortening the path is the pipeline refusing to commit toward
    // unknown space; the endpoint it picked is where the drone will actually fly
    // this cycle. Silent when the whole path survives. Detail, like the other
    // successful-corridor lines below: DEBUG_TRAJGEN only.
    if (cfg_.debug_trajgen && (committed.back() - epath.back()).norm() > 1e-6) {
      DRONE_LOG_INFO("[trajgen] corridor: truncated to " << committed.size() << " wp / "
                     << polylineLength(committed) << " m of " << polylineLength(epath)
                     << " m — " << describeCut());
    }

    // Obstacle points for the decomposition: the conservative grid's occupied
    // and shell cells (real obstacles AND unobserved space; the raw map's
    // occupied leaves when there is no grid) within a window
    // around the committed prefix. Windowed on purpose — a whole room at 5 cm
    // is 1e5-1e6 voxels and DecompUtil scans the list per segment, which would
    // never fit the 1 Hz trajgen budget. The window is the prefix's AABB grown
    // by the region-growth extent plus the margin, so nothing that could bound
    // a region is missed. Coarse (merged) leaves are expanded to resolution
    // voxels, so a single big leaf does not under-represent a solid block as
    // one point.
    const auto obstacles = [&]() {
      Eigen::Vector3d lo = committed.front(), hi = committed.front();
      for (const auto& w : committed) {
        lo = lo.cwiseMin(w);
        hi = hi.cwiseMax(w);
      }
      const double pad = planning::corridorObstacleWindowPad(params);
      lo.array() -= pad;
      hi.array() += pad;
      std::vector<Eigen::Vector3d> pts;
      if (cons_grid) {  // occupied + shell cells: real obstacles and unobserved space
        cons_grid->obstaclesIn(lo, hi, pts);
        // Mapped obstacles first, then the shell, which the corridor holds to
        // unknown_margin (CorridorParams::first_unknown).
        const auto shell_from =
            std::stable_partition(pts.begin(), pts.end(), [&](const Eigen::Vector3d& q) {
              return cons_grid->at(q.x(), q.y(), q.z()) != planning::ConservativeGrid::kShell;
            });
        params.first_unknown = static_cast<std::size_t>(shell_from - pts.begin());
        return pts;
      }
      const octomap::point3d bmin(static_cast<float>(lo.x()), static_cast<float>(lo.y()),
                                  static_cast<float>(lo.z()));
      const octomap::point3d bmax(static_cast<float>(hi.x()), static_cast<float>(hi.y()),
                                  static_cast<float>(hi.z()));
      for (auto it = map->begin_leafs_bbx(bmin, bmax), end = map->end_leafs_bbx();
           it != end; ++it) {
        if (!map->isNodeOccupied(*it)) continue;
        const double size = it.getSize();
        const octomap::point3d c = it.getCoordinate();
        if (size <= res * 1.5) {
          pts.emplace_back(c.x(), c.y(), c.z());
          continue;
        }
        const double half = (size - res) / 2.0;
        for (double dx = -half; dx <= half + 1e-6; dx += res)
          for (double dy = -half; dy <= half + 1e-6; dy += res)
            for (double dz = -half; dz <= half + 1e-6; dz += res)
              pts.emplace_back(c.x() + dx, c.y() + dy, c.z() + dz);
      }
      return pts;
    }();

    std::vector<Eigen::Vector3d> resampled;
    std::vector<planning::ConvexRegion> regions;
    std::string why;
    planning::CorridorAttempt attempt;
    double start_margin = params.margin;
    double end_pullback = 0.0;
    planning::CorridorRepairs repairs;
    const bool built = planning::buildCorridor(obstacles, committed, params, resampled, regions,
                                               &why, cfg_.debug_planner_viz ? &attempt : nullptr,
                                               &start_margin, &end_pullback, &repairs);
    trajgen_corridor_time_ = now() - t_corridor;
    if (!built) {
      snapshotRegions(attempt, /*accepted=*/false);
      // The path's tightest clearance from each kind of hazard against what the
      // corridor holds it to: mapped obstacles (raw field) at CORRIDOR_MARGIN,
      // never-observed space (where the conservative field reads nearer than
      // the raw one) at UNKNOWN_MARGIN when smaller. Context, not the cause:
      // the parenthesis says which check failed.
      std::ostringstream tight;
      tight << "tightest clearance on the path: ";
      const auto obs_edt = cons_grid ? clearanceField(map, cfg_.clearance_threshold) : nullptr;
      if (cons_grid) {
        const auto obs_fn = obs_edt ? makeClearanceFn(obs_edt, cfg_.clearance_threshold)
                                    : planning::CorridorClearanceFn(
                                          [md = cfg_.clearance_threshold](double, double, double) {
                                            return md;
                                          });
        double to_obs = std::numeric_limits<double>::infinity();
        double to_unknown = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i + 1 < committed.size(); ++i) {
          const Eigen::Vector3d d = committed[i + 1] - committed[i];
          const int n = std::max(1, static_cast<int>(std::ceil(d.norm() / 0.05)));
          for (int k = 0; k <= n; ++k) {
            const Eigen::Vector3d q = committed[i] + (static_cast<double>(k) / n) * d;
            const double o = obs_fn(q.x(), q.y(), q.z());
            const double c = cons_fn(q.x(), q.y(), q.z());
            to_obs = std::min(to_obs, o);
            if (c < o - 1e-6) to_unknown = std::min(to_unknown, c);
          }
        }
        tight << "mapped obstacles " << to_obs << " m (needs " << params.margin
              << " m, CORRIDOR_MARGIN), unknown space ";
        if (std::isfinite(to_unknown)) {
          tight << to_unknown << " m";
        } else {
          tight << "never nearer than the obstacles";
        }
        tight << " (needs " << std::min(params.margin, cfg_.unknown_margin)
              << " m, UNKNOWN_MARGIN)";
      } else {
        tight << minConservativeClearance(committed) << " m (needs " << params.margin
              << " m, CORRIDOR_MARGIN)";
      }
      DRONE_LOG_INFO("[trajgen] corridor: decomposition FAILED (" << why << ") over "
                     << committed.size() << " wp / " << polylineLength(committed) << " m — "
                     << tight.str() << ", " << obstacles.size()
                     << " obstacle pts -> no new trajectory");
      return false;
    } else {
      // The details of a corridor that worked: how much clearance the first
      // stretch actually has, how far the end was pulled back, whether thin
      // joints were repaired. DEBUG_TRAJGEN only (the start relaxation used to
      // be unconditional; turned off with the rest on request, 2026-09-30).
      if (cfg_.debug_trajgen && start_margin < params.margin - 1e-6) {
        DRONE_LOG_INFO("[trajgen] corridor: start margin relaxed to " << start_margin
                       << " m (of " << params.margin << " m) over the first "
                       << params.start_relax_dist
                       << " m — the drone is hemmed in; full margin applies beyond that");
      }
      if (cfg_.debug_trajgen && end_pullback > 1e-6) {
        DRONE_LOG_INFO("[trajgen] corridor: end pulled back " << end_pullback
                       << " m along the path to fit inside the shrunk corridor (CORRIDOR_MARGIN "
                       << params.margin << " m)");
      }
      // A squeeze on the path: consecutive regions only overlapped once repaired.
      // Safe (every joint still passed the overlap test), but it explains an
      // unusually large region count and a slower solve.
      if (cfg_.debug_trajgen && (repairs.bridges > 0 || repairs.split_rounds > 0)) {
        DRONE_LOG_INFO("[trajgen] corridor: repaired thin joints with " << repairs.bridges
                       << " bridge region(s) and " << repairs.split_rounds
                       << " split round(s) -> " << regions.size() << " regions");
      }
      planning::CorridorTrajectoryOptimizer optimizer(
          planning::CorridorLimits{cfg_.vmax, cfg_.amax, cfg_.jmax});
      optimizer.setTimeBudget(cfg_.traj_solve_budget);
      optimizer.setGroupCut(cfg_.traj_group_cut);
      optimizer.setGroupEdgeFactor(cfg_.traj_group_edge_factor);
      optimizer.setPathWeight(cfg_.traj_path_weight);
      optimizer.setDebug(cfg_.debug_trajgen);
      optimizer.setAbortFlag(&solve_abort_);
      if (trajgen_stub_len_ > 0.0) optimizer.setStartSeedBoost(trajgen_stub_len_, 2.0);
      snapshotRegions(attempt, /*accepted=*/true);
      if (solve_cancel_.load()) return false;  // cancellation checkpoint
      // `start` carries the splice state. Its position is path.front() by
      // construction (the caller rooted the path there), so it satisfies
      // regions[0]; the derivatives are what make the engage continuous.
      const double t_qp = now();
      const bool solved = optimizer.optimizeTrajectory(start, resampled, regions, traj, pin_waypoints);
      trajgen_qp_time_ = now() - t_qp;
      if (solved) {
        traj.t0 = t0;
        if (info) {
          info->corridor = true;
          info->start_margin = start_margin;
          info->trunc_end = committed.back();
        }
        if (cfg_.debug_trajgen) {
          DRONE_LOG_INFO("[trajgen] corridor: OK " << regions.size() << " regions / "
                         << polylineLength(committed) << " m / " << traj.total_duration << " s");
        }
        return true;
      }
      // Quantify the corridor's SHAPE, not just the fact that nothing fit it.
      // Three bench stalls in a row (2026-09-23/25) came back as "QP INFEASIBLE"
      // with nothing to say which region was at fault, and each time the seed
      // growth had run all the way out — infeasible from 4.8 s to 54 s. That
      // pattern can only be geometric: lengthening every segment loosens the
      // velocity, acceleration and jerk rows and leaves the corridor rows
      // untouched, so a problem still infeasible at 11x the time is infeasible at
      // any time, and the old message naming VMAX/AMAX/JMAX pointed at the one
      // thing it could not be. These are the numbers that decide whether a
      // degree-7 C4 spline can thread the chain at all:
      //   - how far the start sits inside region 0. The start position AND its
      //     rest derivatives pin the first four Bezier control points exactly
      //     there, so a start on the region's boundary has no room to leave it.
      //   - each region's inradius (the largest ball it contains): a sliver
      //     region cannot hold eight control points however roomy its
      //     neighbours are.
      //   - each joint's overlap depth, which is what C0 continuity must land
      //     the junction inside.
      // regionOverlapDepth against itself is the inradius; both are a small LP,
      // microseconds each, and only on a tick that has already failed.
      std::ostringstream geom;
      if (!regions.empty()) {
        double start_slack = std::numeric_limits<double>::infinity();
        for (int r = 0; r < regions.front().A.rows(); ++r) {
          start_slack =
              std::min(start_slack, regions.front().b(r) - regions.front().A.row(r).dot(start.pos));
        }
        geom << " — start " << start_slack << " m inside region 0, inradii [";
        for (std::size_t i = 0; i < regions.size(); ++i) {
          if (i) geom << ", ";
          geom << planning::regionOverlapDepth(regions[i], regions[i]);
        }
        geom << "] m, joint overlaps [";
        for (std::size_t i = 0; i + 1 < regions.size(); ++i) {
          if (i) geom << ", ";
          geom << planning::regionOverlapDepth(regions[i], regions[i + 1]);
        }
        geom << "] m";
      }
      DRONE_LOG_INFO("[trajgen] corridor: QP INFEASIBLE over "
                     << regions.size() << " regions / " << polylineLength(committed) << " m"
                     << geom.str()
                     << " -> no new trajectory (the [corridor-qp] line says whether a longer time "
                        "allocation could ever have helped; if the seed growth ran out, the "
                        "corridor shape is the fault, not VMAX/AMAX/JMAX)");
    }

    // Every corridor failure path ends here, and it stages NOTHING. There used
    // to be a plain min-snap fallback on the truncated prefix, which was a
    // mistake: min-snap knows nothing about obstacles, so the one situation
    // that produced it — the corridor stage saying it cannot guarantee a safe
    // trajectory — is exactly the situation in which an unchecked polynomial is
    // least defensible. Staging nothing means the tracker rides out whatever it
    // already has. If that one is no longer valid the monitor sends no health
    // signal and keeps asking for a replacement, so once HEALTH_TIMEOUT passes
    // the tracker latches the current position and holds. Standing still is the
    // only honest answer when the corridor cannot certify moving, and it
    // recovers the moment the map or the path allows a corridor again.
    return false;
  }

  // Plain min-snap fallback (USE_CORRIDOR_QP off). NOTE: this path is still
  // rest-to-rest. It inherits the splice POSITION — the caller rooted `path`
  // there — and the lead-time anchor, but its solver has no way to accept a
  // start velocity, so engaging it while moving steps the velocity reference
  // from whatever the vehicle was doing to zero. That is the stutter the
  // corridor path was just fixed for. Acceptable only because this mode is the
  // obstacle-blind one already, and nothing should fly it near obstacles; give
  // MinSnapTrajectory a start-derivative boundary before relying on it.
  planning::MinSnapTimeOptimizer optimizer;
  if (!optimizer.optimizeTrajectory(path, traj)) return false;
  traj.t0 = t0;
  if (info) info->trunc_end = Eigen::Vector3d(path.back()[0], path.back()[1], path.back()[2]);
  return true;
}

void AutonomyCore::stagePending(const common::Trajectory& traj) {
  staged_count_.fetch_add(1);
  std::lock_guard<std::mutex> lock(traj_mutex_);
  pending_ = traj;
  last_planned_ = traj;
  last_planned_at_ = now();
  has_last_planned_ = true;
  has_pending_ = true;
}

void AutonomyCore::runPreset(const common::State& state, const Eigen::Isometry3d& world_from_map,
                             const planning::MapHandle& map,
                             const ConsGridHandle& conservative,
                             const std::vector<Eigen::Vector3d>& waypoints) {
  if (waypoints.size() < 2) {
    DRONE_LOG_INFO("[preset] ignored: need at least two waypoints, got " << waypoints.size());
    return;
  }
  // The corridor pipeline needs a map to truncate and grow regions against. With
  // none, drop the request rather than silently doing nothing — the vehicle stays
  // on POS_SP.
  if (!map) {
    DRONE_LOG_INFO("[preset] ignored: no map yet — the corridor pipeline needs one");
    return;
  }

  DRONE_LOG_INFO("[preset] firing one-shot trajectory through " << waypoints.size()
                 << " waypoints");

  // Anchor at rest on the current state. firePreset dropped any goal and the
  // vehicle is holding POS_SP, so spliceAnchor finds no still-tracked outgoing
  // trajectory and falls back to rest at the measured position — exactly the
  // clean rest-to-rest start this test wants. Re-root the first waypoint onto the
  // anchor like any committed path.
  const double t_gen = now();
  const SpliceAnchor anchor = spliceAnchor(state, t_gen, world_from_map);
  std::vector<std::vector<double>> path;
  path.reserve(waypoints.size());
  for (const auto& w : waypoints) path.push_back({w.x(), w.y(), w.z()});
  path.front() = {anchor.start.pos.x(), anchor.start.pos.y(), anchor.start.pos.z()};

  common::Trajectory traj;
  // Pin the waypoints. For a preset the shape IS the test — with the junctions
  // free the QP is pinned only at the two ends and takes the cheapest route the
  // corridor allows, which for a closed square (last waypoint == first) is
  // barely moving at all. Planning deliberately leaves them free; see runTrajgen.
  const bool ok = runTrajgen(path, anchor.t0, anchor.start,
                             conservativeField(map, conservative,
                                               std::max(cfg_.clearance_threshold,
                                                        cfg_.frontier_margin)),
                             map, conservative,
                             cfg_.treat_unknown_as_hazard ? makeUnknownFn(map, conservative)
                                                          : planning::CorridorUnknownFn{},
                             traj, /*pin_waypoints=*/true);
  if (!ok) {
    // runTrajgen has already logged which stage failed and why. Stage nothing;
    // the vehicle keeps holding POS_SP. preset_active_ stays false, so control is
    // never handed to a trajectory that was not built.
    DRONE_LOG_INFO("[preset] trajectory generation FAILED — staying on POS_SP");
    return;
  }

  double path_len = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    path_len += std::hypot(std::hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1]),
                           path[i][2] - path[i - 1][2]);
  }

  // A preset starts at rest, so staging moves t0 to after the solve; preset_end_
  // below then follows it rather than cutting the one-shot short by the solve time.
  const common::Trajectory staged = stagePlanned(anchor, traj, world_from_map);
  {
    // Presets are never monitored (they are kept fresh by stepControl), and what
    // the monitor had recorded is superseded by this one.
    std::lock_guard<std::mutex> lock(traj_mutex_);
    records_.clear();
  }
  // Hold the one-shot for its whole duration, then release (see stepControl). The
  // trajectory plays in absolute time from its t0, so its end is t0 + duration.
  preset_end_.store(staged.t0 + staged.total_duration);
  preset_active_.store(true);
  DRONE_LOG_INFO("[preset] trajectory staged: " << path_len << " m / "
                 << staged.total_duration << " s — holding kTracking until it completes");
}

AutonomyCore::SpliceAnchor AutonomyCore::spliceAnchor(const common::State& state, double t_now,
                                                      const Eigen::Isometry3d& world_from_map,
                                                      double min_t0) const {
  const Eigen::Isometry3d map_from_world = world_from_map.inverse();
  SpliceAnchor anchor;
  anchor.t0 = std::max(t_now + kTrajgenLead, min_t0);

  common::Trajectory outgoing;
  bool have = false;
  double staged_at = 0.0;
  {
    std::lock_guard<std::mutex> lock(traj_mutex_);
    if (has_last_planned_) {
      outgoing = last_planned_;
      staged_at = last_planned_at_;
      have = true;
    }
  }

  // Only splice onto a trajectory the tracker is still following. Once it has
  // latched a hover over it (health timeout, emergency, divergence) the vehicle
  // is no longer on that curve, and matching its state would step the reference
  // rather than smooth it.
  (void)staged_at;
  const bool still_tracked = have && !outgoing.empty() && !tracker_holding_.load() &&
                             !cfg_.bench_replan_from_state;  // bench: always the measured state
  if (still_tracked) {
    // The outgoing trajectory is world-frame (it is what the tracker flies), so
    // sample it there and only then express the result in the map frame.
    anchor.start = common::transformMotion(map_from_world, common::sampleMotion(outgoing, anchor.t0));
    anchor.from_trajectory = true;
  } else {
    // Rest at the measured position. Note sampleMotion would also return rest if
    // the outgoing trajectory had simply run out (it ends at rest by
    // construction) — this branch is for having no usable outgoing curve at all.
    anchor.start.pos = map_from_world * state.pos;
    anchor.from_trajectory = false;
  }
  return anchor;
}

void AutonomyCore::restampRestStart(const SpliceAnchor& anchor, common::Trajectory& traj) const {
  if (anchor.from_trajectory) return;
  // kLeadMin (two control ticks) rather than now() itself, so the tracker
  // promotes it at its own beginning instead of a tick in.
  traj.t0 = now() + kLeadMin;
}

common::Trajectory AutonomyCore::stagePlanned(const SpliceAnchor& anchor,
                                              const common::Trajectory& map_traj,
                                              const Eigen::Isometry3d& world_from_map) {
  common::Trajectory traj = common::transformTrajectory(world_from_map, map_traj);
  restampRestStart(anchor, traj);
  stagePending(traj);
  return traj;
}

std::shared_ptr<const planning::DistanceField> AutonomyCore::clearanceField(
    const planning::MapHandle& map, double maxdist) {
  std::lock_guard<std::mutex> lock(edt_mutex_);
  // No obstacles => clearance is uniform => no field needed (validity treats
  // everything as free, cost reduces to length). Drop any stale cache.
  if (!map || map->size() == 0) {
    edt_.reset();
    edt_source_map_.reset();
    edt_superseded_.clear();
    return nullptr;
  }
  // Normal case: setMap already built this map's field before publishing it.
  // A map that has since been superseded (a planner snapshotted it just before
  // a newer one arrived) is served the newer field rather than rebuilt: the
  // newer field only knows about more obstacles, and rebuilding would put the
  // whole build back on this planner thread.
  if (edt_ && maxdist == edt_maxdist_ &&
      (map == edt_source_map_ || wasSuperseded(edt_superseded_, map))) {
    return edt_;
  }
  // Fallback, built here under the lock: a maxdist change (CLEARANCE_THRESHOLD
  // set live — keying on the map alone used to leave that unapplied until the
  // next octomap, which on a static bench scene may never come), or a map that
  // did not come through setMap. Rare, and it serialises so the two planner
  // threads never build the same field twice. Always for the NEWEST map: a
  // superseded one must not become the cache's source, or the newer map would
  // then be served an older field.
  const planning::MapHandle target =
      (edt_source_map_ && wasSuperseded(edt_superseded_, map)) ? edt_source_map_ : map;
  planner_field_builds_.fetch_add(1);
  auto field = buildEdt(target, maxdist);
  if (target != edt_source_map_) pushSuperseded(edt_superseded_, edt_source_map_, kSupersededHistory);
  edt_ = std::move(field);
  edt_source_map_ = target;
  edt_maxdist_ = maxdist;
  viz_sampled_map_.reset();  // debug clearance samples are of the old field
  return edt_;
}

std::shared_ptr<const planning::DistanceField> AutonomyCore::conservativeField(
    const planning::MapHandle& map, const ConsGridHandle& grid, double maxdist) {
  std::lock_guard<std::mutex> lock(edt_mutex_);
  if (grid && !grid->empty()) {
    // Same prebuilt / superseded / fallback logic as clearanceField, keyed on
    // the grid.
    if (cons_edt_ && maxdist == cons_edt_maxdist_ &&
        (grid == cons_edt_source_grid_ || wasSuperseded(cons_grid_superseded_, grid))) {
      return cons_edt_;
    }
    const ConsGridHandle target =
        (cons_edt_source_grid_ && wasSuperseded(cons_grid_superseded_, grid)) ? cons_edt_source_grid_
                                                                               : grid;
    planner_field_builds_.fetch_add(1);
    auto field = buildEdt(target, maxdist);
    if (target != cons_edt_source_grid_) {
      pushSuperseded(cons_grid_superseded_, cons_edt_source_grid_, kSupersededHistory);
    }
    cons_edt_ = std::move(field);
    cons_edt_source_grid_ = target;
    cons_edt_source_map_.reset();
    cons_edt_maxdist_ = maxdist;
    return cons_edt_;
  }
  if (!map || map->size() == 0) {
    cons_edt_.reset();
    cons_edt_source_map_.reset();
    cons_edt_source_grid_.reset();
    cons_edt_superseded_.clear();
    cons_grid_superseded_.clear();
    return nullptr;
  }
  // No conservative grid => the conservative view IS the search map; reuse its
  // field rather than building a second identical one.
  if (edt_ && (map == edt_source_map_ || wasSuperseded(edt_superseded_, map))) return edt_;
  if (cons_edt_ && maxdist == cons_edt_maxdist_ &&
      (map == cons_edt_source_map_ || wasSuperseded(cons_edt_superseded_, map))) {
    return cons_edt_;
  }
  const planning::MapHandle target =
      (cons_edt_source_map_ && wasSuperseded(cons_edt_superseded_, map)) ? cons_edt_source_map_
                                                                         : map;
  planner_field_builds_.fetch_add(1);
  auto field = buildEdt(target, maxdist);
  if (target != cons_edt_source_map_) {
    pushSuperseded(cons_edt_superseded_, cons_edt_source_map_, kSupersededHistory);
  }
  cons_edt_ = std::move(field);
  cons_edt_source_map_ = target;
  cons_edt_source_grid_.reset();
  cons_edt_maxdist_ = maxdist;
  return cons_edt_;
}

bool AutonomyCore::applyClearanceObjective(planning::GeometricPlanner& planner,
                                           const planning::MapHandle& map,
                                           const ConsGridHandle& cons) {
  auto edt = clearanceField(map, search_cfg_.clearance_threshold);
  if (!edt) return false;
  const double cons_md = std::max(search_cfg_.clearance_threshold, search_cfg_.frontier_margin);
  // Legacy mode (corridor QP off): the conservative field, when there is one, is
  // the single obstacle model, collision check included.
  if (!search_cfg_.use_corridor_qp && cons) {
    if (auto cons_edt = conservativeField(map, cons, cons_md)) edt = cons_edt;
  }
  // The search map's field drives the collision check (clearance > margin). It
  // has to be this view and not the conservative one, because the conservative
  // view stamps the frontier as occupied and a validity check against that
  // would wall the search inside the mapped region entirely.
  planner.setClearance(makeClearanceFn(edt, search_cfg_.clearance_threshold),
                       search_cfg_.clearance_weight, search_cfg_.clearance_threshold);

  // The cost, however, is scored against the conservative field whenever one
  // exists as a distinct map. The reason is that the search and the downstream
  // truncation were previously measuring different things: truncation cuts the
  // committed prefix where conservative clearance falls below frontier_margin,
  // while the cost only ever saw the optimistic field, so the search had no way
  // to know what would get it cut and happily returned paths that ran tangent
  // to the frontier. Scoring both against the same field aligns them, and the
  // prefix that survives truncation gets longer as a result.
  //
  // The shell is the never-observed voxels bordering free space, not the whole
  // unknown volume, so this field is
  // min(distance to a real obstacle, distance to that shell). It is therefore a
  // strict refinement of the optimistic field: identical wherever the shell is
  // not the nearest source, and additionally repulsive near the shell. Nothing
  // about real-obstacle avoidance is lost by scoring against it.
  //
  // Skipped with no conservative grid (frontier treatment off), and in the legacy
  // non-corridor mode, where the conservative field already is the validity
  // field above.
  if (cons && search_cfg_.use_corridor_qp) {
    if (auto cons_edt = conservativeField(map, cons, cons_md)) {
      planner.setCostClearance(makeClearanceFn(cons_edt, search_cfg_.clearance_threshold),
                               search_cfg_.frontier_weight);
    }
  }

  // Charge for routing through unobserved space, when the operator has asked for
  // unmapped space to count as a hazard at all. Gated on the flag itself and NOT
  // on `cons` being non-null: the host only builds a conservative view once a
  // frontier cloud has arrived, so keying off it made a late or missing
  // /octomap_frontier quietly disable this even with the flag on. This term
  // needs no frontier cloud — it reads the raw octree directly.
  //
  // The predicate reads the SEARCH map, not the conservative one, because
  // frontier stamping writes voxels into its copy and any stamped point that did
  // not already exist becomes a known cell there — which would make a sliver of
  // genuinely unobserved space read as observed. The raw map is the honest
  // record of what the sensors have seen.
  //
  // This never touches validity, so unknown space stays enterable and a goal
  // beyond the frontier is still accepted — it just stops being free.
  if (search_cfg_.treat_unknown_as_hazard) {
    planner.setUnknownPenalty(makeUnknownFn(map), search_cfg_.unknown_weight);
  }
  return true;
}

void AutonomyCore::searchLoop() {
  std::uint64_t run = 0;  // search ticks taken (advanced only while a goal and map exist)

  // Idle diagnostics. A tick that cannot plan produces no [plan] line at all,
  // which from outside is indistinguishable from a crashed worker — "I sent a
  // goal and nothing happens" has two very different causes (no goal reached
  // the core, or no map has). Name the missing precondition instead of going
  // quiet: log on every change of reason, then periodically so a persistent
  // stall stays visible without spamming at the tick rate. Thread locals, so
  // this costs nothing once planning is running.
  enum class Idle { kNone, kNoGoal, kNoMap, kNoFrame };
  Idle idle = Idle::kNone;
  double last_idle_log = -1.0e9;

  while (running_.load()) {
    const double t = now();

    common::State state;
    planning::MapHandle map;
    ConsGridHandle conservative;
    common::Goal goal;
    bool has_goal = false;
    bool new_goal = false;
    // One snapshot per cycle, used to bring the state into the map frame.
    Eigen::Isometry3d world_from_map = Eigen::Isometry3d::Identity();
    bool has_frame = false;
    {
      std::lock_guard<std::mutex> lock(io_mutex_);
      state = state_;
      world_from_map = world_from_map_;
      has_frame = has_map_to_world_;
      map = map_;
      conservative = conservative_map_;
      goal = goal_;
      has_goal = has_goal_;
      new_goal = new_goal_;
      new_goal_ = false;
      // Take any new config here, once per cycle, so everything this cycle does
      // sees one consistent Config — and so it applies whether or not the
      // vehicle is armed (stepControl, which used to be the only consumer, only
      // runs armed).
      if (search_config_dirty_) {
        search_cfg_ = pending_config_;
        search_config_dirty_ = false;
      }
    }

    // A new goal invalidates the committed path regardless of its collision
    // validity (the monitor check below only tests for collisions, not whether
    // the path still targets the current goal). Drop it here on this thread so
    // the tick below replans from the drone's current position toward the new
    // goal.
    if (new_goal) setCommittedPath({});

    // The tracker abandoned the trajectory (the vehicle got too far from its
    // reference) and is holding position. Same treatment as a new goal: drop the
    // committed path so this tick searches again from where the vehicle actually
    // is. stepControl has already cleared the splice source and the monitor's
    // records, so the monitor asks for a new trajectory at once and it starts
    // from rest at the measured position.
    if (search_stale_path_.exchange(false)) {
      setCommittedPath({});
    }
    if (search_replan_requested_.exchange(false)) {
      setCommittedPath({});
      if (has_goal) {
        DRONE_LOG_INFO("[plan] the tracker abandoned its trajectory (diverged or no health signal): replanning from the vehicle's position");
      }
    }

    // Without the map->world transform, map-frame obstacles and the world-frame
    // vehicle cannot be put side by side, and assuming identity is exactly the
    // offset this exists to prevent. Hold off; the idle branch says why.
    const bool frame_ok = has_frame || !search_cfg_.require_map_to_world;

    if (!preset_active_.load() && has_goal && map && frame_ok) {
      run++;
      // Local copy for this tick. The trajgen thread reads the shared one
      // whenever it likes, so everything below works on this snapshot and
      // publishes through setCommittedPath.
      std::vector<std::vector<double>> committed_path = committedPath();
      // The search, the committed-path cost from the drone and every other
      // planning quantity use the vehicle position in the map frame.
      // Where to plan FROM. Not the vehicle's position: while a trajectory is being
      // flown, the trajectory built on this search will be spliced onto it at the
      // point it reaches by the time that trajectory takes over — after this
      // search, the monitor noticing the new plan, and the splice lead. Planned
      // from the vehicle instead, the path started metres behind where the new
      // trajectory had to start (bench 2026-09-29: splice point ~3 m off the new
      // path on a goal change at cruise, corridors infeasible until the old
      // trajectory ran out). Rest starts (none flown, hovering, bench flag) plan
      // from the measured position, which is where they start.
      Eigen::Vector3d pos_map = world_from_map.inverse() * state.pos;
      bool predicted_start = false;
      {
        const double budget = std::min(search_cfg_.rrt_solve_time,
                                       committed_path.empty() ? search_cfg_.rrt_replan_solve_time
                                                              : search_cfg_.rrt_solve_time);
        // The monitor ticks at traj_monitor_rate, so it notices the new plan on
        // average half a period after the search ends.
        const double engage =
            now() + budget + 0.5 / std::max(search_cfg_.traj_monitor_rate, 0.5) + kTrajgenLead;
        std::lock_guard<std::mutex> lock(traj_mutex_);
        if (has_last_planned_ && !last_planned_.empty() && !tracker_holding_.load() &&
            !search_cfg_.bench_replan_from_state) {
          pos_map = world_from_map.inverse() * common::sampleMotion(last_planned_, engage).pos;
          predicted_start = true;
        }
      }
      (void)predicted_start;
      const std::vector<double> start = {pos_map.x(), pos_map.y(), pos_map.z()};
      const std::vector<double> goal_vec = {goal.pos.x(), goal.pos.y(), goal.pos.z()};

      // Commit a path: keep it on the worker and publish it to trajgen.
      auto adopt = [this, &committed_path](const std::vector<std::vector<double>>& p) {
        committed_path = p;
        setCommittedPath(p);
      };

      // Which map view the geometric search (and the committed-path monitor)
      // runs on. Corridor mode searches the OPTIMISTIC view — unknown reads as
      // free, so the informed planners (BIT*/AIT*/EIT*) accept a goal beyond
      // the mapped frontier instead of refusing it ("no goal states
      // available"); safety against unknown space is enforced downstream by
      // truncation + corridor, not by the search. Without corridor mode the
      // legacy single-map behavior is preserved: the conservative field, when
      // there is one, is the one obstacle model (applyClearanceObjective).
      const planning::MapHandle search_map = map;

      // One planner per tick. Set the clearance fields FIRST, so the
      // committed-path re-check below sees the same obstacle model the search
      // would. The search map's field is the obstacle model for the collision
      // check; the conservative field, when it is a distinct view, additionally
      // scores the cost so the search is repelled by the frontier it would
      // otherwise skim (see applyClearanceObjective). Both are cached and
      // rebuilt only on a map change, so this is cheap on the common
      // still-valid tick.
      planning::GeometricPlanner planner(search_map, search_cfg_.rrt_solve_time);
      planner.setPlannerType(search_cfg_.planner_type);
      planner.setBestEffort(search_cfg_.best_effort_goal);
      planner.setEscapeRamp(search_cfg_.escape_ramp_dist);
      applyClearanceObjective(planner, search_map, conservative);
      // Never-observed space here is the grid's view when there is one (the
      // keep-out around the drone counts as seen, so it can still climb off the
      // bench), else the raw map's.
      if (search_cfg_.treat_unknown_as_hazard) {
        planner.setUnknownSlopeLimit(makeUnknownFn(search_map, conservative),
                                     search_cfg_.max_unknown_slope,
                                     search_cfg_.unknown_slope_weight);
      }

      // Debug-only: re-sample the clearance field when the map changes (the EDT
      // is now current for this tick). Gated so a regular flight never walks the
      // grid. Sampling only on map change keeps even a debug run cheap on a
      // static scene.
      if (search_cfg_.debug_planner_viz && search_map != vizSampledMap()) {
        auto samples = sampleClearanceField(search_cfg_.clearance_threshold);
        {
          std::lock_guard<std::mutex> lock(traj_mutex_);
          last_clearance_samples_ = std::move(samples);
        }
        setVizSampledMap(search_map);
      }

      const bool had_path = !committed_path.empty();
      const bool path_invalid = !had_path || !planner.isPathValid(committed_path);

      // IMPROVE every Nth tick, N = improve period / monitor period (≈10 with the
      // defaults), so the RRT_MONITOR_PERIOD / RRT_IMPROVE_PERIOD params still set
      // both cadences. Skipped with no committed path or no obstacles mapped
      // (clearance is then uniform, so there is nothing to improve — a blocked path
      // still replans below regardless).
      const long n = std::lround(search_cfg_.rrt_improve_period /
                                 std::max(search_cfg_.rrt_monitor_period, 1.0e-3));
      const std::uint64_t improve_interval = static_cast<std::uint64_t>(std::max<long>(1, n));
      const bool improve_run = (run % improve_interval == 0) && had_path && map->size() > 0;

      // Diagnostics on the committed path (the field is already set). The cost is
      // split into its length + clearance-penalty terms for the logs below;
      // committed_clr is the tightest distance to a wall along the path.
      const auto committed = planner.costBreakdown(committed_path);
      const double committed_clr = planner.minClearance(committed_path);

      // Stream a cost as "T (len L N% + obst O N% + frontier F N% + unknown U
      // N%)": the path's length, the proximity penalty near mapped obstacles,
      // the same near the frontier (only when the cost is scored on the
      // frontier-stamped field), and the unknown-space surcharge (only when it
      // is configured), each with its share of the total, so a high score reads
      // as long, wall-hugging, frontier-hugging or routed through unmapped
      // space, which want different knobs (length: none; obst/frontier:
      // CLEARANCE_WEIGHT / CLEARANCE_THRESHOLD; unknown: UNKNOWN_WEIGHT).
      const bool show_unknown = search_cfg_.treat_unknown_as_hazard && search_cfg_.unknown_weight > 0.0;
      auto fmtCost = [show_unknown](const planning::GeometricPlanner::CostBreakdown& cb) {
        std::ostringstream os;
        os << std::fixed << std::setprecision(2) << cb.total;
        if (!std::isfinite(cb.total) || cb.total <= 0.0) return os.str();
        const auto term = [&](const char* name, double v) {
          os << name << v << " " << std::lround(100.0 * v / cb.total) << "%";
        };
        term(" (len ", cb.length);
        term(cb.split ? " + obst " : " + clr ", cb.clearance - cb.frontier);
        if (cb.split) term(" + frontier ", cb.frontier);
        if (show_unknown || cb.unknown > 0.0) term(" + unknown ", cb.unknown);
        if (cb.steep > 0.0) term(" + steep ", cb.steep);
        os << ")";
        return os.str();
      };

      // Straight-line distance from a path's endpoint to the goal. In best-effort
      // mode a path may deliberately stop short (at the frontier edge), so this is
      // how much of the goal still remains; the committed and a candidate path are
      // compared on it to see which reaches closer.
      auto gapToGoal = [&goal_vec](const std::vector<std::vector<double>>& p) {
        if (p.empty()) return std::numeric_limits<double>::infinity();
        const auto& e = p.back();
        const double dx = e[0] - goal_vec[0], dy = e[1] - goal_vec[1], dz = e[2] - goal_vec[2];
        return std::sqrt(dx * dx + dy * dy + dz * dz);
      };

      if (path_invalid || improve_run) {
        std::vector<std::vector<double>> candidate;
        // A path is needed now: a short budget. Improving one can take the full one.
        // A path is needed now: stop at RRT_REPLAN_SOLVE_TIME if one reaching the
        // goal has been found by then, else keep going to RRT_SOLVE_TIME. An
        // improve gets the full RRT_SOLVE_TIME.
        planner.setPlanningTime(search_cfg_.rrt_solve_time);
        planner.setEarlyStop(path_invalid ? search_cfg_.rrt_replan_solve_time : 0.0);
        const double t_search = now();
        search_running_.store(true);
        const bool solved = planner.planPath(start, goal_vec, candidate);
        search_running_.store(false);
        last_search_time_.store(now() - t_search);

        // A goal the collision check rejects is planned to at the nearest valid
        // point instead (see GeometricPlanner::projectGoal), so the drone will
        // deliberately stop short of what was commanded. Carried as a suffix on
        // this tick's plan line rather than a line of its own: it is a property of
        // the solve, and the searches are already rate-limited to the monitor /
        // improve cadences. Silent in the common case where the goal was valid.
        std::string goal_note;
        if (std::isinf(planner.lastGoalProjection())) {
          goal_note = " [goal (" + std::to_string(goal_vec[0]) + ", " +
                      std::to_string(goal_vec[1]) + ", " + std::to_string(goal_vec[2]) +
                      ") is inside an obstacle: no point clearing " +
                      std::to_string(planner.collisionMargin()) + "m nearby]";
        } else if (planner.lastGoalProjection() > 0.0) {
          const auto& pg = planner.lastPlanningGoal();
          std::ostringstream os;
          os << " [goal projected " << planner.lastGoalProjection() << "m to clear "
             << planner.collisionMargin() << "m -> (" << pg[0] << ", " << pg[1] << ", "
             << pg[2] << ")]";
          goal_note = os.str();
        }

        if (path_invalid) {
          // Forced replan — adopt unconditionally (safety outranks optimality).
          if (solved) {
            const auto cand = planner.costBreakdown(candidate);
            adopt(candidate);
            if (had_path)
              DRONE_LOG_INFO("[plan] #" << run << " " << planning::toString(search_cfg_.planner_type) <<" MONITOR committed BLOCKED (clr="
                             << committed_clr << "m < " << planner.collisionMargin()
                             << "m) -> REPLAN cost=" << fmtCost(cand) << " clr="
                             << planner.minClearance(candidate) << "m" << goal_note);
            else
              DRONE_LOG_INFO("[plan] #" << run << " " << planning::toString(search_cfg_.planner_type) <<" MONITOR no committed path -> PLAN cost="
                             << fmtCost(cand) << " clr="
                             << planner.minClearance(candidate) << "m" << goal_note);
          } else {
            DRONE_LOG_INFO("[plan] #" << run << " " << planning::toString(search_cfg_.planner_type) <<" MONITOR committed "
                           << (had_path ? "BLOCKED" : "absent")
                           << " -> REPLAN FAILED (no path to goal), holding" << goal_note);
          }
        } else {
          // Improve — switch only past the hysteresis margin. RRT* is randomised,
          // so without the margin a near-identical re-solve would chatter.
          //
          // Two things can make a candidate better. (1) Reach: in best-effort mode a
          // fresh solve may end closer to the goal than the committed path — new
          // space was mapped, so the frontier, and the reachable point nearest the
          // goal, moved forward. A candidate that closes the goal gap by more than
          // kGoalProgress is adopted outright: advancing toward the goal outranks
          // path cost, and its necessarily-longer route would otherwise lose the
          // cost test below precisely because it reaches further. (2) Cost, at
          // comparable reach: score the candidate against the committed path's
          // *remaining* cost from the drone's current position, not its full
          // original cost. Both are rooted at the drone (`start`), so the margin is
          // a fair like-for-like test; billing the full committed cost would let a
          // candidate win merely because the drone advanced (its root creeps toward
          // the goal while the committed cost still charges the traversed prefix).
          constexpr double kGoalProgress = 0.25;  // min gap reduction [m] to count as advancing
          const double committed_gap = gapToGoal(committed_path);
          const double cand_gap =
              solved ? planner.lastGoalGap() : std::numeric_limits<double>::infinity();
          const auto remaining = remainingCommittedSuffix(committed_path, start);
          const auto remaining_cb = planner.costBreakdown(remaining);
          const double remaining_cost = remaining_cb.total;
          const double threshold = search_cfg_.replan_improve_ratio * remaining_cost;
          const auto cand = planner.costBreakdown(candidate);
          if (solved && cand_gap < committed_gap - kGoalProgress) {
            adopt(candidate);
            DRONE_LOG_INFO("[plan] #" << run << " " << planning::toString(search_cfg_.planner_type) <<" IMPROVE best-effort ADVANCE gap "
                           << committed_gap << "m -> " << cand_gap << "m (goal) cost=" << fmtCost(cand) << " clr="
                           << planner.minClearance(candidate) << "m" << goal_note);
          } else if (solved && cand_gap <= committed_gap + kGoalProgress && cand.total <= threshold) {
            adopt(candidate);
            DRONE_LOG_INFO("[plan] #" << run << " " << planning::toString(search_cfg_.planner_type) <<" IMPROVE remaining cost=" << fmtCost(remaining_cb)
                           << " (committed=" << fmtCost(committed) << ") -> ADOPT cost=" << fmtCost(cand) << " clr="
                           << planner.minClearance(candidate) << "m" << goal_note);
          } else {
            DRONE_LOG_INFO("[plan] #" << run << " " << planning::toString(search_cfg_.planner_type) <<" IMPROVE remaining cost=" << fmtCost(remaining_cb)
                           << " clr=" << committed_clr << "m gap=" << committed_gap << "m candidate cost="
                           << (solved ? fmtCost(cand) : std::string("inf")) << " gap="
                           << (solved ? std::to_string(cand_gap) : std::string("inf"))
                           << "m -> keep" << goal_note);
          }
        }
      } else {
        // Monitor only: committed path still clear of obstacles. gap is how far the
        // committed path's end still is from the goal — ~0 once the goal is reached,
        // or the best-effort closest-approach distance while the drone is ratcheting
        // toward a goal beyond the mapped frontier.
        DRONE_LOG_INFO("[plan] #" << run << " " << planning::toString(search_cfg_.planner_type) <<" MONITOR committed cost=" << fmtCost(committed)
                       << " clr=" << committed_clr << "m gap=" << gapToGoal(committed_path) << "m -> OK");
      }

    } else if (!preset_active_.load()) {
      const Idle reason = !has_goal ? Idle::kNoGoal : !map ? Idle::kNoMap : Idle::kNoFrame;
      if (reason != idle || t - last_idle_log >= kIdleLogPeriod) {
        if (reason == Idle::kNoGoal) {
          DRONE_LOG_INFO("[plan] idle: no goal set — nothing to plan toward");
        } else if (reason == Idle::kNoMap) {
          DRONE_LOG_INFO("[plan] idle: goal set but no map yet — waiting for the first "
                         "octomap; the planner cannot run without one");
        } else {
          DRONE_LOG_INFO("[plan] idle: goal and map set but no map->world transform yet — "
                         "planning is paused until the host supplies one (is RTAB-Map publishing?)");
        }
        idle = reason;
        last_idle_log = t;
      }
    }

    // The loop ticks at the monitor cadence — every iteration is one monitor tick
    // — but wakes at once for a new goal or a divergence replan, which used to
    // wait up to a whole period for the next tick. Clamp to a small floor so a
    // mis-set period cannot turn this into a busy loop.
    const double wake = t + std::max(search_cfg_.rrt_monitor_period, 0.01);
    while (running_.load() && now() < wake) {
      {
        std::lock_guard<std::mutex> lock(io_mutex_);
        if (new_goal_) break;
      }
      if (search_replan_requested_.load() || search_stale_path_.load()) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
}

void AutonomyCore::recordStaged(const common::Trajectory& staged, const TrajgenInfo& info,
                                std::uint64_t path_version,
                                const std::vector<std::vector<double>>& path) {
  TrajRecord r;
  r.traj = staged;
  r.first_segment_end =
      staged.t0 + (staged.segment_times.empty() ? 0.0 : staged.segment_times.front());
  r.start_margin = info.start_margin;
  r.corridor = info.corridor;
  r.path_version = path_version;
  r.trunc_end_arc = arcLengthAlong(path, info.trunc_end);
  std::lock_guard<std::mutex> lock(traj_mutex_);
  records_.push_back(std::move(r));
  while (records_.size() > 2) records_.erase(records_.begin());
}

planning::TrajectoryCheckParams AutonomyCore::checkParams(const Config& c, double start_margin,
                                                          double first_segment_end,
                                                          double resolution) {
  planning::TrajectoryCheckParams p;
  p.margin = c.corridor_margin;
  p.start_margin = start_margin;
  p.first_segment_end = first_segment_end;
  // The same tolerance truncation's floor uses: one voxel or 5%, whichever is
  // more lenient.
  p.slack = resolution;
  p.rel = kTruncationTolerance;
  p.sample_step = 0.05;
  // Per-axis limits allow up to sqrt(3) x vmax in norm.
  p.max_speed = std::sqrt(3.0) * c.vmax;
  p.emergency_horizon = c.emergency_horizon;
  p.emergency_factor = c.emergency_factor;
  return p;
}

namespace {
// "EVASION+NEW_PLAN", "WAYPOINT", or "improve" when nothing was needed.
template <class N>
std::string needsName(const N& n) {
  std::string s;
  const auto add = [&s](const char* x) {
    if (!s.empty()) s += "+";
    s += x;
  };
  if (n.evasion) add("OBSTACLE_EVASION");
  if (n.new_plan) add("NEW_PLAN");
  if (n.waypoint) add("WAYPOINT_ADVANCED");
  return s.empty() ? std::string("improve") : s;
}
}  // namespace

void AutonomyCore::trajgenLoop() {
  while (running_.load()) {
    common::State state;
    planning::MapHandle map;
    ConsGridHandle conservative;
    bool preset_pending = false;
    std::vector<Eigen::Vector3d> preset_waypoints;
    // One snapshot per solve, used both to bring the state into the map frame and
    // to take the finished trajectory back out (see stagePlanned).
    Eigen::Isometry3d world_from_map = Eigen::Isometry3d::Identity();
    bool has_frame = false;
    {
      std::lock_guard<std::mutex> lock(io_mutex_);
      state = state_;
      world_from_map = world_from_map_;
      has_frame = has_map_to_world_;
      map = map_;
      conservative = conservative_map_;
      preset_pending = preset_pending_;
      preset_pending_ = false;
      if (preset_pending) preset_waypoints = preset_waypoints_;
      if (trajgen_config_dirty_) {
        cfg_ = pending_config_;
        trajgen_config_dirty_ = false;
      }
    }
    // Divergence is handled through the records and the splice epoch (see
    // stepControl); the monitor sees the trajectory gone and asks for a new one.
    trajgen_replan_requested_.store(false);

    const bool frame_ok = has_frame || !cfg_.require_map_to_world;

    // One-shot preset trajectory: bypass the geometric search entirely and solve
    // the corridor-QP trajectory once through the given waypoints, then hold it
    // (kept fresh by stepControl) until it completes. It lives on this thread
    // because it is trajectory generation; the search thread stands down while
    // preset_active_ is set, and firePreset dropped any goal.
    if (preset_pending && !frame_ok) {
      DRONE_LOG_INFO("[preset] ignored: no map->world transform yet — the preset square is in "
                     "the map frame and cannot be placed without it (is RTAB-Map publishing?)");
    } else if (preset_pending) {
      runPreset(state, world_from_map, map, conservative, preset_waypoints);
    }
    if (preset_active_.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      continue;
    }

    // Take the job the monitor posted, if any. The cancel flags are reset here,
    // at the START of a job, so a cancel aimed at the previous job that arrives
    // after it finished cannot hit this one.
    SolveJob job;
    bool have = false;
    {
      std::lock_guard<std::mutex> lock(job_mutex_);
      if (has_job_) {
        job = job_;
        has_job_ = false;
        have = true;
        solve_cancel_.store(false);
        solve_abort_.store(false);
      }
    }
    if (!have) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }

    SolveResult result;
    result.job = job;
    result.epoch = splice_epoch_.load();
    std::uint64_t version = 0;
    const std::vector<std::vector<double>> committed_path = committedPath(&version);
    result.version = version;
    result.world_from_map = world_from_map;
    result.path = committed_path;
    if (cfg_.plan_trajectory && map && frame_ok && !committed_path.empty()) {
      // Anchor the solve on the state the vehicle will be in when it engages, at
      // least kStageGap after the trajectory staged before it (which may still be
      // waiting in the tracker), and root the path there so the corridor is grown
      // around the point the trajectory actually starts from. Rooting at the
      // measured position instead would leave the splice point outside region 0
      // whenever there is tracking error, and the QP's start equality would then
      // be infeasible against region 0's faces.
      const double t_gen = now();
      double min_t0 = 0.0;
      {
        std::lock_guard<std::mutex> lock(traj_mutex_);
        if (has_last_planned_) min_t0 = last_planned_.t0 + kStageGap;
      }
      result.anchor = spliceAnchor(state, t_gen, world_from_map, min_t0);
      // Root the path at the anchor AND drop the waypoints already behind it.
      // Only overwriting the first waypoint (as this used to) leaves the path
      // doubling back to waypoints the vehicle has passed once it is beyond the
      // first one, so truncation and the corridor worked on a path that ran
      // backwards before going on (bench 2026-09-29: "root 3.17 m from the search
      // start", a 3.6 m path handed over as 5.8 m, corridors infeasible).
      const std::vector<double> root = {result.anchor.start.pos.x(), result.anchor.start.pos.y(),
                                        result.anchor.start.pos.z()};
      const std::vector<std::vector<double>> rooted =
          remainingCommittedSuffix(committed_path, root);
      // Braking stub. A splice pins the start velocity; when the new path leaves
      // at a sharp angle to it (a goal to the side or behind, at cruise), the
      // corridor grown along the new path gives the vehicle ~1 m to brake in its
      // old direction and the QP is infeasible at every time allocation (0.6 m/s
      // reversal, measured). Lead the path in with a straight stub along the
      // current velocity so the first corridor region holds the braking.
      // Truncation checks the stub like the rest of the path, so it never commits
      // into an obstacle; if there is no room to stop there, the solve fails.
      //
      // How long it must be is not the physical stopping distance: the Bezier hull
      // constraint is conservative, and on the bench (2026-09-29) stubs of 0.70-0.84
      // m (1.5x the stop) were QP-infeasible while 1.13 m worked. So the solve
      // tries a longer stub, in place, when one fails — each try ~0.1 s — rather
      // than failing back to the monitor, whose retries moved the anchor along the
      // old trajectory a little further from the (fixed) path every time.
      double stub_ideal = 0.0;
      Eigen::Vector3d stub_dir = Eigen::Vector3d::Zero();
      if (result.anchor.from_trajectory && rooted.size() >= 2) {
        const Eigen::Vector3d v = result.anchor.start.vel;
        const double speed = v.norm();
        const Eigen::Vector3d first =
            Eigen::Vector3d(rooted[1][0], rooted[1][1], rooted[1][2]) -
            Eigen::Vector3d(rooted[0][0], rooted[0][1], rooted[0][2]);
        constexpr double kStubMinSpeed = 0.1;  // [m/s]
        constexpr double kStubCosAngle = 0.5;  // path leaving > 60 deg off the velocity
        if (speed > kStubMinSpeed && first.norm() > 1e-6 &&
            v.dot(first) / (speed * first.norm()) < kStubCosAngle) {
          const double a = std::max(cfg_.amax, 1e-3), j = std::max(cfg_.jmax, 1e-3);
          const double ramp = speed >= a * a / j ? speed / a + a / j : 2.0 * std::sqrt(speed / j);
          stub_ideal = 0.5 * speed * ramp;  // jerk-limited stopping distance
          stub_dir = v / speed;
        }
      }
      const auto pathWithStub = [&](double factor) {
        std::vector<std::vector<double>> path = rooted;
        if (stub_ideal > 0.0) {
          const Eigen::Vector3d tip = Eigen::Vector3d(path[0][0], path[0][1], path[0][2]) +
                                      factor * stub_ideal * stub_dir;
          path.insert(path.begin() + 1, {tip.x(), tip.y(), tip.z()});
        }
        return path;
      };
      // How far the anchor is off the committed path, for the truncation log.
      const double root_shift =
          distanceToPath(committed_path, Eigen::Vector3d(root[0], root[1], root[2]));
      std::ostringstream stub_note;
      // Truncation stops at unobserved space only when the operator asked for
      // it. The predicate reads the STAMPED view: there the ball around the drone
      // has been marked free and the shell occupied, so "never observed" means
      // the same thing to truncation, the monitor and the corridor (whose
      // obstacles come from the same view).
      const double t_field = now();
      const auto cons_field = conservativeField(
          map, conservative, std::max(cfg_.clearance_threshold, cfg_.frontier_margin));
      const double field_time = now() - t_field;
      const auto unknown_fn = cfg_.treat_unknown_as_hazard ? makeUnknownFn(map, conservative)
                                                           : planning::CorridorUnknownFn{};
      const double factors_with_stub[] = {1.5, 2.25, 3.4};
      const int tries = stub_ideal > 0.0 ? 3 : 1;
      for (int k = 0; k < tries; ++k) {
        const double factor = stub_ideal > 0.0 ? factors_with_stub[k] : 0.0;
        trajgen_stub_len_ = factor * stub_ideal;
        result.ok = runTrajgen(pathWithStub(factor), result.anchor.t0, result.anchor.start,
                               cons_field, map, conservative, unknown_fn, result.traj,
                               /*pin_waypoints=*/false, root_shift, &result.info);
        if (stub_ideal > 0.0) {
          stub_note.str("");
          stub_note << " | braking stub " << factor * stub_ideal << " m (x" << factor << " the stop, try "
                    << k + 1 << ")";
        }
        // Retry longer only while the deadline allows: the anchor is fixed, so a
        // try must finish leaving the hand-over margin before it.
        if (result.ok || solve_cancel_.load() || result.anchor.t0 - now() < 0.5) break;
      }
      trajgen_stub_len_ = 0.0;
      result.cancelled = solve_cancel_.load();
      // A failure with the anchor far off the path means the path is stale (it was
      // planned from a predicted start the trajectory has since left): drop it so
      // the search re-plans from a fresh one, instead of retrying it.
      constexpr double kStalePathShift = 1.0;  // [m]
      if (!result.ok && !result.cancelled && root_shift > kStalePathShift) {
        DRONE_LOG_INFO("[trajgen] the splice point is " << root_shift
                       << " m off the committed path: dropping it so the search re-plans from "
                          "where the trajectory is now");
        search_stale_path_.store(true);
      }
      const double solve_time = now() - t_gen;
      DRONE_LOG_INFO("[trajgen] solve for " << needsName(job.needs) << ": " << solve_time
                     << " s (field " << field_time << " s, corridor " << trajgen_corridor_time_
                     << " s, QP " << trajgen_qp_time_ << " s) | "
                     << (result.anchor.from_trajectory ? "splice" : "rest") << ", starts "
                     << result.anchor.t0 - t_gen << " s after the solve began | search last "
                     << last_search_time_.load() << " s"
                     << (search_running_.load() ? ", one running now" : "") << stub_note.str()
                     << " | " << (result.cancelled ? "CANCELLED" : result.ok ? "OK" : "FAILED"));
    }
    std::lock_guard<std::mutex> lock(job_mutex_);
    result_ = std::move(result);
    has_result_ = true;
  }
}

// The trajectory monitor. It is the one place that decides anything about
// trajectories; the solver only runs the solves it is given. Each tick:
//   1. Work out the NEEDS of what the tracker will fly (the trajectory being
//      flown and any staged after it): OBSTACLE_EVASION (the latest one fails
//      its safety check on the current map), NEW_PLAN (it was built for an older
//      committed path, or there is none), WAYPOINT_ADVANCED (truncation's stop
//      point has moved on TRAJ_EXTEND_DIST, or at all within TRAJ_EXTEND_HORIZON
//      of its end). Plus an emergency stop when an unsafe point is too close.
//   2. Judge a result the solver just returned, and re-judge the one in the
//      waiting room, against those needs (see `judge` below). One that would go
//      through switches off the needs it satisfies; one that no longer
//      qualifies is scrapped. The waiting one is handed over once the
//      trajectory staged before it has engaged.
//   3. Health signal while neither OBSTACLE_EVASION nor NEW_PLAN is on.
//   4. Decide the solver: cancel the solve in flight if a need it cannot meet
//      has come up (see below), and start one, with a snapshot of the needs, if
//      any need is on or TRAJ_IMPROVE_PERIOD has passed since the last solve that
//      finished uninterrupted.
// It ticks at traj_monitor_rate, and at once when a result arrives or a
// hand-over is due, since those cannot wait up to a tick.
void AutonomyCore::monitorLoop() {
  bool in_flight = false;
  SolveJob flight;  // the monitor's copy, the one whose `needs` are authoritative
  std::uint64_t next_id = 1;
  bool waiting = false;
  SolveResult room;  // the waiting room
  double last_completed = -1.0e9;
  double last_failed = -1.0e9;  // when a solve last FAILED; see the retry gap below
  double next_tick = now();
  std::string last_needs = "";
  std::string last_evasion;
  // How long after the previous trajectory's start the tracker's slot is free
  // again: it promotes at t0 on its next control tick, and a hand-over read on
  // that same tick must not overwrite it first.
  constexpr double kSlotMargin = 0.05;
  constexpr double kWaypointTol = 0.1;  // [m] a stop point this close counts as reached

  const auto postJob = [&](const Needs& needs, std::uint64_t version) {
    flight = SolveJob{next_id++, needs, version};
    in_flight = true;
    std::lock_guard<std::mutex> lock(job_mutex_);
    job_ = flight;
    has_job_ = true;
  };
  const auto cancelJob = [&](const std::string& why) {
    DRONE_LOG_INFO("[trajmon] solve for " << needsName(flight.needs) << " CANCELLED: " << why);
    {
      std::lock_guard<std::mutex> lock(job_mutex_);
      has_job_ = false;  // not picked up yet: never start it
    }
    solve_cancel_.store(true);
    solve_abort_.store(true);
    in_flight = false;
  };

  while (running_.load()) {
    double t = now();

    // Collect a result — only the one for the job in flight; a cancelled job's
    // leftovers are ignored.
    SolveResult res;
    bool got = false;
    {
      std::lock_guard<std::mutex> lock(job_mutex_);
      if (has_result_) {
        res = std::move(result_);
        has_result_ = false;
        got = in_flight && res.job.id == flight.id;
      }
    }
    if (got) {
      in_flight = false;
      res.job = flight;  // its needs as the monitor last amended them
      if (!res.cancelled) last_completed = t;
    }
    double prev_t0 = -1.0e9;
    {
      std::lock_guard<std::mutex> lock(traj_mutex_);
      if (has_last_planned_) prev_t0 = last_planned_.t0;
    }
    const bool handover_due = waiting && t >= prev_t0 + kSlotMargin;
    if (!got && !handover_due && t < next_tick) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }

    planning::MapHandle map;
    ConsGridHandle conservative;
    Eigen::Isometry3d world_from_map = Eigen::Isometry3d::Identity();
    Eigen::Vector3d measured_pos = Eigen::Vector3d::Zero();  // world frame
    {
      std::lock_guard<std::mutex> lock(io_mutex_);
      map = map_;
      conservative = conservative_map_;
      world_from_map = world_from_map_;
      measured_pos = state_.pos;
      if (monitor_config_dirty_) {
        monitor_cfg_ = pending_config_;
        monitor_config_dirty_ = false;
      }
    }
    const Config& c = monitor_cfg_;
    if (t >= next_tick) next_tick = t + 1.0 / std::max(c.traj_monitor_rate, 0.5);
    if (!c.plan_trajectory || preset_active_.load() || !map) {
      waiting = false;
      continue;
    }

    // ---- 1. Needs of what the tracker will fly ------------------------------
    std::vector<TrajRecord> recs;
    {
      std::lock_guard<std::mutex> lock(traj_mutex_);
      while (records_.size() > 1 && records_[1].traj.t0 <= t) records_.erase(records_.begin());
      recs = records_;
    }
    std::uint64_t version = 0;
    const std::vector<std::vector<double>> path = committedPath(&version);
    const double maxd = std::max(c.clearance_threshold, c.frontier_margin);
    const Eigen::Isometry3d map_from_world = world_from_map.inverse();
    const bool corridor = c.use_corridor_qp;
    const auto field = corridor ? conservativeField(map, conservative, maxd) : nullptr;
    const planning::CorridorClearanceFn cons_clearance =
        field ? makeClearanceFn(field, maxd) : planning::CorridorClearanceFn{};
    // The trajectory check holds mapped obstacles to corridor_margin and
    // never-observed space to unknown_margin, as the corridor was built; the
    // WAYPOINT truncation re-run uses truncation's own split (see the solver).
    const auto obstacle_field =
        field && conservative ? clearanceField(map, c.clearance_threshold) : nullptr;
    const planning::CorridorClearanceFn clearance =
        field && conservative
            ? makeSplitClearanceFn(cons_clearance, obstacle_field, c.clearance_threshold,
                                   c.corridor_margin - c.unknown_margin)
            : cons_clearance;
    const planning::CorridorClearanceFn trunc_clearance =
        field && conservative
            ? makeSplitClearanceFn(cons_clearance, obstacle_field, c.clearance_threshold,
                                   c.frontier_margin - c.unknown_margin)
            : cons_clearance;
    // Read from the stamped view, like truncation's (see the solver).
    const planning::CorridorUnknownFn unknown =
        c.treat_unknown_as_hazard ? makeUnknownFn(map, conservative)
                                  : planning::CorridorUnknownFn{};

    Needs needs;
    bool emergency = false;
    std::string evasion_note;
    double trunc_arc_now = -1.0;  // truncation's stop point now, as arc length along the path
    if (recs.empty()) {
      needs.new_plan = !path.empty();
    } else {
      needs.new_plan = recs.back().path_version != version;
      if (field && recs.back().corridor) {
        // Each trajectory over the stretch it will actually be flown. Only a
        // failure in the LAST one is an evasion need: an earlier one's stretch is
        // flown before anything new could engage, so it is the emergency stop's.
        bool latest_fails = false;
        planning::TrajectoryCheck worst;
        bool any_fail = false;
        for (std::size_t i = 0; i < recs.size(); ++i) {
          const bool last = i + 1 == recs.size();
          const double until = last ? 0.0 : recs[i + 1].traj.t0;
          if (!last && until <= t) continue;
          auto p = checkParams(c, recs[i].start_margin, recs[i].first_segment_end,
                               map->getResolution());
          p.emergency_horizon =
              std::max(0.0, c.emergency_horizon - std::max(0.0, recs[i].traj.t0 - t));
          const auto check = planning::checkTrajectory(clearance, unknown, recs[i].traj,
                                                       map_from_world, t, until, p);
          if (!check.ok) {
            if (!any_fail || check.worst_time < worst.worst_time) worst = check;
            any_fail = true;
            if (last) latest_fails = true;
          }
          emergency = emergency || check.emergency;
        }
        needs.evasion = latest_fails;
        if (any_fail) {
          std::ostringstream os;
          os << (worst.unknown ? std::string("never-observed space")
                               : "clearance " + std::to_string(worst.worst_clearance) +
                                     " m < required " + std::to_string(worst.worst_required) + " m")
             << " " << worst.worst_time - t << " s ahead at (" << worst.worst_point.x() << ", "
             << worst.worst_point.y() << ", " << worst.worst_point.z() << ")";
          evasion_note = os.str();
        }
      }
      // WAYPOINT_ADVANCED: truncation re-run on the committed path, rooted where
      // the reference is now, against where it stopped when the latest
      // trajectory was built. Meaningless across a plan change.
      if (field && recs.back().corridor && !needs.new_plan && path.size() >= 2) {
        const TrajRecord& active =
            (recs.size() > 1 && t < recs.back().traj.t0) ? recs.front() : recs.back();
        // Rooted where the reference is now, with the waypoints behind it dropped
        // (see the solver). With bench_replan_from_state the solver starts every
        // trajectory from the measured position instead, and on a disarmed bench
        // the reference runs on while the drone stays put: rooted at the
        // reference, this asked for a stop point the solver, rooted at the drone,
        // could never reach, and every WAYPOINT solve was scrapped in a loop. So
        // root it where the solver will.
        const Eigen::Vector3d ref =
            map_from_world * (c.bench_replan_from_state ? measured_pos
                                                        : common::sampleMotion(active.traj, t).pos);
        std::vector<Eigen::Vector3d> epath;
        for (const auto& w : remainingCommittedSuffix(path, {ref.x(), ref.y(), ref.z()})) {
          epath.emplace_back(w[0], w[1], w[2]);
        }
        const auto committed = planning::truncatePath(
            trunc_clearance, epath, c.frontier_margin * (1.0 - kTruncationTolerance), c.escape_ramp_dist,
            0.05, unknown, nullptr, map->getResolution(), kTruncationTolerance);
        if (committed.size() >= 2) {
          constexpr double kMinAdvance = 0.05;  // below this it is noise, not progress [m]
          trunc_arc_now = arcLengthAlong(path, committed.back());
          const double advance = trunc_arc_now - recs.back().trunc_end_arc;
          const double time_left = recs.back().traj.t0 + recs.back().traj.total_duration - t;
          needs.waypoint = advance >= c.traj_extend_dist ||
                           (advance > kMinAdvance && time_left <= c.traj_extend_horizon);
        }
      }
    }

    // ---- 2. Judge results against the needs ---------------------------------
    // Whether `r` would go through now, and which needs it satisfies:
    //  - OBSTACLE_EVASION on: it must pass the check on the current map; it then
    //    goes through and satisfies EVASION, NEW_PLAN if it was asked for the
    //    current plan, WAYPOINT_ADVANCED if it reaches the advanced stop point.
    //  - else NEW_PLAN on: it goes through only if it was asked for the current
    //    plan, whatever the other flags say; otherwise it is scrapped.
    //  - else WAYPOINT_ADVANCED on: it goes through if it reaches the advanced
    //    stop point, or if it is better as an improve (below).
    //  - else: an improve — it goes through only if it reaches the current
    //    trajectory's stop point sooner.
    // Whatever the case it must pass the check on the current map, and it must
    // not have been spliced onto a trajectory since abandoned.
    const auto judge = [&](const SolveResult& r, Needs* satisfied, std::string* why) {
      *satisfied = Needs{};
      if (!r.ok) {
        *why = "the solve failed";
        return false;
      }
      if (r.epoch != splice_epoch_.load()) {
        *why = "the trajectory it was spliced onto was abandoned";
        return false;
      }
      if (field && r.info.corridor) {
        const auto check = planning::checkTrajectory(
            clearance, unknown, r.traj, Eigen::Isometry3d::Identity(), r.traj.t0, 0.0,
            checkParams(c, r.info.start_margin,
                        r.traj.t0 + (r.traj.segment_times.empty() ? 0.0
                                                                  : r.traj.segment_times.front()),
                        map->getResolution()));
        if (!check.ok) {
          std::ostringstream os;
          os << "not safe on the current map ("
             << (check.unknown ? std::string("never-observed space")
                               : "clearance " + std::to_string(check.worst_clearance) + " m < " +
                                     std::to_string(check.worst_required) + " m")
             << " " << check.worst_time - r.traj.t0 << " s into it)";
          *why = os.str();
          return false;
        }
      }
      const bool current_plan = r.version == version;
      const bool plan_ok = r.job.needs.new_plan && current_plan;
      const bool wp_ok = needs.waypoint && current_plan && trunc_arc_now >= 0.0 &&
                         arcLengthAlong(path, r.info.trunc_end) >= trunc_arc_now - kWaypointTol;
      // Improve: reaches the current trajectory's stop point sooner. Not a
      // comparison of total durations — one that goes further is longer overall.
      // One that does not pass within kReach of that stop point is not better.
      const auto better = [&](std::string* note) {
        if (recs.empty() || !current_plan) return false;
        const TrajRecord& cur = recs.back();
        constexpr double kReach = 0.1;        // [m]
        constexpr double kReachLoose = 0.25;  // closest approach still accepted [m]
        constexpr double kMinGain = 0.05;     // [s]
        const Eigen::Vector3d stop =
            map_from_world *
            common::sampleMotion(cur.traj, cur.traj.t0 + cur.traj.total_duration).pos;
        double best_d = std::numeric_limits<double>::infinity();
        double t_best = -1.0;
        double t_reach = -1.0;
        for (double tau = 0.0; tau <= r.traj.total_duration + 1e-9; tau += 0.02) {
          const double d = (common::sampleMotion(r.traj, r.traj.t0 + tau).pos - stop).norm();
          if (d < best_d) {
            best_d = d;
            t_best = tau;
          }
          if (d <= kReach) {
            t_reach = tau;
            break;
          }
        }
        if (t_reach < 0.0 && best_d <= kReachLoose) t_reach = t_best;
        std::ostringstream os;
        if (t_reach < 0.0) {
          os << "stops " << best_d << " m short of the current stop point";
          *note = os.str();
          return false;
        }
        const double arrival = r.traj.t0 + t_reach;
        const double current_arrival = cur.traj.t0 + cur.traj.total_duration;
        os << "reaches the current stop point in " << arrival - t << " s vs " << current_arrival - t
           << " s";
        *note = os.str();
        return arrival < current_arrival - kMinGain;
      };
      std::string note;
      if (needs.evasion) {
        satisfied->evasion = true;
        satisfied->new_plan = plan_ok;
        satisfied->waypoint = wp_ok;
        return true;
      }
      if (needs.new_plan) {
        if (!plan_ok) {
          *why = "NEW_PLAN is on and it was not built for the current plan";
          return false;
        }
        satisfied->new_plan = true;
        satisfied->waypoint = wp_ok;
        return true;
      }
      if (needs.waypoint) {
        if (wp_ok) {
          satisfied->waypoint = true;
          return true;
        }
        if (better(&note)) return true;
        *why = "does not reach the advanced stop point and is not better (" + note + ")";
        return false;
      }
      if (better(&note)) return true;
      *why = "not better (" + note + ")";
      return false;
    };
    const auto satisfy = [&needs](const Needs& s) {
      if (s.evasion) needs.evasion = false;
      if (s.new_plan) needs.new_plan = false;
      if (s.waypoint) needs.waypoint = false;
    };
    const auto handOver = [&]() {
      // A splice must still start at least kStageMargin from now, or it would
      // reach the tracker too late to engage where it was spliced.
      const double left = room.anchor.t0 - now();
      if (room.anchor.from_trajectory && left < kStageMargin) {
        DRONE_LOG_INFO("[trajmon] waiting " << needsName(room.job.needs)
                       << " trajectory SCRAPPED at hand-over: only " << left
                       << " s before its start (< " << kStageMargin << " s)");
        waiting = false;
        return;
      }
      const common::Trajectory staged = stagePlanned(room.anchor, room.traj, room.world_from_map);
      recordStaged(staged, room.info, room.version, room.path);
      DRONE_LOG_INFO("[trajmon] " << needsName(room.job.needs) << " trajectory STAGED: "
                     << staged.total_duration << " s, starts in " << staged.t0 - now() << " s ("
                     << (room.anchor.from_trajectory ? "splice" : "rest") << ")");
      waiting = false;
    };

    if (got && !res.cancelled) {
      Needs s;
      std::string why;
      if (judge(res, &s, &why)) {
        satisfy(s);
        room = std::move(res);
        waiting = true;
        DRONE_LOG_INFO("[trajmon] result for " << needsName(room.job.needs)
                       << " ACCEPTED, satisfies " << (s.any() ? needsName(s) : std::string("none (better)"))
                       << " -> waiting room");
      } else {
        DRONE_LOG_INFO("[trajmon] result for " << needsName(res.job.needs) << " SCRAPPED: " << why);
        if (!res.ok) last_failed = t;
      }
    } else if (waiting) {
      // Re-judged every tick it waits: the needs may have changed since.
      Needs s;
      std::string why;
      if (judge(room, &s, &why)) {
        satisfy(s);
      } else {
        DRONE_LOG_INFO("[trajmon] waiting " << needsName(room.job.needs)
                       << " trajectory SCRAPPED: " << why);
        waiting = false;
      }
    }
    if (waiting) {
      {
        std::lock_guard<std::mutex> lock(traj_mutex_);
        if (has_last_planned_) prev_t0 = last_planned_.t0;
      }
      t = now();
      if (t >= prev_t0 + kSlotMargin) handOver();
    }

    // ---- 3. Emergency and health --------------------------------------------
    if (emergency) {
      DRONE_LOG_ERROR("[trajmon] EMERGENCY STOP: " << evasion_note);
      emergency_request_.store(true);
      // Everything in flight, waiting or staged was spliced onto the trajectory
      // being abandoned: drop it all here (not in stepControl, which does not run
      // on a disarmed bench), so the next trajectory starts at rest.
      if (in_flight) cancelJob("emergency stop");
      waiting = false;
      splice_epoch_.fetch_add(1);
      std::lock_guard<std::mutex> lock(traj_mutex_);
      has_pending_ = false;
      has_last_planned_ = false;
      last_planned_ = common::Trajectory{};
      records_.clear();
    }
    if (!recs.empty() && !needs.evasion && !needs.new_plan) heartbeat_.store(true);

    // ---- 4. Decide the solver -----------------------------------------------
    if (in_flight) {
      const Needs& f = flight.needs;
      if (!f.any() && needs.any()) {
        cancelJob("an improve, and " + needsName(needs) + " came up");
      } else if (!f.evasion && !f.new_plan && f.waypoint && (needs.evasion || needs.new_plan)) {
        cancelJob("WAYPOINT_ADVANCED only, and " + needsName(needs) + " came up");
      } else if (f.new_plan && version != flight.version) {
        if (f.evasion) {
          // Evasion is never cut short: let it finish, it can still go through
          // as an evasion. It no longer satisfies NEW_PLAN, so that stays on.
          flight.needs.new_plan = false;
          DRONE_LOG_INFO("[trajmon] a newer plan arrived during an OBSTACLE_EVASION solve: it "
                         "keeps running for the evasion; NEW_PLAN stays on");
        } else {
          cancelJob("a newer plan arrived");
        }
      }
    }
    // A failed solve is not retried at once: the same inputs fail the same way, and
    // the anchor moves along the outgoing trajectory meanwhile. The gap gives the
    // map, the path or the trajectory time to change.
    constexpr double kRetryGap = 0.3;  // [s]
    if (!in_flight && !waiting && !path.empty() && !emergency && t - last_failed >= kRetryGap) {
      if (needs.any()) {
        postJob(needs, version);
        DRONE_LOG_INFO("[trajmon] solve started for " << needsName(needs)
                       << (needs.evasion ? " (" + evasion_note + ")" : std::string()));
      } else if (!recs.empty() && t - last_completed >= c.traj_improve_period &&
                 t < recs.back().traj.t0 + recs.back().traj.total_duration) {
        // An improve is judged on reaching the current stop point sooner, which a
        // trajectory that has already arrived cannot be beaten on.
        postJob(needs, version);
      }
    }

    const std::string now_needs = needs.any() ? needsName(needs) : std::string("none");
    if (now_needs != last_needs) {
      DRONE_LOG_INFO("[trajmon] needs: " << now_needs
                     << (needs.evasion || needs.new_plan ? " (no health signal)" : ""));
      last_needs = now_needs;
    }
  }
}

}  // namespace drone_core::autonomy
