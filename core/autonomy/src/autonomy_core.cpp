#include "drone_core/autonomy/autonomy_core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <ompl/util/Console.h>

#include "drone_core/common/logging.hpp"
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

double steadyNowSeconds() {
  const auto t = std::chrono::steady_clock::now().time_since_epoch();
  return std::chrono::duration<double>(t).count();
}

// Build a Euclidean distance transform over the occupancy octree, capped at
// maxdist (metres) so its cost is bounded by the obstacle envelope rather than
// the whole volume. getDistance() then gives O(1) clearance lookups. Points
// outside the map's bounding box read as "no obstacle within maxdist", matching
// the planner's treat-unknown-as-free policy.
std::shared_ptr<DynamicEDTOctomap> buildEdt(const std::shared_ptr<octomap::OcTree>& tree,
                                            double maxdist) {
  double xmin, ymin, zmin, xmax, ymax, zmax;
  tree->getMetricMin(xmin, ymin, zmin);
  tree->getMetricMax(xmax, ymax, zmax);
  const octomap::point3d bbx_min(static_cast<float>(xmin), static_cast<float>(ymin),
                                 static_cast<float>(zmin));
  const octomap::point3d bbx_max(static_cast<float>(xmax), static_cast<float>(ymax),
                                 static_cast<float>(zmax));
  auto edt = std::make_shared<DynamicEDTOctomap>(static_cast<float>(maxdist), tree.get(),
                                                 bbx_min, bbx_max,
                                                 /*treatUnknownAsOccupied=*/false);
  edt->update();
  return edt;
}

// Wrap a distance field as a clearance oracle. The planner's objective, the
// planner's validity check and the corridor stages all take the same
// std::function signature and all need the same out-of-bounds convention:
// getDistance returns a negative sentinel outside the field's bounding box,
// which means nothing is mapped nearby, so report the saturation distance
// instead. Shared rather than written out at each call site so the convention
// cannot drift apart between them.
planning::CorridorClearanceFn makeClearanceFn(std::shared_ptr<DynamicEDTOctomap> edt,
                                              double maxd) {
  return [edt = std::move(edt), maxd](double x, double y, double z) {
    const double d = edt->getDistance(octomap::point3d(
        static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)));
    return d < 0.0 ? maxd : d;
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
    : cfg_(config), clock_(steadyNowSeconds) {
  tracker_.setPositionGains(cfg_.pos_p);
  tracker_.setVelocityGains(cfg_.vel_p, cfg_.vel_i, cfg_.vel_d);
  tracker_.setHoverThrust(cfg_.hover_thrust);
  tracker_.enableFeedforward(cfg_.enable_feedforward);
  tracker_.setStaleTimeout(cfg_.stale_timeout);

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

void AutonomyCore::setState(const common::State& state) {
  std::lock_guard<std::mutex> lock(io_mutex_);
  state_ = state;
}

void AutonomyCore::setMap(const planning::MapHandle& map,
                          const planning::MapHandle& conservative) {
  std::lock_guard<std::mutex> lock(io_mutex_);
  map_ = map;
  conservative_map_ = conservative;
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

void AutonomyCore::applyConfig(const Config& config) {
  std::lock_guard<std::mutex> lock(io_mutex_);
  pending_config_ = config;
  config_dirty_ = true;
}

void AutonomyCore::reset() {
  tracker_.reset();
  // The tracker just dropped its trajectories, so there is nothing left to
  // splice onto. Clearing this makes the next replan anchor at rest on the
  // measured position, which is what a fresh engage needs.
  std::lock_guard<std::mutex> lock(traj_mutex_);
  has_pending_ = false;
  has_last_planned_ = false;
  last_planned_ = common::Trajectory{};
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
    config = pending_config_;
    config_dirty = config_dirty_;
    config_dirty_ = false;
  }

  if (config_dirty) {
    cfg_ = config;
    tracker_.setPositionGains(cfg_.pos_p);
    tracker_.setVelocityGains(cfg_.vel_p, cfg_.vel_i, cfg_.vel_d);
    tracker_.setHoverThrust(cfg_.hover_thrust);
    tracker_.enableFeedforward(cfg_.enable_feedforward);
    tracker_.setStaleTimeout(cfg_.stale_timeout);
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

  return tracker_.update(state, t, dt);
}

void AutonomyCore::startPlanner() {
  if (running_.exchange(true)) return;
  worker_ = std::thread(&AutonomyCore::plannerLoop, this);
}

void AutonomyCore::stopPlanner() {
  if (!running_.exchange(false)) return;
  if (worker_.joinable()) worker_.join();
}

bool AutonomyCore::planOnce() {
  common::State state;
  planning::MapHandle map;
  planning::MapHandle conservative;
  common::Goal goal;
  bool has_goal = false;
  {
    std::lock_guard<std::mutex> lock(io_mutex_);
    state = state_;
    map = map_;
    conservative = conservative_map_;
    goal = goal_;
    has_goal = has_goal_;
  }

  if (!has_goal || !map) return false;

  std::vector<std::vector<double>> path;
  if (!runGlobalPlan(state, goal, map, path)) return false;

  // Same splice anchoring as the worker's trajgen tick — see spliceAnchor.
  const SpliceAnchor anchor = spliceAnchor(state, now());
  if (!path.empty()) {
    path.front() = {anchor.start.pos.x(), anchor.start.pos.y(), anchor.start.pos.z()};
  }

  common::Trajectory traj;
  const planning::MapHandle cons = conservative ? conservative : map;
  // Truncation stops at unobserved space only when the operator asked for it —
  // see the worker's copy of this for why it keys off the flag and not off
  // whether a conservative view exists. The predicate reads the RAW map.
  if (!runTrajgen(path, anchor.t0, anchor.start, conservativeField(cons), cons,
                  cfg_.treat_unknown_as_hazard ? makeUnknownFn(map)
                                               : planning::CorridorUnknownFn{},
                  traj))
    return false;

  stagePending(traj);
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

std::vector<std::vector<double>> AutonomyCore::geometricPath() const {
  std::lock_guard<std::mutex> lock(traj_mutex_);
  return last_geometric_path_;
}

planning::GeometricPlanner::SearchTree AutonomyCore::searchTree() const {
  std::lock_guard<std::mutex> lock(traj_mutex_);
  return last_search_tree_;
}

std::vector<std::array<double, 4>> AutonomyCore::clearanceSamples() const {
  std::lock_guard<std::mutex> lock(traj_mutex_);
  return last_clearance_samples_;
}

AutonomyCore::CorridorSnapshot AutonomyCore::corridorSnapshot() const {
  std::lock_guard<std::mutex> lock(traj_mutex_);
  return last_corridor_;
}

std::vector<std::array<double, 4>> AutonomyCore::sampleClearanceField() const {
  std::vector<std::array<double, 4>> out;

  // Sample whichever field the cost is actually scored against, since tuning
  // clearance_weight / clearance_threshold by eye only works if the picture
  // shows what the objective sees. That is the conservative field whenever one
  // exists as a distinct view, and the search field otherwise. Both are current
  // for this tick: applyClearanceObjective runs before this and populates them.
  const bool use_cons = cons_edt_ && cons_edt_source_map_ &&
                        cons_edt_source_map_ != edt_source_map_;
  const auto& field = use_cons ? cons_edt_ : edt_;
  const auto& source = use_cons ? cons_edt_source_map_ : edt_source_map_;
  if (!field || !source) return out;

  double xmin, ymin, zmin, xmax, ymax, zmax;
  source->getMetricMin(xmin, ymin, zmin);
  source->getMetricMax(xmax, ymax, zmax);

  constexpr double step = 0.15;  // grid spacing [m] — coarse, debug-only
  const double maxd = cfg_.clearance_threshold;
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
  planning::GeometricPlanner planner(map, cfg_.rrt_solve_time);
  planner.setPlannerType(cfg_.planner_type);
  planner.setBestEffort(cfg_.best_effort_goal);
  const std::vector<double> start = {state.pos.x(), state.pos.y(), state.pos.z()};
  const std::vector<double> goal_vec = {goal.pos.x(), goal.pos.y(), goal.pos.z()};
  return planner.planPath(start, goal_vec, path);
}

bool AutonomyCore::runTrajgen(const std::vector<std::vector<double>>& path, double t0,
                              const common::MotionState& start,
                              const std::shared_ptr<DynamicEDTOctomap>& cons_edt,
                              const planning::MapHandle& cons_map,
                              const planning::CorridorUnknownFn& unknown_fn,
                              common::Trajectory& traj) {
  if (path.size() < 2) return false;

  if (cfg_.use_corridor_qp && cons_edt && cons_map) {
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

    planning::CorridorParams params;
    params.max_segment_len = cfg_.max_segment_len;
    params.margin = cfg_.corridor_margin;
    params.local_bbox = cfg_.corridor_bbox;
    // Same distance the truncation ramp uses, and for the same reason: it is
    // how far from the drone we accept reduced clearance in exchange for being
    // able to move at all. Sharing the parameter keeps the two stages from
    // disagreeing about where the vehicle stops being a special case.
    params.start_relax_dist = cfg_.escape_ramp_dist;
    // Obstacle points are voxel centres; the corridor must clear the voxel's
    // worst-case corner, so tell it the map's half-diagonal.
    params.voxel_half_diagonal = cons_map->getResolution() * std::sqrt(3.0) / 2.0;

    // Truncation enforces the configured frontier margin exactly — it is NOT
    // floored by the corridor margin, so FRONTIER_MARGIN means what it says and
    // the two stages can be tuned independently. Whatever it is set to, a
    // committed point must still have strictly positive clearance, so the
    // prefix can never reach into an occupied or unknown voxel.
    // When the caller supplied the unknown predicate it is an absolute stop,
    // exempt from the escape ramp: the ramp trades margin for mobility against a
    // hazard whose distance we can measure, and unobserved space is not that.
    // When it did not — TREAT_FRONTIER_AS_OBSTACLE off — truncation is purely
    // the clearance walk it always was, and unmapped space reads as free.
    const auto committed =
        planning::truncatePath(cons_fn, epath, cfg_.frontier_margin, cfg_.escape_ramp_dist,
                               /*sample_step=*/0.05, unknown_fn);

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
      const char* why = "clearance below the ramped margin";
      if (unknown_fn && epath.size() >= 2) {
        const Eigen::Vector3d d = epath[1] - epath[0];
        const double n = d.norm();
        if (n > 1e-9) {
          const Eigen::Vector3d q = epath[0] + (0.05 / n) * d;
          if (unknown_fn(q.x(), q.y(), q.z())) why = "path leaves observed space at the drone";
        }
      }
      DRONE_LOG_INFO("[trajgen] corridor: truncation empty (" << why
                     << ") -> no new trajectory");
      return false;
    }
    snapshotCommitted(committed);

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
    // unknown space — worth a line, since the endpoint it picked is where the
    // drone will actually fly this cycle. Silent when the whole path survives.
    if ((committed.back() - epath.back()).norm() > 1e-6) {
      DRONE_LOG_INFO("[trajgen] corridor: truncated to " << committed.size() << " wp / "
                     << polylineLength(committed) << " m of " << polylineLength(epath)
                     << " m (frontier margin " << cfg_.frontier_margin << " m)");
    }

    // Obstacle points for the decomposition: occupied leaves of the
    // conservative map (real obstacles AND stamped frontier) within a window
    // around the committed prefix. Windowed on purpose — a whole room at 5 cm
    // is 1e5-1e6 voxels and DecompUtil scans the list per segment, which would
    // never fit the 1 Hz trajgen budget. The window is the prefix's AABB grown
    // by the region-growth extent plus the margin, so nothing that could bound
    // a region is missed. Coarse (unpruned) leaves are expanded to resolution
    // voxels, mirroring toOcTree() in the ROS node, so a single big leaf does
    // not under-represent a solid block as one point.
    const auto obstacles = [&]() {
      Eigen::Vector3d lo = committed.front(), hi = committed.front();
      for (const auto& w : committed) {
        lo = lo.cwiseMin(w);
        hi = hi.cwiseMax(w);
      }
      const double pad = planning::corridorObstacleWindowPad(params);
      lo.array() -= pad;
      hi.array() += pad;
      const double res = cons_map->getResolution();
      std::vector<Eigen::Vector3d> pts;
      const octomap::point3d bmin(static_cast<float>(lo.x()), static_cast<float>(lo.y()),
                                  static_cast<float>(lo.z()));
      const octomap::point3d bmax(static_cast<float>(hi.x()), static_cast<float>(hi.y()),
                                  static_cast<float>(hi.z()));
      for (auto it = cons_map->begin_leafs_bbx(bmin, bmax), end = cons_map->end_leafs_bbx();
           it != end; ++it) {
        if (!cons_map->isNodeOccupied(*it)) continue;
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
    if (!planning::buildCorridor(obstacles, committed, params, resampled, regions, &why,
                                 cfg_.debug_planner_viz ? &attempt : nullptr, &start_margin)) {
      snapshotRegions(attempt, /*accepted=*/false);
      DRONE_LOG_INFO("[trajgen] corridor: decomposition FAILED (" << why << ") over "
                     << committed.size() << " wp / " << polylineLength(committed)
                     << " m — tightest conservative clearance "
                     << minConservativeClearance(committed) << " m vs required " << params.margin
                     << " m (CORRIDOR_MARGIN), " << obstacles.size()
                     << " obstacle pts -> no new trajectory");
      return false;
    } else {
      // A corridor bought with reduced clearance around the vehicle is not the
      // same event as one that cleared the full margin, so it never passes
      // silently — this is the one number that says how much protection the
      // first stretch of the flown trajectory actually has. Logged
      // unconditionally (not behind debug_planner_viz, unlike the OK line): it
      // is a safety fact, not a debugging aid.
      if (start_margin < params.margin - 1e-6) {
        DRONE_LOG_INFO("[trajgen] corridor: start margin relaxed to " << start_margin
                       << " m (of " << params.margin << " m) over the first "
                       << params.start_relax_dist
                       << " m — the drone is hemmed in; full margin applies beyond that");
      }
      planning::CorridorTrajectoryOptimizer optimizer(
          planning::CorridorLimits{cfg_.vmax, cfg_.amax, cfg_.jmax});
      snapshotRegions(attempt, /*accepted=*/true);
      // `start` carries the splice state. Its position is path.front() by
      // construction (the caller rooted the path there), so it satisfies
      // regions[0]; the derivatives are what make the engage continuous.
      if (optimizer.optimizeTrajectory(start, resampled, regions, traj)) {
        traj.t0 = t0;
        if (cfg_.debug_planner_viz) {
          DRONE_LOG_INFO("[trajgen] corridor: OK " << regions.size() << " regions / "
                         << polylineLength(committed) << " m / " << traj.total_duration << " s");
        }
        return true;
      }
      DRONE_LOG_INFO("[trajgen] corridor: QP INFEASIBLE over " << regions.size() << " regions / "
                     << polylineLength(committed) << " m — corridor built but no trajectory "
                     << "fits it within VMAX/AMAX/JMAX -> no new trajectory");
    }

    // Every corridor failure path ends here, and it stages NOTHING. There used
    // to be a plain min-snap fallback on the truncated prefix, which was a
    // mistake: min-snap knows nothing about obstacles, so the one situation
    // that produced it — the corridor stage saying it cannot guarantee a safe
    // trajectory — is exactly the situation in which an unchecked polynomial is
    // least defensible. Staging nothing means the tracker rides out whatever it
    // already has and, if this persists past STALE_TIMEOUT, latches the current
    // position and holds. Standing still is the only honest answer when the
    // corridor cannot certify moving. Note the caller does not advance
    // last_trajgen_ on false, so this retries next tick and recovers the moment
    // the map or the path allows a corridor again.
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
  return true;
}

void AutonomyCore::stagePending(const common::Trajectory& traj) {
  std::lock_guard<std::mutex> lock(traj_mutex_);
  pending_ = traj;
  last_planned_ = traj;
  last_planned_at_ = now();
  has_last_planned_ = true;
  has_pending_ = true;
}

AutonomyCore::SpliceAnchor AutonomyCore::spliceAnchor(const common::State& state,
                                                      double t_now) const {
  SpliceAnchor anchor;
  anchor.t0 = t_now + trajgen_lead_;

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

  // Only splice onto a trajectory the tracker is still following. Past the stale
  // timeout it has latched a hover, so the vehicle is no longer on that curve
  // and matching its state would step the reference rather than smooth it.
  const bool still_tracked = have && !outgoing.empty() &&
                             (t_now - staged_at) <= cfg_.stale_timeout;
  if (still_tracked) {
    anchor.start = common::sampleMotion(outgoing, anchor.t0);
    anchor.from_trajectory = true;
  } else {
    // Rest at the measured position. Note sampleMotion would also return rest if
    // the outgoing trajectory had simply run out (it ends at rest by
    // construction) — this branch is for having no usable outgoing curve at all.
    anchor.start.pos = state.pos;
    anchor.from_trajectory = false;
  }
  return anchor;
}

std::shared_ptr<DynamicEDTOctomap> AutonomyCore::clearanceField(
    const planning::MapHandle& map) {
  // No obstacles => clearance is uniform => no field needed (validity treats
  // everything as free, cost reduces to length). Drop any stale cache.
  if (!map || map->size() == 0) {
    edt_.reset();
    edt_source_map_.reset();
    return nullptr;
  }
  // Rebuild only when the map object itself changed. This is what makes the EDT
  // collision check cheaper than the per-state octree scan it replaces: a static
  // scene reuses one field across many ticks instead of rebuilding it each time.
  if (map != edt_source_map_) {
    edt_ = buildEdt(map, cfg_.clearance_threshold);
    edt_source_map_ = map;
  }
  return edt_;
}

std::shared_ptr<DynamicEDTOctomap> AutonomyCore::conservativeField(
    const planning::MapHandle& map) {
  if (!map || map->size() == 0) {
    cons_edt_.reset();
    cons_edt_source_map_.reset();
    return nullptr;
  }
  // No frontier information => the conservative view IS the search map; reuse
  // its field rather than building a second identical EDT.
  if (map == edt_source_map_ && edt_) return edt_;
  if (map != cons_edt_source_map_) {
    cons_edt_ = buildEdt(map, std::max(cfg_.clearance_threshold, cfg_.frontier_margin));
    cons_edt_source_map_ = map;
  }
  return cons_edt_;
}

bool AutonomyCore::applyClearanceObjective(planning::GeometricPlanner& planner,
                                           const planning::MapHandle& map,
                                           const planning::MapHandle& cons) {
  auto edt = clearanceField(map);
  if (!edt) return false;
  // The search map's field drives the collision check (clearance > margin). It
  // has to be this view and not the conservative one, because the conservative
  // view stamps the frontier as occupied and a validity check against that
  // would wall the search inside the mapped region entirely.
  planner.setClearance(makeClearanceFn(edt, cfg_.clearance_threshold),
                       cfg_.clearance_weight, cfg_.clearance_threshold);

  // The cost, however, is scored against the conservative field whenever one
  // exists as a distinct map. The reason is that the search and the downstream
  // truncation were previously measuring different things: truncation cuts the
  // committed prefix where conservative clearance falls below frontier_margin,
  // while the cost only ever saw the optimistic field, so the search had no way
  // to know what would get it cut and happily returned paths that ran tangent
  // to the frontier. Scoring both against the same field aligns them, and the
  // prefix that survives truncation gets longer as a result.
  //
  // The frontier is stamped as a thin shell of known-free voxels bordering
  // unknown space, not as the whole unknown volume, so this field is
  // min(distance to a real obstacle, distance to that shell). It is therefore a
  // strict refinement of the optimistic field: identical wherever the shell is
  // not the nearest source, and additionally repulsive near the shell. Nothing
  // about real-obstacle avoidance is lost by scoring against it.
  //
  // Skipped when the two views are the same object, which is the case with
  // frontier stamping off and in the legacy non-corridor mode where the search
  // already runs on the conservative view. There the override would be a no-op
  // at best, and calling conservativeField would just hand back the same field.
  if (cons && cons != map) {
    if (auto cons_edt = conservativeField(cons)) {
      planner.setCostClearance(makeClearanceFn(cons_edt, cfg_.clearance_threshold));
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
  if (cfg_.treat_unknown_as_hazard) {
    planner.setUnknownPenalty(makeUnknownFn(map), cfg_.unknown_weight);
  }
  return true;
}

void AutonomyCore::plannerLoop() {
  std::uint64_t run = 0;  // planner ticks taken (advanced only while a goal and map exist)

  // Idle diagnostics. A tick that cannot plan produces no [plan] line at all,
  // which from outside is indistinguishable from a crashed worker — "I sent a
  // goal and nothing happens" has two very different causes (no goal reached
  // the core, or no map has). Name the missing precondition instead of going
  // quiet: log on every change of reason, then periodically so a persistent
  // stall stays visible without spamming at the tick rate. Worker-thread
  // locals, so this costs nothing once planning is running.
  enum class Idle { kNone, kNoGoal, kNoMap };
  Idle idle = Idle::kNone;
  double last_idle_log = -1.0e9;

  while (running_.load()) {
    const double t = now();

    common::State state;
    planning::MapHandle map;
    planning::MapHandle conservative;
    common::Goal goal;
    bool has_goal = false;
    bool new_goal = false;
    {
      std::lock_guard<std::mutex> lock(io_mutex_);
      state = state_;
      map = map_;
      conservative = conservative_map_;
      goal = goal_;
      has_goal = has_goal_;
      new_goal = new_goal_;
      new_goal_ = false;
    }

    // A new goal invalidates the committed path regardless of its collision
    // validity (the monitor check below only tests for collisions, not whether
    // the path still targets the current goal). Drop it here on the worker
    // thread so the tick below replans from the drone's current position toward
    // the new goal.
    if (new_goal) cached_path_.clear();

    if (has_goal && map) {
      ++run;
      const std::vector<double> start = {state.pos.x(), state.pos.y(), state.pos.z()};
      const std::vector<double> goal_vec = {goal.pos.x(), goal.pos.y(), goal.pos.z()};

      // Commit a path: keep it on the worker and republish for visualisation.
      auto adopt = [this](const std::vector<std::vector<double>>& p) {
        cached_path_ = p;
        std::lock_guard<std::mutex> lock(traj_mutex_);
        last_geometric_path_ = p;
      };

      // Which map view the geometric search (and the committed-path monitor)
      // runs on. Corridor mode searches the OPTIMISTIC view — unknown reads as
      // free, so the informed planners (BIT*/AIT*/EIT*) accept a goal beyond
      // the mapped frontier instead of refusing it ("no goal states
      // available"); safety against unknown space is enforced downstream by
      // truncation + corridor, not by the search. Without corridor mode the
      // legacy single-map behavior is preserved: the conservative
      // (frontier-stamped) view, when present, is the one obstacle model.
      const planning::MapHandle search_map =
          (!cfg_.use_corridor_qp && conservative) ? conservative : map;

      // One planner per tick. Set the clearance fields FIRST, so the
      // committed-path re-check below sees the same obstacle model the search
      // would. The search map's field is the obstacle model for the collision
      // check; the conservative field, when it is a distinct view, additionally
      // scores the cost so the search is repelled by the frontier it would
      // otherwise skim (see applyClearanceObjective). Both are cached and
      // rebuilt only on a map change, so this is cheap on the common
      // still-valid tick.
      planning::GeometricPlanner planner(search_map, cfg_.rrt_solve_time);
      planner.setPlannerType(cfg_.planner_type);
      planner.setBestEffort(cfg_.best_effort_goal);
      planner.setRecordTree(cfg_.debug_planner_viz);
      applyClearanceObjective(planner, search_map, conservative);

      // Debug-only: re-sample the clearance field when the map changes (the EDT
      // is now current for this tick). Gated so a regular flight never walks the
      // grid. Sampling only on map change keeps even a debug run cheap on a
      // static scene.
      if (cfg_.debug_planner_viz && search_map != viz_sampled_map_) {
        auto samples = sampleClearanceField();
        {
          std::lock_guard<std::mutex> lock(traj_mutex_);
          last_clearance_samples_ = std::move(samples);
        }
        viz_sampled_map_ = search_map;
      }

      const bool had_path = !cached_path_.empty();
      const bool path_invalid = !had_path || !planner.isPathValid(cached_path_);

      // IMPROVE every Nth tick, N = improve period / monitor period (≈10 with the
      // defaults), so the RRT_MONITOR_PERIOD / RRT_IMPROVE_PERIOD params still set
      // both cadences. Skipped with no committed path or no obstacles mapped
      // (clearance is then uniform, so there is nothing to improve — a blocked path
      // still replans below regardless).
      const long n = std::lround(cfg_.rrt_improve_period /
                                 std::max(cfg_.rrt_monitor_period, 1.0e-3));
      const std::uint64_t improve_interval = static_cast<std::uint64_t>(std::max<long>(1, n));
      const bool improve_run = (run % improve_interval == 0) && had_path && map->size() > 0;

      // Diagnostics on the committed path (the field is already set). The cost is
      // split into its length + clearance-penalty terms for the logs below;
      // committed_clr is the tightest distance to a wall along the path.
      const auto committed = planner.costBreakdown(cached_path_);
      const double committed_clr = planner.minClearance(cached_path_);

      // Stream a cost as "total (len=L + clr_cost=C + unk_cost=U)". The unknown
      // term is shown only when it is being charged, so the common line stays
      // as it was — but when a route does leave mapped space, the number that
      // explains the score is right there rather than hidden inside clr_cost.
      auto fmtCost = [](const planning::GeometricPlanner::CostBreakdown& cb) {
        std::ostringstream os;
        os << cb.total << " (len=" << cb.length << " + clr_cost=" << cb.clearance;
        if (cb.unknown > 0.0) os << " + unk_cost=" << cb.unknown;
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
        const bool solved = planner.planPath(start, goal_vec, candidate);

        // Debug-only: capture the tree this solve built (even if it failed — that
        // is exactly when seeing where it explored is most useful).
        if (cfg_.debug_planner_viz) {
          std::lock_guard<std::mutex> lock(traj_mutex_);
          last_search_tree_ = planner.searchTree();
        }

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
              DRONE_LOG_INFO("[plan] #" << run << " " << planning::toString(cfg_.planner_type) <<" MONITOR committed BLOCKED (clr="
                             << committed_clr << "m < " << planner.collisionMargin()
                             << "m) -> REPLAN cost=" << fmtCost(cand) << " clr="
                             << planner.minClearance(candidate) << "m" << goal_note);
            else
              DRONE_LOG_INFO("[plan] #" << run << " " << planning::toString(cfg_.planner_type) <<" MONITOR no committed path -> PLAN cost="
                             << fmtCost(cand) << " clr="
                             << planner.minClearance(candidate) << "m" << goal_note);
          } else {
            DRONE_LOG_INFO("[plan] #" << run << " " << planning::toString(cfg_.planner_type) <<" MONITOR committed "
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
          const double committed_gap = gapToGoal(cached_path_);
          const double cand_gap =
              solved ? planner.lastGoalGap() : std::numeric_limits<double>::infinity();
          const auto remaining = remainingCommittedSuffix(cached_path_, start);
          const double remaining_cost = planner.pathCost(remaining);
          const double threshold = cfg_.replan_improve_ratio * remaining_cost;
          const auto cand = planner.costBreakdown(candidate);
          if (solved && cand_gap < committed_gap - kGoalProgress) {
            adopt(candidate);
            DRONE_LOG_INFO("[plan] #" << run << " " << planning::toString(cfg_.planner_type) <<" IMPROVE best-effort ADVANCE gap "
                           << committed_gap << "m -> " << cand_gap << "m (goal) cost=" << fmtCost(cand) << " clr="
                           << planner.minClearance(candidate) << "m" << goal_note);
          } else if (solved && cand_gap <= committed_gap + kGoalProgress && cand.total <= threshold) {
            adopt(candidate);
            DRONE_LOG_INFO("[plan] #" << run << " " << planning::toString(cfg_.planner_type) <<" IMPROVE remaining cost=" << remaining_cost
                           << " (committed=" << fmtCost(committed) << ") -> ADOPT cost=" << fmtCost(cand) << " clr="
                           << planner.minClearance(candidate) << "m" << goal_note);
          } else {
            DRONE_LOG_INFO("[plan] #" << run << " " << planning::toString(cfg_.planner_type) <<" IMPROVE remaining cost=" << remaining_cost
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
        DRONE_LOG_INFO("[plan] #" << run << " " << planning::toString(cfg_.planner_type) <<" MONITOR committed cost=" << fmtCost(committed)
                       << " clr=" << committed_clr << "m gap=" << gapToGoal(cached_path_) << "m -> OK");
      }

      // Trajectory generation is the next pipeline stage; while plan_trajectory is
      // false the worker is a pure geometric planner and control keeps following
      // the direct setpoint (the tracker is never handed a trajectory). When
      // enabled it re-anchors min-snap to where the vehicle is now, on its cadence.
      if (cfg_.plan_trajectory && !cached_path_.empty() &&
          t - last_trajgen_ >= cfg_.trajgen_period) {
        // Anchor the replan on the state the vehicle will be in when it
        // engages, not on where it is now, and root the path there so the
        // corridor is grown around the point the trajectory actually starts
        // from. Rooting at the measured position instead would leave the splice
        // point outside region 0 whenever there is tracking error, and the QP's
        // start equality would then be infeasible against region 0's faces.
        const double t_gen = now();
        const SpliceAnchor anchor = spliceAnchor(state, t_gen);
        std::vector<std::vector<double>> path = cached_path_;
        path.front() = {anchor.start.pos.x(), anchor.start.pos.y(), anchor.start.pos.z()};
        common::Trajectory traj;
        const planning::MapHandle cons = conservative ? conservative : map;
        // Truncation stops at unobserved space only when the operator asked for
        // it. With treat_unknown_as_hazard false the intended mode is the legacy
        // one throughout: nothing distinguishes unknown from free, in the cost
        // or in truncation.
        //
        // Keyed off the flag and NOT off `conservative` being non-null. The host
        // only builds that view once a frontier cloud has arrived, so keying off
        // it made a late or missing /octomap_frontier quietly drop this guard
        // even with the flag on. Truncation needs no frontier cloud to make this
        // check — the predicate reads the RAW map, never the stamped copy, since
        // stamping writes voxels into that copy and a stamped point that did not
        // already exist would make genuinely unobserved space read as observed.
        const bool ok = runTrajgen(path, anchor.t0, anchor.start, conservativeField(cons), cons,
                                   cfg_.treat_unknown_as_hazard ? makeUnknownFn(map)
                                                                : planning::CorridorUnknownFn{},
                                   traj);

        // Re-measure the lead from what this solve actually cost. Timed around
        // the whole call, failures included: a corridor that fails late has
        // still burned the time, and the next replan has to budget for it.
        const double solve_time = now() - t_gen;
        trajgen_solve_max_ = std::max(solve_time, kLeadMaxDecay * trajgen_solve_max_);
        trajgen_lead_ = std::clamp(kLeadSafetyFactor * trajgen_solve_max_, kLeadMin, kLeadMax);
        if (solve_time > anchor.t0 - t_gen) {
          // The trajectory is due to engage before it was finished, so it will
          // start slightly past its own beginning. Self-correcting (the lead
          // just grew), but worth naming: this is the one case that puts a step
          // in the reference, and a persistent one means the solve is too slow
          // for TRAJGEN_PERIOD, not that the lead is mistuned.
          DRONE_LOG_INFO("[trajgen] solve " << solve_time << " s overran its "
                         << (anchor.t0 - t_gen) << " s lead — engaging late; lead now "
                         << trajgen_lead_ << " s");
        }

        if (ok) {
          stagePending(traj);
          last_trajgen_ = t;
        }
      }
      idle = Idle::kNone;  // planning again; a later stall re-reports immediately
    } else {
      const Idle reason = !has_goal ? Idle::kNoGoal : Idle::kNoMap;
      if (reason != idle || t - last_idle_log >= kIdleLogPeriod) {
        if (reason == Idle::kNoGoal) {
          DRONE_LOG_INFO("[plan] idle: no goal set — nothing to plan toward");
        } else {
          DRONE_LOG_INFO("[plan] idle: goal set but no map yet — waiting for the first "
                         "octomap; the planner cannot run without one");
        }
        idle = reason;
        last_idle_log = t;
      }
    }

    // The loop ticks at the monitor cadence — every iteration is one monitor tick.
    // Clamp to a small floor so a mis-set period cannot turn this into a busy loop.
    std::this_thread::sleep_for(
        std::chrono::duration<double>(std::max(cfg_.rrt_monitor_period, 0.01)));
  }
}

}  // namespace drone_core::autonomy
