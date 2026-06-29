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
#include "drone_core/control/flatness_mapper.hpp"

namespace drone_core::autonomy {

namespace {

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

void AutonomyCore::setMap(const planning::MapHandle& map) {
  std::lock_guard<std::mutex> lock(io_mutex_);
  map_ = map;
}

void AutonomyCore::setGoal(const common::Goal& goal) {
  std::lock_guard<std::mutex> lock(io_mutex_);
  goal_ = goal;
  has_goal_ = true;
  // Force a fresh geometric plan toward the new goal on the next worker cycle:
  // an empty committed path reads as invalid, so the loop replans immediately.
  cached_path_.clear();
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
  common::Goal goal;
  bool has_goal = false;
  {
    std::lock_guard<std::mutex> lock(io_mutex_);
    state = state_;
    map = map_;
    goal = goal_;
    has_goal = has_goal_;
  }

  if (!has_goal || !map) return false;

  std::vector<std::vector<double>> path;
  if (!runGlobalPlan(state, goal, map, path)) return false;

  common::Trajectory traj;
  if (!runTrajgen(path, now(), traj)) return false;

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

std::vector<std::array<double, 4>> AutonomyCore::sampleClearanceField() const {
  std::vector<std::array<double, 4>> out;
  if (!edt_ || !edt_source_map_) return out;

  double xmin, ymin, zmin, xmax, ymax, zmax;
  edt_source_map_->getMetricMin(xmin, ymin, zmin);
  edt_source_map_->getMetricMax(xmax, ymax, zmax);

  constexpr double step = 0.15;  // grid spacing [m] — coarse, debug-only
  const double maxd = cfg_.clearance_threshold;
  for (double x = xmin; x <= xmax; x += step) {
    for (double y = ymin; y <= ymax; y += step) {
      for (double z = zmin; z <= zmax; z += step) {
        double d = edt_->getDistance(octomap::point3d(
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
  const std::vector<double> start = {state.pos.x(), state.pos.y(), state.pos.z()};
  const std::vector<double> goal_vec = {goal.pos.x(), goal.pos.y(), goal.pos.z()};
  return planner.planPath(start, goal_vec, path);
}

bool AutonomyCore::runTrajgen(const std::vector<std::vector<double>>& path, double t0,
                              common::Trajectory& traj) {
  if (path.size() < 2) return false;
  planning::MinSnapTimeOptimizer optimizer;
  if (!optimizer.optimizeTrajectory(path, traj)) return false;
  traj.t0 = t0;
  return true;
}

void AutonomyCore::stagePending(const common::Trajectory& traj) {
  std::lock_guard<std::mutex> lock(traj_mutex_);
  pending_ = traj;
  last_planned_ = traj;
  has_pending_ = true;
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

bool AutonomyCore::applyClearanceObjective(planning::GeometricPlanner& planner,
                                           const planning::MapHandle& map) {
  auto edt = clearanceField(map);
  if (!edt) return false;
  // One field, two consumers: the planner's collision check (clearance > margin)
  // and its cost objective (proximity penalty up to clearance_threshold).
  planner.setClearance(
      [edt, maxd = cfg_.clearance_threshold](double x, double y, double z) {
        const double d = edt->getDistance(octomap::point3d(
            static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)));
        return d < 0.0 ? maxd : d;  // outside the EDT bbx => treat as free
      },
      cfg_.clearance_weight, cfg_.clearance_threshold);
  return true;
}

void AutonomyCore::plannerLoop() {
  std::uint64_t run = 0;  // planner ticks taken (advanced only while a goal and map exist)

  while (running_.load()) {
    const double t = now();

    common::State state;
    planning::MapHandle map;
    common::Goal goal;
    bool has_goal = false;
    {
      std::lock_guard<std::mutex> lock(io_mutex_);
      state = state_;
      map = map_;
      goal = goal_;
      has_goal = has_goal_;
    }

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

      // One planner per tick. Set the clearance field FIRST: it is now the single
      // obstacle model for both the collision check (clearance > margin) and the
      // cost, so the committed-path re-check below sees the same obstacles the
      // search would. The field is cached (rebuilt only on a map change), so this
      // is cheap on the common still-valid tick.
      planning::GeometricPlanner planner(map, cfg_.rrt_solve_time);
      planner.setPlannerType(cfg_.planner_type);
      planner.setRecordTree(cfg_.debug_planner_viz);
      applyClearanceObjective(planner, map);

      // Debug-only: re-sample the clearance field when the map changes (the EDT
      // is now current for this tick). Gated so a regular flight never walks the
      // grid. Sampling only on map change keeps even a debug run cheap on a
      // static scene.
      if (cfg_.debug_planner_viz && map != viz_sampled_map_) {
        auto samples = sampleClearanceField();
        {
          std::lock_guard<std::mutex> lock(traj_mutex_);
          last_clearance_samples_ = std::move(samples);
        }
        viz_sampled_map_ = map;
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
      const double committed_cost = committed.total;
      const double committed_clr = planner.minClearance(cached_path_);

      // Stream a cost as "total (len=L + clr_cost=C)" so each replan line shows
      // whether the score is driven by path length or by obstacle proximity.
      auto fmtCost = [](const planning::GeometricPlanner::CostBreakdown& cb) {
        std::ostringstream os;
        os << cb.total << " (len=" << cb.length << " + clr_cost=" << cb.clearance << ")";
        return os.str();
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

        if (path_invalid) {
          // Forced replan — adopt unconditionally (safety outranks optimality).
          if (solved) {
            const auto cand = planner.costBreakdown(candidate);
            adopt(candidate);
            if (had_path)
              DRONE_LOG_INFO("[plan] #" << run << " " << planning::toString(cfg_.planner_type) <<" MONITOR committed BLOCKED (clr="
                             << committed_clr << "m < " << planner.collisionMargin()
                             << "m) -> REPLAN cost=" << fmtCost(cand) << " clr="
                             << planner.minClearance(candidate) << "m");
            else
              DRONE_LOG_INFO("[plan] #" << run << " " << planning::toString(cfg_.planner_type) <<" MONITOR no committed path -> PLAN cost="
                             << fmtCost(cand) << " clr="
                             << planner.minClearance(candidate) << "m");
          } else {
            DRONE_LOG_INFO("[plan] #" << run << " " << planning::toString(cfg_.planner_type) <<" MONITOR committed "
                           << (had_path ? "BLOCKED" : "absent")
                           << " -> REPLAN FAILED (no path to goal), holding");
          }
        } else {
          // Improve — switch only past the hysteresis margin. RRT* is randomised,
          // so without the margin a near-identical re-solve would chatter.
          const double threshold = cfg_.replan_improve_ratio * committed_cost;
          const auto cand = planner.costBreakdown(candidate);
          if (solved && cand.total <= threshold) {
            adopt(candidate);
            DRONE_LOG_INFO("[plan] #" << run << " " << planning::toString(cfg_.planner_type) <<" IMPROVE committed cost=" << fmtCost(committed)
                           << " -> ADOPT cost=" << fmtCost(cand) << " clr="
                           << planner.minClearance(candidate) << "m");
          } else {
            DRONE_LOG_INFO("[plan] #" << run << " " << planning::toString(cfg_.planner_type) <<" IMPROVE committed cost=" << fmtCost(committed)
                           << " clr=" << committed_clr << "m candidate cost="
                           << (solved ? fmtCost(cand) : std::string("inf"))
                           << " -> keep (need <=" << threshold << ")");
          }
        }
      } else {
        // Monitor only: committed path still clear of obstacles.
        DRONE_LOG_INFO("[plan] #" << run << " " << planning::toString(cfg_.planner_type) <<" MONITOR committed cost=" << fmtCost(committed)
                       << " clr=" << committed_clr << "m -> OK");
      }

      // Trajectory generation is the next pipeline stage; while plan_trajectory is
      // false the worker is a pure geometric planner and control keeps following
      // the direct setpoint (the tracker is never handed a trajectory). When
      // enabled it re-anchors min-snap to where the vehicle is now, on its cadence.
      if (cfg_.plan_trajectory && !cached_path_.empty() &&
          t - last_trajgen_ >= cfg_.trajgen_period) {
        std::vector<std::vector<double>> path = cached_path_;
        path.front() = {state.pos.x(), state.pos.y(), state.pos.z()};
        common::Trajectory traj;
        if (runTrajgen(path, now(), traj)) {
          stagePending(traj);
          last_trajgen_ = t;
        }
      }
    }

    // The loop ticks at the monitor cadence — every iteration is one monitor tick.
    // Clamp to a small floor so a mis-set period cannot turn this into a busy loop.
    std::this_thread::sleep_for(
        std::chrono::duration<double>(std::max(cfg_.rrt_monitor_period, 0.01)));
  }
}

}  // namespace drone_core::autonomy
