#include "drone_core/autonomy/autonomy_core.hpp"

#include <chrono>

#include "drone_core/control/flatness_mapper.hpp"

namespace drone_core::autonomy {

namespace {

double steadyNowSeconds() {
  const auto t = std::chrono::steady_clock::now().time_since_epoch();
  return std::chrono::duration<double>(t).count();
}

}  // namespace

AutonomyCore::AutonomyCore(const Config& config)
    : cfg_(config), clock_(steadyNowSeconds) {
  tracker_.setPositionGains(cfg_.pos_p);
  tracker_.setVelocityGains(cfg_.vel_p, cfg_.vel_i, cfg_.vel_d);
  tracker_.setHoverThrust(cfg_.hover_thrust);
  tracker_.enableFeedforward(cfg_.enable_feedforward);
  tracker_.setStaleTimeout(cfg_.stale_timeout);
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
  // Force a fresh geometric plan toward the new goal on the next worker cycle.
  cached_path_.clear();
  last_rrt_ = -1.0e9;
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

bool AutonomyCore::runGlobalPlan(const common::State& state, const common::Goal& goal,
                                 const planning::MapHandle& map,
                                 std::vector<std::vector<double>>& path) {
  planning::RrtStarPlanner planner(map);
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

void AutonomyCore::plannerLoop() {
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
      // Geometric replan on the slow cadence (or whenever the path was invalidated).
      if (cached_path_.empty() || t - last_rrt_ >= cfg_.rrt_period) {
        std::vector<std::vector<double>> path;
        if (runGlobalPlan(state, goal, map, path)) {
          cached_path_ = path;
          last_rrt_ = t;
        }
      }

      // Local trajectory replan on the mid cadence, re-anchored to where the
      // vehicle actually is now.
      if (!cached_path_.empty() && t - last_trajgen_ >= cfg_.trajgen_period) {
        std::vector<std::vector<double>> path = cached_path_;
        path.front() = {state.pos.x(), state.pos.y(), state.pos.z()};
        common::Trajectory traj;
        if (runTrajgen(path, now(), traj)) {
          stagePending(traj);
          last_trajgen_ = t;
        }
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

}  // namespace drone_core::autonomy
