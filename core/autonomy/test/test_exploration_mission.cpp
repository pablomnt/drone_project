// Goal-directed exploration (AutonomyCore::explorationTick) end to end, on the
// threaded search with short budgets: ADVANCE to the edge of explored space,
// UNCOVER (exit point + a viewpoint facing it) once there, ADVANCE again when
// the map reveals more, DONE at the goal.

#include "drone_core/autonomy/autonomy_core.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

namespace {

int g_failures = 0;
void check(bool ok, const std::string& what) {
  if (!ok) {
    std::cerr << "FAIL: " << what << "\n";
    ++g_failures;
  }
}

using namespace drone_core;
using Mode = autonomy::AutonomyCore::MissionMode;

common::State at(const Eigen::Vector3d& p) {
  common::State s;
  s.pos = p;
  return s;
}

// A free box [x0, x1] x [-1.5, 1.5] x (0, 2.5) with a floor at z = 0; unknown elsewhere.
std::shared_ptr<octomap::OcTree> room(double x0, double x1) {
  auto tree = std::make_shared<octomap::OcTree>(0.1);
  for (double x = x0 + 0.05; x < x1; x += 0.1) {
    for (double y = -1.45; y < 1.5; y += 0.1) {
      tree->updateNode(octomap::point3d(x, y, 0.05), true);
      for (double z = 0.15; z < 2.5; z += 0.1) tree->updateNode(octomap::point3d(x, y, z), false);
    }
  }
  return tree;
}

void setRoom(autonomy::AutonomyCore& core, double x0, double x1, const Eigen::Vector3d& drone) {
  auto tree = room(x0, x1);
  Eigen::Vector3d lo, hi;
  core.mapCrop(lo, hi);
  auto grid = std::make_shared<const planning::ConservativeGrid>(
      *tree, octomap::point3d(drone.x(), drone.y(), drone.z()), 0.3, lo, hi, /*shell=*/true, 1);
  core.setMap(tree, grid);
}

bool waitFor(const std::function<bool()>& cond, double seconds) {
  for (int i = 0; i < static_cast<int>(seconds / 0.05); ++i) {
    if (cond()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return cond();
}

}  // namespace

int main() {
  autonomy::AutonomyCore::Config cfg;
  cfg.require_map_to_world = false;
  cfg.plan_trajectory = false;  // the search's decisions are what is checked
  cfg.use_corridor_qp = true;
  cfg.treat_unknown_as_hazard = true;
  cfg.use_exploration = true;
  cfg.planner_type = planning::PlannerType::EITstar;
  cfg.rrt_solve_time = 0.4;
  cfg.rrt_replan_solve_time = 0.2;
  cfg.rrt_monitor_period = 0.05;
  cfg.retarget_period = 0.5;
  cfg.view_dwell = 0.2;
  autonomy::AutonomyCore core(cfg);

  const Eigen::Vector3d start(0.05, 0.05, 1.25);
  const Eigen::Vector3d goal(8.0, 0.0, 1.25);
  setRoom(core, -1.0, 3.0, start);
  core.setVehicleState(at(start));
  common::Goal g;
  g.pos = goal;
  core.setGoal(g);
  core.startPlanner();

  // 1. ADVANCE toward the east edge of the explored box.
  check(waitFor([&] {
          const auto v = core.missionView();
          return v.mode == Mode::kAdvance && v.has_target;
        }, 3.0),
        "no ADVANCE target");
  auto v = core.missionView();
  check(v.target.x() > 1.0 && v.target.x() < 2.75,
        "ADVANCE target not toward the east edge inside explored space (x " +
            std::to_string(v.target.x()) + ")");
  check(std::isnan(v.target_yaw), "an ADVANCE target carries an end heading");

  // 2. Arrive there: nothing closer is known, so UNCOVER with a viewpoint
  //    facing the exit point at the box's east face.
  core.setVehicleState(at(v.target));
  check(waitFor([&] {
          const auto w = core.missionView();
          return w.mode == Mode::kUncover && w.has_target && w.has_exit;
        }, 5.0),
        "no UNCOVER viewpoint after arriving");
  v = core.missionView();
  check(v.exit.x() > 2.7 && v.exit.x() < 3.3, "exit point not at the east face (x " +
                                                  std::to_string(v.exit.x()) + ")");
  check(std::isfinite(v.target_yaw), "the viewpoint has no end heading");
  if (std::isfinite(v.target_yaw)) {
    const double facing = std::atan2(v.exit.y() - v.target.y(), v.exit.x() - v.target.x());
    check(std::abs(std::remainder(v.target_yaw - facing, 2.0 * M_PI)) < 1e-6,
          "the viewpoint does not face the exit point");
  }

  // 3. At the viewpoint the map reveals the rest of the way: ADVANCE again,
  //    now to the goal itself.
  core.setVehicleState(at(v.target));
  setRoom(core, -1.0, 10.0, v.target);
  check(waitFor([&] {
          const auto w = core.missionView();
          return w.mode == Mode::kAdvance && w.has_target && (w.target - goal).norm() < 0.5;
        }, 5.0),
        "no ADVANCE to the goal once the way was revealed");

  // 4. At the goal: DONE.
  core.setVehicleState(at(goal + Eigen::Vector3d(-0.5, 0.0, 0.0)));
  check(waitFor([&] { return core.missionView().mode == Mode::kDone; }, 3.0), "not DONE at the goal");

  core.stopPlanner();
  if (g_failures == 0) {
    std::cout << "exploration_mission: all checks passed\n";
    return 0;
  }
  std::cerr << "exploration_mission: " << g_failures << " failure(s)\n";
  return 1;
}
