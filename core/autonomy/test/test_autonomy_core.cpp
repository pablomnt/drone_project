// Integration check for the orchestrator: plan against a synthetic map, track
// the trajectory, fall back to hover-hold when guidance goes stale, and run the
// background planner thread without deadlocking.

#include "drone_core/autonomy/autonomy_core.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

namespace {

int g_failures = 0;

void check(bool ok, const char* what) {
  if (!ok) {
    std::cerr << "FAIL: " << what << "\n";
    ++g_failures;
  }
}

drone_core::common::State airborneAt(const Eigen::Vector3d& p) {
  drone_core::common::State s;
  s.pos = p;
  return s;
}

}  // namespace

int main() {
  using namespace drone_core;

  // Synchronous path: drive the core with a controllable clock.
  {
    autonomy::AutonomyCore::Config cfg;
    cfg.stale_timeout = 0.5;
    autonomy::AutonomyCore core(cfg);

    double fake_time = 100.0;
    core.setClock([&fake_time]() { return fake_time; });

    auto octree = std::make_shared<octomap::OcTree>(0.1);
    core.setMap(octree);
    core.setState(airborneAt(Eigen::Vector3d(0.0, 0.0, 1.0)));
    core.reset();

    common::Goal goal;
    goal.pos = Eigen::Vector3d(3.0, 0.0, 1.0);
    core.setGoal(goal);

    check(core.planOnce(), "planOnce produced a trajectory");

    // A freshly planned trajectory is anchored a lead time ahead of the solve,
    // so it does not engage on the tick that produced it — the tracker holds it
    // until its t0. Stepping control at the planning instant must therefore
    // still show no tracking; this is the property that makes a generous lead
    // free rather than a source of reference jumps.
    core.stepControl(0.02);
    check(core.inHoverHold(), "staged trajectory does not engage before its t0");

    // Past the lead it takes over. planOnce never updates the measured lead, so
    // this is still the kLeadMin floor (0.04 s); 0.1 s clears it without eating
    // into the 0.5 s stale timeout this core is configured with.
    fake_time += 0.1;
    core.setState(airborneAt(Eigen::Vector3d(0.0, 0.0, 1.0)));
    auto cmd = core.stepControl(0.02);
    check(!core.inHoverHold(), "tracking after a fresh trajectory");
    check(core.hasTrajectory(), "trajectory is active");
    check(cmd.thrust >= 0.0 && cmd.thrust <= 1.0, "thrust within bounds while tracking");
    check(!core.sampledPlannedPath().empty(), "planned path is sampleable for viz");

    // Let guidance go stale: time advances, no new trajectory arrives.
    fake_time += 5.0;
    core.setState(airborneAt(Eigen::Vector3d(0.0, 0.0, 1.0)));
    core.stepControl(0.02);
    check(core.inHoverHold(), "watchdog fell back to hover-hold when guidance went stale");
  }

  // Corridor-QP mode: planOnce must route trajgen through the corridor
  // pipeline (truncation + box corridor + QP against the conservative EDT) and
  // still stage a trajectory. A mapped floor gives the distance field real
  // obstacles; the corridor's collision margin must then keep the trajectory
  // well off that floor, which plain min-snap would not guarantee.
  {
    autonomy::AutonomyCore::Config cfg;
    cfg.use_corridor_qp = true;
    cfg.rrt_solve_time = 0.5;
    autonomy::AutonomyCore core(cfg);

    double fake_time = 100.0;
    core.setClock([&fake_time]() { return fake_time; });

    // Solid floor at z = 0, with the flight volume above it marked FREE. The
    // free cells are load-bearing: treat_unknown_as_hazard defaults true, so
    // truncation stops at the first cell the map has no node for, and a map that
    // is only an obstacle list reads as "nothing here was ever observed" and
    // commits nothing. Note this holds even though no conservative view is
    // passed — the guard keys off the flag, not off that view, precisely so a
    // missing frontier cloud cannot silently disable it. A real RTAB-Map octomap
    // carries ray-traced free space for the same reason, so writing it here is
    // what makes this a model of the live pipeline rather than an obstacle list.
    // Stepped over integer indices, not by accumulating += 0.1 into a double:
    // the accumulated error drifts across voxel boundaries and leaves unmapped
    // gaps in what is supposed to be a solid block, which now shows up as
    // truncation refusing to commit.
    auto octree = std::make_shared<octomap::OcTree>(0.1);
    for (int ix = -10; ix <= 40; ++ix) {
      for (int iy = -10; iy <= 10; ++iy) {
        const double x = ix * 0.1, y = iy * 0.1;
        octree->updateNode(octomap::point3d(x, y, 0.0), true);
        for (int iz = 1; iz <= 20; ++iz) {
          octree->updateNode(octomap::point3d(x, y, iz * 0.1), false);
        }
      }
    }
    core.setMap(octree);  // no conservative view: the raw map serves both roles
    core.setState(airborneAt(Eigen::Vector3d(0.0, 0.0, 1.0)));
    core.reset();

    common::Goal goal;
    goal.pos = Eigen::Vector3d(3.0, 0.0, 1.0);
    core.setGoal(goal);

    check(core.planOnce(), "corridor-QP planOnce produced a trajectory");
    core.stepControl(0.02);
    check(core.hasTrajectory(), "corridor trajectory is active");

    const auto sampled = core.sampledPlannedPath();
    check(!sampled.empty(), "corridor trajectory is sampleable");
    bool above_floor = !sampled.empty();
    for (const auto& p : sampled) {
      if (p[2] < 0.45) above_floor = false;  // corridor margin (0.5) minus tolerance
    }
    check(above_floor, "corridor trajectory keeps the collision margin off the floor");
  }

  // Background planner thread: should plan and stage without help.
  {
    autonomy::AutonomyCore::Config cfg;
    autonomy::AutonomyCore core(cfg);

    auto octree = std::make_shared<octomap::OcTree>(0.1);
    core.setMap(octree);
    core.setState(airborneAt(Eigen::Vector3d(0.0, 0.0, 1.0)));
    core.reset();

    common::Goal goal;
    goal.pos = Eigen::Vector3d(2.0, 0.0, 1.0);
    core.setGoal(goal);

    core.startPlanner();

    // RRT* spends its full anytime budget per solve, so poll the control step
    // until the worker stages a trajectory (with a generous ceiling).
    drone_core::common::Command cmd;
    bool got_trajectory = false;
    for (int i = 0; i < 60 && !got_trajectory; ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      core.setState(airborneAt(Eigen::Vector3d(0.0, 0.0, 1.0)));
      cmd = core.stepControl(0.02);
      got_trajectory = core.hasTrajectory();
    }
    core.stopPlanner();

    check(got_trajectory, "background planner produced a trajectory");
    check(cmd.thrust >= 0.0 && cmd.thrust <= 1.0, "thrust within bounds (threaded)");
  }

  if (g_failures == 0) {
    std::cout << "autonomy_core: all checks passed\n";
    return 0;
  }
  std::cerr << "autonomy_core: " << g_failures << " failure(s)\n";
  return 1;
}
