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
    core.setVehicleState(airborneAt(Eigen::Vector3d(0.0, 0.0, 1.0)));
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
    core.setVehicleState(airborneAt(Eigen::Vector3d(0.0, 0.0, 1.0)));
    auto cmd = core.stepControl(0.02);
    check(!core.inHoverHold(), "tracking after a fresh trajectory");
    check(core.hasTrajectory(), "trajectory is active");
    check(cmd.thrust >= 0.0 && cmd.thrust <= 1.0, "thrust within bounds while tracking");
    check(!core.sampledPlannedPath().empty(), "planned path is sampleable for viz");

    // Let guidance go stale: time advances, no new trajectory arrives.
    fake_time += 5.0;
    core.setVehicleState(airborneAt(Eigen::Vector3d(0.0, 0.0, 1.0)));
    core.stepControl(0.02);
    check(core.inHoverHold(), "watchdog fell back to hover-hold when guidance went stale");
  }

  // Divergence: the reference stays fresh by the clock, but the vehicle has
  // been knocked far away from it. This must trip hover-hold on its own
  // (independent of stale_timeout, set generously large here so only the
  // distance check can fire) and, critically, must make the NEXT plan root at
  // the vehicle's real position rather than splicing onto the abandoned
  // trajectory's reference.
  {
    autonomy::AutonomyCore::Config cfg;
    cfg.stale_timeout = 5.0;
    cfg.max_tracking_error = 1.0;
    autonomy::AutonomyCore core(cfg);

    double fake_time = 100.0;
    core.setClock([&fake_time]() { return fake_time; });

    auto octree = std::make_shared<octomap::OcTree>(0.1);
    core.setMap(octree);
    core.setVehicleState(airborneAt(Eigen::Vector3d(0.0, 0.0, 1.0)));
    core.reset();

    common::Goal goal;
    goal.pos = Eigen::Vector3d(3.0, 0.0, 1.0);
    core.setGoal(goal);
    check(core.planOnce(), "planOnce produced a trajectory");

    fake_time += 0.1;
    core.setVehicleState(airborneAt(Eigen::Vector3d(0.0, 0.0, 1.0)));
    core.stepControl(0.02);
    check(!core.inHoverHold(), "tracking right after engaging, undiverged");

    // Knock the vehicle 5 m sideways of the reference without advancing time,
    // so stale_timeout cannot be what trips this.
    const Eigen::Vector3d diverged_pos(0.0, 5.0, 1.0);
    core.setVehicleState(airborneAt(diverged_pos));
    core.stepControl(0.02);
    check(core.inHoverHold(), "divergence alone falls back to hover-hold");

    // The next plan must root at the vehicle's ACTUAL position, not wherever
    // the abandoned trajectory's reference had got to (near the original
    // start, (0,0,1)) — that would be exactly the bug this fix closes.
    check(core.planOnce(), "replan after divergence still produces a trajectory");
    const auto path = core.sampledPlannedPath();
    check(!path.empty(), "replanned path is sampleable");
    if (!path.empty()) {
      const Eigen::Vector3d front(path.front()[0], path.front()[1], path.front()[2]);
      check((front - diverged_pos).norm() < 0.2,
            "replan after divergence starts at the real position, not the stale reference");
    }

    // Still holding before the recovery trajectory's t0: the divergence must be
    // acted on once, not on every held tick, or this would discard the recovery
    // trajectory it is waiting for.
    core.stepControl(0.02);
    check(core.inHoverHold(), "still holding before the recovery trajectory engages");
    check(!core.sampledPlannedPath().empty(), "recovery trajectory kept while holding");

    fake_time += 0.1;
    core.stepControl(0.02);
    check(!core.inHoverHold(), "recovery trajectory from the real position is tracked");
  }

  // Bench replan flag. Nothing flies the trajectory, so the drone stays at its
  // start while the clock runs. A normal replan splices onto where the
  // trajectory says the drone is by now (further along); with the flag it must
  // start at the measured position. Both run, so the check proves a difference.
  for (const bool bench : {false, true}) {
    autonomy::AutonomyCore::Config cfg;
    cfg.stale_timeout = 5.0;
    cfg.rrt_solve_time = 0.2;
    cfg.bench_replan_from_state = bench;
    autonomy::AutonomyCore core(cfg);

    double fake_time = 100.0;
    core.setClock([&fake_time]() { return fake_time; });

    auto octree = std::make_shared<octomap::OcTree>(0.1);
    core.setMap(octree);
    const Eigen::Vector3d start(0.0, 0.0, 1.0);
    core.setVehicleState(airborneAt(start));
    core.reset();

    common::Goal goal;
    goal.pos = Eigen::Vector3d(3.0, 0.0, 1.0);
    core.setGoal(goal);
    check(core.planOnce(), "bench-flag case: first plan");

    fake_time += 1.5;  // the unflown trajectory has moved on; the drone has not
    check(core.planOnce(), "bench-flag case: replan");
    const auto path = core.sampledPlannedPath();
    check(!path.empty(), "bench-flag case: replan is sampleable");
    if (!path.empty()) {
      const double off = (Eigen::Vector3d(path.front()[0], path.front()[1], path.front()[2]) -
                          start).norm();
      if (bench) {
        check(off < 0.05, "BENCH_TEST_REPLAN_DISABLER: replan starts at the measured position");
      } else {
        check(off > 0.2, "without the bench flag the replan splices ahead (test contrast)");
      }
    }
  }

  // A start at rest is re-anchored to after the solve. The clock jumps 2 s
  // straight after its first read inside planOnce (the splice anchor), as if the
  // solve took that long: with t0 fixed at the anchor, the trajectory would be
  // overdue by the time it was staged and engage partway along on the very next
  // tick.
  {
    autonomy::AutonomyCore::Config cfg;
    cfg.stale_timeout = 0.5;
    autonomy::AutonomyCore core(cfg);

    double fake_time = 100.0;
    bool slow_clock = false;
    core.setClock([&]() {
      const double t = fake_time;
      if (slow_clock) {
        fake_time += 2.0;
        slow_clock = false;
      }
      return t;
    });

    auto octree = std::make_shared<octomap::OcTree>(0.1);
    core.setMap(octree);
    core.setVehicleState(airborneAt(Eigen::Vector3d(0.0, 0.0, 1.0)));
    core.reset();

    common::Goal goal;
    goal.pos = Eigen::Vector3d(3.0, 0.0, 1.0);
    core.setGoal(goal);

    slow_clock = true;
    const double t_before = fake_time;
    check(core.planOnce(), "slow planOnce produced a trajectory");
    slow_clock = false;
    check(fake_time - t_before >= 1.0, "the slow clock actually advanced during the solve");

    core.stepControl(0.02);
    check(core.inHoverHold(), "rest start does not engage on the tick after a slow solve");

    fake_time += 0.1;
    core.setVehicleState(airborneAt(Eigen::Vector3d(0.0, 0.0, 1.0)));
    core.stepControl(0.02);
    check(!core.inHoverHold(), "rest start engages once its re-anchored t0 has passed");
  }

  // Map/world frames. The core plans in the map frame and hands the tracker a
  // world-frame trajectory, converting with the host's map->world transform. A
  // transform with a real offset, heading change and tilt makes every missed
  // conversion visible as a position error of a metre or more:
  //  - without a transform (and require_map_to_world set) nothing is planned;
  //  - the staged trajectory starts at the vehicle's WORLD position (a missing
  //    conversion on the way in or out would start it somewhere else) and ends at
  //    the MAP-frame goal expressed in the world frame;
  //  - a replan while tracking starts exactly where the outgoing world-frame
  //    trajectory is at the splice instant, which only holds if the splice state
  //    is sampled in world and converted into map before solving.
  {
    autonomy::AutonomyCore::Config cfg;
    cfg.stale_timeout = 2.0;
    cfg.rrt_solve_time = 0.2;
    cfg.require_map_to_world = true;
    autonomy::AutonomyCore core(cfg);

    double fake_time = 100.0;
    core.setClock([&fake_time]() { return fake_time; });

    auto octree = std::make_shared<octomap::OcTree>(0.1);
    core.setMap(octree);
    const Eigen::Vector3d p_world(0.2, -0.1, 1.0);
    core.setVehicleState(airborneAt(p_world));
    core.reset();

    common::Goal goal;
    goal.pos = Eigen::Vector3d(2.0, 0.5, 1.0);  // map frame
    core.setGoal(goal);

    check(!core.planOnce(), "frames: no plan before a map->world transform arrives");

    Eigen::Isometry3d world_from_map = Eigen::Isometry3d::Identity();
    world_from_map.rotate(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()));
    world_from_map.rotate(Eigen::AngleAxisd(0.03, Eigen::Vector3d::UnitY()));
    world_from_map.pretranslate(Eigen::Vector3d(1.0, -0.5, 0.2));
    core.setMapToWorld(world_from_map);

    check(core.planOnce(), "frames: plans once the transform is set");
    const auto first = core.sampledPlannedPath(0.01);
    check(first.size() > 60, "frames: trajectory long enough to splice into");
    if (first.size() > 60) {
      const Eigen::Vector3d start(first.front()[0], first.front()[1], first.front()[2]);
      const Eigen::Vector3d end(first.back()[0], first.back()[1], first.back()[2]);
      check((start - p_world).norm() < 1e-4,
            "frames: trajectory starts at the vehicle's world position");
      check((end - world_from_map * goal.pos).norm() < 0.35,
            "frames: trajectory ends at the map-frame goal expressed in world");
      check((end - goal.pos).norm() > 0.8,
            "frames: trajectory end is NOT the raw map-frame goal (conversion applied)");
    }

    // Engage it (rest start: t0 = 100.04), then replan half a second in while
    // it is being tracked. The new t0 is 100.54, i.e. 0.50 s into the old one.
    core.stepControl(0.02);
    fake_time += 0.1;
    core.setVehicleState(airborneAt(p_world));
    core.stepControl(0.02);
    check(!core.inHoverHold(), "frames: tracking the first trajectory");
    fake_time += 0.4;
    core.setVehicleState(airborneAt(p_world));

    check(core.planOnce(), "frames: replan while tracking");
    const auto second = core.sampledPlannedPath(0.01);
    if (first.size() > 60 && !second.empty()) {
      const Eigen::Vector3d old_at_splice(first[50][0], first[50][1], first[50][2]);
      const Eigen::Vector3d new_start(second.front()[0], second.front()[1], second.front()[2]);
      check((new_start - old_at_splice).norm() < 1e-4,
            "frames: replan starts where the outgoing world trajectory is at the splice");
    }
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
    core.setVehicleState(airborneAt(Eigen::Vector3d(0.0, 0.0, 1.0)));
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
    core.setVehicleState(airborneAt(Eigen::Vector3d(0.0, 0.0, 1.0)));
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
      core.setVehicleState(airborneAt(Eigen::Vector3d(0.0, 0.0, 1.0)));
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
