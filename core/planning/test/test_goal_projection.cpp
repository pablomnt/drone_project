// Goal projection: what happens when the requested goal is not a state the
// collision check accepts. OMPL discards invalid goal states and the planners
// then abort with "no goal states are available", so a goal parked near a wall
// used to yield no path at all. planPath must instead plan to the nearest valid
// point and report how far it moved.
//
// Analytic clearance fields only (no octree, no EDT) and short solve budgets, so
// this stays in the seconds and can run on every planning edit — unlike the
// full `planner` test.

#include "drone_core/planning/geometric_planner.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>

namespace {

int failures = 0;

void expect(bool ok, const std::string& what) {
  if (!ok) {
    std::cerr << "FAIL: " << what << "\n";
    ++failures;
  }
}

// Clearance field of a solid axis-aligned box: Euclidean distance from the point
// to the box, 0 on or inside it. Same signature the live planner gets from its
// EDT, so this exercises the O(1) validity path rather than the octree fallback.
auto boxClearance(double x0, double y0, double z0, double x1, double y1, double z1) {
  return [=](double x, double y, double z) {
    const double dx = std::max({x0 - x, x - x1, 0.0});
    const double dy = std::max({y0 - y, y - y1, 0.0});
    const double dz = std::max({z0 - z, z - z1, 0.0});
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  };
}

}  // namespace

int main() {
  using namespace drone_core::planning;

  auto empty = std::make_shared<octomap::OcTree>(0.1);
  const std::vector<double> start = {0.0, 0.0, 1.0};

  // A wall filling x >= 2.0. The margin is 0.5 m, so everything with x > 1.5 is
  // invalid — the case from the bench: a goal a few centimetres off a surface.
  const auto wall = boxClearance(2.0, -10.0, -10.0, 10.0, 10.0, 10.0);
  const double margin = GeometricPlanner(empty, 0.1).collisionMargin();

  // 1. A goal inside the margin must still produce a path. Before projection the
  //    solve failed outright ("no goal states available"), which is the bug.
  {
    GeometricPlanner planner(empty, /*planning_time=*/0.5);
    planner.setClearance(wall, /*weight=*/1.0, /*threshold=*/1.0);
    planner.setBestEffort(true);
    const std::vector<double> goal = {1.9, 0.0, 1.0};  // 0.1 m off the wall
    std::vector<std::vector<double>> path;
    expect(planner.planPath(start, goal, path) && !path.empty(),
           "no path to a goal 0.1 m off a wall");
    expect(planner.lastGoalProjection() > 0.0, "goal inside the margin was not projected");

    // 2. The projected goal must clear the margin, and the path must end there —
    //    i.e. the search respects kCollisionMargin and stops short rather than
    //    flying to the commanded point.
    const auto& pg = planner.lastPlanningGoal();
    expect(wall(pg[0], pg[1], pg[2]) > margin,
           "projected goal does not clear the collision margin");
    if (!path.empty()) {
      const auto& end = path.back();
      expect(wall(end[0], end[1], end[2]) > margin,
             "path endpoint does not clear the collision margin");
    }

    // 3. It must not move further than it has to: the goal is 0.1 m from the wall
    //    and needs 0.5 m, so ~0.4 m of displacement, plus at most one search step.
    expect(planner.lastGoalProjection() < 0.5,
           "goal projected further than necessary");

    // 4. The gap is reported against the goal the caller asked for, not the one
    //    OMPL solved to, so the host's best-effort progress logic stays honest.
    expect(planner.lastGoalGap() >= planner.lastGoalProjection() - 1e-9,
           "goal gap measured against the projected goal instead of the requested one");
  }

  // 5. A goal already in free space is untouched — projection is inert on the
  //    common path.
  {
    GeometricPlanner planner(empty, /*planning_time=*/0.5);
    planner.setClearance(wall, /*weight=*/1.0, /*threshold=*/1.0);
    const std::vector<double> goal = {1.0, 0.0, 1.0};
    std::vector<std::vector<double>> path;
    expect(planner.planPath(start, goal, path) && !path.empty(),
           "no path to a goal in open space");
    expect(planner.lastGoalProjection() == 0.0, "valid goal was projected anyway");
    expect(planner.lastGoalGap() <= GeometricPlanner::goalFlexibility(),
           "path to a reachable open-space goal reported a gap");
  }

  // 6. Strict mode judges the shortfall against the requested goal: a projection
  //    inside kGoalFlexibility is arrival, a larger one is not.
  {
    GeometricPlanner near_miss(empty, /*planning_time=*/0.5);
    near_miss.setClearance(wall, /*weight=*/1.0, /*threshold=*/1.0);
    std::vector<std::vector<double>> path;
    // 0.3 m off the wall: needs ~0.2 m of projection, inside the 0.3 m tolerance.
    expect(near_miss.planPath(start, {1.7, 0.0, 1.0}, path),
           "strict mode rejected a goal it could reach within kGoalFlexibility");

    GeometricPlanner far_miss(empty, /*planning_time=*/0.5);
    far_miss.setClearance(wall, /*weight=*/1.0, /*threshold=*/1.0);
    path.clear();
    // 0.5 m *inside* the wall: needs ~1.0 m of projection, well outside it.
    expect(!far_miss.planPath(start, {2.5, 0.0, 1.0}, path),
           "strict mode accepted a path ending a metre from the requested goal");
  }

  // 7. A goal buried deeper than kGoalProjectRadius has no nearby free point that
  //    could be called the same goal: refuse outright, in both modes, rather than
  //    relocate it somewhere arbitrary. lastGoalProjection() reports infinity so
  //    the host can say why.
  {
    GeometricPlanner planner(empty, /*planning_time=*/0.5);
    planner.setClearance(wall, /*weight=*/1.0, /*threshold=*/1.0);
    planner.setBestEffort(true);
    std::vector<std::vector<double>> path;
    expect(!planner.planPath(start, {5.0, 0.0, 1.0}, path),
           "planned to a goal buried deep inside an obstacle");
    expect(std::isinf(planner.lastGoalProjection()),
           "deeply buried goal did not report an infinite projection");
  }

  // 8. Unknown space still reads as free (the clearance field saturates away from
  //    mapped obstacles), so a goal beyond the frontier is valid and untouched —
  //    projection must not interfere with best-effort chasing of a far goal.
  {
    GeometricPlanner planner(empty, /*planning_time=*/0.5);
    planner.setClearance([](double, double, double) { return 1.0; },
                         /*weight=*/1.0, /*threshold=*/1.0);
    planner.setBestEffort(true);
    std::vector<std::vector<double>> path;
    expect(planner.planPath(start, {8.0, 4.0, 1.0}, path) && !path.empty(),
           "no path to a goal in unmapped space");
    expect(planner.lastGoalProjection() == 0.0, "goal in unmapped space was projected");
  }

  // 9. The walled-off goal from the `planner` test, which projection changes the
  //    route to but not the outcome: a goal at the centre of a 1 m block. Strict
  //    mode still refuses it, best-effort still returns a path that stops short by
  //    more than kGoalFlexibility. Duplicated here because the `planner` test is
  //    too slow to run on a routine edit, and this is the assertion most likely to
  //    be disturbed by a change to the projection bounds.
  {
    const auto block = boxClearance(1.5, -0.5, 0.5, 2.5, 0.5, 1.5);
    const std::vector<double> buried = {2.0, 0.0, 1.0};  // block centre

    GeometricPlanner strict(empty, /*planning_time=*/0.5);
    strict.setClearance(block, /*weight=*/4.0, /*threshold=*/1.0);
    std::vector<std::vector<double>> path;
    expect(!strict.planPath(start, buried, path),
           "strict planPath accepted a path to a walled-off goal");

    GeometricPlanner effort(empty, /*planning_time=*/0.5);
    effort.setClearance(block, /*weight=*/4.0, /*threshold=*/1.0);
    effort.setBestEffort(true);
    path.clear();
    expect(effort.planPath(start, buried, path) && !path.empty(),
           "best-effort planPath returned no path toward a walled-off goal");
    expect(effort.lastGoalGap() > GeometricPlanner::goalFlexibility(),
           "best-effort path unexpectedly reached the buried goal");
    if (!path.empty()) {
      const auto& end = path.back();
      expect(block(end[0], end[1], end[2]) > margin,
             "best-effort path to a buried goal entered the collision margin");
    }
  }

  if (failures == 0) {
    std::cout << "goal_projection: all checks passed\n";
    return 0;
  }
  return 1;
}
