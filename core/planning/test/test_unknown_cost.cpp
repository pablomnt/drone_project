// The unknown-space surcharge: a flat extra cost per metre of path routed
// through space that has never been observed, plus the matching hard stop in
// truncatePath.
//
// The bug this addresses: the clearance field saturates at its threshold, so a
// point further than the threshold from a mapped obstacle scores a proximity
// penalty of exactly zero. That covers the interior of unmapped space and
// everything outside the field's bounding box, so unknown space was the cheapest
// airspace in the problem. With the frontier stamped as an obstacle there is a
// gradient near the shell, but the shell has gaps (the sensor's field of view,
// and the deliberately unstamped ball around the vehicle), and past it the
// penalty returns to zero. Raising CLEARANCE_WEIGHT cannot fix that: it
// multiplies a zero.
//
// Analytic fields and predicates, short solve budgets — fast enough to run on
// every planning edit.

#include "drone_core/planning/corridor.hpp"
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

void expectNear(double got, double want, double tol, const std::string& what) {
  if (!(std::abs(got - want) <= tol)) {
    std::cerr << "FAIL: " << what << " (got " << got << ", want " << want << " +/- " << tol
              << ")\n";
    ++failures;
  }
}

// The scene: everything with x > 2 has never been observed. Mapped space is
// otherwise wide open, so the clearance field is saturated everywhere and
// contributes no penalty at all — which is the point. Only the unknown
// surcharge can distinguish the two halves.
constexpr double kFrontierX = 2.0;
bool unknownBeyondFrontier(double x, double, double) { return x > kFrontierX; }

// Tolerance for a charged-cost assertion. OMPL integrates the objective
// trapezoidally over states interpolated at the state-validity resolution, which
// for this state space is a step of roughly half a metre. The surcharge is a
// step function, so the segment straddling the frontier is averaged across and
// the total lands within about (step * weight) of the exact value. Tests
// therefore assert the charge to a few percent over a long path rather than to
// the metre, and lean on the qualitative checks for the rest.
constexpr double kIntegrationSlack = 3.0;

}  // namespace

int main() {
  using namespace drone_core::planning;

  auto empty = std::make_shared<octomap::OcTree>(0.1);
  const auto wideOpen = [](double, double, double) { return 1.0; };

  // 1. The core claim: with a saturated clearance field, cost is pure length —
  //    the two halves of the world are indistinguishable — until the surcharge
  //    is applied. A 14 m path with 12 m of it beyond the frontier must then
  //    cost 14 (length) + 12 * weight.
  {
    const std::vector<std::vector<double>> path = {{0, 0, 1}, {14, 0, 1}};

    GeometricPlanner plain(empty, /*planning_time=*/0.1);
    plain.setClearance(wideOpen, /*weight=*/100.0, /*threshold=*/1.0);
    expectNear(plain.pathCost(path), 14.0, 1e-6,
               "a saturated clearance field should charge nothing, even at weight 100");

    GeometricPlanner charged(empty, /*planning_time=*/0.1);
    charged.setClearance(wideOpen, /*weight=*/100.0, /*threshold=*/1.0);
    charged.setUnknownPenalty(unknownBeyondFrontier, /*weight=*/10.0);
    expectNear(charged.pathCost(path), 14.0 + 12.0 * 10.0, kIntegrationSlack,
               "unknown surcharge not charged per metre beyond the frontier");

    // The breakdown must attribute it to the unknown term, not to clearance:
    // the two call for opposite fixes, so a log that confuses them misleads.
    const auto cb = charged.costBreakdown(path);
    expectNear(cb.length, 14.0, 1e-6, "breakdown length wrong");
    expectNear(cb.clearance, 0.0, 1e-6, "breakdown blamed clearance for the unknown surcharge");
    expectNear(cb.unknown, 120.0, kIntegrationSlack, "breakdown unknown term wrong");
    expectNear(cb.total, cb.length + cb.clearance + cb.unknown, 1e-9,
               "breakdown terms do not sum to the total");
  }

  // 2. Flat, not a ramp: cost must keep accruing the further in you go. Twice
  //    the depth into unknown space, twice the surcharge. A distance-based
  //    penalty would saturate and stop charging, which is exactly how the old
  //    behaviour let a route dive deep for free.
  {
    GeometricPlanner planner(empty, /*planning_time=*/0.1);
    planner.setClearance(wideOpen, /*weight=*/1.0, /*threshold=*/1.0);
    planner.setUnknownPenalty(unknownBeyondFrontier, /*weight=*/10.0);
    const double shallow = planner.costBreakdown({{0, 0, 1}, {8, 0, 1}}).unknown;   // 6 m
    const double deep = planner.costBreakdown({{0, 0, 1}, {14, 0, 1}}).unknown;     // 12 m
    expectNear(deep, 2.0 * shallow, 2.0 * kIntegrationSlack,
               "unknown surcharge does not accrue linearly with depth");
  }

  // 3. A weight of 0, or no predicate, must leave the cost exactly as it was.
  {
    const std::vector<std::vector<double>> path = {{0, 0, 1}, {4, 0, 1}};
    GeometricPlanner off(empty, /*planning_time=*/0.1);
    off.setClearance(wideOpen, /*weight=*/1.0, /*threshold=*/1.0);
    off.setUnknownPenalty(unknownBeyondFrontier, /*weight=*/0.0);
    expectNear(off.pathCost(path), 4.0, 1e-6, "zero weight still charged for unknown space");
    expectNear(off.costBreakdown(path).unknown, 0.0, 1e-9,
               "zero weight reported a non-zero unknown term");
  }

  // 4. The surcharge must not make unknown space INVALID. A goal beyond the
  //    frontier has to stay reachable, or the optimistic search loses the whole
  //    reason it exists: EIT* would discard the goal state and abort, and
  //    best-effort could no longer chase a goal as the map grows.
  {
    GeometricPlanner planner(empty, /*planning_time=*/0.5);
    planner.setClearance(wideOpen, /*weight=*/1.0, /*threshold=*/1.0);
    planner.setUnknownPenalty(unknownBeyondFrontier, /*weight=*/50.0);
    std::vector<std::vector<double>> path;
    expect(planner.planPath({0, 0, 1}, {5, 0, 1}, path) && !path.empty(),
           "an expensive-unknown goal beyond the frontier became unreachable");
    expect(planner.lastGoalProjection() == 0.0,
           "a goal in unknown space was projected, so the surcharge leaked into validity");
    if (!path.empty()) {
      expect(path.back()[0] > kFrontierX,
             "the path stopped at the frontier instead of reaching the goal beyond it");
    }
  }

  // 5. The routing fix, which is the actual bug. Mapped space is a corridor
  //    -1 <= y <= 1 for x <= 2; beyond x = 2 everything is unknown. The goal
  //    sits at (4, 0), so any path must end in unknown space. But a detour that
  //    swings wide through unknown space to get there must lose to the direct
  //    route, which enters as late as possible. Scored directly rather than via
  //    a solve, so the assertion does not depend on planner randomness.
  {
    GeometricPlanner planner(empty, /*planning_time=*/0.1);
    planner.setClearance(wideOpen, /*weight=*/1.0, /*threshold=*/1.0);
    planner.setUnknownPenalty(unknownBeyondFrontier, /*weight=*/10.0);

    // Straight in: 2 m of unknown.
    const double direct = planner.pathCost({{0, 0, 1}, {4, 0, 1}});
    // Out through a gap and around: shorter in mapped space but 4+ m of unknown.
    const double around =
        planner.pathCost({{0, 0, 1}, {1, 0, 1}, {3, 3, 1}, {4, 0, 1}});
    expect(around > direct,
           "a wide detour through unknown space still beats the direct route");
  }

  // 6. Truncation's hard stop. The clearance oracle says everything is fine —
  //    which is what a gap in the stamped shell looks like — so only the
  //    predicate can cut the path. Without it the whole 4 m is committed.
  {
    const std::vector<Eigen::Vector3d> path = {{0, 0, 1}, {4, 0, 1}};
    const CorridorClearanceFn clear = [](double, double, double) { return 1.0; };

    const auto uncut = truncatePath(clear, path, /*margin=*/0.5, /*escape_ramp=*/1.0);
    expectNear(uncut.back().x(), 4.0, 1e-6,
               "a clearance-only truncation should commit the whole path here");

    const auto cut = truncatePath(clear, path, /*margin=*/0.5, /*escape_ramp=*/1.0,
                                  /*sample_step=*/0.05, unknownBeyondFrontier);
    expect(cut.size() >= 2, "unknown-aware truncation produced no committable prefix");
    expectNear(cut.back().x(), kFrontierX, 0.06,
               "truncation did not stop at the edge of observed space");
  }

  // 7. The stop is exempt from the escape ramp. The ramp trades margin for the
  //    ability to move at all, which is defensible against a hazard whose
  //    distance we can measure. It is not defensible against space we have never
  //    looked at, so even a point close to the drone must be cut.
  {
    const CorridorClearanceFn clear = [](double, double, double) { return 1.0; };
    // Frontier at x = 0.2, well inside a 1 m escape ramp.
    const auto nearUnknown = [](double x, double, double) { return x > 0.2; };
    const auto cut = truncatePath(clear, {{0, 0, 1}, {4, 0, 1}}, /*margin=*/0.5,
                                  /*escape_ramp=*/1.0, /*sample_step=*/0.05, nearUnknown);
    expect(cut.back().x() <= 0.25,
           "the escape ramp let truncation commit into unknown space near the drone");
  }

  if (failures == 0) {
    std::cout << "unknown_cost: all checks passed\n";
    return 0;
  }
  return 1;
}
