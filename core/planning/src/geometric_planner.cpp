#include "drone_core/planning/geometric_planner.hpp"

#include "drone_core/common/logging.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <string>
#include <limits>
#include <utility>

#include <ompl/base/PlannerData.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/samplers/informed/PathLengthDirectInfSampler.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/informedtrees/ABITstar.h>
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/informedtrees/EITstar.h>

namespace drone_core::planning {

namespace {

// Single-integral cost that rewards short, high-clearance paths that stay in
// mapped space. The per-state cost is 1 (so the integral over the path equals
// its length when there is no other term) plus two additions: an
// obstacle-proximity penalty that grows as a state approaches an obstacle and
// saturates at the clearance threshold, and a flat surcharge wherever the state
// sits in space that has never been observed. Integrating over arc length makes
// the total comparable between any two paths regardless of how many waypoints
// each has.
class ClearanceObjective : public ompl::base::StateCostIntegralObjective {
public:
  // `clearance` is the obstacle field, weighted by `weight`. `frontier`, when
  // set, is a field that also counts the frontier (never farther from a hazard
  // than `clearance`); the extra penalty it gives over `clearance` is weighted
  // by `frontier_weight`. With the two weights equal that is exactly the single
  // penalty on `frontier`. Either may be null.
  ClearanceObjective(const ompl::base::SpaceInformationPtr& si,
                     GeometricPlanner::ClearanceFn clearance, double weight,
                     GeometricPlanner::ClearanceFn frontier, double frontier_weight,
                     double threshold, GeometricPlanner::UnknownFn is_unknown,
                     double unknown_weight, GeometricPlanner::UnknownFn steep_unknown = {},
                     double slope_tan = 0.0, double steep_weight = 0.0)
      : ompl::base::StateCostIntegralObjective(si, /*enableMotionCostInterpolation=*/true),
        clearance_(std::move(clearance)), weight_(weight), frontier_(std::move(frontier)),
        frontier_weight_(frontier_weight), threshold_(threshold),
        unknown_(std::move(is_unknown)), unknown_weight_(unknown_weight),
        steep_unknown_(std::move(steep_unknown)), slope_tan_(slope_tan), steep_weight_(steep_weight) {
    // Register a state->goal cost-to-go heuristic (distance to the goal region).
    // This is what RRT*'s informed sampler and the BIT*-lineage goal heuristic
    // query via hasCostToGoHeuristic()/costToGo(); without it OMPL warns that
    // informed sampling "will have little to no effect". Admissible for the same
    // reason as motionCostHeuristic below: our integrand is >= 1, so true cost is
    // always >= geometric distance.
    setCostToGoHeuristic(&ompl::base::goalRegionCostToGo);
  }

  // The state integral, plus the steep-into-unknown term (see
  // GeometricPlanner::setUnknownSlopeLimit): steep_weight x the vertical metres
  // beyond the slope limit, times the fraction of the edge in unknown space.
  ompl::base::Cost motionCost(const ompl::base::State* s1,
                              const ompl::base::State* s2) const override {
    const ompl::base::Cost base = ompl::base::StateCostIntegralObjective::motionCost(s1, s2);
    if (!(steep_weight_ > 0.0) || !(slope_tan_ > 0.0) || !steep_unknown_) return base;
    const auto* a = s1->as<ompl::base::RealVectorStateSpace::StateType>();
    const auto* b = s2->as<ompl::base::RealVectorStateSpace::StateType>();
    const double dx = b->values[0] - a->values[0], dy = b->values[1] - a->values[1],
                 dz = b->values[2] - a->values[2];
    const double excess = std::abs(dz) - slope_tan_ * std::hypot(dx, dy);
    if (!(excess > 0.0)) return base;
    constexpr double kStep = 0.1;  // [m]
    const int n = std::max(1, static_cast<int>(std::ceil(std::sqrt(dx * dx + dy * dy + dz * dz) / kStep)));
    int unknown = 0;
    for (int i = 0; i <= n; ++i) {
      const double t = static_cast<double>(i) / n;
      unknown += steep_unknown_(a->values[0] + t * dx, a->values[1] + t * dy, a->values[2] + t * dz) ? 1 : 0;
    }
    return ompl::base::Cost(base.value() + steep_weight_ * excess * unknown / (n + 1));
  }

  ompl::base::Cost stateCost(const ompl::base::State* state) const override {
    if (!clearance_ && !frontier_ && !unknown_) return ompl::base::Cost(1.0);
    const auto* pos = state->as<ompl::base::RealVectorStateSpace::StateType>();
    const double x = pos->values[0], y = pos->values[1], z = pos->values[2];

    double penalty = 0.0;
    const double obst = clearance_ ? std::max(0.0, threshold_ - clearance_(x, y, z)) : 0.0;
    penalty += weight_ * obst;
    if (frontier_) {
      // Only what the frontier adds over the obstacles: zero wherever a mapped
      // obstacle is at least as close as the frontier.
      const double both = std::max(0.0, threshold_ - frontier_(x, y, z));
      penalty += frontier_weight_ * std::max(0.0, both - obst);
    }
    // Flat, not proportional to anything: the point is that every metre spent in
    // unobserved space costs the same, so a route diving deep into it keeps
    // paying. See GeometricPlanner::setUnknownPenalty.
    if (unknown_ && unknown_(x, y, z)) {
      penalty += unknown_weight_;
    }
    return ompl::base::Cost(1.0 + penalty);
  }

  // Admissible cost-to-go lower bound for an edge: the straight-line distance
  // between the two states. The per-state integrand is 1 + penalty >= 1, so the
  // true motion cost is always >= geometric length >= this distance — never an
  // overestimate. The base StateCostIntegralObjective returns ~0 here, which
  // would leave the heuristic-driven planners (BIT*/ABIT*) essentially blind; a
  // real heuristic is what makes them search toward the goal.
  ompl::base::Cost motionCostHeuristic(const ompl::base::State* s1,
                                       const ompl::base::State* s2) const override {
    return ompl::base::Cost(si_->distance(s1, s2));
  }

  // Sample the informed set directly. Without this OMPL falls back to rejection
  // sampling for a state-cost integral: it draws from the whole state space and
  // keeps only states inside the informed set. Once the best solution is short —
  // an improve search with the vehicle near the end of its path — that set is a
  // tiny fraction of the space, almost every draw is rejected, and the sampling
  // loop does not check the planner's stop condition, so the solve hangs far past
  // its budget (bench 2026-09-28/29: 59-63 s with EIT*, and ABIT* too; reproduced
  // offline with start and goal 2-30 cm apart). The path-length ellipsoid is a
  // valid superset here: the integrand is >= 1, so any state on a path of cost c
  // lies within length c of start plus goal. Sampling it directly loses no
  // candidates and costs one draw per sample.
  ompl::base::InformedSamplerPtr allocInformedStateSampler(
      const ompl::base::ProblemDefinitionPtr& probDefn, unsigned int maxNumberCalls) const override {
    return std::make_shared<ompl::base::PathLengthDirectInfSampler>(probDefn, maxNumberCalls);
  }

private:
  GeometricPlanner::ClearanceFn clearance_;
  double weight_;
  GeometricPlanner::ClearanceFn frontier_;
  double frontier_weight_;
  double threshold_;
  GeometricPlanner::UnknownFn unknown_;
  double unknown_weight_;
  GeometricPlanner::UnknownFn steep_unknown_;
  double slope_tan_;
  double steep_weight_;
};

}  // namespace

GeometricPlanner::GeometricPlanner(const MapHandle& octree, double planning_time)
    : octree_ptr_(octree), planning_time_(planning_time) {
  // Position only. An SE(3) space was used before, but nothing reads orientation
  // (the collision check is a sphere, the trajectory takes positions) and OMPL
  // counts the rotation angle between two states as distance. Random samples get
  // random rotations while start and goal have none, so every detour through a
  // sample paid ~2 m of phantom length, and the informed planners pruned nearly
  // all samples as unable to improve a solution — a short query came back as the
  // bare start-goal edge even when a cheaper bend existed.
  space_ = std::make_shared<ompl::base::RealVectorStateSpace>(3);

  // Bounds tuned for the office test environment: above the floor, below eye
  // level (see kSearchLow / kSearchHigh).
  ompl::base::RealVectorBounds bounds(3);
  for (int i = 0; i < 3; ++i) {
    bounds.setLow(i, kSearchLow[i]);
    bounds.setHigh(i, kSearchHigh[i]);
  }

  space_->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);

  si_ = std::make_shared<ompl::base::SpaceInformation>(space_);
  si_->setStateValidityChecker([this](const ompl::base::State* state) {
    return isStateValid(state);
  });
  // OMPL takes this as a FRACTION of the space's maximum extent, not a distance:
  // the old 0.01 over these bounds (~42.6 m diagonal) checked segments only every
  // ~0.43 m, so a path could graze an obstacle between checks and then be cut by
  // truncatePath, which samples every 5 cm. Convert the step to that fraction so
  // the search checks the same points truncation will.
  si_->setStateValidityCheckingResolution(kValidityCheckStep / space_->getMaximumExtent());
  si_->setup();
}

void GeometricPlanner::setConservative(UnknownFn is_unknown, ClearanceFn conservative,
                                       double unknown_margin) {
  cons_unknown_fn_ = std::move(is_unknown);
  cons_clearance_fn_ = std::move(conservative);
  cons_unknown_margin_ = std::max(0.0, unknown_margin);
}

void GeometricPlanner::setUnknownSlopeLimit(UnknownFn is_unknown, double max_slope_deg,
                                            double steep_weight) {
  slope_unknown_fn_ = std::move(is_unknown);
  max_slope_tan_ = max_slope_deg > 0.0 && max_slope_deg < 90.0
                       ? std::tan(max_slope_deg * M_PI / 180.0)
                       : 0.0;
  steep_weight_ = std::max(0.0, steep_weight);
}

void GeometricPlanner::anchorStart(double x, double y, double z) const {
  start_pos_ = {x, y, z};
  start_floor_unknown_ =
      conservativeActive()
          ? std::max(0.0, cons_clearance_fn_(x, y, z) -
                              0.5 * (octree_ptr_ ? octree_ptr_->getResolution() : 0.0))
          : 0.0;
  // The start's clearance less half a voxel: the EDT reports cell-to-cell
  // distances, so sliding along a wall that is not axis-aligned reads a few
  // centimetres of jitter that is quantisation, not approach. Deliberately
  // tighter than truncation's floor (one voxel or 5% of the clearance,
  // whichever is more lenient; see truncatePath), so the search stays the
  // stricter stage and a path it accepts is not cut near the drone. Without a
  // field (standalone octree fallback) there is no clearance to floor at, and 0
  // leaves the plain ramp.
  start_floor_ = 0.0;
  if (clearance_fn_) {
    const double half_voxel = octree_ptr_ ? 0.5 * octree_ptr_->getResolution() : 0.0;
    start_floor_ = std::max(0.0, clearance_fn_(x, y, z) - half_voxel);
  }
}

bool GeometricPlanner::isStateValid(const ompl::base::State* state) {
  const auto* pos = state->as<ompl::base::RealVectorStateSpace::StateType>();
  return positionValid(pos->values[0], pos->values[1], pos->values[2]);
}

bool GeometricPlanner::positionValid(double x, double y, double z) const {
  // Margin to enforce here: it ramps linearly from 0 at the start to the full
  // collision margin escape_ramp_ metres out, so a parked/lifting drone sitting
  // within the margin of the mapped floor can still root the search without ever
  // entering an obstacle. This is the same ramp truncatePath applies, over the
  // same distance, so a path the search accepts is not then cut near the drone
  // for climbing away from an obstacle more slowly than truncation demands (a
  // hard exemption sphere allowed exactly that). The ramp centre (start_pos_) is
  // anchored in planPath / isPathValid.
  //
  // The ramp is floored at the start's own clearance (start_floor_, see
  // anchorStart): a drone already inside the margin may root a path that climbs
  // away, but not one that brings it any closer to an obstacle than it already
  // is. Without the floor, a start 0.2 m off a wall could head straight at it
  // until clearance fell below the rising ramp, ~0.07 m. Capped at the full
  // margin, so it only ever matters within escape_ramp_ of the start.
  const double ex = x - start_pos_[0];
  const double ey = y - start_pos_[1];
  const double ez = z - start_pos_[2];
  const double from_start = std::sqrt(ex * ex + ey * ey + ez * ez);

  // The slope cone (see setUnknownSlopeLimit): never-observed space steeper
  // than the limit from the start is where the camera will not be looking.
  if (slopeActive() && std::abs(ez) > max_slope_tan_ * std::hypot(ex, ey) &&
      slope_unknown_fn_(x, y, z)) {
    return false;
  }

  for (const auto& e : exclusions_) {
    const double dx = x - e.x(), dy = y - e.y(), dz = z - e.z();
    if (dx * dx + dy * dy + dz * dz < exclusion_radius_ * exclusion_radius_ && from_start > 1e-9) {
      return false;
    }
  }

  if (conservativeActive()) {
    // The one exempt point: a goal in unknown space (see setConservative).
    if (exempt_goal_ && x == exempt_goal_pos_[0] && y == exempt_goal_pos_[1] &&
        z == exempt_goal_pos_[2]) {
      return true;
    }
    const bool is_start = from_start < 1e-9;
    if (!is_start && cons_unknown_fn_(x, y, z)) return false;
    const double need_unknown =
        escape_ramp_ > 0.0
            ? std::min(cons_unknown_margin_,
                       std::max(start_floor_unknown_,
                                cons_unknown_margin_ * std::min(1.0, from_start / escape_ramp_)))
            : cons_unknown_margin_;
    if (!is_start && !(cons_clearance_fn_(x, y, z) > need_unknown)) return false;
  }

  const double margin =
      escape_ramp_ > 0.0
          ? std::min(kCollisionMargin,
                     std::max(start_floor_,
                              kCollisionMargin * std::min(1.0, from_start / escape_ramp_)))
          : kCollisionMargin;

  // Preferred path: a single O(1) clearance lookup when a clearance field is set
  // (the live planner always sets one). The field is the 3D Euclidean distance to
  // the nearest obstacle, so this enforces vertical clearance too — unlike the
  // horizontal-only fallback below — and is far cheaper than the cell scan.
  // Outside the field / unknown space reads as free (it returns the saturation
  // distance), matching the treat-unknown-as-free policy.
  //
  // This is deliberately clearance_fn_ and never the cost field: the cost may be
  // scored against a conservative view in which the mapped frontier reads as an
  // obstacle (see setCostClearance), and testing validity against that would
  // make the frontier a closed surface the search could not cross anywhere.
  if (clearance_fn_) {
    return clearance_fn_(x, y, z) > margin;
  }

  // Fallback for standalone use (no clearance field, e.g. unit tests): scan the
  // octree directly, inflating obstacles by `margin` in the horizontal plane.
  // Unknown space (null node) is free. The motion checker samples every
  // kValidityCheckStep, which is not finer than a 5 cm voxel, so at margin 0
  // (the start of the escape ramp) a segment could clip a voxel corner between
  // samples; any margin above half the step closes that.
  if (!octree_ptr_) return false;
  const double res = octree_ptr_->getResolution();
  for (double dx = -margin; dx <= margin; dx += res) {
    for (double dy = -margin; dy <= margin; dy += res) {
      const octomap::point3d query(x + dx, y + dy, z);
      const auto* node = octree_ptr_->search(query);
      if (node != nullptr && octree_ptr_->isNodeOccupied(node)) {
        return false;
      }
    }
  }
  return true;
}

bool GeometricPlanner::projectGoal(const std::vector<double>& goal,
                                   std::array<double, 3>& out) const {
  const auto& bounds = space_->as<ompl::base::RealVectorStateSpace>()->getBounds();
  // Nudge inside the bounds rather than onto them: OMPL's bounds test is
  // inclusive, but a goal sitting exactly on the ceiling plane leaves the search
  // no room on one side, and the inset is far below the map resolution.
  constexpr double kBoundsInset = 1.0e-3;
  auto clampToBounds = [&](std::array<double, 3>& p) {
    for (int i = 0; i < 3; ++i) {
      p[i] = std::min(std::max(p[i], bounds.low[i] + kBoundsInset),
                      bounds.high[i] - kBoundsInset);
    }
  };

  std::array<double, 3> base{goal[0], goal[1], goal[2]};
  clampToBounds(base);
  if (positionValid(base[0], base[1], base[2])) {
    out = base;
    return true;
  }

  // Escape directions: the six axes first (a voxel obstacle's shortest way out is
  // usually normal to one of its faces), then a Fibonacci lattice over the sphere
  // for everything else — deterministic, no clustering at the poles, and no trig
  // table to keep in sync with the count.
  std::array<std::array<double, 3>, 6 + kGoalProjectDirections> dirs{
      {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}}};
  const double golden = M_PI * (3.0 - std::sqrt(5.0));
  for (int i = 0; i < kGoalProjectDirections; ++i) {
    const double dz = 1.0 - 2.0 * (static_cast<double>(i) + 0.5) / kGoalProjectDirections;
    const double r = std::sqrt(std::max(0.0, 1.0 - dz * dz));
    const double phi = golden * static_cast<double>(i);
    dirs[6 + i] = {r * std::cos(phi), r * std::sin(phi), dz};
  }

  // Walk outward. The first radius that yields any valid point is the answer:
  // everything closer to the goal has already been rejected, so no later radius
  // can do better.
  const int steps = static_cast<int>(std::floor(kGoalProjectRadius / kGoalProjectStep));
  for (int s = 1; s <= steps; ++s) {
    const double radius = s * kGoalProjectStep;
    bool found = false;
    double best_start_dist = std::numeric_limits<double>::infinity();
    std::array<double, 3> best{};
    for (const auto& d : dirs) {
      std::array<double, 3> cand{base[0] + radius * d[0], base[1] + radius * d[1],
                                 base[2] + radius * d[2]};
      clampToBounds(cand);
      if (!positionValid(cand[0], cand[1], cand[2])) continue;
      const double sx = cand[0] - start_pos_[0];
      const double sy = cand[1] - start_pos_[1];
      const double sz = cand[2] - start_pos_[2];
      const double start_dist = sx * sx + sy * sy + sz * sz;
      if (start_dist < best_start_dist) {
        best_start_dist = start_dist;
        best = cand;
        found = true;
      }
    }
    if (found) {
      out = best;
      return true;
    }
  }
  return false;
}

double GeometricPlanner::minClearance(const std::vector<std::vector<double>>& path) const {
  if (!clearance_fn_ || path.size() < 2) return std::numeric_limits<double>::infinity();
  // Only the region where the validity check enforces the full margin is
  // meaningful here: within escape_ramp_ of the start the required margin ramps
  // down to 0 (so a parked/lifting drone can root the search), so a tight
  // clearance there is expected and would not block the path. Anchor the ramp
  // at the first waypoint, mirroring isPathValid, and skip sampled points inside
  // it so this reports the lowest clearance among the points held to the full
  // margin. Returns +infinity if every sampled point is inside the ramp.
  const std::array<double, 3> center{path.front()[0], path.front()[1], path.front()[2]};
  auto insideEscape = [&](double x, double y, double z) {
    const double dx = x - center[0], dy = y - center[1], dz = z - center[2];
    return dx * dx + dy * dy + dz * dz < escape_ramp_ * escape_ramp_;
  };
  double mind = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const auto& a = path[i];
    const auto& b = path[i + 1];
    const double seg = std::hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]);
    const int steps = std::max(1, static_cast<int>(std::ceil(seg / 0.05)));
    for (int s = 0; s <= steps; ++s) {
      const double u = static_cast<double>(s) / steps;
      const double x = a[0] + u * (b[0] - a[0]);
      const double y = a[1] + u * (b[1] - a[1]);
      const double z = a[2] + u * (b[2] - a[2]);
      if (insideEscape(x, y, z)) continue;
      mind = std::min(mind, clearance_fn_(x, y, z));
    }
  }
  return mind;
}

bool GeometricPlanner::isPathValid(const std::vector<std::vector<double>>& path) const {
  if (path.size() < 2) return false;
  exempt_goal_ = false;  // only planPath's own goal is ever exempt

  // Anchor the start-state exemption at the path's first waypoint so a committed
  // path that begins on the floor (the takeoff pose) does not fail this periodic
  // re-check and force a needless replan. Mirrors what planPath exempted.
  anchorStart(path.front()[0], path.front()[1], path.front()[2]);

  auto makeState = [this](const std::vector<double>& p) {
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> s(space_);
    s->values[0] = p[0];
    s->values[1] = p[1];
    s->values[2] = p[2];
    return s;
  };

  // Walk every segment: both endpoints must be valid and the straight-line
  // motion between them collision-free at the space's checking resolution. This
  // reuses the exact collision model (bounds + inflated occupancy) the planner
  // searched with, so "valid" here means the same thing it did at plan time.
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const auto a = makeState(path[i]);
    const auto b = makeState(path[i + 1]);
    if (!si_->isValid(a.get())) return false;
    if (!si_->isValid(b.get())) return false;
    if (!si_->checkMotion(a.get(), b.get())) return false;
  }
  return true;
}

void GeometricPlanner::setClearance(ClearanceFn clearance, double weight, double threshold) {
  clearance_fn_ = std::move(clearance);
  clearance_weight_ = weight;
  clearance_threshold_ = threshold;
}

void GeometricPlanner::setUnknownPenalty(UnknownFn is_unknown, double weight) {
  unknown_fn_ = std::move(is_unknown);
  unknown_weight_ = weight;
}

void GeometricPlanner::setCostClearance(ClearanceFn clearance, double frontier_weight) {
  // Deliberately does not touch clearance_fn_, so a host may call the two
  // setters in either order (see the header note on order-independence).
  cost_clearance_fn_ = std::move(clearance);
  frontier_weight_ = frontier_weight;
}

ompl::base::PlannerPtr GeometricPlanner::makePlanner() const {
  // Build the selected OMPL planner and apply its PlannerConfig sub-struct. Only
  // the construction/parameters differ between planners; the SpaceInformation,
  // validity checker and objective are shared. setup() is left to planPath (it
  // runs after setProblemDefinition).
  switch (planner_type_) {
    case PlannerType::BITstar: {
      auto p = std::make_shared<ompl::geometric::BITstar>(si_);
      const auto& c = params_.bitstar;
      // OMPL renames a k-nearest BIT* itself, with a warning; name it first so
      // the warning does not print on every search.
      if (c.use_k_nearest) p->setName("kBITstar");
      p->setSamplesPerBatch(c.samples_per_batch);
      p->setRewireFactor(c.rewire_factor);
      p->setUseKNearest(c.use_k_nearest);
      p->setPruning(c.pruning);
      p->setJustInTimeSampling(c.jit_sampling);
      return p;
    }
    case PlannerType::ABITstar: {
      // ABIT* IS-A BIT*, so it takes the BitStar fields plus its own inflation /
      // truncation knobs.
      auto p = std::make_shared<ompl::geometric::ABITstar>(si_);
      const auto& c = params_.bitstar;
      if (c.use_k_nearest) p->setName("kABITstar");  // see BIT* above
      p->setSamplesPerBatch(c.samples_per_batch);
      p->setRewireFactor(c.rewire_factor);
      p->setUseKNearest(c.use_k_nearest);
      p->setPruning(c.pruning);
      p->setJustInTimeSampling(c.jit_sampling);
      const auto& a = params_.abitstar;
      p->setInitialInflationFactor(a.initial_inflation);
      p->setInflationScalingParameter(a.inflation_scaling);
      p->setTruncationScalingParameter(a.truncation_scaling);
      return p;
    }
    case PlannerType::AITstar: {
      auto p = std::make_shared<ompl::geometric::AITstar>(si_);
      const auto& c = params_.aitstar;
      p->setBatchSize(c.batch_size);
      p->setRewireFactor(c.rewire_factor);
      p->setUseKNearest(c.use_k_nearest);
      return p;
    }
    case PlannerType::EITstar: {
      auto p = std::make_shared<ompl::geometric::EITstar>(si_);
      const auto& c = params_.eitstar;
      p->setBatchSize(c.batch_size);
      p->setUseKNearest(c.use_k_nearest);
      p->setSuboptimalityFactor(c.suboptimality);
      p->setRadiusFactor(c.radius_factor);
      return p;
    }
    case PlannerType::RRTstar:
    default: {
      auto p = std::make_shared<ompl::geometric::RRTstar>(si_);
      const auto& c = params_.rrtstar;
      // Set the step size before setup(): setup() only auto-sizes the range when
      // it is still zero, so this overrides OMPL's extent-fraction default. <=0
      // keeps the auto behaviour.
      if (c.range > 0.0) p->setRange(c.range);
      p->setGoalBias(c.goal_bias);
      p->setRewireFactor(c.rewire_factor);
      p->setInformedSampling(c.informed);
      p->setKNearest(c.k_nearest);
      return p;
    }
  }
}

ompl::base::OptimizationObjectivePtr GeometricPlanner::makeObjective(
    bool include_unknown, bool include_obstacles, bool include_frontier,
    bool include_steep) const {
  // With no clearance function the per-state cost is a constant 1, so this is
  // exactly a path-length objective; with one it adds the proximity penalty.
  // With a separate cost field (see setCostClearance) the obstacle penalty is
  // scored on the validity field and the frontier's extra on the cost field,
  // each with its own weight; with only one field, that field at
  // clearance_weight_. Everything that scores a path — pathCost, costBreakdown,
  // the shortcut pass — therefore agrees with what the search minimised.
  // The include_* switches drop a term's weight to 0 (costBreakdown only).
  const bool split = static_cast<bool>(clearance_fn_) && static_cast<bool>(cost_clearance_fn_);
  return std::make_shared<ClearanceObjective>(
      si_, split ? clearance_fn_ : costClearanceFn(), include_obstacles ? clearance_weight_ : 0.0,
      split ? cost_clearance_fn_ : ClearanceFn{}, include_frontier ? frontier_weight_ : 0.0,
      clearance_threshold_,
      (include_unknown && unknownPenaltyActive()) ? unknown_fn_ : UnknownFn{},
      unknown_weight_, slope_unknown_fn_, max_slope_tan_, include_steep ? steep_weight_ : 0.0);
}

bool GeometricPlanner::planPath(const std::vector<double>& start_vec,
                              const std::vector<double>& goal_vec,
                              std::vector<std::vector<double>>& result_path) {
  ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start(space_);
  ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal(space_);

  for (int i = 0; i < 3; ++i) start->values[i] = start_vec[i];

  // A start outside the box would be refused by OMPL ("invalid start state
  // (invalid bounds)") and, since the vehicle cannot move without a plan, never
  // recover. Grow the box just enough to hold it. The goal is not treated this
  // way: a goal outside the box is clamped into it, not followed out.
  {
    auto* rv = space_->as<ompl::base::RealVectorStateSpace>();
    ompl::base::RealVectorBounds b = rv->getBounds();
    constexpr double kStartPad = 0.1;  // [m]
    bool grown = false;
    for (int i = 0; i < 3; ++i) {
      if (start_vec[i] < b.low[i]) {
        b.low[i] = start_vec[i] - kStartPad;
        grown = true;
      } else if (start_vec[i] > b.high[i]) {
        b.high[i] = start_vec[i] + kStartPad;
        grown = true;
      }
    }
    if (grown) {
      rv->setBounds(b);
      DRONE_LOG_INFO("[plan] start (" << start_vec[0] << ", " << start_vec[1] << ", "
                     << start_vec[2] << ") is outside the search box; grew it to hold the start");
    }
  }

  // Only the OMPL solve is bounded by planning_time_ — goal projection, the
  // debug tree capture and the post-processing shortcut are not, and the solve
  // itself only checks its termination condition between iterations. A search
  // taking tens of seconds against a 1 s budget has been seen on the bench, so
  // when this call overruns badly, say where the time went. Silent otherwise:
  // four clock reads on a solve that behaved.
  const auto t_begin = std::chrono::steady_clock::now();
  auto lap = [](std::chrono::steady_clock::time_point& mark) {
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - mark).count();
    mark = now;
    return dt;
  };
  auto mark = t_begin;
  double t_project = 0.0, t_setup = 0.0, t_solve = 0.0, t_tree = 0.0, t_post = 0.0;
  const auto reportOverrun = [&](const char* outcome) {
    const double total = std::chrono::duration<double>(
                             std::chrono::steady_clock::now() - t_begin).count();
    if (total <= 1.5 * planning_time_) return;
    DRONE_LOG_INFO("[plan] search OVERRAN its " << planning_time_ << " s budget: " << total
                   << " s total (goal projection " << t_project << " s, setup " << t_setup
                   << " s, solve " << t_solve << " s, tree capture " << t_tree
                   << " s, shortcut " << t_post << " s) — " << outcome);
  };

  // Anchor the start-state collision exemption at this solve's start. Must
  // precede projectGoal, which validates candidates under the same exemption.
  anchorStart(start_vec[0], start_vec[1], start_vec[2]);

  // Move the goal to the nearest state the collision check accepts, if it does
  // not accept the requested one. OMPL drops invalid goal states inside
  // PlannerInputStates::nextGoal and the planner then has nothing to grow
  // toward, so an unprojected goal near a wall produces no path at all — not a
  // path that stops at the margin. Projecting is what turns "invalid goal" back
  // into the ordinary "goal we approach as closely as the margin allows" case.
  std::array<double, 3> planning_goal{};
  exempt_goal_ = false;
  // The exemption lives for this search only, whichever way it returns.
  struct ExemptReset {
    bool& flag;
    ~ExemptReset() { flag = false; }
  } exempt_reset{exempt_goal_};
  if (conservativeActive() && best_effort_ && !positionValid(goal_vec[0], goal_vec[1], goal_vec[2])) {
    // A goal the conservative check refuses (in or next to unknown space, or
    // near an obstacle): keep it where it is, exempt only that point, and let
    // best effort return the reachable point nearest it (see setConservative).
    // Projection would look for a valid point within kGoalProjectRadius, which
    // in unknown space there usually is not.
    const auto& bounds = space_->as<ompl::base::RealVectorStateSpace>()->getBounds();
    constexpr double kInset = 1e-3;  // [m], as projectGoal
    for (int i = 0; i < 3; ++i) {
      planning_goal[i] = std::clamp(goal_vec[i], bounds.low[i] + kInset, bounds.high[i] - kInset);
    }
    exempt_goal_ = true;
    exempt_goal_pos_ = planning_goal;
  } else if (!projectGoal(goal_vec, planning_goal)) {
    last_goal_projection_ = std::numeric_limits<double>::infinity();
    last_goal_gap_ = std::numeric_limits<double>::infinity();
    t_project = lap(mark);
    reportOverrun("no valid goal nearby, never solved");
    return false;
  }
  t_project = lap(mark);
  last_planning_goal_ = planning_goal;
  last_goal_projection_ = std::sqrt(std::pow(planning_goal[0] - goal_vec[0], 2) +
                                    std::pow(planning_goal[1] - goal_vec[1], 2) +
                                    std::pow(planning_goal[2] - goal_vec[2], 2));

  for (int i = 0; i < 3; ++i) goal->values[i] = planning_goal[i];

  // Start and (projected) goal coincide: the path is the point itself, and there
  // is nothing to search. Not handed to OMPL, because a best cost of zero makes
  // the informed set a single point that the direct sampler can never draw from,
  // and RRT* then spins past its budget exactly like the rejection-sampling stall
  // allocInformedStateSampler exists to prevent.
  constexpr double kCoincident = 1e-3;  // [m]
  if (std::sqrt(std::pow(planning_goal[0] - start_vec[0], 2) +
                std::pow(planning_goal[1] - start_vec[1], 2) +
                std::pow(planning_goal[2] - start_vec[2], 2)) < kCoincident) {
    result_path.push_back({start_vec[0], start_vec[1], start_vec[2]});
    result_path.push_back({planning_goal[0], planning_goal[1], planning_goal[2]});
    last_goal_gap_ = last_goal_projection_;
    if (record_tree_) last_tree_ = SearchTree{};
    return true;
  }

  auto pdef = std::make_shared<ompl::base::ProblemDefinition>(si_);
  pdef->setStartAndGoalStates(start, goal);
  pdef->setOptimizationObjective(makeObjective());

  auto planner = makePlanner();
  planner->setProblemDefinition(pdef);
  planner->setup();
  t_setup = lap(mark);

  // The budget, or (setEarlyStop) less once a path reaching the goal exists.
  // Planners report such paths as they find them through the intermediate
  // solution callback; hasExactSolution covers those that only add one to the
  // problem definition.
  ompl::base::PlannerTerminationCondition ptc =
      ompl::base::timedPlannerTerminationCondition(planning_time_);
  bool found_exact = false;
  if (early_stop_ > 0.0 && early_stop_ < planning_time_) {
    pdef->setIntermediateSolutionCallback(
        [&found_exact](const ompl::base::Planner*, const std::vector<const ompl::base::State*>&,
                       const ompl::base::Cost) { found_exact = true; });
    const auto t_start = std::chrono::steady_clock::now();
    const double early = early_stop_;
    ptc = ompl::base::plannerOrTerminationCondition(
        ptc, ompl::base::PlannerTerminationCondition([&found_exact, &pdef, t_start, early]() {
          const double t =
              std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
          return t >= early && (found_exact || pdef->hasExactSolution());
        }));
  }
  const ompl::base::PlannerStatus solved = planner->solve(ptc);
  t_solve = lap(mark);

  // Capture the tree for debug visualisation before the planner goes out of
  // scope. Done regardless of success (the tree of a *failed* solve is just as
  // useful to look at) but only when recording is on, so a flight build never
  // pays for getPlannerData().
  // Skipped when the search sampled too many states to copy quickly: getPlannerData
  // runs without any time limit, and a short search (start near goal) now samples
  // its small informed set densely enough that copying it took 40 s (bench
  // 2026-09-29) with the solve itself on budget. Every sample is state-checked, so
  // that counter is the size proxy; planners without it are always captured.
  std::size_t sampled = 0;
  if (record_tree_) {
    const auto props = planner->getPlannerProgressProperties();
    const auto it = props.find("state collision checks INTEGER");
    if (it != props.end()) sampled = static_cast<std::size_t>(std::stoull(it->second()));
  }
  if (record_tree_ && sampled > kMaxTreeCaptureStates) {
    last_tree_ = SearchTree{};
    DRONE_LOG_INFO("[plan] search tree not captured for viz: " << sampled
                   << " states sampled (limit " << kMaxTreeCaptureStates << ")");
  } else if (record_tree_) {
    last_tree_ = SearchTree{};
    ompl::base::PlannerData data(si_);
    planner->getPlannerData(data);
    const unsigned int n = data.numVertices();
    last_tree_.nodes.reserve(n);
    for (unsigned int i = 0; i < n; ++i) {
      const auto* pos =
          data.getVertex(i).getState()->as<ompl::base::RealVectorStateSpace::StateType>();
      last_tree_.nodes.push_back({pos->values[0], pos->values[1], pos->values[2]});
    }
    for (unsigned int i = 0; i < n; ++i) {
      std::vector<unsigned int> out;
      data.getEdges(i, out);
      for (unsigned int j : out)
        last_tree_.edges.emplace_back(static_cast<int>(i), static_cast<int>(j));
    }
  }

  t_tree = lap(mark);
  if (!solved) {
    last_goal_gap_ = std::numeric_limits<double>::infinity();
    reportOverrun("no solution");
    return false;
  }

  auto path = std::static_pointer_cast<ompl::geometric::PathGeometric>(pdef->getSolutionPath());

  // Distance from the returned solution's endpoint to the goal the *caller* asked
  // for, which with a projected goal is not the one OMPL solved to. Measured off
  // the endpoint rather than taken from getSolutionDifference() so the two cases
  // report on the same reference point: the host compares this against its own
  // endpoint-to-goal distances when scoring best-effort progress, and a path that
  // stopped at the projection is genuinely still that far from the commanded
  // point. Recorded before post-processing, which never moves the endpoint.
  last_goal_gap_ = std::numeric_limits<double>::infinity();
  if (path->getStateCount() > 0) {
    const auto* epos = path->getState(path->getStateCount() - 1)
                           ->as<ompl::base::RealVectorStateSpace::StateType>();
    last_goal_gap_ = std::sqrt(std::pow(epos->values[0] - goal_vec[0], 2) +
                               std::pow(epos->values[1] - goal_vec[1], 2) +
                               std::pow(epos->values[2] - goal_vec[2], 2));
  }

  // A solution can stop short of the requested goal two ways: the tree never
  // connected to the goal (OMPL calls that an approximate solution), or the goal
  // was projected before the solve because the requested point was not a valid
  // state. Both are the same thing to the caller — the vehicle would end up this
  // far from where it was sent — so strict mode gates on the one number that
  // covers both, and rejects the plan rather than committing to a route that
  // ends somewhere else. The caller then treats it as no path (infinite cost).
  // Best-effort mode keeps it: it is the path to the closest point that is both
  // reachable and safe, and the caller uses lastGoalGap() to act on the shortfall.
  if (!best_effort_ && last_goal_gap_ > kGoalFlexibility) {
    return false;
  }

  // Straighten the jagged planner result. Deliberately skip B-spline smoothing so
  // the original waypoints survive for the minimum-snap optimiser downstream.
  // With no clearance field, plain length-only simplifyMax is correct and cheap.
  if (!clearanceMode()) {
    ompl::geometric::PathSimplifier simplifier(si_);
    simplifier.simplifyMax(*path);
  }

  for (std::size_t i = 0; i < path->getStateCount(); ++i) {
    const auto* pos = path->getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
    result_path.push_back({pos->values[0], pos->values[1], pos->values[2]});
  }

  // In clearance mode, use a cost-aware shortcut instead of simplifyMax: it cuts
  // the planner's zig-zag but, by gating on the clearance-aware cost, keeps the
  // detours the objective actually wanted (see shortcutClearanceAware).
  if (clearanceMode()) {
    shortcutClearanceAware(result_path);
  }
  t_post = lap(mark);
  reportOverrun("solved");
  return true;
}

void GeometricPlanner::shortcutClearanceAware(
    std::vector<std::vector<double>>& waypoints) const {
  if (waypoints.size() < 3) return;

  auto makeState = [this](const std::vector<double>& p) {
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> s(space_);
    s->values[0] = p[0];
    s->values[1] = p[1];
    s->values[2] = p[2];
    return s;
  };
  auto motionValid = [&](const std::vector<double>& a, const std::vector<double>& b) {
    const auto sa = makeState(a);
    const auto sb = makeState(b);
    return si_->checkMotion(sa.get(), sb.get());
  };

  // Anchor the start-state collision exemption at the first waypoint, mirroring
  // isPathValid, so a bypass near the takeoff pose is judged with the same
  // relaxed margin the search used (motionValid -> isStateValid reads start_pos_).
  anchorStart(waypoints.front()[0], waypoints.front()[1], waypoints.front()[2]);

  double current = costBreakdown(waypoints).total;
  bool changed = true;
  while (changed && waypoints.size() > 2) {
    changed = false;
    for (std::size_t i = 1; i + 1 < waypoints.size();) {
      const double seg = std::hypot(waypoints[i + 1][0] - waypoints[i - 1][0],
                                    waypoints[i + 1][1] - waypoints[i - 1][1],
                                    waypoints[i + 1][2] - waypoints[i - 1][2]);
      if (seg <= kMaxShortcutSegment && motionValid(waypoints[i - 1], waypoints[i + 1])) {
        std::vector<std::vector<double>> candidate = waypoints;
        candidate.erase(candidate.begin() + static_cast<std::ptrdiff_t>(i));
        const double cost = costBreakdown(candidate).total;
        if (cost <= current + 1e-9) {
          waypoints.swap(candidate);
          current = cost;
          changed = true;
          continue;  // re-examine index i (now the following vertex)
        }
      }
      ++i;
    }
  }
}

double GeometricPlanner::pathCost(const std::vector<std::vector<double>>& path) const {
  return costBreakdown(path).total;
}

GeometricPlanner::CostBreakdown
GeometricPlanner::costBreakdown(const std::vector<std::vector<double>>& path) const {
  constexpr double inf = std::numeric_limits<double>::infinity();
  if (path.size() < 2) return {inf, inf, inf, inf};

  ompl::geometric::PathGeometric geo(si_);
  for (const auto& p : path) {
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> s(space_);
    s->values[0] = p[0];
    s->values[1] = p[1];
    s->values[2] = p[2];
    geo.append(s.get());
  }
  // The objective's per-state cost is 1 + proximity penalty + unknown surcharge,
  // so the total minus the geometric length isolates what the penalties added.
  // Splitting those two apart needs a second pass with the surcharge switched
  // off — worth it because they call for opposite fixes (hugging a wall vs.
  // routing through unmapped space), and skipped entirely when no surcharge is
  // configured, which is the case that must stay cheap.
  const double total = geo.cost(makeObjective()).value();
  const double length = geo.length();
  double unknown = 0.0;
  if (unknownPenaltyActive()) {
    unknown = total - geo.cost(makeObjective(/*include_unknown=*/false)).value();
  }
  double steep = 0.0;
  if (slopeActive() && steep_weight_ > 0.0) {
    steep = total - geo.cost(makeObjective(true, true, true, /*include_steep=*/false)).value();
  }
  CostBreakdown cb{length, total - length - unknown - steep, unknown, total};
  cb.steep = steep;
  // The frontier's share, with the obstacle weight switched off. The integral
  // is linear in the per-state terms, so the obstacle share is the rest.
  if (cost_clearance_fn_ && clearance_fn_) {
    const double frontier =
        geo.cost(makeObjective(/*include_unknown=*/false, /*include_obstacles=*/false,
                               /*include_frontier=*/true, /*include_steep=*/false))
            .value() -
        length;
    cb.frontier = std::clamp(frontier, 0.0, std::max(cb.clearance, 0.0));
    cb.split = true;
  }
  return cb;
}

}  // namespace drone_core::planning
