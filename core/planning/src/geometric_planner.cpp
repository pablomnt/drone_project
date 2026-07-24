#include "drone_core/planning/geometric_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>

#include <ompl/base/PlannerData.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
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
  ClearanceObjective(const ompl::base::SpaceInformationPtr& si,
                     GeometricPlanner::ClearanceFn clearance, double weight, double threshold,
                     GeometricPlanner::UnknownFn is_unknown, double unknown_weight)
      : ompl::base::StateCostIntegralObjective(si, /*enableMotionCostInterpolation=*/true),
        clearance_(std::move(clearance)), weight_(weight), threshold_(threshold),
        unknown_(std::move(is_unknown)), unknown_weight_(unknown_weight) {
    // Register a state->goal cost-to-go heuristic (distance to the goal region).
    // This is what RRT*'s informed sampler and the BIT*-lineage goal heuristic
    // query via hasCostToGoHeuristic()/costToGo(); without it OMPL warns that
    // informed sampling "will have little to no effect". Admissible for the same
    // reason as motionCostHeuristic below: our integrand is >= 1, so true cost is
    // always >= geometric distance.
    setCostToGoHeuristic(&ompl::base::goalRegionCostToGo);
  }

  ompl::base::Cost stateCost(const ompl::base::State* state) const override {
    if (!clearance_ && !unknown_) return ompl::base::Cost(1.0);
    const auto* se3 = state->as<ompl::base::SE3StateSpace::StateType>();
    const auto* pos = se3->as<ompl::base::RealVectorStateSpace::StateType>(0);
    const double x = pos->values[0], y = pos->values[1], z = pos->values[2];

    double penalty = 0.0;
    if (clearance_) {
      penalty += weight_ * std::max(0.0, threshold_ - clearance_(x, y, z));
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

private:
  GeometricPlanner::ClearanceFn clearance_;
  double weight_;
  double threshold_;
  GeometricPlanner::UnknownFn unknown_;
  double unknown_weight_;
};

}  // namespace

GeometricPlanner::GeometricPlanner(const MapHandle& octree, double planning_time)
    : octree_ptr_(octree), planning_time_(planning_time) {
  // SE(3) state space leaves room for orientation should the vehicle footprint
  // ever become asymmetric, even though the current collision check is radial.
  space_ = std::make_shared<ompl::base::SE3StateSpace>();

  // Bounds tuned for the office test environment.
  ompl::base::RealVectorBounds bounds(3);
  bounds.setLow(0, -15.0);
  bounds.setHigh(0, 15.0);
  bounds.setLow(1, -15.0);
  bounds.setHigh(1, 15.0);

  // Keep the search above the floor and below eye level.
  bounds.setLow(2, -1.5);
  bounds.setHigh(2, 2.5);

  space_->as<ompl::base::SE3StateSpace>()->setBounds(bounds);

  si_ = std::make_shared<ompl::base::SpaceInformation>(space_);
  si_->setStateValidityChecker([this](const ompl::base::State* state) {
    return isStateValid(state);
  });
  si_->setStateValidityCheckingResolution(0.01);
  si_->setup();
}

bool GeometricPlanner::isStateValid(const ompl::base::State* state) {
  const auto* se3state = state->as<ompl::base::SE3StateSpace::StateType>();
  const auto* pos = se3state->as<ompl::base::RealVectorStateSpace::StateType>(0);
  return positionValid(pos->values[0], pos->values[1], pos->values[2]);
}

bool GeometricPlanner::positionValid(double x, double y, double z) const {
  // Margin to enforce here: the full collision margin in general, the reduced
  // start margin within the escape sphere so a parked/lifting drone sitting within
  // the margin of the mapped floor can still root the search without ever entering
  // an obstacle. The sphere centre (start_pos_) is anchored in planPath /
  // isPathValid.
  const double ex = x - start_pos_[0];
  const double ey = y - start_pos_[1];
  const double ez = z - start_pos_[2];
  const bool near_start =
      ex * ex + ey * ey + ez * ez <= kStartEscapeRadius * kStartEscapeRadius;
  const double margin = near_start ? kStartMargin : kCollisionMargin;

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
  // Unknown space (null node) is free. The motion checker samples finer (0.01 m)
  // than the voxel size, so a segment cannot tunnel through an occupied voxel
  // between samples even when margin is 0.
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
  const auto& bounds = space_->as<ompl::base::SE3StateSpace>()->getBounds();
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
  // Only the region the validity check actually enforces is meaningful here:
  // points within kStartEscapeRadius of the start are exempt (their margin drops
  // to kStartMargin so a parked/lifting drone can root the search), so a tight
  // clearance there is expected and would not block the path. Anchor the sphere
  // at the first waypoint, mirroring isPathValid, and skip sampled points inside
  // it so this reports the lowest clearance among the points that could actually
  // flag the path. Returns +infinity if every sampled point is inside the sphere.
  const std::array<double, 3> center{path.front()[0], path.front()[1], path.front()[2]};
  auto insideEscape = [&](double x, double y, double z) {
    const double dx = x - center[0], dy = y - center[1], dz = z - center[2];
    return dx * dx + dy * dy + dz * dz <= kStartEscapeRadius * kStartEscapeRadius;
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

  // Anchor the start-state exemption at the path's first waypoint so a committed
  // path that begins on the floor (the takeoff pose) does not fail this periodic
  // re-check and force a needless replan. Mirrors what planPath exempted.
  start_pos_ = {path.front()[0], path.front()[1], path.front()[2]};

  auto makeState = [this](const std::vector<double>& p) {
    ompl::base::ScopedState<ompl::base::SE3StateSpace> s(space_);
    s->setXYZ(p[0], p[1], p[2]);
    s->as<ompl::base::SO3StateSpace::StateType>(1)->setIdentity();
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

void GeometricPlanner::setCostClearance(ClearanceFn clearance) {
  // Deliberately does not touch clearance_fn_, so a host may call the two
  // setters in either order (see the header note on order-independence).
  cost_clearance_fn_ = std::move(clearance);
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
    bool include_unknown) const {
  // With no clearance function the per-state cost is a constant 1, so this is
  // exactly a path-length objective; with one it adds the proximity penalty.
  // The field sampled here is the cost field, which is the validity field
  // unless the host set a separate one (see setCostClearance). Everything that
  // scores a path — pathCost, costBreakdown, the shortcut pass — therefore
  // agrees with what the search minimised.
  return std::make_shared<ClearanceObjective>(
      si_, costClearanceFn(), clearance_weight_, clearance_threshold_,
      (include_unknown && unknownPenaltyActive()) ? unknown_fn_ : UnknownFn{},
      unknown_weight_);
}

bool GeometricPlanner::planPath(const std::vector<double>& start_vec,
                              const std::vector<double>& goal_vec,
                              std::vector<std::vector<double>>& result_path) {
  ompl::base::ScopedState<ompl::base::SE3StateSpace> start(space_);
  ompl::base::ScopedState<ompl::base::SE3StateSpace> goal(space_);

  start->setXYZ(start_vec[0], start_vec[1], start_vec[2]);
  start->as<ompl::base::SO3StateSpace::StateType>(1)->setIdentity();

  // Anchor the start-state collision exemption at this solve's start. Must
  // precede projectGoal, which validates candidates under the same exemption.
  start_pos_ = {start_vec[0], start_vec[1], start_vec[2]};

  // Move the goal to the nearest state the collision check accepts, if it does
  // not accept the requested one. OMPL drops invalid goal states inside
  // PlannerInputStates::nextGoal and the planner then has nothing to grow
  // toward, so an unprojected goal near a wall produces no path at all — not a
  // path that stops at the margin. Projecting is what turns "invalid goal" back
  // into the ordinary "goal we approach as closely as the margin allows" case.
  std::array<double, 3> planning_goal{};
  if (!projectGoal(goal_vec, planning_goal)) {
    last_goal_projection_ = std::numeric_limits<double>::infinity();
    last_goal_gap_ = std::numeric_limits<double>::infinity();
    return false;
  }
  last_planning_goal_ = planning_goal;
  last_goal_projection_ = std::sqrt(std::pow(planning_goal[0] - goal_vec[0], 2) +
                                    std::pow(planning_goal[1] - goal_vec[1], 2) +
                                    std::pow(planning_goal[2] - goal_vec[2], 2));

  goal->setXYZ(planning_goal[0], planning_goal[1], planning_goal[2]);
  goal->as<ompl::base::SO3StateSpace::StateType>(1)->setIdentity();

  auto pdef = std::make_shared<ompl::base::ProblemDefinition>(si_);
  pdef->setStartAndGoalStates(start, goal);
  pdef->setOptimizationObjective(makeObjective());

  auto planner = makePlanner();
  planner->setProblemDefinition(pdef);
  planner->setup();

  const ompl::base::PlannerStatus solved = planner->solve(planning_time_);

  // Capture the tree for debug visualisation before the planner goes out of
  // scope. Done regardless of success (the tree of a *failed* solve is just as
  // useful to look at) but only when recording is on, so a flight build never
  // pays for getPlannerData().
  if (record_tree_) {
    last_tree_ = SearchTree{};
    ompl::base::PlannerData data(si_);
    planner->getPlannerData(data);
    const unsigned int n = data.numVertices();
    last_tree_.nodes.reserve(n);
    for (unsigned int i = 0; i < n; ++i) {
      const auto* st =
          data.getVertex(i).getState()->as<ompl::base::SE3StateSpace::StateType>();
      const auto* pos = st->as<ompl::base::RealVectorStateSpace::StateType>(0);
      last_tree_.nodes.push_back({pos->values[0], pos->values[1], pos->values[2]});
    }
    for (unsigned int i = 0; i < n; ++i) {
      std::vector<unsigned int> out;
      data.getEdges(i, out);
      for (unsigned int j : out)
        last_tree_.edges.emplace_back(static_cast<int>(i), static_cast<int>(j));
    }
  }

  if (!solved) {
    last_goal_gap_ = std::numeric_limits<double>::infinity();
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
    const auto* end = path->getState(path->getStateCount() - 1)
                          ->as<ompl::base::SE3StateSpace::StateType>();
    const auto* epos = end->as<ompl::base::RealVectorStateSpace::StateType>(0);
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
    const auto* state = path->getState(i)->as<ompl::base::SE3StateSpace::StateType>();
    const auto* pos = state->as<ompl::base::RealVectorStateSpace::StateType>(0);
    result_path.push_back({pos->values[0], pos->values[1], pos->values[2]});
  }

  // In clearance mode, use a cost-aware shortcut instead of simplifyMax: it cuts
  // the planner's zig-zag but, by gating on the clearance-aware cost, keeps the
  // detours the objective actually wanted (see shortcutClearanceAware).
  if (clearanceMode()) {
    shortcutClearanceAware(result_path);
  }
  return true;
}

void GeometricPlanner::shortcutClearanceAware(
    std::vector<std::vector<double>>& waypoints) const {
  if (waypoints.size() < 3) return;

  auto makeState = [this](const std::vector<double>& p) {
    ompl::base::ScopedState<ompl::base::SE3StateSpace> s(space_);
    s->setXYZ(p[0], p[1], p[2]);
    s->as<ompl::base::SO3StateSpace::StateType>(1)->setIdentity();
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
  start_pos_ = {waypoints.front()[0], waypoints.front()[1], waypoints.front()[2]};

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
  if (path.size() < 2) return {inf, inf, inf};

  ompl::geometric::PathGeometric geo(si_);
  for (const auto& p : path) {
    ompl::base::ScopedState<ompl::base::SE3StateSpace> s(space_);
    s->setXYZ(p[0], p[1], p[2]);
    s->as<ompl::base::SO3StateSpace::StateType>(1)->setIdentity();
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
  return {length, total - length - unknown, unknown, total};
}

}  // namespace drone_core::planning
