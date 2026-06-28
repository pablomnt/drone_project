#include "drone_core/planning/rrt_star_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>

#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/geometric/PathGeometric.h>

namespace drone_core::planning {

namespace {

// Single-integral cost that rewards short, high-clearance paths. The per-state
// cost is 1 (so the integral over the path equals its length when there is no
// clearance term) plus an obstacle-proximity penalty that grows as a state
// approaches an obstacle and saturates at the clearance threshold. Integrating
// over arc length makes the total comparable between any two paths regardless of
// how many waypoints each has.
class ClearanceObjective : public ompl::base::StateCostIntegralObjective {
public:
  ClearanceObjective(const ompl::base::SpaceInformationPtr& si,
                     RrtStarPlanner::ClearanceFn clearance, double weight, double threshold)
      : ompl::base::StateCostIntegralObjective(si, /*enableMotionCostInterpolation=*/true),
        clearance_(std::move(clearance)), weight_(weight), threshold_(threshold) {}

  ompl::base::Cost stateCost(const ompl::base::State* state) const override {
    double penalty = 0.0;
    if (clearance_) {
      const auto* se3 = state->as<ompl::base::SE3StateSpace::StateType>();
      const auto* pos = se3->as<ompl::base::RealVectorStateSpace::StateType>(0);
      const double d = clearance_(pos->values[0], pos->values[1], pos->values[2]);
      penalty = weight_ * std::max(0.0, threshold_ - d);
    }
    return ompl::base::Cost(1.0 + penalty);
  }

private:
  RrtStarPlanner::ClearanceFn clearance_;
  double weight_;
  double threshold_;
};

}  // namespace

RrtStarPlanner::RrtStarPlanner(const MapHandle& octree, double planning_time)
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

bool RrtStarPlanner::isStateValid(const ompl::base::State* state) {
  const auto* se3state = state->as<ompl::base::SE3StateSpace::StateType>();
  const auto* pos = se3state->as<ompl::base::RealVectorStateSpace::StateType>(0);

  // Margin to enforce here: the full collision margin in general, the reduced
  // start margin within the escape sphere so a parked/lifting drone sitting within
  // the margin of the mapped floor can still root the search without ever entering
  // an obstacle. The sphere centre (start_pos_) is anchored in planPath /
  // isPathValid.
  const double ex = pos->values[0] - start_pos_[0];
  const double ey = pos->values[1] - start_pos_[1];
  const double ez = pos->values[2] - start_pos_[2];
  const bool near_start =
      ex * ex + ey * ey + ez * ez <= kStartEscapeRadius * kStartEscapeRadius;
  const double margin = near_start ? kStartMargin : kCollisionMargin;

  // Preferred path: a single O(1) clearance lookup when a clearance field is set
  // (the live planner always sets one). The field is the 3D Euclidean distance to
  // the nearest obstacle, so this enforces vertical clearance too — unlike the
  // horizontal-only fallback below — and is far cheaper than the cell scan.
  // Outside the field / unknown space reads as free (it returns the saturation
  // distance), matching the treat-unknown-as-free policy.
  if (clearance_fn_) {
    return clearance_fn_(pos->values[0], pos->values[1], pos->values[2]) > margin;
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
      const octomap::point3d query(pos->values[0] + dx, pos->values[1] + dy,
                                   pos->values[2]);
      const auto* node = octree_ptr_->search(query);
      if (node != nullptr && octree_ptr_->isNodeOccupied(node)) {
        return false;
      }
    }
  }
  return true;
}

double RrtStarPlanner::minClearance(const std::vector<std::vector<double>>& path) const {
  if (!clearance_fn_ || path.size() < 2) return std::numeric_limits<double>::infinity();
  double mind = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const auto& a = path[i];
    const auto& b = path[i + 1];
    const double seg = std::hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]);
    const int steps = std::max(1, static_cast<int>(std::ceil(seg / 0.05)));
    for (int s = 0; s <= steps; ++s) {
      const double u = static_cast<double>(s) / steps;
      mind = std::min(mind, clearance_fn_(a[0] + u * (b[0] - a[0]),
                                          a[1] + u * (b[1] - a[1]),
                                          a[2] + u * (b[2] - a[2])));
    }
  }
  return mind;
}

bool RrtStarPlanner::isPathValid(const std::vector<std::vector<double>>& path) const {
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

void RrtStarPlanner::setClearance(ClearanceFn clearance, double weight, double threshold) {
  clearance_fn_ = std::move(clearance);
  clearance_weight_ = weight;
  clearance_threshold_ = threshold;
}

ompl::base::OptimizationObjectivePtr RrtStarPlanner::makeObjective() const {
  // With no clearance function the per-state cost is a constant 1, so this is
  // exactly a path-length objective; with one it adds the proximity penalty.
  return std::make_shared<ClearanceObjective>(si_, clearance_fn_, clearance_weight_,
                                              clearance_threshold_);
}

bool RrtStarPlanner::planPath(const std::vector<double>& start_vec,
                              const std::vector<double>& goal_vec,
                              std::vector<std::vector<double>>& result_path) {
  ompl::base::ScopedState<ompl::base::SE3StateSpace> start(space_);
  ompl::base::ScopedState<ompl::base::SE3StateSpace> goal(space_);

  start->setXYZ(start_vec[0], start_vec[1], start_vec[2]);
  start->as<ompl::base::SO3StateSpace::StateType>(1)->setIdentity();

  // Anchor the start-state collision exemption at this solve's start.
  start_pos_ = {start_vec[0], start_vec[1], start_vec[2]};

  goal->setXYZ(goal_vec[0], goal_vec[1], goal_vec[2]);
  goal->as<ompl::base::SO3StateSpace::StateType>(1)->setIdentity();

  auto pdef = std::make_shared<ompl::base::ProblemDefinition>(si_);
  pdef->setStartAndGoalStates(start, goal);
  pdef->setOptimizationObjective(makeObjective());

  auto planner = std::make_shared<ompl::geometric::RRTstar>(si_);
  planner->setProblemDefinition(pdef);
  planner->setup();

  const ompl::base::PlannerStatus solved = planner->ompl::base::Planner::solve(planning_time_);
  if (!solved) {
    return false;
  }

  // OMPL reports both exact and approximate solutions as "solved". An approximate
  // solution stops short of the goal (the tree never connected to it). Accept one
  // only if its endpoint is within kGoalFlexibility of the goal; otherwise reject
  // the plan — the caller then treats this as no path (infinite cost) rather than
  // committing the vehicle to a route that ends partway. getSolutionDifference()
  // is the distance from the returned path's end to the goal.
  if (pdef->hasApproximateSolution() && pdef->getSolutionDifference() > kGoalFlexibility) {
    return false;
  }

  auto path = std::static_pointer_cast<ompl::geometric::PathGeometric>(pdef->getSolutionPath());

  // Straighten the jagged RRT* result. Deliberately skip B-spline smoothing so
  // the original waypoints survive for the minimum-snap optimiser downstream.
  // Skip this entirely when clearance-aware: simplifyMax shortcuts purely to cut
  // length and collision-checks only, so it would straighten the path back toward
  // the obstacles the clearance objective deliberately routed around.
  if (!clearance_fn_) {
    ompl::geometric::PathSimplifier simplifier(si_);
    simplifier.simplifyMax(*path);
  }

  for (std::size_t i = 0; i < path->getStateCount(); ++i) {
    const auto* state = path->getState(i)->as<ompl::base::SE3StateSpace::StateType>();
    const auto* pos = state->as<ompl::base::RealVectorStateSpace::StateType>(0);
    result_path.push_back({pos->values[0], pos->values[1], pos->values[2]});
  }
  return true;
}

double RrtStarPlanner::pathCost(const std::vector<std::vector<double>>& path) const {
  if (path.size() < 2) return std::numeric_limits<double>::infinity();

  ompl::geometric::PathGeometric geo(si_);
  for (const auto& p : path) {
    ompl::base::ScopedState<ompl::base::SE3StateSpace> s(space_);
    s->setXYZ(p[0], p[1], p[2]);
    s->as<ompl::base::SO3StateSpace::StateType>(1)->setIdentity();
    geo.append(s.get());
  }
  return geo.cost(makeObjective()).value();
}

}  // namespace drone_core::planning
