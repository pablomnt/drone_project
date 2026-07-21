// Unit tests for the corridor-constrained min-snap QP (CorridorTrajectoryOptimizer).
// Analytic corridors only — no map, no ROS — mirroring the oracle idiom of
// test_planner.cpp: hand-built boxes stand in for the EDT-grown corridor, so the
// solver's guarantees (containment, limits, continuity) are checked directly.

#include "drone_core/planning/corridor_trajectory.hpp"

#include <cmath>
#include <iostream>
#include <vector>

using drone_core::common::Trajectory;
using drone_core::planning::Box;
using drone_core::planning::CorridorLimits;
using drone_core::planning::CorridorTrajectoryOptimizer;

namespace {

constexpr double kTol = 1e-3;

// d-th derivative of a degree-7 monomial segment at local time t.
double evalDeriv(const Eigen::VectorXd& c, int d, double t) {
  double v = 0.0;
  for (int k = d; k < c.size(); ++k) {
    double fall = 1.0;
    for (int j = 0; j < d; ++j) fall *= (k - j);
    v += c(k) * fall * std::pow(t, k - d);
  }
  return v;
}

// Full validity audit of a solved corridor trajectory: boundary conditions,
// per-segment box containment, per-axis dynamic limits, and C0-C4 junction
// continuity. Returns the number of violated checks (0 == valid).
int checkTrajectory(const Trajectory& traj, const Eigen::Vector3d& start,
                    const Eigen::Vector3d& goal, const std::vector<Box>& boxes,
                    const CorridorLimits& limits, const char* label) {
  int failures = 0;
  const int S = static_cast<int>(traj.segment_times.size());

  // Boundary: exact position ends, rest (v = a = j = 0) both ends.
  const double Tl = traj.segment_times.back();
  const Eigen::Vector3d p0(evalDeriv(traj.coeffs_x.front(), 0, 0.0),
                           evalDeriv(traj.coeffs_y.front(), 0, 0.0),
                           evalDeriv(traj.coeffs_z.front(), 0, 0.0));
  const Eigen::Vector3d pT(evalDeriv(traj.coeffs_x.back(), 0, Tl),
                           evalDeriv(traj.coeffs_y.back(), 0, Tl),
                           evalDeriv(traj.coeffs_z.back(), 0, Tl));
  if ((p0 - start).norm() > kTol || (pT - goal).norm() > kTol) {
    std::cerr << "FAIL(" << label << "): boundary positions off (start err "
              << (p0 - start).norm() << ", goal err " << (pT - goal).norm() << ")\n";
    ++failures;
  }
  for (int d = 1; d <= 3; ++d) {
    if (std::abs(evalDeriv(traj.coeffs_x.front(), d, 0.0)) > kTol ||
        std::abs(evalDeriv(traj.coeffs_x.back(), d, Tl)) > kTol) {
      std::cerr << "FAIL(" << label << "): boundary derivative order " << d
                << " not at rest\n";
      ++failures;
    }
  }

  // Containment + dynamic limits, sampled finely per segment.
  bool contained = true, limited = true;
  for (int s = 0; s < S; ++s) {
    const double T = traj.segment_times[s];
    const std::vector<const Eigen::VectorXd*> cs = {&traj.coeffs_x[s], &traj.coeffs_y[s],
                                                    &traj.coeffs_z[s]};
    for (int i = 0; i <= 400; ++i) {
      const double t = T * i / 400.0;
      Eigen::Vector3d p;
      for (int ax = 0; ax < 3; ++ax) p(ax) = evalDeriv(*cs[ax], 0, t);
      if (!boxes[s].contains(p, kTol)) contained = false;
      const double lim[4] = {0.0, limits.vmax, limits.amax, limits.jmax};
      for (int d = 1; d <= 3; ++d) {
        for (int ax = 0; ax < 3; ++ax) {
          if (std::abs(evalDeriv(*cs[ax], d, t)) > lim[d] + kTol) limited = false;
        }
      }
    }
  }
  if (!contained) {
    std::cerr << "FAIL(" << label << "): sampled trajectory leaves its corridor box\n";
    ++failures;
  }
  if (!limited) {
    std::cerr << "FAIL(" << label << "): sampled trajectory exceeds a per-axis limit\n";
    ++failures;
  }

  // C0-C4 continuity at each junction.
  for (int s = 0; s + 1 < S; ++s) {
    const double T = traj.segment_times[s];
    const std::vector<const Eigen::VectorXd*> a = {&traj.coeffs_x[s], &traj.coeffs_y[s],
                                                   &traj.coeffs_z[s]};
    const std::vector<const Eigen::VectorXd*> b = {&traj.coeffs_x[s + 1],
                                                   &traj.coeffs_y[s + 1],
                                                   &traj.coeffs_z[s + 1]};
    for (int d = 0; d <= 4; ++d) {
      for (int ax = 0; ax < 3; ++ax) {
        const double diff = std::abs(evalDeriv(*a[ax], d, T) - evalDeriv(*b[ax], d, 0.0));
        if (diff > (d < 3 ? kTol : 1e-2)) {
          std::cerr << "FAIL(" << label << "): junction " << s << " discontinuity at order "
                    << d << " axis " << ax << " (diff " << diff << ")\n";
          ++failures;
        }
      }
    }
  }

  return failures;
}

}  // namespace

int main() {
  int failures = 0;
  const CorridorLimits limits;  // vmax 1.0, amax 1.5, jmax 3.0
  const CorridorTrajectoryOptimizer opt(limits);

  // L-shaped corridor: +x then +y, one box per segment, overlapping around the
  // corner waypoint (2, 0, 1).
  const Eigen::Vector3d start(0.0, 0.0, 1.0);
  const Eigen::Vector3d corner(2.0, 0.0, 1.0);
  const Eigen::Vector3d goal(2.0, 2.0, 1.0);
  const std::vector<Box> boxes = {
      {Eigen::Vector3d(-0.3, -0.3, 0.7), Eigen::Vector3d(2.3, 0.3, 1.3)},
      {Eigen::Vector3d(1.7, -0.3, 0.7), Eigen::Vector3d(2.3, 2.3, 1.3)},
  };
  const std::vector<double> times = {4.0, 4.0};

  // Inner solve at a fixed, generous time allocation.
  Trajectory traj;
  double cost = 0.0;
  if (!opt.solveQP(start, goal, times, boxes, traj, &cost)) {
    std::cerr << "FAIL: feasible L-corridor QP reported infeasible\n";
    return 1;  // everything below depends on this solve
  }
  if (traj.segment_times.size() != 2 || traj.coeffs_x.size() != 2 || cost <= 0.0) {
    std::cerr << "FAIL: solution shape wrong (segments=" << traj.segment_times.size()
              << " cost=" << cost << ")\n";
    ++failures;
  }
  failures += checkTrajectory(traj, start, goal, boxes, limits, "fixed-times");

  // The trajectory must not be pinned to the corner waypoint: the whole point of
  // the corridor formulation is that the path is free within the boxes, so the
  // junction position just has to sit in the overlap of both boxes.
  {
    const double T0 = traj.segment_times[0];
    const Eigen::Vector3d pj(evalDeriv(traj.coeffs_x[0], 0, T0),
                             evalDeriv(traj.coeffs_y[0], 0, T0),
                             evalDeriv(traj.coeffs_z[0], 0, T0));
    if (!boxes[0].contains(pj, kTol) || !boxes[1].contains(pj, kTol)) {
      std::cerr << "FAIL: junction point outside the box overlap\n";
      ++failures;
    }
  }

  // Outer loop: BOBYQA time search from the velocity-consistent seed. The
  // result must be feasible (all the same guarantees hold at the found times)
  // and its duration should beat the deliberately generous fixed allocation
  // above while staying physically sane (2 m per leg at vmax = 1 m/s,
  // rest-to-rest, can't be quicker than the straight-line time).
  {
    Trajectory opt_traj;
    if (!opt.optimizeTrajectory({start, corner, goal}, boxes, opt_traj)) {
      std::cerr << "FAIL: optimizeTrajectory found no feasible allocation\n";
      ++failures;
    } else {
      failures += checkTrajectory(opt_traj, start, goal, boxes, limits, "optimized");
      if (opt_traj.total_duration > times[0] + times[1] + kTol) {
        std::cerr << "FAIL: optimized duration " << opt_traj.total_duration
                  << "s worse than the generous seed (8s)\n";
        ++failures;
      }
      if (opt_traj.total_duration < 2.0) {
        std::cerr << "FAIL: optimized duration " << opt_traj.total_duration
                  << "s is physically impossible at vmax\n";
        ++failures;
      }
    }
  }

  // Infeasible by time: 2 m per segment at vmax = 1 m/s cannot fit in 0.5 s.
  {
    Trajectory t2;
    if (opt.solveQP(start, goal, {0.5, 0.5}, boxes, t2)) {
      std::cerr << "FAIL: too-short time allocation reported feasible\n";
      ++failures;
    }
  }

  // Infeasible by corridor: goal outside the last box conflicts with the
  // end-position equality.
  {
    Trajectory t3;
    if (opt.solveQP(start, Eigen::Vector3d(2.0, 5.0, 1.0), times, boxes, t3)) {
      std::cerr << "FAIL: goal outside the corridor reported feasible\n";
      ++failures;
    }
  }

  // --- Corridor generation against an analytic clearance oracle (a solid wall
  // occupying the half-space x >= 3, so clearance(p) = max(0, 3 - x)). Same
  // oracle idiom the live system uses with its conservative EDT.
  const drone_core::planning::CorridorClearanceFn wall = [](double x, double, double) {
    return std::max(0.0, 3.0 - x);
  };
  const drone_core::planning::CorridorParams params;  // 2.0 m cap, 0.1 step, 0.5 margin, 3.0 growth

  // resamplePath: a 5 m segment under a 2 m cap splits into 3 equal pieces,
  // preserving the exact endpoints.
  {
    const std::vector<Eigen::Vector3d> path = {{0, 0, 0}, {5, 0, 0}};
    const auto r = drone_core::planning::resamplePath(path, params.max_segment_len);
    bool ok = r.size() == 4 && r.front() == path.front() && r.back() == path.back();
    for (size_t i = 0; ok && i + 1 < r.size(); ++i) {
      if ((r[i + 1] - r[i]).norm() > params.max_segment_len + kTol) ok = false;
    }
    if (!ok) {
      std::cerr << "FAIL: resamplePath split wrong (" << r.size() << " points)\n";
      ++failures;
    }
  }

  // growBox: a segment parallel to the wall. The +x face must stop where the
  // margin meets the wall (x = 3 - margin = 2.5); unobstructed faces grow to the
  // cap. The box must contain both endpoints.
  {
    const Eigen::Vector3d a(0, 0, 1), b(1, 0, 1);
    Box box;
    if (!drone_core::planning::growBox(wall, a, b, params, box)) {
      std::cerr << "FAIL: growBox failed in free space\n";
      ++failures;
    } else {
      const bool xface_ok = box.hi.x() <= 2.5 + kTol && box.hi.x() >= 2.5 - params.step - kTol;
      const bool caps_ok = std::abs(box.lo.x() - (0.0 - params.max_box_growth)) < kTol &&
                           std::abs(box.hi.y() - params.max_box_growth) < kTol &&
                           std::abs(box.lo.z() - (1.0 - params.max_box_growth)) < kTol;
      if (!xface_ok || !caps_ok || !box.contains(a) || !box.contains(b)) {
        std::cerr << "FAIL: growBox geometry wrong (hi.x=" << box.hi.x()
                  << " lo=" << box.lo.transpose() << " hi=" << box.hi.transpose() << ")\n";
        ++failures;
      }
    }
  }

  // growBox: a segment whose own AABB clips the wall cannot seed a box.
  {
    Box box;
    if (drone_core::planning::growBox(wall, {2, 0, 1}, {4, 0, 1}, params, box)) {
      std::cerr << "FAIL: growBox grew a box through the wall\n";
      ++failures;
    }
  }

  // The corridor margin is what decides whether a tight passage is usable at
  // all, which is why it is a live parameter (CORRIDOR_MARGIN) rather than a
  // constant. A segment at x=2.6 has 0.4 m to the wall: unusable at the default
  // 0.5 m margin, fine at 0.3 m. Box growth is a strictly harder test than the
  // planner's centreline check, so this knob usually has to sit below the
  // planner's collision margin for a corridor to exist in tight indoor space.
  {
    const Eigen::Vector3d a(2.6, 0, 1), b(2.6, 1, 1);
    drone_core::planning::CorridorParams tight = params;
    Box box;
    tight.margin = 0.5;
    if (drone_core::planning::growBox(wall, a, b, tight, box)) {
      std::cerr << "FAIL: growBox succeeded at 0.5 m margin with only 0.4 m of room\n";
      ++failures;
    }
    tight.margin = 0.3;
    if (!drone_core::planning::growBox(wall, a, b, tight, box)) {
      std::cerr << "FAIL: growBox failed at 0.3 m margin with 0.4 m of room\n";
      ++failures;
    } else if (box.hi.x() > 3.0 - 0.3 + kTol) {
      std::cerr << "FAIL: relaxed-margin box breaches its own margin (hi.x=" << box.hi.x()
                << ", wall at 3.0)\n";
      ++failures;
    }
  }

  // buildCorridor: resample + one overlapping box per segment, all respecting
  // the wall margin; and a path into the wall must fail cleanly.
  {
    std::vector<Eigen::Vector3d> resampled;
    std::vector<Box> cboxes;
    const std::vector<Eigen::Vector3d> path = {{0, 0, 1}, {2.4, 0, 1}};
    if (!drone_core::planning::buildCorridor(wall, path, params, resampled, cboxes)) {
      std::cerr << "FAIL: buildCorridor failed along the wall\n";
      ++failures;
    } else {
      bool ok = cboxes.size() == resampled.size() - 1 && cboxes.size() >= 2;
      for (size_t s = 0; ok && s < cboxes.size(); ++s) {
        if (!cboxes[s].contains(resampled[s], kTol) ||
            !cboxes[s].contains(resampled[s + 1], kTol))
          ok = false;                                  // must contain its endpoints
        if (cboxes[s].hi.x() > 2.5 + kTol) ok = false; // must respect the wall margin
        if (s > 0 && !cboxes[s - 1].contains(resampled[s], kTol))
          ok = false;                                  // junction point in both boxes
      }
      if (!ok) {
        std::cerr << "FAIL: buildCorridor boxes malformed\n";
        ++failures;
      }
    }

    std::vector<Eigen::Vector3d> r2;
    std::vector<Box> b2;
    if (drone_core::planning::buildCorridor(wall, {{2, 0, 1}, {4, 0, 1}}, params, r2, b2) ||
        !r2.empty() || !b2.empty()) {
      std::cerr << "FAIL: buildCorridor through the wall did not fail cleanly\n";
      ++failures;
    }
  }

  // End-to-end: oracle -> corridor -> time-optimised QP. The generated corridor
  // must let the full solver produce a valid trajectory.
  {
    std::vector<Eigen::Vector3d> resampled;
    std::vector<Box> cboxes;
    const std::vector<Eigen::Vector3d> path = {{0, 0, 1}, {2, 0, 1}, {2, 2, 1}};
    if (!drone_core::planning::buildCorridor(wall, path, params, resampled, cboxes)) {
      std::cerr << "FAIL: buildCorridor failed on the L path\n";
      ++failures;
    } else {
      Trajectory t4;
      if (!opt.optimizeTrajectory(resampled, cboxes, t4)) {
        std::cerr << "FAIL: end-to-end corridor trajectory infeasible\n";
        ++failures;
      } else {
        failures += checkTrajectory(t4, resampled.front(), resampled.back(), cboxes, limits,
                                    "end-to-end");
      }
    }
  }

  // --- Truncation against a conservative oracle (frontier stamped occupied).
  // Frontier fills y >= 2, wall fills x >= 3: conservative clearance is the min
  // of the two distances, exactly what a frontier-stamped EDT reports.
  {
    const drone_core::planning::CorridorClearanceFn conservative = [](double x, double y,
                                                                      double) {
      return std::min(std::max(0.0, 3.0 - x), std::max(0.0, 2.0 - y));
    };
    const double frontier_margin = 0.75;

    // A path heading into the frontier is cut where clearance meets the margin
    // (y = 2 - 0.75 = 1.25), never committing into unknown space.
    {
      const std::vector<Eigen::Vector3d> path = {{0, 0, 1}, {0, 3, 1}};
      const auto t = drone_core::planning::truncatePath(conservative, path, frontier_margin);
      if (t.size() < 2 || std::abs(t.back().y() - 1.25) > 0.06 ||
          t.back().x() != 0.0 || conservative(t.back().x(), t.back().y(), 1.0) <
                                     frontier_margin - kTol) {
        std::cerr << "FAIL: frontier truncation cut wrong (end y="
                  << (t.size() >= 2 ? t.back().y() : -1.0) << ")\n";
        ++failures;
      }
    }

    // A path that stays in safe known space passes through untouched.
    {
      const std::vector<Eigen::Vector3d> path = {{0, 0, 1}, {1, 0, 1}, {1, 1, 1}};
      const auto t = drone_core::planning::truncatePath(conservative, path, frontier_margin);
      if (t.size() != 3 || (t.back() - path.back()).norm() > kTol) {
        std::cerr << "FAIL: safe path was truncated (size " << t.size() << ")\n";
        ++failures;
      }
    }

    // A start already hugging the frontier (inside the escape sphere) may still
    // root a short committed prefix instead of truncating to nothing.
    {
      const std::vector<Eigen::Vector3d> path = {{0, 1.9, 1}, {0, 0, 1}};
      const auto t = drone_core::planning::truncatePath(conservative, path, frontier_margin);
      if (t.size() < 2 || (t.back() - path.back()).norm() > kTol) {
        std::cerr << "FAIL: escape-sphere start could not root a path\n";
        ++failures;
      }
    }
  }

  // A start below the collision margin above a mapped floor (clearance = z)
  // roots via the escape sphere; the climb-out survives whole.
  {
    const drone_core::planning::CorridorClearanceFn floor = [](double, double, double z) {
      return std::max(0.0, z);
    };
    const std::vector<Eigen::Vector3d> path = {{0, 0, 0.3}, {0, 0, 1.5}};
    const auto t = drone_core::planning::truncatePath(floor, path, 0.5);
    if (t.size() < 2 || (t.back() - path.back()).norm() > kTol) {
      std::cerr << "FAIL: floor-parked start truncated its climb-out\n";
      ++failures;
    }
  }

  if (failures == 0) {
    std::cout << "test_corridor: all checks passed\n";
    return 0;
  }
  std::cerr << "test_corridor: " << failures << " check(s) failed\n";
  return 1;
}
