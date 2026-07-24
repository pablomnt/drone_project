// Unit tests for the corridor pipeline: truncation against a clearance oracle,
// convex-region decomposition against synthetic obstacle clouds, and the
// corridor-constrained min-snap QP. No map, no ROS — analytic fields and
// hand-built point clouds only, mirroring the oracle idiom of test_planner.cpp.

#include "drone_core/planning/corridor.hpp"
#include "drone_core/planning/corridor_trajectory.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

using drone_core::common::Trajectory;
using drone_core::planning::ConvexRegion;
using drone_core::planning::CorridorLimits;
using drone_core::planning::CorridorParams;
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

// A MotionState at rest at `p`: the boundary condition for the first plan of a
// flight, and what every replan used before they learned to splice onto a
// moving vehicle.
drone_core::common::MotionState restAt(const Eigen::Vector3d& p) {
  drone_core::common::MotionState s;
  s.pos = p;
  return s;
}

// An axis-aligned box as a ConvexRegion (6 unit-normal half-spaces). Lets the
// QP tests state a corridor directly, independent of the decomposition.
ConvexRegion boxRegion(const Eigen::Vector3d& lo, const Eigen::Vector3d& hi) {
  ConvexRegion r;
  r.A.resize(6, 3);
  r.b.resize(6);
  for (int ax = 0; ax < 3; ++ax) {
    Eigen::Vector3d n = Eigen::Vector3d::Zero();
    n(ax) = 1.0;
    r.A.row(2 * ax) = n;             //  x_ax <=  hi
    r.b(2 * ax) = hi(ax);
    r.A.row(2 * ax + 1) = -n;        // -x_ax <= -lo
    r.b(2 * ax + 1) = -lo(ax);
  }
  return r;
}

// Full validity audit of a solved corridor trajectory: boundary conditions,
// per-segment region containment, per-axis dynamic limits, and C0-C4 junction
// continuity. Returns the number of violated checks (0 == valid).
int checkTrajectory(const Trajectory& traj, const drone_core::common::MotionState& start,
                    const Eigen::Vector3d& goal, const std::vector<ConvexRegion>& regions,
                    const CorridorLimits& limits, const char* label) {
  int failures = 0;
  const int S = static_cast<int>(traj.segment_times.size());

  // Boundary: the start matches the requested motion state to third order (a
  // replan splices onto a moving vehicle), the end is an exact position at rest.
  const double Tl = traj.segment_times.back();
  const Eigen::Vector3d p0(evalDeriv(traj.coeffs_x.front(), 0, 0.0),
                           evalDeriv(traj.coeffs_y.front(), 0, 0.0),
                           evalDeriv(traj.coeffs_z.front(), 0, 0.0));
  const Eigen::Vector3d pT(evalDeriv(traj.coeffs_x.back(), 0, Tl),
                           evalDeriv(traj.coeffs_y.back(), 0, Tl),
                           evalDeriv(traj.coeffs_z.back(), 0, Tl));
  if ((p0 - start.pos).norm() > kTol || (pT - goal).norm() > kTol) {
    std::cerr << "FAIL(" << label << "): boundary positions off (start err "
              << (p0 - start.pos).norm() << ", goal err " << (pT - goal).norm() << ")\n";
    ++failures;
  }
  const Eigen::Vector3d* want[4] = {&start.pos, &start.vel, &start.acc, &start.jerk};
  const std::vector<const Eigen::VectorXd*> first = {
      &traj.coeffs_x.front(), &traj.coeffs_y.front(), &traj.coeffs_z.front()};
  const std::vector<const Eigen::VectorXd*> last = {
      &traj.coeffs_x.back(), &traj.coeffs_y.back(), &traj.coeffs_z.back()};
  for (int d = 1; d <= 3; ++d) {
    for (int ax = 0; ax < 3; ++ax) {
      if (std::abs(evalDeriv(*first[ax], d, 0.0) - (*want[d])(ax)) > kTol) {
        std::cerr << "FAIL(" << label << "): start derivative order " << d << " axis " << ax
                  << " is " << evalDeriv(*first[ax], d, 0.0) << ", wanted " << (*want[d])(ax)
                  << "\n";
        ++failures;
      }
      if (std::abs(evalDeriv(*last[ax], d, Tl)) > kTol) {
        std::cerr << "FAIL(" << label << "): end derivative order " << d << " axis " << ax
                  << " not at rest\n";
        ++failures;
      }
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
      if (!regions[s].contains(p, kTol)) contained = false;
      const double lim[4] = {0.0, limits.vmax, limits.amax, limits.jmax};
      for (int d = 1; d <= 3; ++d) {
        for (int ax = 0; ax < 3; ++ax) {
          if (std::abs(evalDeriv(*cs[ax], d, t)) > lim[d] + kTol) limited = false;
        }
      }
    }
  }
  if (!contained) {
    std::cerr << "FAIL(" << label << "): sampled trajectory leaves its corridor region\n";
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

// Smallest distance from `p` to any obstacle point.
double nearestObstacle(const Eigen::Vector3d& p,
                       const std::vector<Eigen::Vector3d>& obstacles) {
  double lo = std::numeric_limits<double>::infinity();
  for (const auto& o : obstacles) lo = std::min(lo, (p - o).norm());
  return lo;
}

// THE safety property of the whole corridor stage: every point inside a region
// is at least `margin` from every obstacle. Grid-samples the region's
// neighbourhood, keeps the points that are inside, and measures. Returns the
// number of violations.
int checkRegionClearance(const std::vector<ConvexRegion>& regions,
                         const std::vector<Eigen::Vector3d>& obstacles,
                         const Eigen::Vector3d& lo, const Eigen::Vector3d& hi,
                         double margin, const char* label) {
  int violations = 0;
  double worst = std::numeric_limits<double>::infinity();
  for (double x = lo.x(); x <= hi.x(); x += 0.1) {
    for (double y = lo.y(); y <= hi.y(); y += 0.1) {
      for (double z = lo.z(); z <= hi.z(); z += 0.1) {
        const Eigen::Vector3d p(x, y, z);
        for (const auto& r : regions) {
          if (!r.contains(p)) continue;
          const double d = nearestObstacle(p, obstacles);
          if (d < margin - kTol) {
            worst = std::min(worst, d);
            ++violations;
          }
        }
      }
    }
  }
  if (violations > 0) {
    std::cerr << "FAIL(" << label << "): " << violations
              << " in-region point(s) closer than the margin (worst " << worst << " m vs "
              << margin << " m)\n";
    return 1;
  }
  return 0;
}

}  // namespace

int main() {
  int failures = 0;
  const CorridorLimits limits;  // vmax 1.0, amax 1.5, jmax 3.0
  const CorridorTrajectoryOptimizer opt(limits);

  // ---------------------------------------------------------------- QP -----
  // L-shaped corridor stated directly as two overlapping box regions. Same
  // geometry the axis-aligned implementation was tested on, so these assertions
  // are a regression check across the coupled-QP rewrite.
  const Eigen::Vector3d start(0.0, 0.0, 1.0);
  const Eigen::Vector3d corner(2.0, 0.0, 1.0);
  const Eigen::Vector3d goal(2.0, 2.0, 1.0);
  const std::vector<ConvexRegion> regions = {
      boxRegion({-0.3, -0.3, 0.7}, {2.3, 0.3, 1.3}),
      boxRegion({1.7, -0.3, 0.7}, {2.3, 2.3, 1.3}),
  };
  const std::vector<double> times = {4.0, 4.0};

  Trajectory traj;
  double cost = 0.0;
  if (!opt.solveQP(start, goal, times, regions, traj, &cost)) {
    std::cerr << "FAIL: feasible L-corridor QP reported infeasible\n";
    return 1;  // everything below depends on this solve
  }
  if (traj.segment_times.size() != 2 || traj.coeffs_x.size() != 2 || cost <= 0.0) {
    std::cerr << "FAIL: solution shape wrong (segments=" << traj.segment_times.size()
              << " cost=" << cost << ")\n";
    ++failures;
  }
  failures += checkTrajectory(traj, restAt(start), goal, regions, limits, "fixed-times");

  // The trajectory must not be pinned to the corner waypoint: the point of the
  // corridor formulation is that the path is free within the regions, so the
  // junction only has to sit in the overlap of both.
  {
    const double T0 = traj.segment_times[0];
    const Eigen::Vector3d pj(evalDeriv(traj.coeffs_x[0], 0, T0),
                             evalDeriv(traj.coeffs_y[0], 0, T0),
                             evalDeriv(traj.coeffs_z[0], 0, T0));
    if (!regions[0].contains(pj, kTol) || !regions[1].contains(pj, kTol)) {
      std::cerr << "FAIL: junction point outside the region overlap\n";
      ++failures;
    }
  }

  // Outer BOBYQA time search from the velocity-consistent seed.
  {
    Trajectory opt_traj;
    if (!opt.optimizeTrajectory({start, corner, goal}, regions, opt_traj)) {
      std::cerr << "FAIL: optimizeTrajectory found no feasible allocation\n";
      ++failures;
    } else {
      failures += checkTrajectory(opt_traj, restAt(start), goal, regions, limits, "optimized");
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
    if (opt.solveQP(start, goal, {0.5, 0.5}, regions, t2)) {
      std::cerr << "FAIL: too-short time allocation reported feasible\n";
      ++failures;
    }
  }

  // Infeasible by corridor: goal outside the last region conflicts with the
  // end-position equality.
  {
    Trajectory t3;
    if (opt.solveQP(start, Eigen::Vector3d(2.0, 5.0, 1.0), times, regions, t3)) {
      std::cerr << "FAIL: goal outside the corridor reported feasible\n";
      ++failures;
    }
  }

  // ------------------------------------------------ moving start (splice) ---
  // A replan does not begin at rest: it begins wherever the outgoing trajectory
  // had the vehicle at the switch instant. The QP must reproduce that state
  // exactly at t = 0 — position, velocity, acceleration and jerk — while still
  // ending at rest inside the corridor and respecting the per-axis limits.
  //
  // Note how modest the start derivatives have to be. The corridor constraint
  // bounds the Bezier CONTROL POINTS, and the low-index control points are
  // built from exactly these derivatives scaled by the segment time: with
  // b3 = 3Tv/7 + 3T^2 a/42 + T^3 j/210, a 4 s segment multiplies the start jerk
  // by ~0.3. So an aggressive splice state on a long segment can push the hull
  // out of a narrow region and make the QP infeasible even though the curve
  // itself would have stayed inside. That is the hull's conservatism, not a
  // physical limit, and it is a real operational effect: replanning while
  // manoeuvring hard in a tight corridor is when this stage is most likely to
  // refuse. Shorter segments (MAX_SEGMENT_LEN) weaken it.
  {
    drone_core::common::MotionState moving;
    moving.pos = start;
    moving.vel = Eigen::Vector3d(0.6, 0.05, 0.0);   // running down the first leg
    moving.acc = Eigen::Vector3d(-0.2, 0.1, 0.0);   // easing off, drifting toward the corner
    moving.jerk = Eigen::Vector3d(0.1, 0.0, 0.0);

    Trajectory mt;
    if (!opt.solveQP(moving, goal, times, regions, mt)) {
      std::cerr << "FAIL: moving-start QP reported infeasible on the L-corridor\n";
      ++failures;
    } else {
      failures += checkTrajectory(mt, moving, goal, regions, limits, "moving-start");
    }

    // The same through the outer time search, which has to seed an allocation
    // that can both carry the initial speed and still brake to rest.
    Trajectory mo;
    if (!opt.optimizeTrajectory(moving, {start, corner, goal}, regions, mo)) {
      std::cerr << "FAIL: moving-start optimizeTrajectory found no feasible allocation\n";
      ++failures;
    } else {
      failures += checkTrajectory(mo, moving, goal, regions, limits, "moving-start-optimized");
    }
  }

  // A start moving fast enough that it cannot stop inside the corridor must be
  // refused, not silently clipped: braking from v0 within distance d needs
  // a >= v0^2/(2d) no matter how the time is allocated, so no allocation exists.
  // This is the one genuinely new infeasibility a moving start introduces.
  {
    const std::vector<ConvexRegion> stub = {boxRegion({-0.05, -0.3, 0.7}, {0.35, 0.3, 1.3})};
    drone_core::common::MotionState fast;
    fast.pos = start;
    fast.vel = Eigen::Vector3d(1.0, 0.0, 0.0);  // 1 m/s into a 0.3 m box, amax 1.5
    Trajectory ft;
    if (opt.optimizeTrajectory(fast, {start, Eigen::Vector3d(0.3, 0.0, 1.0)}, stub, ft)) {
      std::cerr << "FAIL: start too fast to stop in the corridor reported feasible\n";
      ++failures;
    }
  }

  // ------------------------------------------------------- decomposition ---
  CorridorParams params;
  params.margin = 0.4;
  params.voxel_half_diagonal = 0.0;  // obstacles are exact points in these tests
  params.local_bbox = Eigen::Vector3d(2.0, 2.0, 1.0);

  // Build a wall of obstacle points on a plane, spaced finely enough that the
  // decomposition sees a surface rather than isolated specks.
  const auto wallPoints = [](double fixed, int axis, double lo, double hi, double zlo,
                             double zhi) {
    std::vector<Eigen::Vector3d> pts;
    for (double u = lo; u <= hi; u += 0.1) {
      for (double z = zlo; z <= zhi; z += 0.1) {
        Eigen::Vector3d p;
        p(axis) = fixed;
        p((axis + 1) % 3) = u;
        p(2) = z;
        if (axis == 2) p(2) = fixed;
        pts.push_back(p);
      }
    }
    return pts;
  };

  // THE decisive regression: a diagonal segment whose axis-aligned bounding box
  // clips obstacles the path itself is nowhere near. growBox seeded on that AABB
  // failed here; a polyhedron grown along the segment must succeed at the same
  // margin. Obstacle clusters sit at the two off-diagonal AABB corners, ~1.41 m
  // from the path — far outside the 0.4 m margin.
  {
    std::vector<Eigen::Vector3d> obs;
    for (double d = -0.3; d <= 0.3; d += 0.1) {
      for (double z = 0.7; z <= 1.3; z += 0.1) {
        obs.emplace_back(2.0 + d, 0.0, z);   // corner (2, 0)
        obs.emplace_back(0.0, 2.0 + d, z);   // corner (0, 2)
      }
    }
    const std::vector<Eigen::Vector3d> path = {{0, 0, 1}, {2, 2, 1}};
    std::vector<Eigen::Vector3d> resampled;
    std::vector<ConvexRegion> diag;
    CorridorParams dp = params;
    dp.max_segment_len = 4.0;  // keep it a single diagonal segment
    if (!drone_core::planning::buildCorridor(obs, path, dp, resampled, diag)) {
      std::cerr << "FAIL: diagonal segment (the growBox regression) produced no corridor\n";
      ++failures;
    } else {
      failures += checkRegionClearance(diag, obs, {-1, -1, 0.5}, {3, 3, 1.5}, dp.margin,
                                       "diagonal");
      // Every region must actually admit the segment it was grown around.
      for (std::size_t s = 0; s < diag.size(); ++s) {
        if (!diag[s].contains(0.5 * (resampled[s] + resampled[s + 1]))) {
          std::cerr << "FAIL: diagonal region " << s << " excludes its own segment midpoint\n";
          ++failures;
        }
      }
    }
  }

  // A corridor between two parallel walls: regions must clear both by the
  // margin, cover every segment, and overlap enough at the junctions for the QP
  // to have a feasible C0 handover.
  {
    std::vector<Eigen::Vector3d> obs = wallPoints(1.2, 0, -1.0, 4.0, 0.5, 1.5);
    const auto right = wallPoints(-1.2, 0, -1.0, 4.0, 0.5, 1.5);
    obs.insert(obs.end(), right.begin(), right.end());

    const std::vector<Eigen::Vector3d> path = {{0, 0, 1}, {0, 3, 1}};
    std::vector<Eigen::Vector3d> resampled;
    std::vector<ConvexRegion> corridor;
    if (!drone_core::planning::buildCorridor(obs, path, params, resampled, corridor)) {
      std::cerr << "FAIL: straight corridor between walls produced no regions\n";
      ++failures;
    } else {
      if (corridor.size() != resampled.size() - 1) {
        std::cerr << "FAIL: region count " << corridor.size() << " != segments "
                  << resampled.size() - 1 << "\n";
        ++failures;
      }
      failures += checkRegionClearance(corridor, obs, {-1.5, -0.5, 0.5}, {1.5, 3.5, 1.5},
                                       params.margin, "walls");

      // End-to-end: the generated corridor must actually admit a trajectory.
      Trajectory wt;
      if (!opt.optimizeTrajectory(resampled, corridor, wt)) {
        std::cerr << "FAIL: no trajectory fits the generated wall corridor\n";
        ++failures;
      } else {
        failures += checkTrajectory(wt, restAt(resampled.front()), resampled.back(), corridor, limits,
                                    "end-to-end");
      }
    }
  }

  // A gap narrower than twice the margin: the decomposition succeeds, then the
  // shrink collapses the region. The corridor must be rejected AND the attempt
  // must survive with the pre-shrink geometry intact — that pair is what makes
  // a failed tick legible in RViz (raw outlines present, shrunk ones gone
  // ⇒ the margin ate it), so it is a contract, not a convenience.
  {
    std::vector<Eigen::Vector3d> obs = wallPoints(0.25, 0, -1.0, 2.0, 0.5, 1.5);
    const auto other = wallPoints(-0.25, 0, -1.0, 2.0, 0.5, 1.5);  // 0.5 m gap, margin 0.4
    obs.insert(obs.end(), other.begin(), other.end());

    std::vector<Eigen::Vector3d> r3;
    std::vector<ConvexRegion> c3;
    std::string why;
    drone_core::planning::CorridorAttempt attempt;
    CorridorParams np = params;
    np.start_relax_dist = 0.0;  // uniform margin: this asserts the shrink itself
    const bool ok = drone_core::planning::buildCorridor(obs, {{0, 0, 1}, {0, 1.5, 1}}, np,
                                                        r3, c3, &why, &attempt);
    if (ok) {
      std::cerr << "FAIL: a 0.5 m gap was accepted at a 0.4 m margin\n";
      ++failures;
    } else if (!r3.empty() || !c3.empty()) {
      std::cerr << "FAIL: rejected corridor left its primary outputs populated\n";
      ++failures;
    } else if (attempt.raw.empty() || attempt.shrunk.empty()) {
      std::cerr << "FAIL: attempt was not preserved through a rejected corridor\n";
      ++failures;
    } else if (why.empty()) {
      std::cerr << "FAIL: rejection gave no reason\n";
      ++failures;
    } else {
      // The raw region must be drawable (it has volume); the shrunk one must
      // not be (it collapsed). That contrast is the whole diagnostic.
      const bool raw_drawable =
          !drone_core::planning::regionFaceLoops(attempt.raw.front()).empty();
      const bool shrunk_drawable =
          !drone_core::planning::regionFaceLoops(attempt.shrunk.front()).empty();
      if (!raw_drawable) {
        std::cerr << "FAIL: raw attempted region has no drawable faces\n";
        ++failures;
      }
      if (shrunk_drawable) {
        std::cerr << "FAIL: shrunk region still has volume in a 0.5 m gap at 0.4 m margin\n";
        ++failures;
      }
    }
  }

  // ------------------------------------------------- start relaxation ------
  // A drone parked closer to something than the margin. truncatePath ramps its
  // requirement to zero at the vehicle so a path can still be rooted; the
  // corridor has to meet it there or the pipeline refuses to move a drone that
  // is merely near a wall. Obstacle sits 0.25 m BEHIND the start so only the
  // first region is affected — later regions' growth windows do not reach it,
  // which is what lets the test assert the relaxation is bounded in extent.
  {
    std::vector<Eigen::Vector3d> obs;
    for (double x = -0.3; x <= 0.3; x += 0.1) {
      for (double z = 0.7; z <= 1.3; z += 0.1) obs.emplace_back(x, -0.25, z);
    }
    const std::vector<Eigen::Vector3d> path = {{0, 0, 1}, {0, 3, 1}};

    // Control: with the relaxation disabled this is exactly the failure seen on
    // the bench — the full shrink puts the first region's back face in front of
    // the drone. The pair is the point; either half alone proves nothing.
    {
      CorridorParams sp = params;
      sp.start_relax_dist = 0.0;
      std::vector<Eigen::Vector3d> r;
      std::vector<ConvexRegion> c;
      std::string why;
      if (drone_core::planning::buildCorridor(obs, path, sp, r, c, &why)) {
        std::cerr << "FAIL: uniform margin accepted a start 0.25 m from an obstacle "
                     "at a 0.4 m margin\n";
        ++failures;
      }
    }

    CorridorParams sp = params;
    sp.start_relax_dist = 1.0;
    std::vector<Eigen::Vector3d> r;
    std::vector<ConvexRegion> c;
    std::string why;
    double start_margin = -1.0;
    if (!drone_core::planning::buildCorridor(obs, path, sp, r, c, &why, nullptr,
                                             &start_margin)) {
      std::cerr << "FAIL: start relaxation did not rescue a hemmed-in start (" << why << ")\n";
      ++failures;
    } else {
      // Relaxed to what the geometry allows, and no further: the drone has
      // 0.25 m, so that is what the first region keeps — not zero, and not the
      // full margin either.
      if (start_margin > 0.25 + kTol || start_margin < 0.25 - 0.01) {
        std::cerr << "FAIL: start margin " << start_margin
                  << " m is not the 0.25 m the geometry actually allows\n";
        ++failures;
      }
      if (!c.front().contains(r.front())) {
        std::cerr << "FAIL: relaxed first region still excludes the drone\n";
        ++failures;
      }
      // Bounded in extent: the split puts the relaxed region's far end at
      // start_relax_dist, so full margin resumes there.
      if (std::abs((r[1] - r[0]).norm() - sp.start_relax_dist) > kTol) {
        std::cerr << "FAIL: first segment is " << (r[1] - r[0]).norm()
                  << " m, not the relax distance " << sp.start_relax_dist << " m\n";
        ++failures;
      }
      if (c.size() < 2) {
        std::cerr << "FAIL: no region beyond the relaxed one to carry the full margin\n";
        ++failures;
      } else {
        const std::vector<ConvexRegion> beyond(c.begin() + 1, c.end());
        failures += checkRegionClearance(beyond, obs, {-1, -0.5, 0.5}, {1, 3.5, 1.5},
                                         sp.margin, "beyond-relaxation");
      }
      // And it still has to produce a flyable trajectory, not just geometry.
      Trajectory rt;
      if (!opt.optimizeTrajectory(r, c, rt)) {
        std::cerr << "FAIL: no trajectory fits the relaxed corridor\n";
        ++failures;
      } else {
        failures += checkTrajectory(rt, restAt(r.front()), r.back(), c, limits, "relaxed-start");
      }
    }
  }

  // The relaxation has a floor that is geometry, not taste: below half a voxel
  // the region would contain points inside an occupied cell's actual volume, so
  // a drone that close is a hard failure rather than a smaller margin.
  {
    CorridorParams sp = params;
    sp.start_relax_dist = 1.0;
    sp.voxel_half_diagonal = 0.05;
    std::vector<Eigen::Vector3d> obs;
    for (double x = -0.3; x <= 0.3; x += 0.1) {
      for (double z = 0.7; z <= 1.3; z += 0.1) obs.emplace_back(x, -0.03, z);
    }
    std::vector<Eigen::Vector3d> r;
    std::vector<ConvexRegion> c;
    std::string why;
    if (drone_core::planning::buildCorridor(obs, {{0, 0, 1}, {0, 3, 1}}, sp, r, c, &why) ||
        !r.empty() || !c.empty()) {
      std::cerr << "FAIL: a start inside a voxel was relaxed into instead of rejected\n";
      ++failures;
    }
  }

  // A path driven straight into a wall must fail cleanly (both outputs cleared)
  // so the caller falls back rather than flying a half-built corridor.
  {
    const auto obs = wallPoints(1.0, 1, -1.0, 1.0, 0.5, 1.5);  // wall at y = 1
    std::vector<Eigen::Vector3d> r2;
    std::vector<ConvexRegion> c2;
    if (drone_core::planning::buildCorridor(obs, {{0, 0, 1}, {0, 2, 1}}, params, r2, c2) ||
        !r2.empty() || !c2.empty()) {
      std::cerr << "FAIL: corridor through a wall did not fail cleanly\n";
      ++failures;
    }
  }

  // ---------------------------------------------------------- truncation ---
  // Unchanged: still driven by the clearance oracle, not the point cloud.
  {
    const drone_core::planning::CorridorClearanceFn conservative = [](double x, double y,
                                                                      double) {
      return std::min(std::max(0.0, 3.0 - x), std::max(0.0, 2.0 - y));
    };
    const double frontier_margin = 0.75;

    // A path heading into the frontier is cut where clearance meets the ramped
    // requirement, never committing into unknown space.
    {
      const std::vector<Eigen::Vector3d> path = {{0, 0, 1}, {0, 3, 1}};
      const auto t = drone_core::planning::truncatePath(conservative, path, frontier_margin);
      if (t.size() < 2 || std::abs(t.back().y() - 1.25) > 0.06 || t.back().x() != 0.0) {
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

    // The ramp distance is decoupled from the margin for a reason: ramping over
    // the margin makes the requirement climb at 1 m/m and it meets the shrinking
    // clearance almost immediately; ramping over 1 m commits measurably further.
    {
      const std::vector<Eigen::Vector3d> path = {{0, 0.8, 1}, {0, 3, 1}};
      const auto harsh = drone_core::planning::truncatePath(conservative, path,
                                                            frontier_margin, frontier_margin);
      const auto gentle =
          drone_core::planning::truncatePath(conservative, path, frontier_margin, 1.0);
      if (harsh.size() < 2 || gentle.size() < 2) {
        std::cerr << "FAIL: ramp comparison truncated to nothing\n";
        ++failures;
      } else if (gentle.back().y() <= harsh.back().y() + kTol) {
        std::cerr << "FAIL: 1 m ramp did not commit further than a margin-length ramp ("
                  << gentle.back().y() << " vs " << harsh.back().y() << ")\n";
        ++failures;
      } else if (conservative(gentle.back().x(), gentle.back().y(), 1.0) <= 0.0) {
        std::cerr << "FAIL: gentle ramp committed into occupied/unknown space\n";
        ++failures;
      }
    }

    // A start already hugging the frontier may still root a committed prefix.
    {
      const std::vector<Eigen::Vector3d> path = {{0, 1.9, 1}, {0, 0, 1}};
      const auto t = drone_core::planning::truncatePath(conservative, path, frontier_margin);
      if (t.size() < 2 || (t.back() - path.back()).norm() > kTol) {
        std::cerr << "FAIL: escape-ramp start could not root a path\n";
        ++failures;
      }
    }
  }

  // A start below the collision margin above a mapped floor roots via the ramp;
  // the climb-out survives whole.
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
