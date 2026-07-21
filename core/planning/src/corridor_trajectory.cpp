#include "drone_core/planning/corridor_trajectory.hpp"

#include <cmath>
#include <limits>
#include <type_traits>
#include <vector>

#include <nlopt.hpp>
#include <osqp.h>

#include "drone_core/common/logging.hpp"

// The solution vector is mapped straight into Eigen doubles; a float-configured
// OSQP build would silently misread it.
static_assert(std::is_same<OSQPFloat, double>::value,
              "OSQP must be built with double precision (no OSQP_USE_FLOAT)");

namespace drone_core::planning {

namespace {

constexpr int kDegree = 7;               // degree-7 min-snap segments
constexpr int kCoeffs = kDegree + 1;     // monomial coefficients per segment

// Degree-7 snap cost block for one segment of duration t: the Hessian of
// integral over [0,t] of snap^2 in monomial coefficients. Mirrors the Q block
// MinSnapTrajectory::solveKKT assembles (same formulation, reused here as the
// QP quadratic cost).
Eigen::Matrix<double, kCoeffs, kCoeffs> snapCostBlock(double t) {
  Eigen::Matrix<double, kCoeffs, kCoeffs> Qi = Eigen::Matrix<double, kCoeffs, kCoeffs>::Zero();
  const double t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t, t6 = t5 * t, t7 = t6 * t;

  Qi(4, 4) = 576.0 * t;
  Qi(4, 5) = 1440.0 * t2;
  Qi(4, 6) = 2880.0 * t3;
  Qi(4, 7) = 5040.0 * t4;

  Qi(5, 4) = Qi(4, 5);
  Qi(5, 5) = 4800.0 * t3;
  Qi(5, 6) = 10800.0 * t4;
  Qi(5, 7) = 20160.0 * t5;

  Qi(6, 4) = Qi(4, 6);
  Qi(6, 5) = Qi(5, 6);
  Qi(6, 6) = 25920.0 * t5;
  Qi(6, 7) = 50400.0 * t6;

  Qi(7, 4) = Qi(4, 7);
  Qi(7, 5) = Qi(5, 7);
  Qi(7, 6) = Qi(6, 7);
  Qi(7, 7) = 100800.0 * t7;
  return Qi;
}

double binomial(int n, int k) {
  double r = 1.0;
  for (int j = 1; j <= k; ++j) r = r * (n - k + j) / j;
  return r;
}

// Row of the d-th derivative of the monomial polynomial evaluated at time t:
// row[k] = k!/(k-d)! * t^(k-d) for k >= d. (t = 0 gives the single entry d!.)
Eigen::RowVectorXd derivRow(int d, double t) {
  Eigen::RowVectorXd row = Eigen::RowVectorXd::Zero(kCoeffs);
  for (int k = d; k < kCoeffs; ++k) {
    double fall = 1.0;
    for (int j = 0; j < d; ++j) fall *= (k - j);
    row(k) = fall * std::pow(t, k - d);
  }
  return row;
}

// Bezier control points of the d-th derivative as linear rows in the segment's
// monomial coefficients: G = M * D, where D maps monomial coefficients to the
// derivative's monomial coefficients (c_d[k] = (k+d)!/k! * c[k+d], degree
// nd = 7-d) and M is the monomial->Bernstein change of basis over [0, T]
// (tau^k = sum_{j>=k} C(j,k)/C(nd,k) B_j^nd(tau), with the T^k scale from
// t = T*tau). Bounding these control points bounds the derivative everywhere on
// the segment, because a Bezier curve lies in the convex hull of its control
// points — this is what makes the corridor and dynamic-limit rows sufficient
// (conservatively) rather than sampled.
Eigen::MatrixXd bezierControlRows(int d, double T) {
  const int nd = kDegree - d;
  Eigen::MatrixXd D = Eigen::MatrixXd::Zero(nd + 1, kCoeffs);
  for (int k = 0; k <= nd; ++k) {
    double fall = 1.0;
    for (int j = 0; j < d; ++j) fall *= (k + d - j);
    D(k, k + d) = fall;
  }
  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(nd + 1, nd + 1);
  for (int j = 0; j <= nd; ++j) {
    for (int k = 0; k <= j; ++k) {
      M(j, k) = std::pow(T, k) * binomial(j, k) / binomial(nd, k);
    }
  }
  return M * D;
}

// Dense-to-CSC conversion for OSQP. upper_only keeps row <= col entries (the
// form OSQP requires for P). Exact zeros are dropped.
struct Csc {
  std::vector<OSQPFloat> x;
  std::vector<OSQPInt> i;
  std::vector<OSQPInt> p;
};

Csc toCsc(const Eigen::MatrixXd& A, bool upper_only) {
  Csc out;
  out.p.reserve(A.cols() + 1);
  out.p.push_back(0);
  for (int c = 0; c < A.cols(); ++c) {
    const int rmax = upper_only ? std::min<int>(c, A.rows() - 1) : A.rows() - 1;
    for (int r = 0; r <= rmax; ++r) {
      if (A(r, c) != 0.0) {
        out.x.push_back(static_cast<OSQPFloat>(A(r, c)));
        out.i.push_back(r);
      }
    }
    out.p.push_back(static_cast<OSQPInt>(out.i.size()));
  }
  return out;
}

}  // namespace

bool CorridorTrajectoryOptimizer::solveQP(const Eigen::Vector3d& start,
                                          const Eigen::Vector3d& goal,
                                          const std::vector<double>& times,
                                          const std::vector<ConvexRegion>& regions,
                                          common::Trajectory& out,
                                          double* cost_out) const {
  const int S = static_cast<int>(times.size());
  if (S < 1 || regions.size() != times.size()) return false;
  for (double t : times) {
    if (!(t > 0.0) || !std::isfinite(t)) return false;
  }

  // ONE coupled QP over all three axes. A polyhedron face row mixes x, y and
  // z, so the per-axis decomposition an axis-aligned box allowed (one
  // factorization, three solves swapping bounds) no longer exists. Variable
  // layout: segment-major, axis-minor — index(s, axis, k) = s*24 + axis*8 + k.
  const int kAxes = 3;
  const int seg_vars = kAxes * kCoeffs;  // 24
  const int n = S * seg_vars;
  const auto idx = [seg_vars](int s, int axis) { return s * seg_vars + axis * kCoeffs; };

  // The QP is solved in per-segment normalized time: tau = t/T_s with scaled
  // coefficients ct_k = c_k * T_s^k, so p(t) = sum ct_k tau^k. Raw monomials
  // over multi-second segments put ~1e9 snap-Hessian entries next to ~1
  // position rows and OSQP stalls at max_iter ("solved inaccurate"); in tau
  // every constraint row is O(1) and the solver converges quickly. The d-th
  // time-derivative picks up a 1/T^d factor and the snap integral becomes
  // ct' (Q(1)/T^7) ct; output coefficients are rescaled back (c_k = ct_k / T^k)
  // so callers still get real-time monomials.

  // Quadratic cost: the same snap Hessian, once per axis per segment,
  // block-diagonal. P = 2Q so the OSQP objective 0.5 x'Px equals c'Qc.
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(n, n);
  for (int s = 0; s < S; ++s) {
    const Eigen::MatrixXd Qs = 2.0 * snapCostBlock(1.0) / std::pow(times[s], 7);
    for (int ax = 0; ax < kAxes; ++ax) {
      P.block(idx(s, ax), idx(s, ax), kCoeffs, kCoeffs) = Qs;
    }
  }

  // Row budget:
  //   equality (l == u): boundary pos + rest vel/acc/jerk at both ends and
  //     C0..C4 continuity per junction, each replicated per axis;
  //   inequality: one row per polyhedron face per position control point
  //     (spanning all three axis blocks), plus per-axis vel/acc/jerk
  //     control-point limits.
  int faces_total = 0;
  for (const auto& r : regions) faces_total += static_cast<int>(r.A.rows());
  const int m_eq = kAxes * (8 + 5 * (S - 1));
  const int m_in = faces_total * kCoeffs + S * kAxes * (7 + 6 + 5);
  const int m = m_eq + m_in;

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(m, n);
  Eigen::VectorXd lower(m), upper(m);

  int row = 0;

  // Start boundary: p(0) = start, v/a/jerk(0) = 0 (rest — matches the existing
  // min-snap; inheriting the live velocity is a Stage-2 improvement).
  for (int d = 0; d <= 3; ++d) {
    const Eigen::RowVectorXd r0 = derivRow(d, 0.0) / std::pow(times[0], d);
    for (int ax = 0; ax < kAxes; ++ax) {
      A.block(row, idx(0, ax), 1, kCoeffs) = r0;
      lower(row) = upper(row) = d == 0 ? start(ax) : 0.0;
      ++row;
    }
  }

  // Interior junctions: derivative d of segment s at its end equals derivative
  // d of segment s+1 at its start, for d = 0..4 (C0 through snap), per axis.
  // In tau the segment end is tau = 1 and each side carries its own 1/T^d.
  for (int s = 0; s + 1 < S; ++s) {
    for (int d = 0; d <= 4; ++d) {
      const Eigen::RowVectorXd re = derivRow(d, 1.0) / std::pow(times[s], d);
      const Eigen::RowVectorXd rs = derivRow(d, 0.0) / std::pow(times[s + 1], d);
      for (int ax = 0; ax < kAxes; ++ax) {
        A.block(row, idx(s, ax), 1, kCoeffs) = re;
        A.block(row, idx(s + 1, ax), 1, kCoeffs) = -rs;
        lower(row) = upper(row) = 0.0;
        ++row;
      }
    }
  }

  // End boundary: p(T) = goal, rest. The rest end doubles as a safety stop if
  // replanning ever halts mid-flight.
  for (int d = 0; d <= 3; ++d) {
    const Eigen::RowVectorXd rT = derivRow(d, 1.0) / std::pow(times[S - 1], d);
    for (int ax = 0; ax < kAxes; ++ax) {
      A.block(row, idx(S - 1, ax), 1, kCoeffs) = rT;
      lower(row) = upper(row) = d == 0 ? goal(ax) : 0.0;
      ++row;
    }
  }

  // Corridor: the j-th position control point of segment s is
  // r_j = (G_pos.row(j) * ct_s^x, ..., ct_s^y, ..., ct_s^z) and must satisfy
  // every face A_f . r_j <= b_f — one row per (face, control point) spanning
  // the three axis blocks. The Bezier hull property lifts the control-point
  // bound to the whole curve.
  const Eigen::MatrixXd G_pos = bezierControlRows(0, 1.0);
  for (int s = 0; s < S; ++s) {
    for (int f = 0; f < regions[s].A.rows(); ++f) {
      for (int j = 0; j < kCoeffs; ++j) {
        for (int ax = 0; ax < kAxes; ++ax) {
          A.block(row, idx(s, ax), 1, kCoeffs) = regions[s].A(f, ax) * G_pos.row(j);
        }
        lower(row) = -OSQP_INFTY;
        upper(row) = regions[s].b(f);
        ++row;
      }
    }
  }

  // Dynamic limits: per-axis vel/acc/jerk control points, unchanged in meaning.
  const double lim[4] = {0.0, limits_.vmax, limits_.amax, limits_.jmax};
  for (int s = 0; s < S; ++s) {
    for (int d = 1; d <= 3; ++d) {
      const Eigen::MatrixXd G = bezierControlRows(d, 1.0) / std::pow(times[s], d);
      for (int ax = 0; ax < kAxes; ++ax) {
        A.block(row, idx(s, ax), G.rows(), kCoeffs) = G;
        for (int r = 0; r < G.rows(); ++r) {
          lower(row + r) = -lim[d];
          upper(row + r) = lim[d];
        }
        row += static_cast<int>(G.rows());
      }
    }
  }

  const Csc Pc = toCsc(P, /*upper_only=*/true);
  const Csc Ac = toCsc(A, /*upper_only=*/false);
  OSQPCscMatrix Pm{n, n, const_cast<OSQPInt*>(Pc.p.data()), const_cast<OSQPInt*>(Pc.i.data()),
                   const_cast<OSQPFloat*>(Pc.x.data()), static_cast<OSQPInt>(Pc.x.size()), -1, 0};
  OSQPCscMatrix Am{m, n, const_cast<OSQPInt*>(Ac.p.data()), const_cast<OSQPInt*>(Ac.i.data()),
                   const_cast<OSQPFloat*>(Ac.x.data()), static_cast<OSQPInt>(Ac.x.size()), -1, 0};
  const std::vector<OSQPFloat> q(n, 0.0);

  OSQPSettings settings;
  osqp_set_default_settings(&settings);
  settings.verbose = 0;
  settings.polishing = 1;    // recover a high-accuracy solution from the ADMM iterate
  settings.eps_abs = 1e-5;   // tight tolerances: corridor rows are safety constraints
  settings.eps_rel = 1e-5;
  // A well-conditioned (tau-normalized) solve converges quickly; only
  // near-infeasible probes from the outer time search grind longer. Cap them
  // well below OSQP's 4000 default so a whole BOBYQA search stays cheap — an
  // unconverged probe is treated as infeasible, which is what it borders on.
  settings.max_iter = 1000;

  std::vector<OSQPFloat> l(m), u(m);
  for (int r = 0; r < m; ++r) {
    l[r] = static_cast<OSQPFloat>(lower(r));
    u[r] = static_cast<OSQPFloat>(upper(r));
  }

  OSQPSolver* solver = nullptr;
  if (osqp_setup(&solver, &Pm, q.data(), &Am, l.data(), u.data(), m, n, &settings) != 0) {
    DRONE_LOG_ERROR("[corridor-qp] OSQP setup failed");
    return false;
  }

  bool ok = true;
  double cost = 0.0;
  Eigen::VectorXd sol;
  if (osqp_solve(solver) != 0) {
    // API-level failure (not a solve outcome) — always worth a line.
    DRONE_LOG_ERROR("[corridor-qp] OSQP solve error");
    ok = false;
  } else if (solver->info->status_val != OSQP_SOLVED) {
    // Primal infeasible, or unconverged at the iteration cap (which only
    // happens bordering infeasibility) — either way there is no trustworthy
    // trajectory. Silent: the outer time search probes this region on every
    // run and scores it as a penalty; the caller's fallback does the one-line
    // reporting if a *final* solve ends up here.
    ok = false;
  } else {
    sol = Eigen::Map<const Eigen::VectorXd>(
        reinterpret_cast<const double*>(solver->solution->x), n);
    cost = solver->info->obj_val;
  }
  osqp_cleanup(solver);
  if (!ok) return false;

  out = common::Trajectory{};
  out.segment_times = times;
  out.total_duration = 0.0;
  for (int s = 0; s < S; ++s) {
    // Undo the tau normalization: c_k = ct_k / T^k gives real-time monomials.
    Eigen::VectorXd rescale(kCoeffs);
    for (int k = 0; k < kCoeffs; ++k) rescale(k) = std::pow(times[s], -k);
    out.coeffs_x.push_back(sol.segment(idx(s, 0), kCoeffs).cwiseProduct(rescale));
    out.coeffs_y.push_back(sol.segment(idx(s, 1), kCoeffs).cwiseProduct(rescale));
    out.coeffs_z.push_back(sol.segment(idx(s, 2), kCoeffs).cwiseProduct(rescale));
    out.total_duration += times[s];
  }
  if (cost_out) *cost_out = cost;
  return true;
}

namespace {

// Context handed to the NLopt objective (mirrors TimeOptimizerContext for the
// plain min-snap optimiser).
struct CorridorTimeContext {
  const CorridorTrajectoryOptimizer* solver;
  Eigen::Vector3d start;
  Eigen::Vector3d goal;
  const std::vector<ConvexRegion>* regions;
  double time_penalty;
  double infeasible_penalty;
};

// Feasibility-aware objective: QP snap cost + time penalty for a candidate
// allocation, or a large flat penalty when the QP is infeasible at these times
// (too short for the limits / corridor), which pushes BOBYQA back toward the
// feasible region the generous seed starts in.
double corridorTimeObjective(const std::vector<double>& x, std::vector<double>& grad,
                             void* data) {
  (void)grad;
  const auto* ctx = static_cast<const CorridorTimeContext*>(data);
  double total = 0.0;
  for (double t : x) total += t;

  common::Trajectory traj;
  double cost = 0.0;
  if (!ctx->solver->solveQP(ctx->start, ctx->goal, x, *ctx->regions, traj, &cost)) {
    return ctx->infeasible_penalty;
  }
  return cost + ctx->time_penalty * total;
}

}  // namespace

bool CorridorTrajectoryOptimizer::optimizeTrajectory(
    const std::vector<Eigen::Vector3d>& waypoints, const std::vector<ConvexRegion>& regions,
    common::Trajectory& out) const {
  if (waypoints.size() < 2 || regions.size() != waypoints.size() - 1) return false;
  const int S = static_cast<int>(regions.size());

  // Velocity-consistent seed: long enough to traverse each segment at vmax with
  // some slack. BOBYQA must start on the feasible side (the infeasible region
  // scores a flat penalty, so a search started inside it is blind), and
  // "len/vmax + buffer" alone can still be infeasible once the accel/jerk ramps
  // and the conservative Bezier hull bite — so grow the whole allocation
  // geometrically until the QP accepts it before searching.
  std::vector<double> times(S);
  for (int s = 0; s < S; ++s) {
    times[s] = (waypoints[s + 1] - waypoints[s]).norm() / limits_.vmax + kSeedBuffer;
  }
  {
    common::Trajectory probe;
    int grow = 0;
    while (grow < kMaxSeedGrowth &&
           !solveQP(waypoints.front(), waypoints.back(), times, regions, probe)) {
      for (double& t : times) t *= 1.5;
      ++grow;
    }
    if (grow == kMaxSeedGrowth) return false;  // corridor unusable at any sane duration
  }

  CorridorTimeContext ctx{this, waypoints.front(), waypoints.back(), &regions,
                          kTimePenalty, kInfeasiblePenalty};

  nlopt::opt optimizer(nlopt::LN_BOBYQA, S);
  optimizer.set_min_objective(corridorTimeObjective, &ctx);
  optimizer.set_lower_bounds(std::vector<double>(S, kMinSegmentTime));
  optimizer.set_xtol_rel(1e-2);
  optimizer.set_maxeval(kMaxEvals);

  double min_cost = 0.0;
  try {
    optimizer.optimize(times, min_cost);
  } catch (const std::exception& e) {
    // BOBYQA trouble is not fatal: fall through and try the QP at whatever
    // times we have (the seed if it never improved).
    DRONE_LOG_INFO("[corridor-qp] time search failed (" << e.what()
                   << "), using current allocation");
  }

  // Final solve at the chosen allocation. The search started feasible, but
  // BOBYQA returns its lowest evaluated point, which can sit just inside the
  // infeasible boundary; retry once with a modest stretch before giving up.
  if (solveQP(waypoints.front(), waypoints.back(), times, regions, out)) return true;
  for (double& t : times) t *= 1.5;
  return solveQP(waypoints.front(), waypoints.back(), times, regions, out);
}

}  // namespace drone_core::planning
