#include "drone_core/planning/corridor_trajectory.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

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

bool CorridorTrajectoryOptimizer::solveQP(const common::MotionState& start,
                                          const Eigen::Vector3d& goal,
                                          const std::vector<double>& times,
                                          const std::vector<ConvexRegion>& regions,
                                          common::Trajectory& out,
                                          double* cost_out,
                                          const std::vector<Eigen::Vector3d>* pin_waypoints,
                                          const std::vector<Eigen::Vector3d>* path_waypoints,
                                          double* path_cost_out, std::string* status_out) const {
  const int S = static_cast<int>(times.size());
  if (S < 1 || regions.size() != times.size()) return false;
  // Pinning needs one waypoint per segment boundary. A mismatched list is a
  // caller bug rather than an infeasible problem, so refuse rather than
  // silently solving the unpinned problem and returning a shape nobody asked
  // for — that failure would look exactly like the bug pinning exists to fix.
  if (pin_waypoints && static_cast<int>(pin_waypoints->size()) != S + 1) return false;
  const bool pin = pin_waypoints != nullptr;
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

  // Bezier position control points as a linear map of the scaled coefficients:
  // control point j of a segment is G_pos.row(j) . ct. Hoisted here because both
  // the corridor face rows below and the path term just under it need it.
  const Eigen::MatrixXd G_pos = bezierControlRows(0, 1.0);

  // Linear term. Zero for pure minimum-snap; the path term below is the only
  // thing that ever writes it, since every other cost here is a pure quadratic
  // form in the coefficients.
  Eigen::VectorXd q_vec = Eigen::VectorXd::Zero(n);

  // Path-following term (see setPathWeight). Pull each segment's position
  // control points toward the straight chord between its two waypoints:
  //
  //   lambda * sum_j || G_pos.row(j) . ct  -  chord(j) ||^2
  //
  // Expanded into OSQP's 0.5 x'Px + q'x that is P += 2*lambda*G'G and
  // q += -2*lambda*G'chord, per segment per axis. The dropped constant
  // lambda*||chord||^2 does not move the minimiser; the reported cost below is
  // recomputed from the solution rather than read off obj_val, so it is not
  // missing from the numbers either.
  //
  // Divided by kCoeffs so the weight means "per unit of MEAN squared deviation"
  // and does not silently change meaning if the polynomial degree ever does, and
  // weighted by each segment's CHORD LENGTH so the sum approximates the integral
  // of squared deviation over path length. Without that length factor the term
  // is extensive in the number of segments rather than in distance: every
  // segment contributes eight control points whether it spans 2 m or 15 cm, so
  // splitting a segment in two would double its share of the penalty. That is
  // not hypothetical here — MAX_SEGMENT_LEN, the start relaxation's split at
  // ESCAPE_RAMP_DIST and the thin-joint repair all change the segment count for
  // reasons that have nothing to do with how direct the trajectory should be, and
  // the weight would otherwise have to be retuned every time they did. Snap and
  // the time penalty are both extensive in time, so this also makes all three
  // terms scale consistently with the size of the problem.
  //
  // What this does NOT buy: a result invariant to how the path is chopped. More
  // waypoints genuinely say more about the intended shape — each segment's eight
  // targets cluster along its own chord — so a finely split corridor is pulled
  // harder toward the plan at the same weight (measured: 0.19 m of corner
  // deviation over two segments against 0.05 m over four). That is the term
  // working, not a scaling bug. What the length factor fixes is the penalty
  // DENSITY: cost per metre of path rather than per segment, so the weight keeps
  // one meaning rather than drifting with the segment count.
  //
  // Note also that the targets are points along the chord, not the chord as a
  // set, so the term penalises being at the wrong place ALONG the path as well as
  // off it — it pulls toward a roughly uniform traversal of each segment too.
  const bool use_path = path_weight_ > 0.0 && path_waypoints != nullptr &&
                        static_cast<int>(path_waypoints->size()) == S + 1;
  const double path_lambda = use_path ? path_weight_ / kCoeffs : 0.0;
  // Per-segment weight, chord length included. Also read back after the solve to
  // report the term, so it is computed once here.
  std::vector<double> path_seg_lambda(use_path ? S : 0, 0.0);
  if (use_path) {
    const std::vector<Eigen::Vector3d>& wp = *path_waypoints;
    // Same for every segment and axis: only the chord targets and length differ.
    const Eigen::MatrixXd GtG = G_pos.transpose() * G_pos;
    for (int s = 0; s < S; ++s) {
      // A zero-length segment gets no pull, which is right: it has no chord to
      // be pulled toward. resamplePath never emits consecutive duplicates, so
      // this is a guard rather than a case.
      path_seg_lambda[s] = path_lambda * (wp[s + 1] - wp[s]).norm();
      if (path_seg_lambda[s] <= 0.0) continue;
      for (int ax = 0; ax < kAxes; ++ax) {
        Eigen::VectorXd chord(kCoeffs);
        for (int j = 0; j < kCoeffs; ++j) {
          const double u = static_cast<double>(j) / (kCoeffs - 1);
          chord(j) = wp[s](ax) + u * (wp[s + 1](ax) - wp[s](ax));
        }
        P.block(idx(s, ax), idx(s, ax), kCoeffs, kCoeffs) += 2.0 * path_seg_lambda[s] * GtG;
        q_vec.segment(idx(s, ax), kCoeffs) -=
            2.0 * path_seg_lambda[s] * G_pos.transpose() * chord;
      }
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
  // The pinned variant adds one position row per interior junction per axis on
  // top of the C0..C4 continuity already there: continuity says the segments
  // meet, this says WHERE.
  const int m_eq = kAxes * (8 + (pin ? 6 : 5) * (S - 1));
  const int m_in = faces_total * kCoeffs + S * kAxes * (7 + 6 + 5);
  const int m = m_eq + m_in;

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(m, n);
  Eigen::VectorXd lower(m), upper(m);

  int row = 0;

  // Start boundary: position and its first three derivatives pinned to the
  // state the vehicle will be in when this trajectory takes over. On a replan
  // that is the outgoing trajectory sampled at the switch instant, so the two
  // curves agree to C3 there and the reference — including the flatness
  // feed-forward the controller consumes — crosses the splice with no step. A
  // caller with no such state (first plan of a flight, replan off a hover)
  // passes a default-constructed MotionState and gets the old rest start.
  const std::array<const Eigen::Vector3d*, 4> start_deriv = {&start.pos, &start.vel, &start.acc,
                                                             &start.jerk};
  for (int d = 0; d <= 3; ++d) {
    const Eigen::RowVectorXd r0 = derivRow(d, 0.0) / std::pow(times[0], d);
    for (int ax = 0; ax < kAxes; ++ax) {
      A.block(row, idx(0, ax), 1, kCoeffs) = r0;
      lower(row) = upper(row) = (*start_deriv[d])(ax);
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
    // Optional waypoint interpolation: pin segment s's END position to
    // waypoint s+1. Only one side needs pinning — the C0 row just emitted
    // carries it to segment s+1's start, so constraining both would be
    // redundant rows on an equality the solver already holds. Position only:
    // the velocity, acceleration and jerk at the junction stay free and
    // continuous, so the curve carries speed THROUGH each waypoint instead of
    // stopping at it.
    if (pin) {
      const Eigen::RowVectorXd rp = derivRow(0, 1.0);
      for (int ax = 0; ax < kAxes; ++ax) {
        A.block(row, idx(s, ax), 1, kCoeffs) = rp;
        lower(row) = upper(row) = (*pin_waypoints)[s + 1](ax);
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
  const std::vector<OSQPFloat> q(q_vec.data(), q_vec.data() + n);

  OSQPSettings settings;
  osqp_set_default_settings(&settings);
  settings.verbose = 0;
  settings.polishing = 1;    // recover a high-accuracy solution from the ADMM iterate
  settings.eps_abs = 1e-5;   // tight tolerances: corridor rows are safety constraints
  settings.eps_rel = 1e-5;
  // A well-conditioned (tau-normalized) solve converges quickly; only
  // near-infeasible probes from the outer time search grind longer. Cap them
  // well below OSQP's 4000 default so a whole time search stays cheap — an
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
  Eigen::VectorXd sol;
  if (osqp_solve(solver) != 0) {
    // API-level failure (not a solve outcome) — always worth a line.
    DRONE_LOG_ERROR("[corridor-qp] OSQP solve error");
    ok = false;
  } else if (solver->info->status_val != OSQP_SOLVED) {
    if (status_out) *status_out = solver->info->status;
    // Primal infeasible, or unconverged at the iteration cap (which only
    // happens bordering infeasibility) — either way there is no trustworthy
    // trajectory. Silent: the outer time search probes this region on every
    // run and simply rejects the allocation; it reports the status itself if
    // no allocation is ever accepted.
    ok = false;
  } else {
    sol = Eigen::Map<const Eigen::VectorXd>(
        reinterpret_cast<const double*>(solver->solution->x), n);
  }
  osqp_cleanup(solver);
  if (!ok) return false;

  // Split the objective for the caller: the time search minimises their sum,
  // the debug line wants them apart, and obj_val gives neither (it lumps the two
  // together and is short by the path term's dropped constant). Recomputing from
  // the solution is a handful of 8x8 quadratic forms, so it costs nothing next to
  // the solve itself.
  double snap_cost = 0.0;
  double path_cost = 0.0;
  {
    const Eigen::MatrixXd Q1 = snapCostBlock(1.0);
    for (int s = 0; s < S; ++s) {
      const double inv_t7 = 1.0 / std::pow(times[s], 7);
      for (int ax = 0; ax < kAxes; ++ax) {
        const Eigen::VectorXd ct = sol.segment(idx(s, ax), kCoeffs);
        // 0.5 x'Px with P = 2Q/T^7.
        snap_cost += inv_t7 * ct.dot(Q1 * ct);
        if (use_path) {
          const std::vector<Eigen::Vector3d>& wp = *path_waypoints;
          Eigen::VectorXd chord(kCoeffs);
          for (int j = 0; j < kCoeffs; ++j) {
            const double u = static_cast<double>(j) / (kCoeffs - 1);
            chord(j) = wp[s](ax) + u * (wp[s + 1](ax) - wp[s](ax));
          }
          path_cost += path_seg_lambda[s] * (G_pos * ct - chord).squaredNorm();
        }
      }
    }
  }

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
  if (cost_out) *cost_out = snap_cost;
  if (path_cost_out) *path_cost_out = path_cost;
  return true;
}

namespace {

// Partition the segments into groups that turn the same way, for the per-group
// time cuts in optimizeTrajectory. Returns [first, last] segment index pairs
// covering 0..S-1 in order. The decision is made at the joints (the turn from
// one segment's direction to the next):
//   - a joint turning at most `straight_angle` is straight, and its direction is
//     ignored (the turn axis of a near-zero turn is noise);
//   - a straight joint continues the group, unless that group is an arc: then
//     the arc's last segment runs from its final turn into this straight joint,
//     so it opens the new straight group rather than closing the arc;
//   - a turning joint continues the group only when the joint before it was the
//     same turn (angle within `turn_tolerance`, axis within `axis_tolerance`) —
//     an arc. Anything else opens a new group.
// So a sharp corner between two straights is a boundary between two straight
// groups, and an arc's group holds the segments that lie between two of its
// matching turns.
std::vector<std::pair<int, int>> groupSegments(const std::vector<Eigen::Vector3d>& waypoints,
                                               double straight_angle, double turn_tolerance,
                                               double axis_tolerance) {
  const int S = static_cast<int>(waypoints.size()) - 1;
  std::vector<Eigen::Vector3d> dir(S);
  for (int s = 0; s < S; ++s) {
    const Eigen::Vector3d d = waypoints[s + 1] - waypoints[s];
    const double n = d.norm();
    dir[s] = n > 1e-9 ? Eigen::Vector3d(d / n) : Eigen::Vector3d::Zero();
  }
  struct Turn {
    double angle;          // [rad]
    Eigen::Vector3d axis;  // unit, zero when there is no turn to speak of
  };
  // Joint j sits between segments j - 1 and j.
  const auto turnAt = [&dir](int j) {
    const Eigen::Vector3d c = dir[j - 1].cross(dir[j]);
    const double cn = c.norm();
    return Turn{std::atan2(cn, dir[j - 1].dot(dir[j])),
                cn > 1e-9 ? Eigen::Vector3d(c / cn) : Eigen::Vector3d::Zero()};
  };
  const auto isTurn = [straight_angle](const Turn& t) { return t.angle > straight_angle; };
  const double cos_axis = std::cos(axis_tolerance);
  const auto sameTurn = [&](const Turn& a, const Turn& b) {
    return isTurn(a) && isTurn(b) && std::abs(a.angle - b.angle) <= turn_tolerance &&
           a.axis.dot(b.axis) >= cos_axis;
  };

  std::vector<std::pair<int, int>> groups{{0, 0}};
  enum class Kind { kSingle, kStraight, kArc } kind = Kind::kSingle;  // of groups.back()
  for (int j = 1; j < S; ++j) {
    const Turn t = turnAt(j);
    if (!isTurn(t)) {
      if (kind == Kind::kArc) {
        --groups.back().second;  // an arc group has >= 2 segments, so this leaves one
        groups.push_back({j - 1, j});
      } else {
        groups.back().second = j;
      }
      kind = Kind::kStraight;
    } else if (j >= 2 && sameTurn(turnAt(j - 1), t)) {
      groups.back().second = j;
      kind = Kind::kArc;
    } else {
      groups.push_back({j, j});
      kind = Kind::kSingle;
    }
  }
  return groups;
}

}  // namespace

bool CorridorTrajectoryOptimizer::optimizeTrajectory(
    const common::MotionState& start, const std::vector<Eigen::Vector3d>& waypoints,
    const std::vector<ConvexRegion>& regions, common::Trajectory& out,
    bool pin_waypoints) const {
  if (waypoints.size() < 2 || regions.size() != waypoints.size() - 1) return false;
  const int S = static_cast<int>(regions.size());
  // Passed to every solve below: the allocation being searched has to be an
  // allocation for the shape actually being built.
  const std::vector<Eigen::Vector3d>* pin = pin_waypoints ? &waypoints : nullptr;
  const auto t_begin = std::chrono::steady_clock::now();
  const auto elapsed = [&t_begin]() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t_begin).count();
  };
  const auto since = [](std::chrono::steady_clock::time_point t0) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  };
  const auto withinBudget = [&]() { return time_budget_ <= 0.0 || elapsed() < time_budget_; };
  const auto sum = [](const std::vector<double>& v) {
    double t = 0.0;
    for (double x : v) t += x;
    return t;
  };

  // Every stage below only ever replaces `times` with an allocation the QP has
  // just accepted, and solveQP leaves its output untouched on failure, so `best`
  // always holds the trajectory for the current `times` — no final re-solve.
  common::Trajectory best;
  double best_snap = 0.0;
  double best_path = 0.0;
  // OSQP's own word for why the most recent solve was rejected; see the growth
  // failure below.
  std::string last_status;
  const auto trySolve = [&](const std::vector<double>& t) {
    double snap = 0.0;
    double path = 0.0;
    last_status.clear();
    if (!solveQP(start, waypoints.back(), t, regions, best, &snap, pin, &waypoints, &path,
                 &last_status)) {
      return false;
    }
    best_snap = snap;
    best_path = path;
    return true;
  };

  // Velocity-consistent seed: long enough to traverse each segment at vmax with
  // some slack. "len/vmax + buffer" alone can still be infeasible once the
  // accel/jerk ramps and the conservative Bezier hull bite, so the whole
  // allocation is then grown geometrically until the QP accepts it.
  std::vector<double> times(S);
  for (int s = 0; s < S; ++s) {
    times[s] = (waypoints[s + 1] - waypoints[s]).norm() / limits_.vmax + kSeedBuffer;
  }
  // A moving start needs time to shed that speed on top of the traversal, and
  // the seed above knows nothing about it. Charge the whole braking time to the
  // first segment: it is only a seed. Without this a fast replan starts inside
  // the infeasible region and burns growth iterations getting out.
  times[0] += limits_.amax > 0.0 ? start.vel.cwiseAbs().maxCoeff() / limits_.amax : 0.0;

  // Debug accounting for the one-line breakdown below (debug_ only).
  const double seed_total = sum(times);
  int grow_solves = 0;
  double grow_time = 0.0;
  double grown_total = 0.0;
  int bisect_solves = 0;
  double bisect_time = 0.0;
  double bisected_total = 0.0;
  double bisect_gap = 0.0;  // final (hi - lo) / lo of the bracket
  struct PassLog {
    double cut = 0.0;  // middle-segment fraction tried
    int tried = 0;
    int accepted = 0;
    double before = 0.0;
    double after = 0.0;
    double time = 0.0;
  };
  std::vector<std::pair<int, int>> groups;
  std::vector<PassLog> passes;
  bool budget_hit = false;
  const auto report = [&](const std::string& outcome, const common::Trajectory* result) {
    if (!debug_) return;
    std::ostringstream os;
    os << "[corridor-qp] " << S << " segments | stage 1: " << grow_time << " s, " << grow_solves
       << " QP solve(s), seed " << seed_total << " s -> " << grown_total << " s | ";
    if (bisect_solves == 0) {
      os << "bisect: not run | ";
    } else {
      os << "bisect: " << bisect_time << " s, " << bisect_solves << " QP solve(s) -> "
         << bisected_total << " s (bracket " << 100.0 * bisect_gap << "%) | ";
    }
    if (groups.empty()) {
      os << "groups: not formed | ";
    } else {
      os << "groups: " << groups.size() << " [";
      for (size_t g = 0; g < groups.size(); ++g) {
        os << (g ? " " : "") << groups[g].first;
        if (groups[g].second != groups[g].first) os << "-" << groups[g].second;
      }
      os << "] | ";
    }
    for (const auto& p : passes) {
      os << "cut " << 100.0 * p.cut << "%: " << p.accepted << "/" << p.tried << " accepted, "
         << p.before << " s -> " << p.after << " s, " << p.time << " s | ";
    }
    if (budget_hit) os << "[budget hit] | ";
    os << "total " << elapsed() << " s | " << outcome;
    if (result) {
      os << ", trajectory " << result->total_duration << " s | cost: snap " << best_snap;
      if (path_weight_ > 0.0) os << " + path " << best_path;
    }
    DRONE_LOG_INFO(os.str());
  };

  // Stage 1: grow until feasible. Not cut short by the budget: until a feasible
  // allocation exists there is nothing to fall back on.
  int grow = 0;
  {
    const auto t_grow = std::chrono::steady_clock::now();
    bool feasible = false;
    while (true) {
      ++grow_solves;
      feasible = trySolve(times);
      if (feasible || grow >= kMaxSeedGrowth) break;
      for (double& t : times) t *= 1.5;
      ++grow;
    }
    grow_time = since(t_grow);
    grown_total = sum(times);
    // Note this cannot rescue a corridor that is simply too SHORT to stop in:
    // braking from v0 inside distance d needs a >= v0^2/(2d) whatever the time
    // allocation, so stretching time does not help. That is a real physical
    // refusal (the committed prefix is shorter than the stopping distance) and
    // the caller must treat it as "no trajectory", not as a tuning failure.
    if (!feasible) {  // corridor unusable at any sane duration
      // Say WHICH rejection it was. OSQP reporting "primal infeasible" means the
      // corridor genuinely admits no such curve; "maximum iterations reached" or
      // a polish failure means the solver ran out of road on a hard problem and
      // the code's treat-as-infeasible rule fired. Those want opposite responses
      // — look at the geometry versus loosen the solver or whatever is driving
      // the optimum onto a constraint boundary (TRAJ_PATH_WEIGHT, a start pinned
      // millimetres inside region 0) — and they were indistinguishable in the log.
      report("FAILED (no feasible allocation within the seed growth; OSQP said \"" +
                 (last_status.empty() ? std::string("unknown") : last_status) + "\")",
             nullptr);
      return false;
    }
  }

  // Stage 2: bisect the growth's last step. Growth multiplies every segment by
  // 1.5, so it can land up to 50% past the shortest feasible allocation. From
  // rest, stretching every segment by one factor leaves the curve's shape
  // unchanged and only scales its derivatives down, so feasibility is monotone
  // in the factor and the bracket [previous step, this step] holds exactly one
  // switch. With a moving start the fixed start velocity breaks the scaling
  // slightly, so the switch is only approximately single — harmless, since `hi`
  // is always an allocation the QP accepted and a wrongly-rejected midpoint only
  // leaves the result a little longer. Only possible when growth ran (grow > 0),
  // which is what supplies the infeasible end.
  if (grow > 0) {
    const auto t_bisect = std::chrono::steady_clock::now();
    const std::vector<double> base = times;  // feasible, at factor 1
    double lo = 1.0 / 1.5;                   // infeasible (the previous growth step)
    double hi = 1.0;
    std::vector<double> trial(S);
    while ((hi - lo) / lo > kBisectGap) {
      if (!withinBudget()) {
        budget_hit = true;
        break;
      }
      const double mid = 0.5 * (lo + hi);
      for (int s = 0; s < S; ++s) trial[s] = base[s] * mid;
      ++bisect_solves;
      if (trySolve(trial)) {
        hi = mid;
      } else {
        lo = mid;
      }
    }
    for (int s = 0; s < S; ++s) times[s] = base[s] * hi;
    bisect_gap = (hi - lo) / lo;
    bisect_time = since(t_bisect);
  }
  bisected_total = sum(times);

  // Stage 3: per-group cuts. Uniform scaling makes every segment wait for the
  // tightest one, so segments with slack — typically the straights either side
  // of a corner — are left slower than they need to be. Cutting a segment on its
  // own puts a speed step at both its joints, which the jerk limit often
  // refuses even when cutting its neighbours with it would pass, so segments
  // that turn alike are cut together (see groupSegments). A group's two end
  // segments take only group_edge_factor_ of the cut, easing the speed change
  // into its neighbours. Two passes, the full cut then half of it, each over
  // every group, longest (in time) first so a budget cut-off drops the least
  // valuable tries. Every try re-solves the whole trajectory, so a kept cut can
  // never be broken by a later one.
  if (!budget_hit && group_cut_ > 0.0) {
    groups = groupSegments(waypoints, kGroupStraightAngle, kGroupTurnTolerance,
                           kGroupAxisTolerance);
    std::vector<double> trial(S);
    for (const double cut : {group_cut_, 0.5 * group_cut_}) {
      if (budget_hit) break;
      PassLog log;
      log.cut = cut;
      log.before = sum(times);
      const auto t_pass = std::chrono::steady_clock::now();
      std::vector<int> order(groups.size());
      std::vector<double> group_time(groups.size(), 0.0);
      for (size_t g = 0; g < groups.size(); ++g) {
        order[g] = static_cast<int>(g);
        for (int s = groups[g].first; s <= groups[g].second; ++s) group_time[g] += times[s];
      }
      std::stable_sort(order.begin(), order.end(),
                       [&](int a, int b) { return group_time[a] > group_time[b]; });
      for (const int g : order) {
        if (!withinBudget()) {
          budget_hit = true;
          break;
        }
        trial = times;
        const auto [first, last] = groups[g];
        for (int s = first; s <= last; ++s) {
          const bool edge = s == first || s == last;
          const double frac = edge ? group_edge_factor_ * cut : cut;
          trial[s] = std::max(kMinSegmentTime, times[s] * (1.0 - frac));
        }
        ++log.tried;
        if (trySolve(trial)) {
          times = trial;
          ++log.accepted;
        }
      }
      log.after = sum(times);
      log.time = since(t_pass);
      passes.push_back(log);
    }
  }

  out = best;
  report("OK", &out);
  return true;
}

}  // namespace drone_core::planning
