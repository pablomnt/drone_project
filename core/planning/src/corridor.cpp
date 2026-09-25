#include "drone_core/planning/corridor.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

// DecompUtil (header-only) stays confined to this translation unit: the public
// header trades in plain Eigen half-space data only.
#include <decomp_util/ellipsoid_decomp.h>

namespace drone_core::planning {

namespace {
// Shallowest overlap two consecutive regions may have [m] (radius of the largest
// ball inside both). Any positive depth lets the C0 junction sit in both; this
// only keeps a sliver the solver cannot tell from empty from reaching the QP.
constexpr double kMinRegionOverlap = 0.02;
}  // namespace

std::vector<std::vector<Eigen::Vector3d>> regionFaceLoops(const ConvexRegion& region) {
  std::vector<std::vector<Eigen::Vector3d>> loops;
  const int k = static_cast<int>(region.A.rows());
  if (k < 4) return loops;  // fewer than 4 half-spaces cannot bound a volume
  constexpr double kEps = 1e-6;

  // Candidate vertices: every triple of faces meeting at a point that satisfies
  // all the remaining half-spaces.
  std::vector<Eigen::Vector3d> verts;
  for (int i = 0; i < k; ++i) {
    for (int j = i + 1; j < k; ++j) {
      for (int l = j + 1; l < k; ++l) {
        Eigen::Matrix3d M;
        M.row(0) = region.A.row(i);
        M.row(1) = region.A.row(j);
        M.row(2) = region.A.row(l);
        const double det = M.determinant();
        if (std::abs(det) < 1e-9) continue;  // parallel / coincident faces
        const Eigen::Vector3d v =
            M.inverse() * Eigen::Vector3d(region.b(i), region.b(j), region.b(l));
        if (((region.A * v - region.b).array() > kEps).any()) continue;  // outside
        bool dup = false;
        for (const auto& u : verts) {
          if ((u - v).norm() < 1e-6) { dup = true; break; }
        }
        if (!dup) verts.push_back(v);
      }
    }
  }
  if (verts.size() < 4) return loops;

  // Per face, the vertices lying on it, sorted by angle about the face centroid
  // in the face plane so the ring traces the outline rather than zig-zagging.
  for (int f = 0; f < k; ++f) {
    const Eigen::Vector3d n = region.A.row(f).transpose();
    std::vector<Eigen::Vector3d> on_face;
    for (const auto& v : verts) {
      if (std::abs(n.dot(v) - region.b(f)) < 1e-5) on_face.push_back(v);
    }
    if (on_face.size() < 3) continue;

    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto& v : on_face) centroid += v;
    centroid /= static_cast<double>(on_face.size());

    // Any unit vector in the face plane serves as the angle origin.
    Eigen::Vector3d u = n.unitOrthogonal();
    const Eigen::Vector3d w = n.cross(u).normalized();
    std::sort(on_face.begin(), on_face.end(),
              [&](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
                const Eigen::Vector3d da = a - centroid, db = b - centroid;
                return std::atan2(da.dot(w), da.dot(u)) < std::atan2(db.dot(w), db.dot(u));
              });
    loops.push_back(std::move(on_face));
  }
  return loops;
}

double regionOverlapDepth(const ConvexRegion& a, const ConvexRegion& b,
                          Eigen::Vector3d* center) {
  // A region with no faces is all of space (ConvexRegion::contains agrees).
  if (a.A.rows() == 0 || b.A.rows() == 0) return std::numeric_limits<double>::infinity();

  // Primal: maximise r over (x, r) subject to n_i·x + |n_i| r <= b_i for every
  // face i of both regions — the ball of radius r about x is inside every face.
  // Solved through its dual, which is in standard form with only four equality
  // rows and one column per face:
  //   minimise b·y  subject to  sum_i y_i n_i = 0,  sum_i y_i |n_i| = 1,  y >= 0.
  // By strong duality the optimum equals the primal's largest radius. Dense
  // two-phase simplex with Bland's rule (no cycling on the degenerate, near-
  // parallel faces corridor regions carry). OSQP was tried first and stalled at
  // its iteration cap on exactly those faces, which is ADMM's known weakness on
  // degenerate LPs; this is exact and needs no cap on well-formed input.
  const int m = static_cast<int>(a.A.rows() + b.A.rows());
  constexpr int kRows = 4;
  const int cols = m + kRows;  // face columns, then one artificial per row
  Eigen::MatrixXd T = Eigen::MatrixXd::Zero(kRows, cols + 1);  // last column: rhs
  Eigen::VectorXd face_b(m);
  int col = 0;
  for (const ConvexRegion* region : {&a, &b}) {
    for (int f = 0; f < region->A.rows(); ++f, ++col) {
      const Eigen::Vector3d nrm = region->A.row(f).transpose();
      T.block(0, col, 3, 1) = nrm;
      T(3, col) = nrm.norm();
      face_b(col) = region->b(f);
    }
  }
  for (int r = 0; r < kRows; ++r) T(r, m + r) = 1.0;
  T(3, cols) = 1.0;  // rhs (0, 0, 0, 1) is already non-negative
  std::array<int, kRows> basis = {m, m + 1, m + 2, m + 3};

  constexpr double kEps = 1e-9;
  const int max_pivots = 50 * cols;  // Bland's rule terminates; this only guards bad input
  // Minimise cost·y over the current tableau. Artificial columns may leave the
  // basis but never re-enter once phase 1 is done. False on unbounded or on
  // hitting the pivot guard.
  auto simplex = [&](const Eigen::VectorXd& cost, bool allow_artificial, bool& unbounded) {
    unbounded = false;
    for (int it = 0; it < max_pivots; ++it) {
      int enter = -1;
      const int limit = allow_artificial ? cols : m;
      for (int j = 0; j < limit && enter < 0; ++j) {
        double reduced = cost(j);
        for (int r = 0; r < kRows; ++r) reduced -= cost(basis[r]) * T(r, j);
        if (reduced < -kEps) enter = j;
      }
      if (enter < 0) return true;  // optimal
      int leave = -1;
      double best = std::numeric_limits<double>::infinity();
      for (int r = 0; r < kRows; ++r) {
        if (T(r, enter) <= kEps) continue;
        const double ratio = T(r, cols) / T(r, enter);
        if (leave < 0 || ratio < best - kEps ||
            (std::abs(ratio - best) <= kEps && basis[r] < basis[leave])) {
          best = ratio;
          leave = r;
        }
      }
      if (leave < 0) {
        unbounded = true;
        return false;
      }
      T.row(leave) /= T(leave, enter);
      for (int r = 0; r < kRows; ++r) {
        if (r != leave && T(r, enter) != 0.0) T.row(r) -= T(r, enter) * T.row(leave);
      }
      basis[leave] = enter;
    }
    return false;
  };

  // Phase 1: minimise the artificials to find a feasible dual basis.
  Eigen::VectorXd cost1 = Eigen::VectorXd::Zero(cols);
  cost1.tail(kRows).setOnes();
  bool unbounded = false;
  if (!simplex(cost1, /*allow_artificial=*/true, unbounded)) {
    return -std::numeric_limits<double>::infinity();
  }
  double infeasibility = 0.0;
  for (int r = 0; r < kRows; ++r) {
    if (basis[r] >= m) infeasibility += T(r, cols);
  }
  // No dual solution means the primal is unbounded: the regions share an
  // infinite ball, which closed corridor regions never do.
  if (infeasibility > 1e-7) return std::numeric_limits<double>::infinity();
  // Pivot any artificial still basic at zero out on a face column, so phase 2
  // prices real columns only. A row with no such column is redundant (all face
  // normals lack that component, e.g. a region open along z); its artificial
  // stays at zero and phase 2 never moves it. On bounded regions the phase-1
  // ratio test has already removed them, so this is for degenerate input only.
  for (int r = 0; r < kRows; ++r) {
    if (basis[r] < m) continue;
    for (int j = 0; j < m; ++j) {
      if (std::abs(T(r, j)) > kEps) {
        T.row(r) /= T(r, j);
        for (int k = 0; k < kRows; ++k) {
          if (k != r && T(k, j) != 0.0) T.row(k) -= T(k, j) * T.row(r);
        }
        basis[r] = j;
        break;
      }
    }
  }

  // Phase 2: minimise b·y. Artificials cost nothing and cannot enter.
  Eigen::VectorXd cost2 = Eigen::VectorXd::Zero(cols);
  cost2.head(m) = face_b;
  if (!simplex(cost2, /*allow_artificial=*/false, unbounded)) {
    // Unbounded dual = infeasible primal, impossible here (r can always shrink);
    // treat it, and the pivot guard, as no usable overlap.
    return -std::numeric_limits<double>::infinity();
  }
  double depth = 0.0;
  for (int r = 0; r < kRows; ++r) {
    // An artificial that phase 2 pushed off zero means the basis is no longer a
    // dual solution and the value below would not be the overlap. Fail safe.
    if (basis[r] >= m && std::abs(T(r, cols)) > 1e-7) {
      return -std::numeric_limits<double>::infinity();
    }
    depth += cost2(basis[r]) * T(r, cols);
  }
  if (center) {
    // The primal optimum is the dual's simplex multipliers, pi = c_B B^-1: its
    // first three entries are the ball centre x, its last the radius r. B^-1 is
    // read off the artificial columns, which started as the identity and have
    // had every row operation applied to them since.
    Eigen::Vector4d pi = Eigen::Vector4d::Zero();
    for (int j = 0; j < kRows; ++j) {
      for (int r = 0; r < kRows; ++r) pi(j) += cost2(basis[r]) * T(r, m + j);
    }
    *center = pi.head<3>();
  }
  return depth;
}

std::vector<Eigen::Vector3d> resamplePath(const std::vector<Eigen::Vector3d>& path,
                                          double max_segment_len) {
  // Nothing to subdivide: too few points or a nonsensical cap => pass through.
  if (path.size() < 2 || max_segment_len <= 0.0) return path;

  std::vector<Eigen::Vector3d> out;
  out.reserve(path.size());

  // Push a point only if it differs from the current tail, so shared segment
  // endpoints (b of segment i == a of segment i+1) and degenerate zero-length
  // segments never produce consecutive duplicates.
  const auto pushUnique = [&out](const Eigen::Vector3d& q) {
    if (out.empty() || (out.back().array() != q.array()).any()) out.push_back(q);
  };

  pushUnique(path.front());  // seed the start once; the loop only adds interiors + ends

  for (size_t i = 0; i + 1 < path.size(); ++i) {
    const Eigen::Vector3d& a = path[i];
    const Eigen::Vector3d& b = path[i + 1];
    const double L = (b - a).norm();

    // Fewest equal pieces that keep every piece within the cap (>=1 so a and b
    // stay connected even for a zero-length or already-short segment).
    const int n = std::max(1, static_cast<int>(std::ceil(L / max_segment_len)));

    // Interior split points; the shared endpoints are handled by pushUnique
    // (a was pushed as the previous end / start, b is pushed just below).
    for (int k = 1; k < n; ++k) {
      const double t = static_cast<double>(k) / n;
      pushUnique(a + t * (b - a));
    }
    pushUnique(b);
  }

  // Guarantee the last point is exactly the original endpoint (no rounding on b).
  out.back() = path.back();
  return out;
}

std::vector<Eigen::Vector3d> truncatePath(const CorridorClearanceFn& conservative_clearance,
                                          const std::vector<Eigen::Vector3d>& path,
                                          double margin, double escape_ramp,
                                          double sample_step,
                                          const CorridorUnknownFn& is_unknown,
                                          TruncationCut* cut) {
  if (cut) *cut = TruncationCut{};
  if (path.size() < 2) return path;
  const Eigen::Vector3d& start = path.front();

  // A sampled point is safe when its conservative clearance covers the required
  // margin, which RAMPS LINEARLY from 0 at the drone to the full margin at
  // `escape_ramp` metres out, the same ramp the planner's validity check uses (a
  // drone parked near the mapped floor, or in a small pocket of known-free
  // space, must be able to root a path). A hard exemption sphere instead leaves
  // a dead band just outside it where the full margin applies at once, cutting
  // even a path heading directly away from the hazard. Ramping over a distance INDEPENDENT of the margin is what keeps
  // it usable: ramping over `margin` metres instead makes the requirement rise
  // at 1 m/m, which on a thinly-mapped scene meets the shrinking clearance
  // within centimetres and truncates the path to nothing. Clearance must
  // additionally be strictly positive everywhere (never inside an
  // occupied/unknown voxel), so leniency near the start never means blindness.
  // Unobserved space is an absolute stop, checked BEFORE the clearance test and
  // exempt from the ramp. The ramp trades margin for the ability to move at all,
  // which is a reasonable trade against a hazard whose distance we can measure;
  // it is not a reasonable trade against space we have never looked at. And the
  // clearance test cannot catch this on its own — it measures distance to the
  // stamped frontier shell, which has gaps, so a path leaving through one reads
  // as high-clearance the whole way out. See the header.
  const auto safe = [&](const Eigen::Vector3d& q) {
    if (is_unknown && is_unknown(q.x(), q.y(), q.z())) return false;
    const double d = conservative_clearance(q.x(), q.y(), q.z());
    if (d <= 0.0) return false;
    if (escape_ramp <= 0.0) return d >= margin;  // ramp disabled
    return d >= margin * std::min(1.0, (q - start).norm() / escape_ramp);
  };

  std::vector<Eigen::Vector3d> out;
  out.push_back(start);
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    const Eigen::Vector3d& a = path[i];
    const Eigen::Vector3d& b = path[i + 1];
    const double L = (b - a).norm();
    const int n = std::max(1, static_cast<int>(std::ceil(L / sample_step)));
    for (int k = 1; k <= n; ++k) {
      const Eigen::Vector3d q = a + (static_cast<double>(k) / n) * (b - a);
      if (!safe(q)) {
        if (cut) {
          cut->cut = true;
          cut->unknown = is_unknown && is_unknown(q.x(), q.y(), q.z());
          cut->point = q;
          cut->from_start = (q - start).norm();
          cut->clearance = conservative_clearance(q.x(), q.y(), q.z());
          cut->required = escape_ramp <= 0.0
                              ? margin
                              : margin * std::min(1.0, cut->from_start / escape_ramp);
        }
        // Cut just before the first unsafe sample. The previous sample is the
        // committed endpoint (unless it duplicates the tail, e.g. an unsafe
        // first sample of a segment cutting at the shared waypoint).
        const Eigen::Vector3d last = a + (static_cast<double>(k - 1) / n) * (b - a);
        if ((last - out.back()).norm() > 1e-9) out.push_back(last);
        return out;
      }
    }
    out.push_back(b);
  }
  return out;  // whole path safe
}

double corridorObstacleWindowPad(const CorridorParams& p) {
  // Mirror the bbox sizing in buildCorridor: the widest a region can reach is
  // the lateral window, which is at least the longest segment, plus the shrink.
  const double pull_in = p.margin + p.voxel_half_diagonal;
  return std::max(p.local_bbox.maxCoeff(), p.max_segment_len) + pull_in;
}

bool buildCorridor(const std::vector<Eigen::Vector3d>& obstacles,
                   const std::vector<Eigen::Vector3d>& path,
                   const CorridorParams& p,
                   std::vector<Eigen::Vector3d>& resampled_out,
                   std::vector<ConvexRegion>& regions_out,
                   std::string* reason,
                   CorridorAttempt* attempt,
                   double* start_margin,
                   double* end_pullback,
                   CorridorRepairs* repairs) {
  // The primary outputs are always cleared on failure so a rejected corridor
  // can never be flown; `attempt` deliberately survives so the host can draw
  // what was rejected.
  const auto fail = [&](const std::string& why) {
    if (reason) *reason = why;
    resampled_out.clear();
    regions_out.clear();
    return false;
  };
  resampled_out.clear();
  regions_out.clear();
  if (reason) reason->clear();
  if (attempt) *attempt = CorridorAttempt{};
  if (repairs) *repairs = CorridorRepairs{};
  if (path.size() < 2) return fail("path has fewer than 2 waypoints");

  resampled_out = resamplePath(path, p.max_segment_len);
  if (resampled_out.size() < 2) return fail("resampling produced fewer than 2 waypoints");

  // Confine the start relaxation below to a SHORT first region by splitting the
  // first segment at start_relax_dist. Without this the relaxed region is a
  // whole max_segment_len long, so a drone that needs 10 cm of leniency to get
  // off the ground would fly two metres at reduced margin. The split is what
  // makes the relaxation bounded in extent — a convex region has no interior
  // gradient, so this is the only place that bound can come from. Skipped when
  // the first segment is already short enough (it is then already inside the
  // relax distance) or when the leftover piece would be a sliver, since a
  // near-zero segment gives the QP's time allocation a degenerate T.
  if (p.start_relax_dist > 0.0) {
    constexpr double kMinSplitPiece = 0.15;
    const Eigen::Vector3d& a = resampled_out[0];
    const Eigen::Vector3d& b = resampled_out[1];
    const double L = (b - a).norm();
    if (p.start_relax_dist >= kMinSplitPiece && L > p.start_relax_dist + kMinSplitPiece) {
      resampled_out.insert(resampled_out.begin() + 1,
                           a + (p.start_relax_dist / L) * (b - a));
    }
  }

  // DecompUtil's ellipsoid decomposition: per segment, inflate an ellipsoid
  // spanning it and cut a half-space at each obstacle point in the order they
  // bind, within a window aligned to the segment (local_bbox: x along the
  // path, y/z lateral). Vec3f is double despite the name. NOTE: the
  // (origin, dim) constructor + global bbox path is deliberately avoided — its
  // add_global_bbox has an upstream bug (the -Y plane is placed at +Y's
  // coordinate), and the caller's obstacle window plus local_bbox already
  // bound the regions.
  vec_Vec3f obs;
  obs.reserve(obstacles.size());
  for (const auto& o : obstacles) obs.emplace_back(o.x(), o.y(), o.z());

  // Size the growth window from the geometry rather than a fixed number. Two
  // requirements. (1) It must scale with the segments: a window narrower than
  // the segment is long would clip regions purely because the path is long,
  // which is the bounding-box failure this rewrite exists to remove. (2) The
  // shrink below pulls EVERY face in, including these artificial window planes,
  // so the window must be pull_in larger than the volume we actually want to
  // keep — otherwise the margin silently eats the usable region from the
  // outside. Lateral gets the full segment length, along-track half of it
  // (which also guarantees consecutive regions overlap generously, since each
  // reaches past its endpoints into its neighbour). p.local_bbox acts as a
  // floor, so a caller can ask for a wider window but never a self-defeating
  // one. Sized once from the initial segments and kept through the repairs
  // below, which only ever add shorter segments.
  const double pull_in = p.margin + p.voxel_half_diagonal;
  double seg_max = 0.0;
  for (size_t i = 0; i + 1 < resampled_out.size(); ++i) {
    seg_max = std::max(seg_max, (resampled_out[i + 1] - resampled_out[i]).norm());
  }
  const Eigen::Vector3d want(std::max(p.local_bbox.x(), 0.5 * seg_max),
                             std::max(p.local_bbox.y(), seg_max),
                             std::max(p.local_bbox.z(), seg_max));
  const Eigen::Vector3d bbox = want.array() + pull_in;

  // Grow the UNSHRUNK region around one segment, as plain A/b rows oriented
  // "inside satisfies A p <= b" using the segment midpoint (which the
  // decomposition guarantees is inside). DecompUtil grows each segment of a
  // path independently, so growing them one at a time is the same result as
  // one call over the whole path, and lets a repair grow an extra region
  // without regrowing the rest. Faces are unit-normal on DecompUtil's side
  // already, but normalise defensively so b stays metric.
  const auto growRaw = [&](const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                           ConvexRegion& raw) {
    EllipsoidDecomp3D decomp;
    decomp.set_obs(obs);
    decomp.set_local_bbox(Vec3f(bbox.x(), bbox.y(), bbox.z()));
    decomp.dilate(vec_Vec3f{Vec3f(a.x(), a.y(), a.z()), Vec3f(b.x(), b.y(), b.z())});
    const auto polys = decomp.get_polyhedrons();
    if (polys.size() != 1) return false;
    const Eigen::Vector3d mid = 0.5 * (a + b);
    LinearConstraint3D lc(Vec3f(mid.x(), mid.y(), mid.z()), polys[0].hyperplanes());
    std::vector<Eigen::Vector3d> normals;
    std::vector<double> offsets;
    for (int r = 0; r < lc.A().rows(); ++r) {
      const Eigen::Vector3d n = lc.A().row(r).transpose();
      const double norm = n.norm();
      if (norm < 1e-9) continue;  // degenerate face; drop rather than divide
      normals.push_back(n / norm);
      offsets.push_back(lc.b()(r) / norm);
    }
    const int rows = static_cast<int>(normals.size());
    raw.A.resize(rows, 3);
    raw.b.resize(rows);
    for (int r = 0; r < rows; ++r) {
      raw.A.row(r) = normals[r];
      raw.b(r) = offsets[r];
    }
    return true;
  };
  // Pull every face in by `shrink`. With the full pull_in (margin + voxel half
  // diagonal): DecompUtil's faces touch the obstacle *points*, which are voxel
  // centres, so the extra pull-in makes the margin hold against the voxel's
  // worst-case corner, not just its centre.
  const auto shrunkBy = [](const ConvexRegion& raw, double shrink) {
    ConvexRegion region = raw;
    region.b.array() -= shrink;
    return region;
  };

  // Repairs for consecutive regions that stop overlapping once shrunk (see the
  // overlap check below). Overlap depth is the radius of the largest ball in
  // both regions, and shrinking both by pull_in lowers it by exactly pull_in, so
  // any joint whose unshrunk overlap is thinner than ~pull_in fails however
  // roomy the two regions are on either side of it — typically a joint that
  // falls in a squeeze, like passing under furniture. Two repairs, cheapest
  // first:
  //   - BRIDGE: grow one extra region on a short segment through the deepest
  //     point of the two unshrunk regions' intersection, i.e. centred in the
  //     squeeze, and splice it between them. Only the joint's waypoint changes
  //     (it becomes the bridge segment's two ends), and interior waypoints are
  //     not pinned by the QP, so nothing else moves. Kept only if both new
  //     joints pass the same overlap test.
  //   - SPLIT: halve the two segments either side of the joint and rebuild.
  //     Shorter segments grow rounder regions about the joint, which usually
  //     share more of it. Rebuilds everything, so it is bounded in rounds.
  // Neither is guaranteed: a bridge's overlap with a neighbour is still limited
  // by how thin that neighbour is near the squeeze. The overlap test stays the
  // final word, so a genuinely narrow passage is still refused.
  constexpr int kMaxSplitRounds = 2;
  constexpr double kBridgeHalfLen = 0.25;  // bridge segment half-length [m]
  constexpr double kBridgeDepthGive = 0.01;  // depth traded to centre a bridge near its joint [m]
  constexpr double kMinHalfPiece = 0.15;   // don't split a segment below 2x this [m]
  const std::vector<Eigen::Vector3d> base_start = resampled_out;
  std::vector<Eigen::Vector3d> base = base_start;
  int bridges = 0;
  int split_rounds = 0;
  double pulled_back = 0.0;

  for (;;) {
    resampled_out = base;
    regions_out.clear();
    std::vector<ConvexRegion> raws;  // unshrunk, aligned with regions_out
    // Index into `base` of each resampled_out point, -1 for bridge ends; says
    // which base segments a failing joint sits between when splitting.
    std::vector<int> origin(base.size());
    for (size_t i = 0; i < origin.size(); ++i) origin[i] = static_cast<int>(i);
    if (attempt) *attempt = CorridorAttempt{};
    if (attempt) attempt->resampled = resampled_out;
    bridges = 0;
    pulled_back = 0.0;

    regions_out.reserve(base.size() - 1);
    for (size_t s = 0; s + 1 < resampled_out.size(); ++s) {
      ConvexRegion raw;
      if (!growRaw(resampled_out[s], resampled_out[s + 1], raw)) {
        return fail("decomposition returned the wrong number of regions");
      }

      // The first region gets the largest shrink that still contains the drone,
      // rather than the full one. The QP equality-constrains the trajectory to
      // start at the vehicle's position, so a first region that excludes it is
      // infeasible outright — and truncatePath, by design, hands us a start whose
      // required clearance ramps to ZERO at the drone, so on a thin map the two
      // stages disagree by construction and the corridor is refused every tick.
      // Since every face is a plane and offsets carry metric distance, the
      // vehicle's slack against face r is offsets[r] - n_r.p, and the tightest of
      // those is the most we can pull in. Two things make this safe to do:
      //   - It never relaxes more than it must. Give the drone room and the min
      //     rises above pull_in, the clamp binds, and this is a no-op — the
      //     relaxation heals itself as the map fills in, with no parameter to
      //     retune.
      //   - It never crosses the line from "less margin" into "into the
      //     obstacle": the floor is voxel_half_diagonal, below which the region
      //     would contain points inside an occupied voxel's actual volume rather
      //     than merely close to it. That floor is geometry, not taste, which is
      //     why there is no tunable minimum here.
      // Deliberately NOT applied to later regions: the whole point of the split
      // above is that leniency stops at start_relax_dist.
      double shrink = pull_in;
      if (s == 0 && p.start_relax_dist > 0.0) {
        constexpr double kBoundarySlack = 1e-3;  // keep the QP off an exact face
        double slack = std::numeric_limits<double>::infinity();
        for (int r = 0; r < raw.A.rows(); ++r) {
          slack = std::min(slack, raw.b(r) - raw.A.row(r).dot(resampled_out.front()));
        }
        shrink = std::min(pull_in, slack - kBoundarySlack);
        if (shrink < p.voxel_half_diagonal) shrink = p.voxel_half_diagonal;
      }
      if (s == 0 && start_margin) *start_margin = shrink - p.voxel_half_diagonal;

      ConvexRegion region = shrunkBy(raw, shrink);
      if (attempt) {
        attempt->raw.push_back(raw);
        attempt->shrunk.push_back(region);
      }
      raws.push_back(std::move(raw));
      regions_out.push_back(std::move(region));
    }

    // Validate what the shrink may have destroyed, checking exactly what the QP
    // pins — no more. The start and goal positions are equality-constrained, so
    // they must lie in the first/last region. Interior junction positions are
    // NOT pinned to the waypoints (the trajectory is free within the corridor),
    // so requiring waypoints inside the shrunk regions would reintroduce the
    // "path barely clears, corridor fails" mode this rewrite removes; what C0
    // continuity actually needs is a non-empty INTERSECTION of each consecutive
    // pair, wherever it lies. Checked exactly, as the deepest ball inside both
    // (regionOverlapDepth). This replaced sampling 11 points on the lines from the
    // junction waypoint to the two segment midpoints, which missed any overlap
    // off those lines: on the bench (2026-09-17) it rejected two regions sharing
    // a 0.82 m-radius ball because the junction itself sat 9 cm outside the
    // shrunk first region.
    // With the adaptive shrink above, the first region can only miss the drone if
    // it was already outside the UNSHRUNK region or within half a voxel of it —
    // i.e. the conservative map says the vehicle is in, or touching, an occupied
    // or unknown cell. That is a different fault from a margin that was merely
    // too greedy, and needs a different response (look at the map or the state
    // estimate, not at CORRIDOR_MARGIN), so it says so.
    if (!regions_out.front().contains(resampled_out.front())) {
      return fail(p.start_relax_dist > 0.0
                      ? "the drone's position is inside (or within half a voxel of) an occupied "
                        "or unknown cell on the conservative map"
                      : "margin shrink pushed the first region past the start position");
    }
    // The end, unlike the start, is free to move. Whatever sits there, a truncation
    // cut or a projected goal, is placed right at a clearance limit, and DecompUtil
    // puts every face THROUGH an obstacle point: the face separating the end from
    // its nearest obstacle is closer than that obstacle, and square-on only by
    // luck. The shrink needs margin + voxel_half_diagonal against that face, so an
    // end sitting ~0.5 m from something is shrunk out of its own region almost
    // every time. Rather than refuse the corridor, walk the end back along the path
    // until the shrunk region holds it, dropping trailing regions that hold none of
    // their segment. Full margin is kept everywhere; the vehicle only stops a
    // little earlier, and the next cycle pushes the end forward again. A remaining
    // piece shorter than kMinEndPiece counts as holding none of its segment: a
    // near-zero segment gives the time allocation a degenerate T. Runs before the
    // overlap check, so regions dropped here are never judged.
    constexpr double kPullbackStep = 0.02;  // walk resolution [m]
    constexpr double kMinEndPiece = 0.10;   // shortest last segment kept [m]
    constexpr double kEndSlack = 1e-3;      // keep the pinned end off an exact face
    while (!regions_out.back().contains(resampled_out.back())) {
      const size_t k = regions_out.size() - 1;  // spans resampled_out[k] .. [k + 1]
      const Eigen::Vector3d a = resampled_out[k];
      const Eigen::Vector3d b = resampled_out[k + 1];
      const double L = (b - a).norm();
      bool found = false;
      for (double back = kPullbackStep; L - back >= kMinEndPiece; back += kPullbackStep) {
        const Eigen::Vector3d q = b + (back / L) * (a - b);
        if (regions_out[k].contains(q, -kEndSlack)) {
          resampled_out[k + 1] = q;
          pulled_back += back;
          found = true;
          break;
        }
      }
      if (found) break;
      if (k == 0) {
        std::ostringstream os;
        os << "no point of the path at least " << kMinEndPiece
           << " m from the start fits inside the shrunk corridor";
        return fail(os.str());
      }
      regions_out.pop_back();
      raws.pop_back();
      resampled_out.pop_back();
      origin.pop_back();
      pulled_back += L;
    }

    int split_at = -1;  // base index of the joint to split around, if any
    for (size_t s = 0; s + 1 < regions_out.size();) {
      const double depth = regionOverlapDepth(regions_out[s], regions_out[s + 1]);
      if (depth >= kMinRegionOverlap) {
        ++s;
        continue;
      }

      bool bridged = false;
      Eigen::Vector3d c;
      const double raw_depth = regionOverlapDepth(raws[s], raws[s + 1], &c);
      // The bridge segment must lie in free space for DecompUtil to grow around
      // it. The ball of radius raw_depth about c is inside both unshrunk regions,
      // hence obstacle-free, so a segment of half-length <= raw_depth / 2 is too.
      if (p.bridge_joints && std::isfinite(raw_depth) && raw_depth > kMinRegionOverlap) {
        // Along the direction of travel through the joint, so the bridge is
        // elongated the way the trajectory passes, not across it.
        Eigen::Vector3d dir = resampled_out[s + 2] - resampled_out[s];
        if (dir.norm() < 1e-9) dir = resampled_out[s + 1] - resampled_out[s];
        if (dir.norm() < 1e-9) dir = Eigen::Vector3d::UnitX();
        dir.normalize();
        // The deepest point is often not unique — a squeeze uniform across the
        // path has a whole line of them — and the LP returns whichever vertex it
        // lands on, which can be metres to the side. Slide it back toward the
        // joint waypoint as far as keeps all but kBridgeDepthGive of the depth.
        // The ball radius min_i(b_i - n_i.x) is concave along the line, so the
        // points that keep it form one interval ending at c: bisect for its start.
        {
          const Eigen::Vector3d w = resampled_out[s + 1];
          const auto ball = [&](const Eigen::Vector3d& x) {
            double r = std::numeric_limits<double>::infinity();
            for (const ConvexRegion* q : {&raws[s], &raws[s + 1]}) {
              for (int f = 0; f < q->A.rows(); ++f) r = std::min(r, q->b(f) - q->A.row(f).dot(x));
            }
            return r;
          };
          const double want = raw_depth - kBridgeDepthGive;
          if (ball(w) >= want) {
            c = w;
          } else {
            double lo = 0.0, hi = 1.0;  // ball(w + lo (c - w)) < want <= ball(... hi ...)
            for (int it = 0; it < 30; ++it) {
              const double mid = 0.5 * (lo + hi);
              (ball(w + mid * (c - w)) >= want ? hi : lo) = mid;
            }
            c = w + hi * (c - w);
          }
        }
        const double h = std::min(kBridgeHalfLen, 0.5 * (raw_depth - kBridgeDepthGive));
        const Eigen::Vector3d ba = c - h * dir;
        const Eigen::Vector3d bb = c + h * dir;
        ConvexRegion braw;
        if (growRaw(ba, bb, braw)) {
          ConvexRegion bridge = shrunkBy(braw, pull_in);
          if (regionOverlapDepth(regions_out[s], bridge) >= kMinRegionOverlap &&
              regionOverlapDepth(bridge, regions_out[s + 1]) >= kMinRegionOverlap) {
            // Region s now ends at ba, the bridge spans ba..bb, and region s+1
            // starts at bb.
            resampled_out[s + 1] = ba;
            resampled_out.insert(resampled_out.begin() + s + 2, bb);
            origin[s + 1] = -1;
            origin.insert(origin.begin() + s + 2, -1);
            if (attempt) {
              attempt->raw.push_back(braw);
              attempt->shrunk.push_back(bridge);
            }
            raws.insert(raws.begin() + s + 1, std::move(braw));
            regions_out.insert(regions_out.begin() + s + 1, std::move(bridge));
            ++bridges;
            bridged = true;
            // Don't advance: the loop re-checks both joints the bridge made with
            // the same test every other joint passes, so a corridor can never be
            // accepted on the strength of the check above alone.
          }
        }
      }
      if (bridged) continue;

      std::ostringstream os;
      os << "regions " << s << " and " << s + 1 << " stopped overlapping after the margin shrink ("
         << (std::isfinite(depth) ? "largest ball inside both " + std::to_string(depth) + " m"
                                  : std::string("overlap solve failed"))
         << ", need " << kMinRegionOverlap << " m";
      if (p.bridge_joints) os << "; a bridge region did not fix it";
      if (split_rounds > 0) os << "; after " << split_rounds << " split round(s)";
      os << ")";
      if (split_rounds < kMaxSplitRounds && origin[s + 1] >= 0) {
        split_at = origin[s + 1];
        break;
      }
      return fail(os.str());
    }
    if (split_at < 0) break;  // every joint overlaps

    // Halve the base segments either side of the joint, later one first so the
    // earlier insertion index stays valid. A piece shorter than kMinHalfPiece is
    // a sliver the time allocation handles badly, so such a segment is left
    // whole; if neither can be split there is nothing left to try.
    bool split_any = false;
    const size_t j = static_cast<size_t>(split_at);
    if (j + 1 < base.size() && (base[j + 1] - base[j]).norm() >= 2.0 * kMinHalfPiece) {
      base.insert(base.begin() + j + 1, 0.5 * (base[j] + base[j + 1]));
      split_any = true;
    }
    if (j > 0 && (base[j] - base[j - 1]).norm() >= 2.0 * kMinHalfPiece) {
      base.insert(base.begin() + j, 0.5 * (base[j - 1] + base[j]));
      split_any = true;
    }
    if (!split_any) {
      return fail("regions around waypoint " + std::to_string(j) +
                  " stopped overlapping after the margin shrink and their segments are too "
                  "short to split");
    }
    ++split_rounds;
  }

  if (end_pullback) *end_pullback = pulled_back;
  if (repairs) {
    repairs->bridges = bridges;
    repairs->split_rounds = split_rounds;
  }
  return true;  // regions_out.size() == resampled_out.size() - 1
}

}  // namespace drone_core::planning
