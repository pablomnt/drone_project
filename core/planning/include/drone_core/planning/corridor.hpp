#pragma once

#include <functional>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace drone_core::planning {

// Clearance oracle for corridor generation: distance [m] from a world point to
// the nearest obstacle. Deliberately the same shape as GeometricPlanner's
// ClearanceFn so the host can hand the corridor generator the conservative
// (frontier-stamped) EDT — or a test an analytic function — without the
// generator knowing anything about octomap.
using CorridorClearanceFn = std::function<double(double x, double y, double z)>;

// Convex free region: the space one trajectory segment may occupy, as the
// half-space intersection A·p <= b (one row per face, row normals unit-length
// so b carries metric distance). General polyhedra rather than axis-aligned
// boxes: a box seeded on a diagonal segment's AABB is mostly volume the path
// never visits, and it fails on geometry the drone would never approach —
// polyhedra grown along the path (DecompUtil's ellipsoid inflation) cut at the
// obstacles that actually bind, recovering the free volume next to complex
// nearby geometry. Plain Eigen data on purpose: DecompUtil types never cross
// this public header.
struct ConvexRegion {
  Eigen::Matrix<double, Eigen::Dynamic, 3> A;
  Eigen::VectorXd b;

  bool contains(const Eigen::Vector3d& p, double tol = 0.0) const {
    return A.rows() == 0 || ((A * p - b).array() <= tol).all();
  }
};

// Tunables for corridor construction.
struct CorridorParams {
  double max_segment_len = 2.0;  // resample cap: no segment longer than this [m]
  double margin = 0.5;           // clearance every region keeps from obstacle points [m]
  // Minimum USABLE half-extents of the region-growth window, in the
  // SEGMENT-ALIGNED frame — component 0 is slack along the segment beyond its
  // endpoints, 1 and 2 are lateral — not world axes. Treated as a floor, not a
  // literal size: buildCorridor raises it to scale with the longest segment
  // (lateral >= segment length, along-track >= half of it, so consecutive
  // regions overlap generously) and then adds the margin shrink on top, so the
  // window planes never eat into the volume the trajectory can actually use.
  // Raise this to let regions grow wider than the segments themselves; note
  // that a wider window also means more obstacle points per decomposition.
  Eigen::Vector3d local_bbox{0.0, 0.0, 0.0};
  // Half-diagonal of an occupancy voxel [m]: obstacle points are voxel
  // centres, so faces are pushed this much further in addition to `margin`
  // for the margin to hold against the voxel's worst-case corner. Set from
  // the map resolution (res * sqrt(3) / 2); the default matches 0.05 m.
  double voxel_half_diagonal = 0.0433;
  // Distance from the drone over which the FIRST region's margin may be
  // relaxed [m] — the corridor's counterpart to truncatePath's escape ramp and
  // the planner's start-escape sphere, and the reason a drone parked close to
  // the frontier can get a corridor at all.
  //
  // A convex region cannot be less safe at one end than the other: one plane
  // holds everywhere, so the ramp truncation applies pointwise has no direct
  // analogue here. The relaxation is instead bounded in the two ways that are
  // available. In EXTENT: the first segment is split at this distance, so
  // whatever margin is given up is given up over the first `start_relax_dist`
  // metres only and every later region carries the full `margin`. In MAGNITUDE:
  // the first region is shrunk by the largest amount that still contains the
  // drone (see buildCorridor), never by less than it has to be, so the
  // relaxation disappears on its own as the map fills in around the vehicle.
  //
  // Set <= 0 to disable both — no split, uniform `margin` everywhere, and a
  // drone closer than `margin` to anything mapped or unknown gets no corridor.
  double start_relax_dist = 1.0;
};

// Vertex loops of a region's faces, for visualisation: one entry per face that
// has at least three vertices, each an ordered ring of that face's corners
// (angularly sorted about the face centroid, so drawing it as a closed line
// strip traces the face outline). Vertices are enumerated by intersecting every
// triple of faces and keeping the points that satisfy all the others, which is
// O(faces^3) — fine for the ~10-20 faces a corridor region carries, at debug
// visualisation rates, but not something to call on the flight path.
std::vector<std::vector<Eigen::Vector3d>> regionFaceLoops(const ConvexRegion& region);

// Subdivide any path segment longer than max_segment_len into equal pieces so
// every output segment respects the cap. Keeps the original waypoints; never
// produces consecutive duplicates. A degenerate input (<2 points, cap <= 0)
// passes through unchanged.
std::vector<Eigen::Vector3d> resamplePath(const std::vector<Eigen::Vector3d>& path,
                                          double max_segment_len);


// Truncate a (possibly optimistically planned) path to its safe committed
// prefix: walk it from the start outward, sampling finely, and cut at the first
// point whose clearance under the CONSERVATIVE oracle (frontier stamped as
// occupied, so distance = min(dist to obstacle, dist to unknown)) drops below
// `margin`. Pass max(frontier_margin, collision_margin) as the margin: the
// conservative distance is a lower bound on both hazard distances, so one walk
// enforces the frontier margin against unknown space and (at worst
// over-conservatively) the collision margin against mapped obstacles. This is
// what keeps a best-effort path — which may run through unexplored space — from
// committing the vehicle beyond the mapped frontier; the endpoint ratchets
// forward as the map grows.
//
// Near the start the required clearance RAMPS UP with distance travelled:
//   required = margin * min(1, distance_from_start / escape_ramp)
// reaching the full margin only at `escape_ramp` metres out. This serves the
// planner's start-escape purpose — a drone parked near the mapped floor, or
// sitting in a small pocket of known-free space, can still root a path — without
// a hard sphere's dead band. `escape_ramp` is deliberately decoupled from
// `margin`: tying the two (ramping to full margin over `margin` metres) makes
// the ramp steeper exactly when the margin is large, and on a thinly-mapped
// scene the rising requirement meets the shrinking clearance within
// centimetres, truncating the path to nothing. A longer ramp commits further
// before demanding full clearance. Pass <= 0 to disable the ramp (full margin
// everywhere). Clearance must always be strictly positive regardless, so the
// prefix can never enter an occupied or unknown voxel.
//
// The result is the safe prefix: the original waypoints passed, plus the last
// safe sampled point as its endpoint. A result with fewer than 2 points means
// nothing of the path is safely committable (the caller falls back / holds).
std::vector<Eigen::Vector3d> truncatePath(const CorridorClearanceFn& conservative_clearance,
                                          const std::vector<Eigen::Vector3d>& path,
                                          double margin, double escape_ramp = 1.0,
                                          double sample_step = 0.05);

// Full corridor for a path: resample to the segment cap, then grow one convex
// free region per segment via DecompUtil's ellipsoid decomposition against the
// given obstacle points (occupied + frontier-stamped voxel centres of the
// conservative map, pre-windowed by the caller — this function scans the whole
// list per segment). Each region is shrunk by margin + voxel_half_diagonal
// along every face normal so the trajectory it confines keeps true metric
// clearance from the voxels themselves — except the FIRST, which is shrunk by
// as much of that as still contains the drone (see start_relax_dist, and
// `start_margin` below for what it ended up guaranteeing). The result is then
// validated against exactly what the QP pins: the start and goal positions are
// equality-constrained, so each must lie in its region, and consecutive regions
// must still share a point for the C0 handover (which shrinking can empty).
// On any violated region both outputs are cleared and false is returned (the
// caller falls back).
// On success regions_out.size() == resampled_out.size() - 1.
// `reason`, if given, receives a short human-readable explanation on failure
// (which check rejected the corridor) — the caller's one-line diagnostic
// otherwise cannot distinguish "the margin collapsed a region" from "two
// regions stopped overlapping", which need different responses.
//
// `attempt`, if given, receives the geometry as it was built REGARDLESS of
// whether the corridor is then accepted — the primary outputs are still
// cleared on failure so a rejected corridor can never be flown, but a rejected
// corridor is exactly what you want to look at. Holding both the raw and the
// shrunk regions makes the common failure self-evident on sight: if `raw` has
// volume and `shrunk` has none, the margin ate the region.
struct CorridorAttempt {
  std::vector<Eigen::Vector3d> resampled;  // segments the decomposition ran on
  std::vector<ConvexRegion> raw;           // as DecompUtil built them, no margin applied
  std::vector<ConvexRegion> shrunk;        // after the margin + voxel pull-in
};

// `start_margin`, if given, receives the clearance the FIRST region actually
// guarantees [m] — normally `margin`, but less when the drone sits too close to
// something mapped or unknown for the full shrink to contain it (see
// CorridorParams::start_relax_dist). Reported unconditionally rather than only
// under debug viz: a corridor that succeeded by giving up margin around the
// vehicle is not the same event as one that did not, and the difference must
// not be invisible in flight. Untouched on failure.
bool buildCorridor(const std::vector<Eigen::Vector3d>& obstacles,
                   const std::vector<Eigen::Vector3d>& path,
                   const CorridorParams& p,
                   std::vector<Eigen::Vector3d>& resampled_out,
                   std::vector<ConvexRegion>& regions_out,
                   std::string* reason = nullptr,
                   CorridorAttempt* attempt = nullptr,
                   double* start_margin = nullptr);

// Upper bound on how far from the committed path a region can reach, given the
// same params — i.e. how wide the obstacle window the caller extracts must be
// for the decomposition to see everything that could bound a region. Keeps the
// caller's windowing in step with buildCorridor's internal bbox sizing.
double corridorObstacleWindowPad(const CorridorParams& p);

}  // namespace drone_core::planning
