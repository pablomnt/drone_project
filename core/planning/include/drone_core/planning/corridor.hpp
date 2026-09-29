#pragma once

#include <functional>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "drone_core/common/types.hpp"

namespace drone_core::planning {

// Clearance oracle for corridor generation: distance [m] from a world point to
// the nearest obstacle. Deliberately the same shape as GeometricPlanner's
// ClearanceFn so the host can hand the corridor generator the conservative
// (frontier-stamped) EDT — or a test an analytic function — without the
// generator knowing anything about octomap.
using CorridorClearanceFn = std::function<double(double x, double y, double z)>;

// True when a world point lies in space that has never been observed. Same shape
// as GeometricPlanner's UnknownFn, and for the same reason: the host wires it to
// an octree lookup, a test to an analytic predicate, and neither the corridor
// stages nor the planner need to know which.
using CorridorUnknownFn = std::function<bool(double x, double y, double z)>;

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
  // the planner's validity ramp, and the reason a drone parked close to
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

  // Repair joints whose regions stop overlapping once shrunk by growing an extra
  // "bridge" region centred in the squeeze between them (see buildCorridor).
  // Its two ends replace the joint's waypoint, so callers that PIN interior
  // waypoints (presets) must turn this off: the bridge ends lie off their path.
  // Splitting a failing joint's segments in half is always allowed, since the
  // midpoints stay on the path.
  bool bridge_joints = true;

  // Optional box every region is clipped to (world axes, after the margin
  // shrink), so the trajectory cannot leave it: the search plans inside
  // GeometricPlanner::kSearchLow/High, and a trajectory that left that space by
  // the width of its corridor could strand the vehicle outside the box the
  // search can start from. Unclipped by default (infinite). The box is grown to
  // contain every waypoint of the path handed to buildCorridor — the start may
  // sit slightly outside it (a splice point, or a vehicle already out), and a
  // region that excluded it would make the start equality infeasible.
  Eigen::Vector3d bounds_lo = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
  Eigen::Vector3d bounds_hi = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
};

// What buildCorridor had to do to make consecutive regions overlap.
struct CorridorRepairs {
  int bridges = 0;       // bridge regions spliced in
  int split_rounds = 0;  // times segments around a failing joint were halved and rebuilt
};

// Vertex loops of a region's faces, for visualisation: one entry per face that
// has at least three vertices, each an ordered ring of that face's corners
// (angularly sorted about the face centroid, so drawing it as a closed line
// strip traces the face outline). Vertices are enumerated by intersecting every
// triple of faces and keeping the points that satisfy all the others, which is
// O(faces^3) — fine for the ~10-20 faces a corridor region carries, at debug
// visualisation rates, but not something to call on the flight path.
std::vector<std::vector<Eigen::Vector3d>> regionFaceLoops(const ConvexRegion& region);

// How deeply two regions overlap: the radius [m] of the largest ball inside
// both, from a small exact LP (maximise r subject to n·x + |n| r <= b over the
// faces of both regions). Negative when the regions are disjoint, by roughly how
// far apart they are; zero when they only touch. Exact rather than sampled, so it
// finds an overlap wherever it lies — the C0 handover between consecutive
// corridor segments only needs the junction somewhere in the intersection, not
// near the waypoint. A region with no faces is unbounded and overlaps anything
// (+infinity); a solve that fails returns -infinity, the safe side for a caller
// reading this as overlap depth. Microseconds for corridor-sized regions.
// `center`, if given, receives the centre of that ball — the deepest point of
// the intersection — whenever the returned depth is finite; untouched otherwise.
double regionOverlapDepth(const ConvexRegion& a, const ConvexRegion& b,
                          Eigen::Vector3d* center = nullptr);

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
// The ramp is floored at the start's own clearance less a tolerance, and the
// result capped at `margin`:
//   floor    = clearance(start) - max(start_floor_slack, start_floor_rel * clearance(start))
//   required = min(margin, max(floor, ramp))
// so leniency near the start only ever lets the drone move AWAY from what it is
// already too close to, never (materially) closer. Without the floor a start
// 0.2 m from a wall could head straight at it until clearance fell below the
// rising ramp. The tolerance is whichever of the absolute and the relative one
// is more lenient. The absolute one absorbs the field's quantisation: the host
// passes one voxel, since the drone's reading can jump by more than half a
// voxel when it moves a single cell (bench 2026-09-28: 0.5 vs 0.4743 on
// alternate ticks, cutting the path at 0.25 m then committing it whole). The
// relative one keeps truncation looser than the planner's own floor by the same
// fraction as its margin (the host passes kTruncationTolerance), so a path the
// search accepted is not cut near the drone.
//
// `is_unknown`, when supplied, is an ABSOLUTE stop: the prefix is cut before the
// first sample lying in space that has never been observed, whatever the
// clearance there says and regardless of the escape ramp. This is a genuine hole
// in the clearance test rather than a refinement of it. The conservative field
// measures distance to the stamped frontier shell, so it only reports danger
// within its saturation distance of that shell — and the shell is not reliably
// closed (the sensor's field of view leaves gaps, and the host deliberately
// leaves an unstamped ball around the vehicle). A path leaving mapped space
// through such a gap sees high clearance the whole way and is committed in full.
// Asking the octree whether a cell exists closes that: unobserved is unobserved,
// with no dependence on the shell's geometry, and it is correct outside the
// map's bounding box too. Pass an empty function to disable (clearance-only
// behaviour).
//
// The result is the safe prefix: the original waypoints passed, plus the last
// safe sampled point as its endpoint. A result with fewer than 2 points means
// nothing of the path is safely committable (the caller falls back / holds).
//
// `cut`, when non-null, is filled with where and why the walk stopped: the first
// unsafe sample, its straight-line distance from the start, and either the
// clearance there against the ramped requirement or that it lay in unobserved
// space. `cut->cut` is false when the whole path was safe. Diagnostic only.
struct TruncationCut {
  bool cut = false;
  bool unknown = false;           // stopped by is_unknown, not by clearance
  Eigen::Vector3d point{0, 0, 0};
  double from_start = 0.0;        // straight-line distance from path.front() [m]
  double clearance = 0.0;         // conservative clearance at `point` [m]
  double required = 0.0;          // ramped, floored margin required at `point` [m]
  double root_clearance = 0.0;    // conservative clearance at path.front() [m]
  double floor = 0.0;             // the floor that root clearance gave [m]
};
std::vector<Eigen::Vector3d> truncatePath(const CorridorClearanceFn& conservative_clearance,
                                          const std::vector<Eigen::Vector3d>& path,
                                          double margin, double escape_ramp = 1.0,
                                          double sample_step = 0.05,
                                          const CorridorUnknownFn& is_unknown = {},
                                          TruncationCut* cut = nullptr,
                                          double start_floor_slack = 0.0,
                                          double start_floor_rel = 0.0);

// Re-check a trajectory that is already being flown (or about to be) against
// the CURRENT map, for the trajectory monitor. A trajectory is proved safe only
// against the map it was built on; this is what notices that a newly observed
// obstacle, or the frontier, has since come too close to it.
//
// Holds the trajectory to the clearance it was BUILT with, not to truncation's
// numbers: the corridor keeps `margin` (CORRIDOR_MARGIN) everywhere except the
// first segment, where buildCorridor may have relaxed it to `start_margin` for a
// hemmed-in start. Truncation's FRONTIER_MARGIN would flag every fresh
// trajectory, since the trajectory is only built to keep CORRIDOR_MARGIN and cuts
// corners off the checked path. Each sample must clear its margin less the more
// lenient of `slack` and `rel` x margin; with those at least half a sample step,
// a trajectory straight out of the corridor QP passes by construction (the region
// shrink leaves it >= margin from every voxel centre, which is what the field
// measures).
//
// Samples every `sample_step` metres of travel at most — the step in time is
// sample_step / max_speed — and subtracts half a step from each clearance: the
// field is 1-Lipschitz, so the curve between two samples can be at most that
// much closer than the samples show. The check is therefore exact, not
// probabilistic.
struct TrajectoryCheckParams {
  double margin = 0.4;             // CORRIDOR_MARGIN [m]
  double start_margin = 0.4;       // the relaxed first-segment margin buildCorridor reported [m]
  double first_segment_end = 0.0;  // wall-clock end of the first segment [s]
  double slack = 0.05;             // absolute tolerance [m]
  double rel = 0.05;               // relative tolerance, fraction of the margin
  double sample_step = 0.05;       // max travel between samples [m]
  double max_speed = 1.0;          // bound on the trajectory's speed [m/s], sets the time step
  double emergency_horizon = 2.0;  // how far ahead of `t_from` an emergency is looked for [s]
  double emergency_factor = 0.7;   // emergency below this fraction of the (untolerated) margin
};
struct TrajectoryCheck {
  bool ok = true;
  // Some sample within emergency_horizon of `t_from` is below emergency_factor x
  // its margin, in contact, or in never-observed space: too close to wait for a
  // replacement trajectory.
  bool emergency = false;
  bool unknown = false;          // the worst sample lies in never-observed space
  double worst_time = 0.0;       // wall-clock time of the worst sample [s]
  double worst_clearance = 0.0;  // its clearance, less the half-step [m]
  double worst_required = 0.0;   // what it had to clear [m]
  Eigen::Vector3d worst_point{0, 0, 0};  // in the field's frame
};
// Checks `traj` from wall-clock `t_from` (clamped to its start) until
// `t_until` or its end, whichever is first; pass t_until <= t_from for "to the
// end". `field_from_traj` takes the trajectory's frame into the field's (the
// monitor flies world-frame trajectories against map-frame fields). `is_unknown`
// may be empty.
TrajectoryCheck checkTrajectory(const CorridorClearanceFn& clearance,
                                const CorridorUnknownFn& is_unknown,
                                const common::Trajectory& traj,
                                const Eigen::Isometry3d& field_from_traj, double t_from,
                                double t_until, const TrajectoryCheckParams& params);

// Full corridor for a path: resample to the segment cap, then grow one convex
// free region per segment via DecompUtil's ellipsoid decomposition against the
// given obstacle points (occupied + frontier-stamped voxel centres of the
// conservative map, pre-windowed by the caller — this function scans the whole
// list per segment). Each region is shrunk by margin + voxel_half_diagonal
// along every face normal so the trajectory it confines keeps true metric
// clearance from the voxels themselves — except the FIRST, which is shrunk by
// as much of that as still contains the drone (see start_relax_dist, and
// `start_margin` below for what it ended up guaranteeing). The result is then
// validated against exactly what the QP pins: the start and end positions are
// equality-constrained, so each must lie in its region, and consecutive regions
// must still share a point for the C0 handover (which shrinking can empty). A
// joint that fails that is repaired where possible — a bridge region grown in
// the squeeze, then halving the segments around it and rebuilding (at most
// twice) — so regions_out can hold MORE regions than the resampled path had
// segments, with resampled_out extended to match. An
// end the shrink excludes is not a failure: it is walked back along the path
// until the shrunk region holds it, dropping trailing regions that hold none of
// their segment, so resampled_out.back() may lie short of path.back() (see
// `end_pullback`). Only if nothing beyond the start fits is the corridor refused.
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
//
// `end_pullback`, if given, receives how far along the path the end was moved
// back to fit the shrunk corridor [m], 0 when it fit as given. Reported for the
// same reason: a trajectory that stops short of where it was sent must say so.
// Untouched on failure.
//
// `repairs`, if given, receives how many joints needed a bridge region and how
// many split-and-rebuild rounds ran (see CorridorParams::bridge_joints). A
// repaired corridor is as safe as any other — every joint still passes the
// same overlap test — but it says the path runs through a squeeze.
bool buildCorridor(const std::vector<Eigen::Vector3d>& obstacles,
                   const std::vector<Eigen::Vector3d>& path,
                   const CorridorParams& p,
                   std::vector<Eigen::Vector3d>& resampled_out,
                   std::vector<ConvexRegion>& regions_out,
                   std::string* reason = nullptr,
                   CorridorAttempt* attempt = nullptr,
                   double* start_margin = nullptr,
                   double* end_pullback = nullptr,
                   CorridorRepairs* repairs = nullptr);

// Upper bound on how far from the committed path a region can reach, given the
// same params — i.e. how wide the obstacle window the caller extracts must be
// for the decomposition to see everything that could bound a region. Keeps the
// caller's windowing in step with buildCorridor's internal bbox sizing.
double corridorObstacleWindowPad(const CorridorParams& p);

}  // namespace drone_core::planning
