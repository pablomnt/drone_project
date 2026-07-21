#pragma once

#include <functional>
#include <vector>

#include <Eigen/Dense>

namespace drone_core::planning {

// Clearance oracle for corridor generation: distance [m] from a world point to
// the nearest obstacle. Deliberately the same shape as GeometricPlanner's
// ClearanceFn so the host can hand the corridor generator the conservative
// (frontier-stamped) EDT — or a test an analytic function — without the
// generator knowing anything about octomap.
using CorridorClearanceFn = std::function<double(double x, double y, double z)>;

// Axis-aligned free box: a convex region the trajectory may occupy. Each box
// becomes six linear half-space rows (lo <= p <= hi per axis) in the corridor
// QP.
struct Box {
  Eigen::Vector3d lo;
  Eigen::Vector3d hi;

  bool contains(const Eigen::Vector3d& p, double tol = 0.0) const {
    return (p.array() >= lo.array() - tol).all() && (p.array() <= hi.array() + tol).all();
  }
};

// Tunables for corridor construction.
struct CorridorParams {
  double max_segment_len = 2.0;  // resample cap: no segment longer than this [m]
  double step = 0.10;            // box face growth increment / free-test grid [m]
  double margin = 0.5;           // required clearance for a point to count as free [m]
  double max_box_growth = 3.0;   // cap on how far any face grows from its seed [m]
};

// Subdivide any path segment longer than max_segment_len into equal pieces so
// every output segment respects the cap. Keeps the original waypoints; never
// produces consecutive duplicates. A degenerate input (<2 points, cap <= 0)
// passes through unchanged.
std::vector<Eigen::Vector3d> resamplePath(const std::vector<Eigen::Vector3d>& path,
                                          double max_segment_len);

// Grow a maximal free axis-aligned box around segment [a, b]: seed at the
// segment's tight AABB, then push each of the six faces outward in `step`
// increments while the newly swept slab stays free (every grid sample has
// clearance >= margin) and within max_box_growth. Returns false when even the
// seed AABB clips an obstacle (no free box exists around this segment). The
// resulting box always contains both endpoints.
bool growBox(const CorridorClearanceFn& clearance, const Eigen::Vector3d& a,
             const Eigen::Vector3d& b, const CorridorParams& p, Box& box_out);

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
// forward as the map grows. Near the start the required clearance ramps up with
// distance travelled — required = min(margin, distance from start) — serving
// the planner's start-escape purpose (a drone parked near the mapped floor or
// inside the frontier keep-out can still root a path) without a hard sphere's
// dead band; clearance must always be strictly positive. The result is the safe
// prefix: the original waypoints passed, plus the last safe sampled point as
// its endpoint. A result with fewer than 2 points means nothing of the path is
// safely committable (the caller falls back / holds).
std::vector<Eigen::Vector3d> truncatePath(const CorridorClearanceFn& conservative_clearance,
                                          const std::vector<Eigen::Vector3d>& path,
                                          double margin, double sample_step = 0.05);

// Full corridor for a path: resample to the segment cap, then grow one free box
// per segment. Consecutive boxes overlap by construction (each contains its
// segment's endpoints, and adjacent segments share one). On success
// boxes_out.size() == resampled_out.size() - 1; on any un-growable segment both
// outputs are cleared and false is returned (the caller falls back).
bool buildCorridor(const CorridorClearanceFn& clearance,
                   const std::vector<Eigen::Vector3d>& path,
                   const CorridorParams& p,
                   std::vector<Eigen::Vector3d>& resampled_out,
                   std::vector<Box>& boxes_out);

}  // namespace drone_core::planning
