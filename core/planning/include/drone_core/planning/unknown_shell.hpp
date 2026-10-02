#pragma once

#include <cstddef>

#include <octomap/OcTree.h>

namespace drone_core::planning {

struct UnknownShellStats {
  std::size_t ball_freed = 0;   // never-observed voxels inside the ball, marked free
  std::size_t free_leaves = 0;  // known-free leaves swept
  std::size_t stamped = 0;      // shell voxels stamped occupied
  // Wall time of each stage [ms]: freeing the ball, filling the grid from the
  // leaves (allocation included), the neighbour sweep, and writing the shell
  // into the tree (inner-node refresh included).
  double ball_ms = 0.0;
  double grid_ms = 0.0;
  double sweep_ms = 0.0;
  double stamp_ms = 0.0;
};

// Walls off everything the sensors have never observed, in place:
//   1. every never-observed voxel within `keep_out_radius` of `center` (the
//      drone) is marked free, so the drone is never boxed in by the unobserved
//      space right beside it (behind it, below it) and the shell wraps around
//      that ball instead of leaving a hole in it;
//   2. every never-observed voxel touching a known-free voxel (26-neighbourhood)
//      is stamped occupied.
// The result answers every distance and corridor question exactly as if the
// whole unobserved volume were an obstacle: the nearest unobserved voxel to any
// free point is either behind an obstacle (which is nearer) or touches free
// space, i.e. is in the shell. Voxels already known (free or occupied) are never
// changed outside the ball, and occupied ones never inside it either.
UnknownShellStats stampUnknownShell(octomap::OcTree& tree, const octomap::point3d& center,
                                    double keep_out_radius);

}  // namespace drone_core::planning
