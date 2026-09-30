#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <octomap/OcTree.h>

namespace drone_core::planning {

// The conservative view of the map, as a dense grid of one byte per voxel over
// (a crop of) the raw octree's box. It holds everything the conservative
// consumers need — obstacles for the distance field and the corridor, and the
// "never observed" test — without building a stamped octree: that meant copying
// the whole map and writing every shell voxel into the copy one at a time, the
// two costs that grow fastest with the map.
//
// Built in three steps:
//   1. fill the grid from the raw tree's leaves (merged blocks cover every cell
//      they span): free, occupied, or never observed where the tree has no node;
//   2. mark never-observed cells within `keep_out_radius` of `center` (the
//      drone) as free, so the drone is never boxed in by what it cannot see
//      beside it and the shell wraps around that ball;
//   3. mark every never-observed cell touching a free one (26-neighbourhood) as
//      shell.
// Obstacles for the distance field and the corridor are occupied + shell cells,
// which answers every distance question exactly as if the whole unobserved
// volume were an obstacle (see stampUnknownShell, the octree version this
// replaces and the reference the tests compare against).
//
// The box: the raw tree's box, grown to hold the ball, plus one cell on every
// side (outside the tree is never observed, so the shell must be able to sit
// there), intersected with [crop_lo, crop_hi] (metres, map frame) — the
// planning box grown by the fields' saturation distance, so obstacles just
// outside it still count while map beyond it costs nothing. Cells outside the
// grid read kUnknown.
//
// Immutable once built; any number of threads may query it concurrently.
class ConservativeGrid {
public:
  enum Cell : std::uint8_t { kUnknown = 0, kOccupied = 1, kFree = 2, kShell = 3 };

  struct Stats {
    std::size_t free_cells = 0;  // free cells swept (ball included)
    std::size_t ball_freed = 0;  // never-observed cells inside the ball, marked free
    std::size_t shell = 0;       // cells marked shell
    // Wall time of each step [ms]: filling the grid (allocation included),
    // freeing the ball, the neighbour sweep.
    double fill_ms = 0.0;
    double ball_ms = 0.0;
    double sweep_ms = 0.0;
  };

  // `shell` false skips steps 2 and 3: a plain mirror of `raw` (used by tests
  // that hand-build a conservative tree). `threads` = 0 uses the hardware
  // concurrency.
  ConservativeGrid(const octomap::OcTree& raw, const octomap::point3d& center,
                   double keep_out_radius, const Eigen::Vector3d& crop_lo,
                   const Eigen::Vector3d& crop_hi, bool shell = true, unsigned threads = 0);

  // The cell containing `p` (same key arithmetic as octomap); kUnknown outside
  // the grid, for NaN, or for coordinates beyond octomap's key range.
  Cell at(const octomap::point3d& p) const;
  Cell at(double x, double y, double z) const;
  // Never observed: kUnknown or kShell (the ball counts as observed).
  bool isUnknown(double x, double y, double z) const;
  static bool isObstacle(Cell c) { return c == kOccupied || c == kShell; }

  // Centres of every occupied or shell cell whose centre lies in [lo, hi]
  // (metres), appended to `out` — the corridor's obstacle list and the
  // occupancy viz.
  void obstaclesIn(const Eigen::Vector3d& lo, const Eigen::Vector3d& hi,
                   std::vector<Eigen::Vector3d>& out) const;

  double resolution() const { return res_; }
  // The box, in octomap keys of cell (0, 0, 0) and in cells; x fastest, then
  // y, then z in data().
  int keyOffset() const { return key_offset_; }
  int keyX0() const { return kx0_; }
  int keyY0() const { return ky0_; }
  int keyZ0() const { return kz0_; }
  int sizeX() const { return nx_; }
  int sizeY() const { return ny_; }
  int sizeZ() const { return nz_; }
  const std::uint8_t* data() const { return cells_.data(); }
  bool empty() const { return cells_.empty(); }

  const Stats& stats() const { return stats_; }

private:
  double res_ = 0.0;
  double inv_res_ = 0.0;
  int key_offset_ = 0;  // octomap's tree_max_val: key = floor(coord / res) + this
  int kx0_ = 0, ky0_ = 0, kz0_ = 0;
  int nx_ = 0, ny_ = 0, nz_ = 0;
  std::vector<std::uint8_t> cells_;
  Stats stats_;
};

}  // namespace drone_core::planning
