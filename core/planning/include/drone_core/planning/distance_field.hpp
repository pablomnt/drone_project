#pragma once

#include <cstdint>
#include <memory>
#include <new>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <octomap/OcTree.h>

#include "drone_core/planning/conservative_grid.hpp"

namespace drone_core::planning {

// Exact Euclidean distance field over the voxel grid of an octree's bounding
// box: for every voxel, the distance from its centre to the nearest occupied
// voxel's centre, capped at `maxdist`. Built from scratch in one go with the
// separable squared-distance transform of Felzenszwalb & Huttenlocher (three
// passes, one per axis, each row independent), which replaces DynamicEDT3D:
// that one is built for incremental repair and pays for a priority queue and
// ~25 bytes of state per cell that a from-scratch build never uses.
//
// Immutable once built, so any number of threads may query it concurrently.
class DistanceField {
public:
  // `threads` = 0 uses the hardware concurrency.
  DistanceField(const octomap::OcTree& tree, double maxdist, unsigned threads = 0);
  // The same, with the box cut down to the cells whose centres lie in
  // [crop_lo, crop_hi] (metres). Obstacles outside the crop are ignored, so the
  // caller grows the crop by maxdist beyond where it will query. An empty
  // intersection gives an empty field (every query -1).
  DistanceField(const octomap::OcTree& tree, double maxdist, unsigned threads,
                const Eigen::Vector3d& crop_lo, const Eigen::Vector3d& crop_hi);
  // Over a conservative grid's box, with its occupied and shell cells as the
  // obstacles. Identical to building from an octree whose occupied voxels are
  // exactly those cells, over the same box.
  DistanceField(const ConservativeGrid& grid, double maxdist, unsigned threads = 0);

  // The box's outer corners [m] (lowest corner of cell (0,0,0), highest corner
  // of the last cell); lo > hi for an empty field.
  Eigen::Vector3d boxMin() const;
  Eigen::Vector3d boxMax() const;

  // Distance [m] from the centre of the voxel containing `p` to the nearest
  // occupied voxel centre, capped at maxDist(); 0 inside an occupied voxel.
  // Negative (-1) outside the box — the same convention as
  // DynamicEDTOctomap::getDistance, so callers' out-of-box handling is unchanged.
  float getDistance(const octomap::point3d& p) const;

  double maxDist() const { return maxdist_; }
  double resolution() const { return res_; }
  // The box, in cells.
  int sizeX() const { return nx_; }
  int sizeY() const { return ny_; }
  int sizeZ() const { return nz_; }

private:
  double res_ = 0.0;
  double inv_res_ = 0.0;
  double maxdist_ = 0.0;
  int key_offset_ = 0;         // octomap's tree_max_val: key = floor(coord / res) + this
  int kx0_ = 0, ky0_ = 0, kz0_ = 0;  // key of cell (0, 0, 0)
  int nx_ = 0, ny_ = 0, nz_ = 0;

  // Squared distance in cells², x fastest, then y, then z. Values are exact
  // below far_; far_ itself means "at least maxdist away (or no obstacle)".
  // Stored as uint16 whenever far_ fits (maxdist up to ~255 cells), which
  // halves memory and pass bandwidth; exactly one of the two is non-empty.
  // Allocator that default-initialises (leaves the cells unwritten): the grid is
  // first written by the build's threads, and value-initialising it would be a
  // serial pass of page faults (~10 ms for 10M cells).
  template <typename T>
  struct NoInitAllocator : std::allocator<T> {
    template <typename U>
    struct rebind { using other = NoInitAllocator<U>; };
    NoInitAllocator() = default;
    template <typename U>
    NoInitAllocator(const NoInitAllocator<U>&) noexcept {}
    template <typename U>
    void construct(U* p) noexcept { ::new (static_cast<void*>(p)) U; }
    template <typename U, typename... Args>
    void construct(U* p, Args&&... args) { ::new (static_cast<void*>(p)) U(std::forward<Args>(args)...); }
  };
  std::uint32_t far_ = 0;
  std::vector<std::uint16_t, NoInitAllocator<std::uint16_t>> sq16_;
  std::vector<std::uint32_t, NoInitAllocator<std::uint32_t>> sq32_;
  std::vector<float> lut_;     // sq16_ only: squared cells -> metres, far_ -> maxdist

  // Sets the box (inclusive keys), far_, and runs the passes; `stamp(geometry,
  // bits)` sets the bits of the occupied cells first. Defined and instantiated
  // in distance_field.cpp only.
  template <typename Stamp>
  void build(const int lo[3], const int hi[3], unsigned threads, Stamp&& stamp);
};

}  // namespace drone_core::planning
