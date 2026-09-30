// DistanceField: the exact Euclidean distance field that replaced DynamicEDT3D.
//
// What is checked:
//   1. Every cell of several small maps against a brute-force ground truth
//      (random clutter at a few densities, merged/pruned occupied leaves, a
//      free-only map, single-voxel and empty trees, maxdist 0, and the 16-bit
//      envelope and 32-bit storage paths that only kick in for large maxdist),
//      plus random points inside voxels, points just outside the box and
//      NaN / infinite / far-away / key-wrapping coordinates (all -1).
//   2. A medium room-like map against DynamicEDT3D: ours never reads more than
//      DynamicEDT3D (capped at maxdist), and they rarely differ at all.
//   3. The same map built on 1, 2, 3, 8 and all threads is bit-identical.
//   4. Eight threads querying one field concurrently get the single-threaded
//      answers.
//   5. Build time of DynamicEDT3D vs ours (informational only).
//   6. Crops: a field built with a crop box equals, on every cell of the crop,
//      brute force over the occupied voxels inside the crop only (merged
//      occupied leaves cut by it included), reads -1 outside it, and is empty
//      when the crop misses the map.
//   7. The ConservativeGrid constructor: equal, bit for bit over the grid's
//      box, to the tree constructor on the raw map with the grid's shell cells
//      stamped occupied (uncropped and cropped), and to brute force; the grid's
//      shell is exactly stampUnknownShell's when uncropped.
//   8. boxMin()/boxMax() are the box's outer corners (lo > hi when empty).
//
// The ground truth is computed from occupancy read back cell by cell through
// octomap's own search(), not from the field's octree walk, so a merged leaf
// the field mis-stamped or a box it mis-sized shows up as a mismatch.
//
// Everything runs in a few seconds in Release.

#include "drone_core/planning/distance_field.hpp"
#include "drone_core/planning/conservative_grid.hpp"
#include "drone_core/planning/unknown_shell.hpp"

#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <octomap/OcTree.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <vector>

namespace {

using drone_core::planning::DistanceField;
using octomap::OcTree;
using octomap::OcTreeKey;
using octomap::point3d;

int failures = 0;

void expect(bool ok, const std::string& what) {
  if (!ok) {
    std::cerr << "FAIL: " << what << "\n";
    ++failures;
  }
}

// Key of the cell [0, res) on each axis for octomap's default depth of 16:
// keys below it are negative coordinates.
constexpr int kO = 32768;

enum : std::uint8_t { kUnknown = 0, kFree = 1, kOcc = 2 };

// A box of cells in key space: keys lo[a] .. lo[a] + n[a] - 1 on each axis.
struct Box {
  int lo[3] = {0, 0, 0};
  int n[3] = {0, 0, 0};
  std::size_t cells() const { return static_cast<std::size_t>(n[0]) * n[1] * n[2]; }
  std::size_t index(int x, int y, int z) const {
    return static_cast<std::size_t>(x) + static_cast<std::size_t>(n[0]) *
                                             (static_cast<std::size_t>(y) +
                                              static_cast<std::size_t>(n[1]) * z);
  }
  OcTreeKey key(int x, int y, int z) const {
    return OcTreeKey(static_cast<octomap::key_type>(lo[0] + x),
                     static_cast<octomap::key_type>(lo[1] + y),
                     static_cast<octomap::key_type>(lo[2] + z));
  }
};

// The map as designed: a state per cell of a key-space region.
struct Scene {
  double res = 0.05;
  Box region;
  std::vector<std::uint8_t> state;

  Scene(double r, int lx, int ly, int lz, int nx, int ny, int nz) : res(r) {
    region.lo[0] = lx;
    region.lo[1] = ly;
    region.lo[2] = lz;
    region.n[0] = nx;
    region.n[1] = ny;
    region.n[2] = nz;
    state.assign(region.cells(), kUnknown);
  }
  std::uint8_t& at(int x, int y, int z) { return state[region.index(x, y, z)]; }
  // Inclusive block, clipped to the region.
  void fill(int x0, int y0, int z0, int x1, int y1, int z1, std::uint8_t s) {
    for (int z = std::max(0, z0); z <= std::min(region.n[2] - 1, z1); ++z)
      for (int y = std::max(0, y0); y <= std::min(region.n[1] - 1, y1); ++y)
        for (int x = std::max(0, x0); x <= std::min(region.n[0] - 1, x1); ++x) at(x, y, z) = s;
  }
  // The box the field should cover: lowest to highest key of any known cell.
  Box expectedBox() const {
    int lo[3] = {INT32_MAX, INT32_MAX, INT32_MAX}, hi[3] = {INT32_MIN, INT32_MIN, INT32_MIN};
    for (int z = 0; z < region.n[2]; ++z)
      for (int y = 0; y < region.n[1]; ++y)
        for (int x = 0; x < region.n[0]; ++x) {
          if (state[region.index(x, y, z)] == kUnknown) continue;
          const int c[3] = {x, y, z};
          for (int a = 0; a < 3; ++a) {
            lo[a] = std::min(lo[a], region.lo[a] + c[a]);
            hi[a] = std::max(hi[a], region.lo[a] + c[a]);
          }
        }
    Box b;
    if (lo[0] > hi[0]) return b;
    for (int a = 0; a < 3; ++a) {
      b.lo[a] = lo[a];
      b.n[a] = hi[a] - lo[a] + 1;
    }
    return b;
  }
};

std::unique_ptr<OcTree> buildTree(const Scene& s) {
  auto tree = std::make_unique<OcTree>(s.res);
  for (int z = 0; z < s.region.n[2]; ++z)
    for (int y = 0; y < s.region.n[1]; ++y)
      for (int x = 0; x < s.region.n[0]; ++x) {
        const std::uint8_t st = s.state[s.region.index(x, y, z)];
        if (st != kUnknown) tree->updateNode(s.region.key(x, y, z), st == kOcc);
      }
  tree->prune();
  return tree;
}

// Occupancy of every cell of `box`, read back through octomap's search (which
// returns the covering leaf at whatever depth it was merged to).
std::vector<std::uint8_t> readOccupancy(const OcTree& tree, const Box& box) {
  std::vector<std::uint8_t> occ(box.cells(), 0);
  for (int z = 0; z < box.n[2]; ++z)
    for (int y = 0; y < box.n[1]; ++y)
      for (int x = 0; x < box.n[0]; ++x) {
        const octomap::OcTreeNode* node = tree.search(box.key(x, y, z));
        occ[box.index(x, y, z)] = (node != nullptr && tree.isNodeOccupied(node)) ? 1 : 0;
      }
  return occ;
}

// Ground truth, the obvious way: squared distance in cells from each cell to
// the nearest occupied cell, -1 if there is none. O(cells x occupied).
std::vector<std::int64_t> bruteForceD2(const Box& box, const std::vector<std::uint8_t>& occ) {
  std::vector<std::array<int, 3>> sites;
  for (int z = 0; z < box.n[2]; ++z)
    for (int y = 0; y < box.n[1]; ++y)
      for (int x = 0; x < box.n[0]; ++x)
        if (occ[box.index(x, y, z)]) sites.push_back({x, y, z});
  std::vector<std::int64_t> d2(box.cells(), -1);
  if (sites.empty()) return d2;
  for (int z = 0; z < box.n[2]; ++z)
    for (int y = 0; y < box.n[1]; ++y)
      for (int x = 0; x < box.n[0]; ++x) {
        std::int64_t best = std::numeric_limits<std::int64_t>::max();
        for (const auto& s : sites) {
          const std::int64_t dx = x - s[0], dy = y - s[1], dz = z - s[2];
          best = std::min(best, dx * dx + dy * dy + dz * dz);
        }
        d2[box.index(x, y, z)] = best;
      }
  return d2;
}

// The same for one cell only (spot checks on the bigger map).
std::int64_t bruteForceD2At(const Box& box, const std::vector<std::uint8_t>& occ, int x, int y,
                            int z) {
  std::int64_t best = -1;
  for (int sz = 0; sz < box.n[2]; ++sz)
    for (int sy = 0; sy < box.n[1]; ++sy)
      for (int sx = 0; sx < box.n[0]; ++sx) {
        if (!occ[box.index(sx, sy, sz)]) continue;
        const std::int64_t dx = x - sx, dy = y - sy, dz = z - sz;
        const std::int64_t d = dx * dx + dy * dy + dz * dz;
        if (best < 0 || d < best) best = d;
      }
  return best;
}

// Ground truth for the bigger map, still brute force but local: scan the
// offsets within `radius2` cells² in increasing order and stop at the first
// occupied cell. -1 if nothing lies within radius2 (which must be large enough
// that anything beyond it reads maxdist anyway).
std::vector<std::int64_t> windowedD2(const Box& box, const std::vector<std::uint8_t>& occ,
                                     std::int64_t radius2) {
  struct Off { int dx, dy, dz; std::int64_t d2; };
  std::vector<Off> offs;
  const int r = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(radius2))));
  for (int dz = -r; dz <= r; ++dz)
    for (int dy = -r; dy <= r; ++dy)
      for (int dx = -r; dx <= r; ++dx) {
        const std::int64_t d2 = static_cast<std::int64_t>(dx) * dx + dy * dy + dz * dz;
        if (d2 <= radius2) offs.push_back({dx, dy, dz, d2});
      }
  std::sort(offs.begin(), offs.end(), [](const Off& a, const Off& b) { return a.d2 < b.d2; });
  std::vector<std::int64_t> out(box.cells(), -1);
  for (int z = 0; z < box.n[2]; ++z)
    for (int y = 0; y < box.n[1]; ++y)
      for (int x = 0; x < box.n[0]; ++x) {
        for (const Off& o : offs) {
          const int sx = x + o.dx, sy = y + o.dy, sz = z + o.dz;
          if (sx < 0 || sy < 0 || sz < 0 || sx >= box.n[0] || sy >= box.n[1] || sz >= box.n[2])
            continue;
          if (occ[box.index(sx, sy, sz)]) {
            out[box.index(x, y, z)] = o.d2;
            break;
          }
        }
      }
  return out;
}

// The value the field should read for a cell whose nearest obstacle is sqrt(d2)
// cells away (d2 < 0: none at all).
float truthValue(std::int64_t d2, double res, double maxdist) {
  if (d2 < 0) return static_cast<float>(maxdist);
  return static_cast<float>(std::min(std::sqrt(static_cast<double>(d2)) * res, maxdist));
}

// Equal to float precision. Any real error is at least one cell step in the
// squared distance, i.e. res * (sqrt(d2 + 1) - sqrt(d2)) >= ~1e-4 m for every
// map here, far above this tolerance.
bool sameFloat(float got, float want) {
  return std::abs(got - want) <= 2.0f * FLT_EPSILON * std::max(1.0f, std::abs(want));
}

point3d cellCentre(const OcTree& tree, const Box& box, int x, int y, int z) {
  return tree.keyToCoord(box.key(x, y, z));
}

std::string cellName(const Box& box, int x, int y, int z) {
  return "cell (" + std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z) +
         ") key (" + std::to_string(box.lo[0] + x - kO) + "," + std::to_string(box.lo[1] + y - kO) +
         "," + std::to_string(box.lo[2] + z - kO) + ")";
}

// Every cell centre of the field's box, x fastest (for bitwise comparisons).
std::vector<float> allCells(const DistanceField& df, const OcTree& tree, const Box& box) {
  std::vector<float> v(box.cells());
  for (int z = 0; z < box.n[2]; ++z)
    for (int y = 0; y < box.n[1]; ++y)
      for (int x = 0; x < box.n[0]; ++x)
        v[box.index(x, y, z)] = df.getDistance(cellCentre(tree, box, x, y, z));
  return v;
}

// The full check of one field against the ground truth `d2` over `box`.
void checkField(const DistanceField& df, const OcTree& tree, const Box& box,
                const std::vector<std::int64_t>& d2, double maxdist, const std::string& label,
                std::mt19937& rng) {
  const double res = tree.getResolution();
  expect(df.sizeX() == box.n[0] && df.sizeY() == box.n[1] && df.sizeZ() == box.n[2],
         label + ": box is " + std::to_string(df.sizeX()) + "x" + std::to_string(df.sizeY()) +
             "x" + std::to_string(df.sizeZ()) + ", want " + std::to_string(box.n[0]) + "x" +
             std::to_string(box.n[1]) + "x" + std::to_string(box.n[2]));
  if (df.sizeX() != box.n[0] || df.sizeY() != box.n[1] || df.sizeZ() != box.n[2]) return;

  // 1. Every cell centre.
  int bad = 0;
  for (int z = 0; z < box.n[2]; ++z)
    for (int y = 0; y < box.n[1]; ++y)
      for (int x = 0; x < box.n[0]; ++x) {
        const point3d p = cellCentre(tree, box, x, y, z);
        const float want = truthValue(d2[box.index(x, y, z)], res, maxdist);
        const float got = df.getDistance(p);
        if (!sameFloat(got, want)) {
          if (bad < 5)
            std::cerr << "  " << label << ": " << cellName(box, x, y, z) << " reads " << got
                      << ", brute force " << want << "\n";
          ++bad;
        }
      }
  expect(bad == 0, label + ": " + std::to_string(bad) + " of " + std::to_string(box.cells()) +
                       " cell centres differ from brute force");

  // 2. Random points anywhere in the box grown by two cells on every side: the
  //    expected answer is whatever octomap's own key of the point says (the
  //    cell's truth inside the box, -1 outside).
  std::uniform_real_distribution<double> u(0.0, 1.0);
  bad = 0;
  for (int i = 0; i < 3000; ++i) {
    double c[3];
    for (int a = 0; a < 3; ++a)
      c[a] = (box.lo[a] - kO - 2 + u(rng) * (box.n[a] + 4)) * res;
    const point3d p(static_cast<float>(c[0]), static_cast<float>(c[1]), static_cast<float>(c[2]));
    const OcTreeKey k = tree.coordToKey(p);
    const int cx = k[0] - box.lo[0], cy = k[1] - box.lo[1], cz = k[2] - box.lo[2];
    const bool inside =
        cx >= 0 && cy >= 0 && cz >= 0 && cx < box.n[0] && cy < box.n[1] && cz < box.n[2];
    const float want = inside ? truthValue(d2[box.index(cx, cy, cz)], res, maxdist) : -1.0f;
    const float got = df.getDistance(p);
    if (!sameFloat(got, want)) {
      if (bad < 5)
        std::cerr << "  " << label << ": point (" << p.x() << "," << p.y() << "," << p.z()
                  << ") reads " << got << ", want " << want << "\n";
      ++bad;
    }
  }
  expect(bad == 0, label + ": " + std::to_string(bad) + " random points differ");

  // 3. The layer of cells just outside each face, and points a hair either side
  //    of each face.
  std::uniform_int_distribution<int> ux(0, box.n[0] - 1), uy(0, box.n[1] - 1),
      uz(0, box.n[2] - 1);
  bad = 0;
  int bad_edge = 0;
  for (int i = 0; i < 40; ++i) {
    for (int a = 0; a < 3; ++a) {
      for (int side = 0; side < 2; ++side) {
        int c[3] = {ux(rng), uy(rng), uz(rng)};
        c[a] = side == 0 ? -1 : box.n[a];
        if (df.getDistance(cellCentre(tree, box, c[0], c[1], c[2])) != -1.0f) ++bad;

        // A hundredth of a cell outside the face reads -1, inside reads the
        // boundary cell.
        c[a] = side == 0 ? 0 : box.n[a] - 1;
        point3d p = cellCentre(tree, box, c[0], c[1], c[2]);
        const double face = (side == 0 ? box.lo[a] - kO : box.lo[a] + box.n[a] - kO) * res;
        const double eps = (side == 0 ? -1.0 : 1.0) * 0.01 * res;
        p(a) = static_cast<float>(face + eps);
        if (df.getDistance(p) != -1.0f) ++bad_edge;
        p(a) = static_cast<float>(face - eps);
        if (!sameFloat(df.getDistance(p), truthValue(d2[box.index(c[0], c[1], c[2])], res,
                                                     maxdist)))
          ++bad_edge;
      }
    }
  }
  expect(bad == 0, label + ": " + std::to_string(bad) + " cells just outside the box read != -1");
  expect(bad_edge == 0,
         label + ": " + std::to_string(bad_edge) + " points a hair from a face read wrong");

  // 4. Non-finite and far-away coordinates on each axis, including ones a whole
  //    key range away, where octomap's 16-bit key wraps back onto a cell of
  //    the box.
  const float kNaN = std::numeric_limits<float>::quiet_NaN();
  const float kInf = std::numeric_limits<float>::infinity();
  const point3d centre = cellCentre(tree, box, box.n[0] / 2, box.n[1] / 2, box.n[2] / 2);
  bad = 0;
  for (int a = 0; a < 3; ++a) {
    const float wrap = static_cast<float>(65536.0 * res);
    for (float v : {kNaN, kInf, -kInf, 1e9f, -1e9f, 1e30f, centre(a) + wrap, centre(a) - wrap}) {
      point3d p = centre;
      p(a) = v;
      if (df.getDistance(p) != -1.0f) ++bad;
    }
  }
  expect(bad == 0, label + ": " + std::to_string(bad) + " NaN/inf/far/wrapped points read != -1");
}

// Builds `scene`, checks the tree came out as designed and returns it with its
// box, occupancy and brute-force ground truth.
struct Built {
  std::unique_ptr<OcTree> tree;
  Box box;
  std::vector<std::uint8_t> occ;
  std::vector<std::int64_t> d2;
};

Built build(const Scene& scene, const std::string& label, bool brute_force = true) {
  Built b;
  b.tree = buildTree(scene);
  b.box = scene.expectedBox();
  b.occ = readOccupancy(*b.tree, b.box);
  // The tree must hold exactly the designed occupancy (otherwise the test, not
  // the field, would be wrong).
  int mismatch = 0;
  for (int z = 0; z < b.box.n[2]; ++z)
    for (int y = 0; y < b.box.n[1]; ++y)
      for (int x = 0; x < b.box.n[0]; ++x) {
        const int rx = b.box.lo[0] - scene.region.lo[0] + x;
        const int ry = b.box.lo[1] - scene.region.lo[1] + y;
        const int rz = b.box.lo[2] - scene.region.lo[2] + z;
        const bool designed = scene.state[scene.region.index(rx, ry, rz)] == kOcc;
        if (designed != (b.occ[b.box.index(x, y, z)] != 0)) ++mismatch;
      }
  expect(mismatch == 0, label + ": octree occupancy differs from the scene in " +
                            std::to_string(mismatch) + " cells (test setup)");
  if (brute_force) b.d2 = bruteForceD2(b.box, b.occ);
  return b;
}

// Random clutter: each cell occupied with probability `density`, otherwise
// free with probability `free_frac`, otherwise unknown. The two corner cells
// are known, so the box is exactly the region.
Scene randomScene(double res, const int lo[3], const int n[3], double density, double free_frac,
                  std::mt19937& rng) {
  Scene s(res, lo[0], lo[1], lo[2], n[0], n[1], n[2]);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  for (auto& c : s.state) {
    const double r = u(rng);
    c = r < density ? kOcc : (r < density + (1.0 - density) * free_frac ? kFree : kUnknown);
  }
  if (s.at(0, 0, 0) == kUnknown) s.at(0, 0, 0) = kFree;
  if (s.at(n[0] - 1, n[1] - 1, n[2] - 1) == kUnknown) s.at(n[0] - 1, n[1] - 1, n[2] - 1) = kFree;
  return s;
}

// ---------------------------------------------------------------- check 1

void checkSmallMaps(std::mt19937& rng) {
  // Random clutter, maps straddling the origin on every axis, a few densities
  // and maxdists: one well inside the box, one an exact multiple of the
  // resolution, one about the box diagonal and one beyond it.
  struct Case { double res; int lo[3]; int n[3]; double density; std::vector<double> maxdists; };
  const std::vector<Case> cases = {
      {0.05, {kO - 15, kO - 11, kO - 7}, {30, 24, 20}, 0.002, {0.3, 0.25, 2.0, 10.0}},
      {0.05, {kO - 15, kO - 11, kO - 7}, {30, 24, 20}, 0.02, {0.3, 0.25, 0.0}},
      {0.05, {kO - 20, kO - 3, kO - 17}, {40, 21, 33}, 0.2, {0.3, 1.0}},
      {0.07, {kO - 13, kO - 30, kO - 2}, {25, 31, 22}, 0.01, {0.33, 0.7, 5.0}},
      {0.07, {kO - 13, kO - 30, kO - 2}, {25, 31, 22}, 0.05, {0.33, 0.07}},
  };
  int idx = 0;
  for (const Case& c : cases) {
    const Scene scene = randomScene(c.res, c.lo, c.n, c.density, 0.6, rng);
    const std::string base = "random map " + std::to_string(idx++) + " (res " +
                             std::to_string(c.res) + ", density " + std::to_string(c.density) + ")";
    const Built b = build(scene, base);
    for (double md : c.maxdists) {
      const DistanceField df(*b.tree, md, 3);
      checkField(df, *b.tree, b.box, b.d2, md, base + " maxdist " + std::to_string(md), rng);
    }
    // The windowed brute force used for the medium map agrees with the plain
    // one wherever the plain one is within its radius (sparse map, small
    // radius, so the cut-off really matters).
    if (idx == 1) {
      const std::int64_t r2 = 50;
      const std::vector<std::int64_t> w = windowedD2(b.box, b.occ, r2);
      int bad = 0;
      for (std::size_t i = 0; i < w.size(); ++i)
        if (w[i] != (b.d2[i] >= 0 && b.d2[i] <= r2 ? b.d2[i] : -1)) ++bad;
      expect(bad == 0, base + ": windowed brute force disagrees with plain brute force in " +
                           std::to_string(bad) + " cells (test reference)");
    }
  }

  // Merged occupied leaves: aligned solid blocks that prune to single leaves of
  // 8, 4 and 2 cells, merged free blocks (one of them defines the max corner
  // of the box), and scattered single voxels.
  {
    Scene s(0.05, kO - 16, kO - 16, kO - 16, 32, 32, 32);
    s.fill(0, 0, 0, 15, 15, 15, kFree);        // key -16..-1: a 16-cell free leaf
    s.fill(24, 24, 24, 31, 31, 31, kFree);     // key 8..15: an 8-cell free leaf at the max corner
    s.fill(0, 16, 24, 7, 23, 31, kOcc);        // keys x -16..-9, y 0..7, z 8..15
    s.fill(20, 4, 12, 23, 7, 15, kOcc);        // keys x 4..7, y -12..-9, z -4..-1
    s.fill(10, 10, 16, 11, 11, 17, kOcc);      // keys -6..-5, -6..-5, 0..1
    s.fill(0, 0, 0, 0, 0, 0, kFree);
    std::uniform_int_distribution<int> uc(0, 31);
    for (int i = 0; i < 40; ++i) {
      const int x = uc(rng), y = uc(rng), z = uc(rng);
      if (s.at(x, y, z) == kUnknown) s.at(x, y, z) = kOcc;
    }
    const Built b = build(s, "pruned map");
    bool occ8 = false, occ4 = false, occ2 = false, free8 = false, free16 = false;
    for (auto it = b.tree->begin_leafs(), end = b.tree->end_leafs(); it != end; ++it) {
      const bool o = b.tree->isNodeOccupied(*it);
      const unsigned d = it.getDepth();
      occ8 |= o && d == 13;
      occ4 |= o && d == 14;
      occ2 |= o && d == 15;
      free8 |= !o && d == 13;
      free16 |= !o && d == 12;
    }
    expect(occ8 && occ4 && occ2, "pruned map: the solid blocks did not merge into 8/4/2-cell "
                                 "occupied leaves (test setup)");
    expect(free8 && free16, "pruned map: the free blocks did not merge (test setup)");
    for (double md : {0.2, 0.4, 3.0}) {
      const DistanceField df(*b.tree, md, 4);
      checkField(df, *b.tree, b.box, b.d2, md, "pruned map maxdist " + std::to_string(md), rng);
    }
  }

  // Free voxels only (with unknown gaps): every cell reads maxdist.
  {
    const int lo[3] = {kO - 10, kO - 7, kO - 12}, n[3] = {20, 20, 20};
    const Scene s = randomScene(0.05, lo, n, 0.0, 0.7, rng);
    const Built b = build(s, "free-only map");
    const DistanceField df(*b.tree, 0.35, 2);
    checkField(df, *b.tree, b.box, b.d2, 0.35, "free-only map", rng);
    int not_max = 0;
    for (float v : allCells(df, *b.tree, b.box))
      if (v != static_cast<float>(0.35)) ++not_max;
    expect(not_max == 0, "free-only map: " + std::to_string(not_max) + " cells below maxdist");
  }

  // A box of one voxel, occupied or free, and an empty tree.
  for (std::uint8_t st : {kOcc, kFree}) {
    Scene s(0.05, kO - 1, kO + 5, kO - 3, 1, 1, 1);
    s.at(0, 0, 0) = st;
    const std::string label = st == kOcc ? "single occupied voxel" : "single free voxel";
    const Built b = build(s, label);
    const DistanceField df(*b.tree, 0.5, 4);
    checkField(df, *b.tree, b.box, b.d2, 0.5, label, rng);
    const float v = df.getDistance(cellCentre(*b.tree, b.box, 0, 0, 0));
    expect(v == (st == kOcc ? 0.0f : 0.5f), label + ": reads " + std::to_string(v));
    int bad = 0;
    for (int dz = -1; dz <= 1; ++dz)
      for (int dy = -1; dy <= 1; ++dy)
        for (int dx = -1; dx <= 1; ++dx)
          if ((dx || dy || dz) && df.getDistance(cellCentre(*b.tree, b.box, dx, dy, dz)) != -1.0f)
            ++bad;
    expect(bad == 0, label + ": " + std::to_string(bad) + " of the 26 neighbours read != -1");
  }
  {
    OcTree empty(0.05);
    const DistanceField df(empty, 1.0, 4);
    expect(df.sizeX() == 0 && df.sizeY() == 0 && df.sizeZ() == 0, "empty tree: non-empty box");
    expect(df.getDistance(point3d(0, 0, 0)) == -1.0f && df.getDistance(point3d(0.01f, -0.3f, 2)) == -1.0f,
           "empty tree: a query read != -1");
  }

  // Large maxdist on long boxes, which switch the field's internals away from
  // the common 16-bit windowed pass: first the 16-bit lower-envelope pass
  // (saturation above 128 cells), then 32-bit storage (above 256 cells). Few
  // obstacles, clustered at one corner, so distances run right up to (and past)
  // the cap and the far end saturates.
  {
    Scene s(0.05, kO - 10, kO - 75, kO - 75, 20, 150, 150);
    s.at(0, 0, 0) = kOcc;
    s.at(19, 149, 149) = kFree;
    std::uniform_int_distribution<int> ux(0, 19), uyz(0, 40), uy(0, 149);
    for (int i = 0; i < 20; ++i) s.at(ux(rng), uyz(rng), uyz(rng)) = kOcc;
    for (int i = 0; i < 3; ++i) s.at(ux(rng), uy(rng), uyz(rng)) = kOcc;
    const Built b = build(s, "long map (16-bit)");
    // 6.0 m = 120 cells: the widest windowed pass; 7.0 m = 140 cells and 100 m
    // (capped at the box diagonal): the envelope.
    for (double md : {6.0, 7.0, 100.0}) {
      const DistanceField df(*b.tree, md, 4);
      checkField(df, *b.tree, b.box, b.d2, md, "long map (16-bit) maxdist " + std::to_string(md),
                 rng);
    }
  }
  {
    Scene s(0.05, kO - 1, kO - 100, kO - 100, 3, 200, 200);
    s.at(0, 0, 0) = kOcc;
    s.at(1, 1, 0) = kOcc;
    s.at(2, 0, 3) = kOcc;
    s.at(2, 199, 199) = kFree;
    const Built b = build(s, "long map (32-bit)");
    // 13.3 m = 266 cells: 32-bit with saturation (the far corner is ~281
    // cells away); 1000 m: 32-bit, capped at the box diagonal.
    for (double md : {13.3, 1000.0}) {
      const DistanceField df(*b.tree, md, 4);
      checkField(df, *b.tree, b.box, b.d2, md, "long map (32-bit) maxdist " + std::to_string(md),
                 rng);
    }
  }
}

// ---------------------------------------------------------------- check 2..5

// A 3 x 3 x 2 m room at 5 cm straddling the origin: floor, walls with a door
// and a window, a table, a cabinet, a ball, an unknown pocket, random clutter
// and small random blobs.
Scene roomScene(std::mt19937& rng) {
  Scene s(0.05, kO - 30, kO - 30, kO - 6, 60, 60, 40);
  s.fill(0, 0, 0, 59, 59, 39, kFree);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  for (int z = 0; z < 4; ++z)  // below the floor: mostly unknown
    for (int y = 0; y < 60; ++y)
      for (int x = 0; x < 60; ++x)
        if (u(rng) < 0.6) s.at(x, y, z) = kUnknown;
  s.fill(0, 0, 4, 59, 59, 5, kOcc);           // floor
  s.fill(0, 0, 6, 1, 59, 39, kOcc);           // wall x-, two cells thick
  s.fill(0, 25, 6, 1, 34, 27, kFree);         // door
  s.fill(59, 0, 6, 59, 59, 33, kOcc);         // wall x+, one cell
  s.fill(59, 10, 18, 59, 20, 28, kFree);      // window
  s.fill(0, 0, 6, 59, 0, 30, kOcc);           // wall y-
  s.fill(10, 10, 18, 25, 20, 19, kOcc);       // table top
  for (int lx : {10, 24})
    for (int ly : {10, 19}) s.fill(lx, ly, 6, lx + 1, ly + 1, 17, kOcc);  // legs
  s.fill(40, 5, 6, 49, 12, 30, kOcc);         // cabinet
  s.fill(50, 2, 6, 57, 10, 30, kUnknown);     // unseen pocket behind it
  s.fill(30, 30, 28, 45, 45, 39, kUnknown);   // unseen space under the ceiling
  for (int z = 0; z < 40; ++z)                // a ball, radius 4 cells
    for (int y = 0; y < 60; ++y)
      for (int x = 0; x < 60; ++x)
        if ((x - 30) * (x - 30) + (y - 45) * (y - 45) + (z - 15) * (z - 15) <= 16)
          s.at(x, y, z) = kOcc;
  for (int z = 6; z < 40; ++z)                // clutter
    for (int y = 0; y < 60; ++y)
      for (int x = 0; x < 60; ++x)
        if (s.at(x, y, z) == kFree && u(rng) < 0.0005) s.at(x, y, z) = kOcc;
  std::uniform_int_distribution<int> uxy(2, 56), uz(6, 36);
  for (int i = 0; i < 12; ++i) {              // small unaligned blobs
    const int x = uxy(rng), y = uxy(rng), z = uz(rng);
    s.fill(x, y, z, x + 1, y + 2, z + 1, kOcc);
  }
  s.at(0, 0, 0) = kFree;
  s.at(59, 59, 39) = kFree;
  return s;
}

double msSince(std::chrono::steady_clock::time_point t0) {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

// Compares `df` with DynamicEDT3D over our box (DynamicEDT3D's is one layer
// larger on the max faces, so ours is the common box), with DynamicEDT3D
// capped at maxdist (it reads up to maxdist + one voxel where nothing is in
// range). Ours must equal the brute-force truth `b.d2` and never read above
// DynamicEDT3D, DynamicEDT3D must never read below the truth, and the two may
// differ by more than 1e-4 in at most `max_fraction` of the cells, each by less
// than half a voxel.
void compareWithDynamicEdt(const DistanceField& df, const DynamicEDTOctomap& edt, const Built& b,
                           double maxdist, const std::string& label, double max_fraction,
                           std::mt19937& rng) {
  const OcTree& tree = *b.tree;
  const Box& box = b.box;
  const double res = tree.getResolution();
  std::size_t over = 0, under_truth = 0, differ = 0, edt_outside = 0, differ_bad_truth = 0;
  double max_diff = 0.0, max_under = 0.0, max_raw = 0.0;
  std::vector<std::array<int, 3>> differing;
  for (int z = 0; z < box.n[2]; ++z)
    for (int y = 0; y < box.n[1]; ++y)
      for (int x = 0; x < box.n[0]; ++x) {
        const point3d p = cellCentre(tree, box, x, y, z);
        const float e = edt.getDistance(p);
        if (e < 0.0f) {
          ++edt_outside;
          continue;
        }
        max_raw = std::max(max_raw, static_cast<double>(e));
        const double ecap = std::min(static_cast<double>(e), maxdist);
        const double ours = df.getDistance(p);
        const double truth = truthValue(b.d2[box.index(x, y, z)], res, maxdist);
        if (ours > ecap + 1e-6) ++over;
        if (truth > ecap + 1e-6) {
          ++under_truth;
          max_under = std::max(max_under, truth - ecap);
        }
        const double diff = std::abs(ours - ecap);
        if (diff > 1e-4) {
          ++differ;
          differing.push_back({x, y, z});
          if (!sameFloat(static_cast<float>(ours), static_cast<float>(truth))) ++differ_bad_truth;
        }
        max_diff = std::max(max_diff, diff);
      }
  // DynamicEDT3D's extra layer beyond our max faces (informational).
  int extra = 0;
  for (int a = 0; a < 3; ++a) {
    int c[3] = {box.n[0] / 2, box.n[1] / 2, box.n[2] / 2};
    c[a] = box.n[a];
    extra += edt.getDistance(cellCentre(tree, box, c[0], c[1], c[2])) >= 0.0f;
  }
  const double frac = static_cast<double>(differ) / static_cast<double>(box.cells());
  std::cout << "  " << label << " vs DynamicEDT3D (capped at maxdist " << maxdist << "): "
            << box.cells() << " cells compared, " << differ << " differ by > 1e-4 ("
            << 100.0 * frac << " %), max difference " << max_diff << " m; ours above DynamicEDT3D in "
            << over << " cells; DynamicEDT3D below brute force in " << under_truth << " cells"
            << (under_truth ? " (max " + std::to_string(max_under) + " m)" : std::string())
            << "; DynamicEDT3D uncapped max " << max_raw << " m, extra max-face layer on "
            << extra << "/3 axes\n";
  expect(edt_outside == 0, label + ": DynamicEDT3D reads -1 in " + std::to_string(edt_outside) +
                               " cells of our box (its box should contain ours)");
  expect(over == 0, label + ": ours reads above DynamicEDT3D in " + std::to_string(over) + " cells");
  expect(under_truth == 0, label + ": DynamicEDT3D reads below the brute-force truth in " +
                               std::to_string(under_truth) + " cells");
  expect(differ_bad_truth == 0, label + ": where ours and DynamicEDT3D differ, ours disagrees "
                                        "with brute force in " +
                                    std::to_string(differ_bad_truth) + " cells");
  expect(frac <= max_fraction, label + ": ours and DynamicEDT3D differ in " +
                                   std::to_string(100.0 * frac) + " % of cells (limit " +
                                   std::to_string(100.0 * max_fraction) + " %)");
  expect(max_diff < 0.5 * res, label + ": ours and DynamicEDT3D differ by up to " +
                                   std::to_string(max_diff) + " m (limit half a voxel)");

  // Independent spot checks with the plain O(occupied) brute force: up to 200
  // cells where the two fields differ, and 1500 random cells.
  std::shuffle(differing.begin(), differing.end(), rng);
  if (differing.size() > 200) differing.resize(200);
  std::uniform_int_distribution<int> ux(0, box.n[0] - 1), uy(0, box.n[1] - 1), uz(0, box.n[2] - 1);
  for (int i = 0; i < 1500; ++i) differing.push_back({ux(rng), uy(rng), uz(rng)});
  int bad = 0;
  for (const auto& c : differing) {
    const std::int64_t d2 = bruteForceD2At(box, b.occ, c[0], c[1], c[2]);
    if (!sameFloat(df.getDistance(cellCentre(tree, box, c[0], c[1], c[2])),
                   truthValue(d2, res, maxdist)))
      ++bad;
  }
  expect(bad == 0, label + ": " + std::to_string(bad) +
                       " spot-checked cells differ from the plain brute force");
}

// Sparse slanted plates (discs one cell thick at random orientations) in open
// space, 60 x 60 x 40 cells at 5 cm. DynamicEDT3D's brushfire is exact on the
// room map at 1 m, but on this kind of geometry at larger distances it
// overestimates in a handful of cells, so this is where "never below the truth"
// and "ours never above DynamicEDT3D" are actually exercised.
Scene platesScene(std::mt19937& rng) {
  Scene s(0.05, kO - 30, kO - 25, kO - 20, 60, 60, 40);
  s.fill(0, 0, 0, 59, 59, 39, kFree);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  for (int i = 0; i < 10; ++i) {
    double n[3] = {u(rng) - 0.5, u(rng) - 0.5, u(rng) - 0.5};
    const double len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    const double c[3] = {u(rng) * 60, u(rng) * 60, u(rng) * 40};
    const double r = 5.0 + 10.0 * u(rng);
    for (int z = 0; z < 40; ++z)
      for (int y = 0; y < 60; ++y)
        for (int x = 0; x < 60; ++x) {
          const double d[3] = {x - c[0], y - c[1], z - c[2]};
          const double h = (d[0] * n[0] + d[1] * n[1] + d[2] * n[2]) / len;
          if (std::abs(h) < 0.5 && d[0] * d[0] + d[1] * d[1] + d[2] * d[2] < r * r)
            s.at(x, y, z) = kOcc;
        }
  }
  s.at(0, 0, 0) = kFree;
  s.at(59, 59, 39) = kFree;
  return s;
}

// Seed 5 gives 5 cells where DynamicEDT3D overestimates (seeds 1-12 give 0-5).
constexpr unsigned kPlatesSeed = 5;

void checkPlatesMap(unsigned seed, std::mt19937& rng) {
  constexpr double kMaxDist = 3.0;  // 60 cells
  // Its own generator: which cells DynamicEDT3D gets wrong depends on the exact
  // geometry, so the map must not change when other checks draw more numbers.
  std::mt19937 scene_rng(seed);
  const Scene scene = platesScene(scene_rng);
  Built b = build(scene, "plates map");
  OcTree& tree = *b.tree;
  double x0, y0, z0, x1, y1, z1;
  tree.getMetricMin(x0, y0, z0);
  tree.getMetricMax(x1, y1, z1);
  DynamicEDTOctomap edt(static_cast<float>(kMaxDist), &tree, point3d(x0, y0, z0),
                        point3d(x1, y1, z1), /*treatUnknownAsOccupied=*/false);
  edt.update();
  const DistanceField df(tree, kMaxDist, 4);
  checkField(df, tree, b.box, b.d2, kMaxDist, "plates map", rng);
  compareWithDynamicEdt(df, edt, b, kMaxDist, "plates map", /*max_fraction=*/0.001, rng);
}

void checkMediumMap(std::mt19937& rng) {
  constexpr double kMaxDist = 1.0;
  const Scene scene = roomScene(rng);
  Built b = build(scene, "room map", /*brute_force=*/false);
  OcTree& tree = *b.tree;
  const Box& box = b.box;
  const double res = tree.getResolution();
  std::size_t occupied = 0;
  for (auto o : b.occ) occupied += o;

  // Ground truth over every cell: anything beyond 20 cells (+1 cell² margin)
  // reads maxdist.
  const std::int64_t r2 = static_cast<std::int64_t>(std::floor((kMaxDist / res) * (kMaxDist / res))) + 1;
  auto t0 = std::chrono::steady_clock::now();
  b.d2 = windowedD2(box, b.occ, r2);
  const double brute_ms = msSince(t0);

  // --- 5. timing (informational)
  double x0, y0, z0, x1, y1, z1;
  tree.getMetricMin(x0, y0, z0);
  tree.getMetricMax(x1, y1, z1);
  t0 = std::chrono::steady_clock::now();
  DynamicEDTOctomap edt(static_cast<float>(kMaxDist), &tree, point3d(x0, y0, z0),
                        point3d(x1, y1, z1), /*treatUnknownAsOccupied=*/false);
  edt.update();
  const double edt_ms = msSince(t0);

  t0 = std::chrono::steady_clock::now();
  auto df = std::make_unique<DistanceField>(tree, kMaxDist, 4);
  const double first_ms = msSince(t0);
  double best_ms = first_ms;
  for (int i = 0; i < 4; ++i) {
    t0 = std::chrono::steady_clock::now();
    df = std::make_unique<DistanceField>(tree, kMaxDist, 4);
    best_ms = std::min(best_ms, msSince(t0));
  }
  std::cout << "room map: " << box.n[0] << "x" << box.n[1] << "x" << box.n[2] << " cells ("
            << box.cells() << "), " << occupied << " occupied, maxdist " << kMaxDist << " m\n"
            << "  build: DynamicEDT3D " << edt_ms << " ms, DistanceField (4 threads) " << first_ms
            << " ms first / " << best_ms << " ms best of 5 (brute force reference " << brute_ms
            << " ms)\n";

  // --- 1. (again, on a realistic map) ours == brute force everywhere
  checkField(*df, tree, box, b.d2, kMaxDist, "room map", rng);

  // --- 2. against DynamicEDT3D.
  // Threshold 0.1 % on both maps: observed 0 cells on the room and at most
  // 0.0035 % (5 cells, max 1.8 mm) on the plates over 12 seeds, so a real
  // behavioural difference would show far above it.
  compareWithDynamicEdt(*df, edt, b, kMaxDist, "room map", /*max_fraction=*/0.001, rng);
  {
    // The room never gets 1 m from everything, so compare at 0.5 m too, where
    // most of it saturates (the truth above covers any cap up to 1 m).
    DynamicEDTOctomap edt_half(0.5f, &tree, point3d(x0, y0, z0), point3d(x1, y1, z1), false);
    edt_half.update();
    const DistanceField df_half(tree, 0.5, 4);
    checkField(df_half, tree, box, b.d2, 0.5, "room map maxdist 0.5", rng);
    compareWithDynamicEdt(df_half, edt_half, b, 0.5, "room map (0.5 m)", 0.001, rng);
  }

  // --- 3. thread counts: bit-identical fields.
  const std::vector<float> ref = allCells(*df, tree, box);
  for (unsigned t : {1u, 2u, 3u, 8u, 0u}) {
    const DistanceField other(tree, kMaxDist, t);
    const std::vector<float> v = allCells(other, tree, box);
    expect(std::memcmp(v.data(), ref.data(), v.size() * sizeof(float)) == 0,
           "room map: build with " + std::to_string(t) + " threads differs from 4 threads");
  }

  // --- 4. concurrent queries: every cell centre, random points in and around
  //    the box, and out-of-box points, queried by 8 threads at once.
  std::vector<point3d> pts;
  pts.reserve(box.cells() + 20000);
  for (int z = 0; z < box.n[2]; ++z)
    for (int y = 0; y < box.n[1]; ++y)
      for (int x = 0; x < box.n[0]; ++x) pts.push_back(cellCentre(tree, box, x, y, z));
  std::uniform_real_distribution<double> u(-0.2, 1.2);
  for (int i = 0; i < 20000; ++i)
    pts.emplace_back(static_cast<float>(x0 + u(rng) * (x1 - x0)),
                     static_cast<float>(y0 + u(rng) * (y1 - y0)),
                     static_cast<float>(z0 + u(rng) * (z1 - z0)));
  std::vector<float> single(pts.size());
  for (std::size_t i = 0; i < pts.size(); ++i) single[i] = df->getDistance(pts[i]);

  constexpr int kThreads = 8;
  std::vector<std::vector<float>> results(kThreads, std::vector<float>(pts.size()));
  std::atomic<int> ready{0};
  std::vector<std::thread> pool;
  const DistanceField& shared = *df;
  for (int t = 0; t < kThreads; ++t)
    pool.emplace_back([&, t] {
      ready.fetch_add(1);
      while (ready.load() < kThreads) std::this_thread::yield();  // start together
      // Three passes each, starting at different offsets so the threads hit
      // different parts of the grid at the same moment.
      for (int pass = 0; pass < 3; ++pass) {
        const std::size_t n = pts.size(), start = (n / kThreads) * t;
        for (std::size_t k = 0; k < n; ++k) {
          const std::size_t i = (start + k) % n;
          results[t][i] = shared.getDistance(pts[i]);
        }
      }
    });
  for (auto& th : pool) th.join();
  for (int t = 0; t < kThreads; ++t)
    expect(std::memcmp(results[t].data(), single.data(), single.size() * sizeof(float)) == 0,
           "concurrent queries: thread " + std::to_string(t) + " got different answers");
}


// ---------------------------------------------------------------- check 6..8

using drone_core::planning::ConservativeGrid;

constexpr double kInfD = std::numeric_limits<double>::infinity();

// The cells of `box` whose centres lie in [lo, hi] (octomap's own centres).
Box cropOf(const OcTree& tree, const Box& box, const Eigen::Vector3d& lo, const Eigen::Vector3d& hi) {
  Box b;
  for (int a = 0; a < 3; ++a) {
    int l = box.lo[a], h = box.lo[a] + box.n[a] - 1;
    while (l <= h && !(tree.keyToCoord(static_cast<octomap::key_type>(l)) >= lo[a])) ++l;
    while (h >= l && !(tree.keyToCoord(static_cast<octomap::key_type>(h)) <= hi[a])) --h;
    if (l > h) return Box{};
    b.lo[a] = l;
    b.n[a] = h - l + 1;
  }
  return b;
}

// boxMin/boxMax are the outer corners of `box` (empty: lo > hi on every axis),
// and a point a hair inside each corner is in the field, a hair outside is not.
void checkCorners(const DistanceField& df, const Box& box, double res, const std::string& label) {
  const Eigen::Vector3d lo = df.boxMin(), hi = df.boxMax();
  if (box.cells() == 0) {
    expect((lo.array() > hi.array()).all(), label + ": empty field, but boxMin <= boxMax");
    return;
  }
  bool ok = true;
  for (int a = 0; a < 3; ++a) {
    ok &= lo[a] == (box.lo[a] - kO) * res;
    ok &= hi[a] == (box.lo[a] + box.n[a] - kO) * res;
  }
  expect(ok, label + ": boxMin/boxMax are not the box's outer corners");
  const Eigen::Vector3d e = Eigen::Vector3d::Constant(0.01 * res);
  const auto at = [&](const Eigen::Vector3d& p) {
    return df.getDistance(point3d(static_cast<float>(p.x()), static_cast<float>(p.y()),
                                  static_cast<float>(p.z())));
  };
  expect(at(lo + e) >= 0.0f && at(hi - e) >= 0.0f, label + ": a point just inside a corner reads -1");
  expect(at(lo - e) == -1.0f && at(hi + e) == -1.0f, label + ": a point just outside a corner reads >= 0");
}

void expectEmpty(const DistanceField& df, const point3d& probe, const std::string& label) {
  expect(df.sizeX() == 0 && df.sizeY() == 0 && df.sizeZ() == 0, label + ": non-empty box");
  expect(df.getDistance(probe) == -1.0f && df.getDistance(point3d(0, 0, 0)) == -1.0f,
         label + ": a query read != -1");
  checkCorners(df, Box{}, 0.05, label);
}

Scene carvedScene(std::mt19937& rng);

void checkCrop(std::mt19937& rng) {
  // A random map straddling the origin, and one of solid aligned blocks that
  // prune into big occupied leaves, which the crops cut through.
  std::vector<std::pair<Scene, std::string>> scenes;
  {
    const int lo[3] = {kO - 15, kO - 11, kO - 7}, n[3] = {30, 24, 20};
    scenes.emplace_back(randomScene(0.05, lo, n, 0.02, 0.6, rng), "random map");
  }
  {
    Scene s(0.05, kO - 16, kO - 16, kO - 8, 32, 32, 24);
    s.fill(0, 0, 0, 31, 31, 23, kFree);
    s.fill(8, 8, 8, 15, 15, 15, kOcc);    // an 8-cell occupied leaf
    s.fill(16, 0, 0, 19, 3, 3, kOcc);     // 4-cell
    s.fill(24, 24, 16, 31, 31, 23, kOcc); // 8-cell at the max corner
    scenes.emplace_back(s, "block map");
  }
  scenes.emplace_back(carvedScene(rng), "carved map");  // ragged faces
  for (const auto& [scene, name] : scenes) {
    const Built b = build(scene, name, /*brute_force=*/false);
    const OcTree& tree = *b.tree;
    const double res = tree.getResolution();
    const Eigen::Vector3d bmin((b.box.lo[0] - kO) * res, (b.box.lo[1] - kO) * res, (b.box.lo[2] - kO) * res);
    const Eigen::Vector3d bmax((b.box.lo[0] + b.box.n[0] - kO) * res, (b.box.lo[1] + b.box.n[1] - kO) * res,
                               (b.box.lo[2] + b.box.n[2] - kO) * res);
    const double md = 0.4;
    {
      const DistanceField plain(tree, md, 4);
      checkCorners(plain, b.box, res, name + " uncropped");
      // An infinite crop is the plain constructor, bit for bit.
      const DistanceField inf(tree, md, 4, Eigen::Vector3d::Constant(-kInfD), Eigen::Vector3d::Constant(kInfD));
      const std::vector<float> a = allCells(plain, tree, b.box), c = allCells(inf, tree, b.box);
      expect(inf.sizeX() == plain.sizeX() && inf.sizeY() == plain.sizeY() && inf.sizeZ() == plain.sizeZ() &&
                 std::memcmp(a.data(), c.data(), a.size() * sizeof(float)) == 0,
             name + ": an infinite crop differs from no crop");
    }
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> crops;
    // Random, not aligned to the grid, some reaching past the map.
    std::uniform_real_distribution<double> ulo(-0.1, 0.7), ulen(0.15, 0.75);
    for (int i = 0; i < 6; ++i) {
      Eigen::Vector3d lo, hi;
      for (int a = 0; a < 3; ++a) {
        lo[a] = bmin[a] + ulo(rng) * (bmax[a] - bmin[a]);
        hi[a] = lo[a] + ulen(rng) * (bmax[a] - bmin[a]);
      }
      crops.push_back({lo, hi});
    }
    // Bounds exactly on cell centres (inclusive) and through the big blocks.
    crops.push_back({bmin + Eigen::Vector3d::Constant(10.5 * res), bmin + Eigen::Vector3d::Constant(19.5 * res)});
    crops.push_back({bmin + Eigen::Vector3d(12 * res, -kInfD, 4 * res), Eigen::Vector3d(kInfD, bmin.y() + 20.2 * res, kInfD)});
    // Bounds on the centres of the map's own first and second cells (and
    // last), on each axis.
    for (int a = 0; a < 3; ++a)
      for (int off = 0; off <= 1; ++off) {
        Eigen::Vector3d lo = Eigen::Vector3d::Constant(-kInfD), hi = Eigen::Vector3d::Constant(kInfD);
        lo[a] = bmin[a] + (off + 0.5) * res;
        crops.push_back({lo, Eigen::Vector3d::Constant(kInfD)});
        hi[a] = bmax[a] - (off + 0.5) * res;
        crops.push_back({Eigen::Vector3d::Constant(-kInfD), hi});
      }
    for (const auto& [lo, hi] : crops) {
      const Box cbox = cropOf(tree, b.box, lo, hi);
      const std::string label = name + " cropped to [" + std::to_string(lo.x()) + "," +
                                std::to_string(lo.y()) + "," + std::to_string(lo.z()) + "]..[" +
                                std::to_string(hi.x()) + "," + std::to_string(hi.y()) + "," +
                                std::to_string(hi.z()) + "]";
      expect(cbox.cells() > 0 && cbox.cells() <= b.box.cells(), label + ": empty crop (test setup)");
      // Ground truth: only the occupied voxels inside the crop count.
      const std::vector<std::uint8_t> occ = readOccupancy(tree, cbox);
      const std::vector<std::int64_t> d2 = bruteForceD2(cbox, occ);
      const DistanceField df(tree, md, 3, lo, hi);
      checkField(df, tree, cbox, d2, md, label, rng);
      checkCorners(df, cbox, res, label);
      const DistanceField one(tree, md, 1, lo, hi);
      const std::vector<float> a = allCells(df, tree, cbox), c = allCells(one, tree, cbox);
      expect(std::memcmp(a.data(), c.data(), a.size() * sizeof(float)) == 0,
             label + ": 1 thread differs from 3");
    }
    // Crops that leave nothing: beside the map, between two cell centres,
    // inverted, NaN.
    const point3d probe = cellCentre(tree, b.box, b.box.n[0] / 2, b.box.n[1] / 2, b.box.n[2] / 2);
    const Eigen::Vector3d c(probe.x(), probe.y(), probe.z());
    const Eigen::Vector3d nan = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    int k = 0;
    for (const auto& [lo, hi] : std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>{
             {bmax + Eigen::Vector3d(0.1, -1, -1), bmax + Eigen::Vector3d(1, 1, 1)},
             {c + Eigen::Vector3d::Constant(0.1 * res), c + Eigen::Vector3d::Constant(0.9 * res)},
             {c, c - Eigen::Vector3d::Constant(res)},
             {nan, Eigen::Vector3d::Constant(kInfD)}}) {
      const DistanceField df(tree, md, 4, lo, hi);
      expectEmpty(df, probe, name + " empty crop " + std::to_string(k++));
    }
  }
  {
    OcTree empty(0.05);
    const DistanceField df(empty, 1.0, 4, Eigen::Vector3d::Constant(-1), Eigen::Vector3d::Constant(1));
    expectEmpty(df, point3d(0, 0, 0), "empty tree cropped");
  }
}

// A scene for the grid: a carved free region inside unknown space (so the
// shell is big), clutter inside it and stray obstacles outside.
Scene carvedScene(std::mt19937& rng) {
  // 80 cells along x, so grid rows span two 64-cell words.
  Scene s(0.05, kO - 40, kO - 10, kO - 7, 80, 20, 14);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  for (int z = 0; z < 14; ++z)
    for (int y = 0; y < 20; ++y)
      for (int x = 0; x < 80; ++x) {
        const double d1 = std::hypot(std::hypot(x - 16.0, y - 10.0), z - 7.0);
        const double d2 = std::hypot(std::hypot(x - 58.0, y - 9.0), z - 6.0);
        const double tunnel = std::hypot(y - 10.0, z - 7.0) + (x < 16 || x > 58 ? 99 : 0);
        if (std::min({d1 - 7.0, d2 - 8.0, tunnel - 3.0}) < u(rng) - 0.5)
          s.at(x, y, z) = u(rng) < 0.02 ? kOcc : kFree;
        else if (u(rng) < 0.003)
          s.at(x, y, z) = kOcc;
      }
  s.fill(8, 8, 4, 15, 15, 7, kFree);  // an aligned block, so free leaves merge
  return s;
}

// `raw` with `grid`'s shell cells stamped occupied, the ball freed, and the
// grid box's two opposite corners made known (free) if they were not, so the
// tree's box holds the grid's.
std::unique_ptr<OcTree> stampedFromGrid(const OcTree& raw, const ConservativeGrid& g) {
  auto t = std::make_unique<OcTree>(raw);
  const Box gb = [&] {
    Box b;
    b.lo[0] = g.keyX0(); b.lo[1] = g.keyY0(); b.lo[2] = g.keyZ0();
    b.n[0] = g.sizeX(); b.n[1] = g.sizeY(); b.n[2] = g.sizeZ();
    return b;
  }();
  for (int z = 0; z < gb.n[2]; ++z)
    for (int y = 0; y < gb.n[1]; ++y)
      for (int x = 0; x < gb.n[0]; ++x) {
        const std::uint8_t c = g.data()[gb.index(x, y, z)];
        const OcTreeKey k = gb.key(x, y, z);
        if (c == ConservativeGrid::kShell) t->setNodeValue(k, t->getClampingThresMaxLog(), true);
        else if (c == ConservativeGrid::kFree && !raw.search(k)) t->setNodeValue(k, t->getClampingThresMinLog(), true);
      }
  for (const OcTreeKey& k : {gb.key(0, 0, 0), gb.key(gb.n[0] - 1, gb.n[1] - 1, gb.n[2] - 1)})
    if (!t->search(k)) t->setNodeValue(k, t->getClampingThresMinLog(), true);
  t->updateInnerOccupancy();
  return t;
}

Box gridBox(const ConservativeGrid& g) {
  Box b;
  if (g.empty()) return b;
  b.lo[0] = g.keyX0(); b.lo[1] = g.keyY0(); b.lo[2] = g.keyZ0();
  b.n[0] = g.sizeX(); b.n[1] = g.sizeY(); b.n[2] = g.sizeZ();
  return b;
}

void checkGridField(std::mt19937& rng) {
  std::vector<std::pair<Scene, std::string>> scenes;
  scenes.emplace_back(carvedScene(rng), "carved map");
  {
    const int lo[3] = {kO - 15, kO - 11, kO - 7}, n[3] = {30, 24, 20};
    scenes.emplace_back(randomScene(0.05, lo, n, 0.02, 0.6, rng), "random map");  // unknown everywhere
  }
  for (const auto& [scene, name] : scenes) {
    const Built b = build(scene, name, /*brute_force=*/false);
    const OcTree& raw = *b.tree;
    const double res = raw.getResolution();
    const point3d drone = cellCentre(raw, b.box, b.box.n[0] / 3, b.box.n[1] / 2, b.box.n[2] / 2);
    const double radius = 0.2;

    // The grid's shell is stampUnknownShell's.
    OcTree stamped(raw);
    const auto ref = drone_core::planning::stampUnknownShell(stamped, drone, radius);
    const ConservativeGrid g(raw, drone, radius, Eigen::Vector3d::Constant(-kInfD),
                             Eigen::Vector3d::Constant(kInfD), true, 4);
    const Box gb = gridBox(g);
    {
      std::size_t mismatch = 0, shell = 0;
      for (int z = 0; z < gb.n[2]; ++z)
        for (int y = 0; y < gb.n[1]; ++y)
          for (int x = 0; x < gb.n[0]; ++x) {
            const OcTreeKey k = gb.key(x, y, z);
            const auto* s = stamped.search(k);
            const bool stamped_shell = !raw.search(k) && s && stamped.isNodeOccupied(s);
            const bool grid_shell = g.data()[gb.index(x, y, z)] == ConservativeGrid::kShell;
            mismatch += stamped_shell != grid_shell;
            shell += grid_shell;
          }
      expect(mismatch == 0 && shell == ref.stamped && g.stats().shell == ref.stamped,
             name + ": the grid's shell differs from stampUnknownShell's in " +
                 std::to_string(mismatch) + " cells (" + std::to_string(shell) + " vs " +
                 std::to_string(ref.stamped) + " stamped)");
      expect(shell > 100, name + ": hardly any shell (test setup)");
    }

    // Uncropped and cropped grids against the tree constructor on the stamped
    // equivalent (same crop), and against brute force.
    const Eigen::Vector3d c(drone.x(), drone.y(), drone.z());
    const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> crops = {
        {Eigen::Vector3d::Constant(-kInfD), Eigen::Vector3d::Constant(kInfD)},
        {c - Eigen::Vector3d(0.33, 0.21, 0.4), c + Eigen::Vector3d(0.52, 0.17, 0.08)},
        {c - Eigen::Vector3d(kInfD, 0.1, kInfD), Eigen::Vector3d::Constant(kInfD)},
    };
    for (std::size_t ci = 0; ci < crops.size(); ++ci) {
      const auto& [lo, hi] = crops[ci];
      const ConservativeGrid cg(raw, drone, radius, lo, hi, true, 4);
      const Box cb = gridBox(cg);
      const std::string label = name + " grid, crop " + std::to_string(ci);
      expect(cb.cells() > 0, label + ": empty grid (test setup)");
      const std::unique_ptr<OcTree> eq = stampedFromGrid(raw, cg);
      std::vector<std::uint8_t> occ(cb.cells());
      for (std::size_t i = 0; i < occ.size(); ++i)
        occ[i] = ConservativeGrid::isObstacle(static_cast<ConservativeGrid::Cell>(cg.data()[i]));
      const std::vector<std::int64_t> d2 = bruteForceD2(cb, occ);
      for (double md : {0.3, 7.0}) {
        const std::string l = label + " maxdist " + std::to_string(md);
        const DistanceField fg(cg, md, 4);
        const DistanceField ft(*eq, md, 4, lo, hi);
        expect(fg.sizeX() == cb.n[0] && fg.sizeY() == cb.n[1] && fg.sizeZ() == cb.n[2],
               l + ": the field's box is not the grid's");
        expect(ft.sizeX() == cb.n[0] && ft.sizeY() == cb.n[1] && ft.sizeZ() == cb.n[2],
               l + ": the stamped tree's box is not the grid's (test setup)");
        if (fg.sizeX() != cb.n[0] || ft.sizeX() != cb.n[0]) continue;
        const std::vector<float> a = allCells(fg, raw, cb), t = allCells(ft, raw, cb);
        expect(std::memcmp(a.data(), t.data(), a.size() * sizeof(float)) == 0,
               l + ": differs from the tree constructor on the stamped equivalent");
        checkField(fg, raw, cb, d2, md, l, rng);
        checkCorners(fg, cb, res, l);
        for (unsigned th : {1u, 2u, 8u}) {
          const DistanceField o(cg, md, th);
          const std::vector<float> v = allCells(o, raw, cb);
          expect(std::memcmp(a.data(), v.data(), a.size() * sizeof(float)) == 0,
                 l + ": " + std::to_string(th) + " threads differ from 4");
        }
      }
    }
    // An empty grid gives an empty field.
    const ConservativeGrid eg(raw, drone, radius, c + Eigen::Vector3d(0, 0, 40), c + Eigen::Vector3d(1, 1, 41));
    expect(eg.empty(), name + ": a crop beside the map gave a non-empty grid (test setup)");
    expectEmpty(DistanceField(eg, 1.0, 4), drone, name + " empty grid");
  }
}

}  // namespace

int main() {
  const auto t0 = std::chrono::steady_clock::now();
  std::mt19937 rng(12345);

  checkSmallMaps(rng);
  checkMediumMap(rng);
  checkPlatesMap(kPlatesSeed, rng);
  checkCrop(rng);
  checkGridField(rng);

  // The long maps under several thread counts too (they take the envelope and
  // 32-bit paths, whose work split differs from the windowed pass).
  {
    Scene s(0.05, kO - 1, kO - 100, kO - 100, 3, 200, 200);
    s.at(0, 0, 0) = kOcc;
    s.at(2, 199, 199) = kFree;
    Scene s16(0.05, kO - 10, kO - 75, kO - 75, 20, 150, 150);
    s16.at(0, 0, 0) = kOcc;
    s16.at(5, 100, 30) = kOcc;
    s16.at(19, 149, 149) = kFree;
    for (const auto& [scene, md] : {std::make_pair(&s, 13.3), std::make_pair(&s16, 7.0)}) {
      const Built b = build(*scene, "thread-count map", /*brute_force=*/false);
      const DistanceField one(*b.tree, md, 1);
      const std::vector<float> ref = allCells(one, *b.tree, b.box);
      for (unsigned t : {2u, 3u, 8u}) {
        const DistanceField other(*b.tree, md, t);
        const std::vector<float> v = allCells(other, *b.tree, b.box);
        expect(std::memcmp(v.data(), ref.data(), v.size() * sizeof(float)) == 0,
               "long map maxdist " + std::to_string(md) + ": " + std::to_string(t) +
                   " threads differ from 1 thread");
      }
    }
  }

  std::cout << "total " << msSince(t0) << " ms\n";
  if (failures == 0) {
    std::cout << "distance_field: all checks passed\n";
    return 0;
  }
  std::cerr << failures << " check(s) failed\n";
  return 1;
}
