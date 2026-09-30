#include "drone_core/planning/unknown_shell.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <vector>

namespace drone_core::planning {

UnknownShellStats stampUnknownShell(octomap::OcTree& tree, const octomap::point3d& center,
                                    double keep_out_radius) {
  UnknownShellStats stats;
  const double res = tree.getResolution();
  using Clock = std::chrono::steady_clock;
  const auto ms_since = [](Clock::time_point t) {
    return std::chrono::duration<double, std::milli>(Clock::now() - t).count();
  };
  auto t = Clock::now();

  // 1. The ball: never-observed voxels around the drone become free.
  octomap::OcTreeKey ck;
  if (keep_out_radius > 0.0 && tree.coordToKeyChecked(center, ck)) {
    const int n = static_cast<int>(std::ceil(keep_out_radius / res));
    const double r2 = keep_out_radius * keep_out_radius;
    const float free_log = tree.getClampingThresMinLog();
    for (int dx = -n; dx <= n; ++dx)
      for (int dy = -n; dy <= n; ++dy)
        for (int dz = -n; dz <= n; ++dz) {
          const octomap::OcTreeKey k(static_cast<octomap::key_type>(ck[0] + dx),
                                     static_cast<octomap::key_type>(ck[1] + dy),
                                     static_cast<octomap::key_type>(ck[2] + dz));
          if ((tree.keyToCoord(k) - center).norm_sq() > r2) continue;
          if (tree.search(k)) continue;
          tree.setNodeValue(k, free_log, /*lazy_eval=*/true);
          ++stats.ball_freed;
        }
  }
  stats.ball_ms = ms_since(t);
  if (tree.size() == 0) return stats;
  t = Clock::now();

  // 2. A dense grid of the tree's bounding box plus one voxel on every side
  // (outside the box is never observed), filled from the leaves. Looking the
  // neighbours up in it instead of searching the tree 26 times per free voxel
  // is what makes this affordable on every map.
  double x0, y0, z0, x1, y1, z1;
  tree.getMetricMin(x0, y0, z0);
  tree.getMetricMax(x1, y1, z1);
  const octomap::OcTreeKey kmin = tree.coordToKey(x0 + 0.5 * res, y0 + 0.5 * res, z0 + 0.5 * res);
  const octomap::OcTreeKey kmax = tree.coordToKey(x1 - 0.5 * res, y1 - 0.5 * res, z1 - 0.5 * res);
  const int ox = kmin[0] - 1, oy = kmin[1] - 1, oz = kmin[2] - 1;
  const int nx = kmax[0] - kmin[0] + 3, ny = kmax[1] - kmin[1] + 3, nz = kmax[2] - kmin[2] + 3;
  const std::size_t sx = 1, sy = static_cast<std::size_t>(nx),
                    sz = static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny);
  enum : std::uint8_t { kUnknown = 0, kOccupied = 1, kFree = 2, kShell = 3 };
  std::vector<std::uint8_t> grid(sz * static_cast<std::size_t>(nz), kUnknown);
  const auto index = [&](int x, int y, int z) {
    return static_cast<std::size_t>(x - ox) * sx + static_cast<std::size_t>(y - oy) * sy +
           static_cast<std::size_t>(z - oz) * sz;
  };

  const unsigned max_depth = tree.getTreeDepth();
  for (auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it) {
    const std::uint8_t v = tree.isNodeOccupied(*it) ? kOccupied : kFree;
    if (v == kFree) ++stats.free_leaves;
    const int span = 1 << (max_depth - it.getDepth());
    const octomap::OcTreeKey base = it.getIndexKey();  // lowest-index voxel of the leaf
    for (int i = 0; i < span; ++i)
      for (int j = 0; j < span; ++j)
        for (int l = 0; l < span; ++l) grid[index(base[0] + i, base[1] + j, base[2] + l)] = v;
  }

  stats.grid_ms = ms_since(t);
  t = Clock::now();

  // 3. Every never-observed cell touching a free one (26-neighbourhood) is shell.
  // Free cells never sit on the padding layer, so every neighbour is in the grid.
  std::array<std::ptrdiff_t, 26> offsets{};
  {
    int m = 0;
    for (int dz = -1; dz <= 1; ++dz)
      for (int dy = -1; dy <= 1; ++dy)
        for (int dx = -1; dx <= 1; ++dx)
          if (dx || dy || dz)
            offsets[m++] = dx * static_cast<std::ptrdiff_t>(sx) +
                           dy * static_cast<std::ptrdiff_t>(sy) +
                           dz * static_cast<std::ptrdiff_t>(sz);
  }
  std::vector<std::size_t> shell;
  for (std::size_t c = 0; c < grid.size(); ++c) {
    if (grid[c] != kFree) continue;
    for (const auto off : offsets) {
      const std::size_t q = c + off;
      if (grid[q] == kUnknown) {
        grid[q] = kShell;
        shell.push_back(q);
      }
    }
  }

  stats.sweep_ms = ms_since(t);
  t = Clock::now();

  // 4. Stamp them.
  const float occ_log = tree.getClampingThresMaxLog();
  for (const std::size_t q : shell) {
    const int z = static_cast<int>(q / sz);
    const int y = static_cast<int>((q % sz) / sy);
    const int x = static_cast<int>(q % sy);
    tree.setNodeValue(octomap::OcTreeKey(static_cast<octomap::key_type>(x + ox),
                                         static_cast<octomap::key_type>(y + oy),
                                         static_cast<octomap::key_type>(z + oz)),
                      occ_log, /*lazy_eval=*/true);
  }
  tree.updateInnerOccupancy();
  stats.stamp_ms = ms_since(t);
  stats.stamped = shell.size();
  return stats;
}

}  // namespace drone_core::planning
