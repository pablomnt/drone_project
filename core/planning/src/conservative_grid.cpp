#include "drone_core/planning/conservative_grid.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstring>
#include <exception>
#include <limits>
#include <mutex>
#include <thread>

// Build outline:
//   1. The box of all leaves, by six branch-and-bound descents (one per face)
//      rather than a walk of the whole tree, each stopping once it passes the
//      crop; plus the keep-out's cells.
//   2. Allocate the grid (kUnknown everywhere) and walk only the subtrees that
//      reach into it (parallel over subtrees), stamping each leaf straight in.
//      Leaves are disjoint, so every byte has at most one writer. With 1 and 2
//      together, map outside the crop costs next to nothing.
//   3. The keep-out (serial, a few thousand cells).
//   4. The sweep, as a dilation of the free set by the 3x3x3 cube on bitsets
//      (64 cells per word): pack the free cells of every row whose 26
//      neighbours are all in the grid into bits and dilate them along x, then OR
//      each row's 3x3 block of neighbouring rows (y and z) and mark every
//      never-observed cell under the result as shell. Shell membership depends
//      only on the grid as it stood after the ball (never on a cell another
//      thread has just marked), and the first half only reads the grid while
//      the second only writes each row's own cells, so the result is the same
//      for any thread count and there is no data race.

namespace drone_core::planning {

namespace {

using Clock = std::chrono::steady_clock;
double msSince(Clock::time_point t) {
  return std::chrono::duration<double, std::milli>(Clock::now() - t).count();
}

// Runs fn(task, worker) for task in [0, n) on up to `threads` threads (the
// calling thread is one of them), handing tasks out dynamically. An exception
// from a task is rethrown on the calling thread once all threads have stopped.
// (A copy of DistanceField's.)
template <typename Fn>
void parallelFor(std::size_t n, unsigned threads, Fn&& fn) {
  if (n == 0) return;
  const unsigned t = static_cast<unsigned>(std::min<std::size_t>(threads, n));
  if (t <= 1) {
    for (std::size_t i = 0; i < n; ++i) fn(i, 0u);
    return;
  }
  std::atomic<std::size_t> next{0};
  std::exception_ptr error;
  std::mutex error_mutex;
  auto work = [&](unsigned worker) {
    try {
      for (;;) {
        const std::size_t i = next.fetch_add(1, std::memory_order_relaxed);
        if (i >= n) break;
        fn(i, worker);
      }
    } catch (...) {
      next.store(n, std::memory_order_relaxed);
      std::lock_guard<std::mutex> lock(error_mutex);
      if (!error) error = std::current_exception();
    }
  };
  std::vector<std::thread> pool;
  pool.reserve(t - 1);
  for (unsigned w = 1; w < t; ++w) {
    try {
      pool.emplace_back(work, w);
    } catch (...) {
      break;
    }
  }
  work(0u);
  for (auto& th : pool) th.join();
  if (error) std::rethrow_exception(error);
}

// ------------------------------------------------------------- crop in keys

// Far beyond octomap's 16-bit key range, and small enough that adding the key
// offset cannot overflow an int.
constexpr double kKeyClamp = 1 << 20;

double cellCentre(int key, int offset, double res) {
  return (static_cast<double>(key - offset) + 0.5) * res;  // octomap's keyToCoord
}

// The smallest key whose cell centre is >= v (NaN: an empty range).
int firstKeyAtOrAbove(double v, double res, double inv_res, int offset) {
  if (std::isnan(v)) return INT_MAX / 2;
  const double t = std::ceil(v * inv_res - 0.5);
  if (!(std::abs(t) < kKeyClamp)) return t < 0 ? -static_cast<int>(kKeyClamp) : static_cast<int>(kKeyClamp);
  int k = static_cast<int>(t) + offset;
  while (cellCentre(k, offset, res) < v) ++k;
  while (cellCentre(k - 1, offset, res) >= v) --k;
  return k;
}

// The largest key whose cell centre is <= v (NaN: an empty range).
int lastKeyAtOrBelow(double v, double res, double inv_res, int offset) {
  if (std::isnan(v)) return INT_MIN / 2;
  const double t = std::floor(v * inv_res - 0.5);
  if (!(std::abs(t) < kKeyClamp)) return t < 0 ? -static_cast<int>(kKeyClamp) : static_cast<int>(kKeyClamp);
  int k = static_cast<int>(t) + offset;
  while (cellCentre(k, offset, res) > v) --k;
  while (cellCentre(k + 1, offset, res) <= v) ++k;
  return k;
}

// ------------------------------------------------------------- octree walks

// The extent of the leaves along one axis, by branch and bound: the lowest
// leaf key (`upper` false) or the highest (`upper` true) over all leaves,
// visiting the nearer half of each node's children first and skipping any
// subtree that cannot beat the best so far. Every node of an octomap tree has
// at least one leaf below it, so a node whose near half has any child never
// needs its far half. Stops early once the answer reaches `enough` (the crop
// clamps it there anyway). Visits a small fraction of the tree on a real map.
struct Extent {
  const octomap::OcTree& tree;
  int axis;
  bool upper;
  int enough;
  int best;

  void search(const octomap::OcTreeNode* node, unsigned level, const std::uint16_t k[3]) {
    const int lo = k[axis], hi = k[axis] + (1 << level) - 1;
    if (upper ? hi <= best : lo >= best) return;
    if (level == 0 || !tree.nodeHasChildren(node)) {
      best = upper ? hi : lo;
      return;
    }
    const std::uint16_t half = static_cast<std::uint16_t>(1u << (level - 1));
    for (unsigned pass = 0; pass < 2; ++pass) {
      const unsigned side = upper ? 1 - pass : pass;  // near half first
      for (unsigned i = 0; i < 8; ++i) {
        if (((i >> axis) & 1u) != side || !tree.nodeChildExists(node, i)) continue;
        const std::uint16_t ck[3] = {static_cast<std::uint16_t>(k[0] + ((i & 1) ? half : 0)),
                                     static_cast<std::uint16_t>(k[1] + ((i & 2) ? half : 0)),
                                     static_cast<std::uint16_t>(k[2] + ((i & 4) ? half : 0))};
        search(tree.getNodeChild(node, i), level - 1, ck);
        if (upper ? best >= enough : best <= enough) return;
      }
    }
  }
};

// The leaves' box in keys (inclusive), each bound only as far as `stop_lo` /
// `stop_hi` (beyond them the caller clamps anyway); lo > hi for an empty tree.
void leafBox(const octomap::OcTree& tree, const int stop_lo[3], const int stop_hi[3], int lo[3],
             int hi[3]) {
  for (int a = 0; a < 3; ++a) {
    lo[a] = INT_MAX;
    hi[a] = INT_MIN;
  }
  const octomap::OcTreeNode* root = tree.getRoot();
  if (root == nullptr) return;
  const std::uint16_t k0[3] = {0, 0, 0};
  for (int a = 0; a < 3; ++a) {
    Extent lower{tree, a, false, stop_lo[a], INT_MAX};
    lower.search(root, tree.getTreeDepth(), k0);
    Extent higher{tree, a, true, stop_hi[a], INT_MIN};
    higher.search(root, tree.getTreeDepth(), k0);
    lo[a] = lower.best;
    hi[a] = higher.best;
  }
}

// Stamps every leaf reaching into the grid's box straight into the grid,
// skipping subtrees that miss it. Leaves are disjoint, so concurrent walkers
// write disjoint bytes.
struct Filler {
  const octomap::OcTree& tree;
  int k0[3], n[3];  // the grid's first key and size
  std::uint8_t* cells;
  std::size_t sy, sz;

  bool misses(unsigned level, const std::uint16_t k[3]) const {
    const int span = 1 << level;
    for (int a = 0; a < 3; ++a)
      if (k[a] + span - 1 < k0[a] || k[a] >= k0[a] + n[a]) return true;
    return false;
  }

  void stamp(const octomap::OcTreeNode* node, unsigned level, const std::uint16_t k[3]) const {
    const int span = 1 << level;
    int a0[3], a1[3];
    for (int a = 0; a < 3; ++a) {
      a0[a] = std::max(0, k[a] - k0[a]);
      a1[a] = std::min(n[a] - 1, k[a] - k0[a] + span - 1);
    }
    const std::uint8_t v = tree.isNodeOccupied(node) ? ConservativeGrid::kOccupied : ConservativeGrid::kFree;
    const std::size_t len = static_cast<std::size_t>(a1[0] - a0[0] + 1);
    for (int z = a0[2]; z <= a1[2]; ++z)
      for (int y = a0[1]; y <= a1[1]; ++y)
        std::memset(cells + static_cast<std::size_t>(z) * sz + static_cast<std::size_t>(y) * sy + a0[0], v,
                    len);
  }

  void walk(const octomap::OcTreeNode* node, unsigned level, const std::uint16_t k[3]) const {
    if (misses(level, k)) return;
    bool inner = false;
    if (level > 0) {
      const std::uint16_t half = static_cast<std::uint16_t>(1u << (level - 1));
      for (unsigned i = 0; i < 8; ++i) {
        if (!tree.nodeChildExists(node, i)) continue;
        inner = true;
        const std::uint16_t ck[3] = {static_cast<std::uint16_t>(k[0] + ((i & 1) ? half : 0)),
                                     static_cast<std::uint16_t>(k[1] + ((i & 2) ? half : 0)),
                                     static_cast<std::uint16_t>(k[2] + ((i & 4) ? half : 0))};
        walk(tree.getNodeChild(node, i), level - 1, ck);
      }
    }
    if (!inner) stamp(node, level, k);
  }
};

struct Frontier {
  const octomap::OcTreeNode* node;
  unsigned level;
  std::uint16_t k[3];
};

// Splits the top of the tree (within the grid's box) into enough independent
// subtrees for `threads` workers and fills from them in parallel.
void fillTree(const Filler& filler, unsigned threads) {
  const octomap::OcTree& tree = filler.tree;
  const octomap::OcTreeNode* root = tree.getRoot();
  if (root == nullptr) return;
  std::vector<Frontier> frontier;
  const std::uint16_t k0[3] = {0, 0, 0};
  if (!filler.misses(tree.getTreeDepth(), k0)) frontier.push_back({root, tree.getTreeDepth(), {0, 0, 0}});
  const std::size_t want = static_cast<std::size_t>(threads) * 32;
  for (unsigned round = 0; round < 16 && !frontier.empty() && frontier.size() < want; ++round) {
    std::vector<Frontier> next;
    next.reserve(frontier.size() * 8);
    for (const Frontier& f : frontier) {
      bool inner = false;
      if (f.level > 0) {
        const std::uint16_t half = static_cast<std::uint16_t>(1u << (f.level - 1));
        for (unsigned i = 0; i < 8; ++i) {
          if (!tree.nodeChildExists(f.node, i)) continue;
          inner = true;
          const Frontier c{tree.getNodeChild(f.node, i), f.level - 1,
                           {static_cast<std::uint16_t>(f.k[0] + ((i & 1) ? half : 0)),
                            static_cast<std::uint16_t>(f.k[1] + ((i & 2) ? half : 0)),
                            static_cast<std::uint16_t>(f.k[2] + ((i & 4) ? half : 0))}};
          if (!filler.misses(c.level, c.k)) next.push_back(c);
        }
      }
      if (!inner) filler.stamp(f.node, f.level, f.k);
    }
    frontier.swap(next);
  }
  parallelFor(frontier.size(), threads, [&](std::size_t i, unsigned) {
    filler.walk(frontier[i].node, frontier[i].level, frontier[i].k);
  });
}

// ------------------------------------------------------------- bit packing

// Packs a per-byte flag of up to 64 cells into one word, bit i = cell i. `lsb`
// maps 8 cells loaded as a little-endian word to a word whose byte i is 1 if
// cell i is flagged and 0 otherwise; the multiply gathers the 8 low bits into
// the top byte (no two partial products share a bit, so nothing carries).
template <typename Lsb, typename Flag>
inline std::uint64_t pack(const std::uint8_t* p, int n, Lsb lsb, Flag flag) {
  std::uint64_t out = 0;
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  if (n == 64) {
    for (int i = 0; i < 8; ++i) {
      std::uint64_t x;
      std::memcpy(&x, p + 8 * i, 8);
      out |= ((lsb(x) * 0x0102040810204080ULL) >> 56) << (8 * i);
    }
    return out;
  }
#endif
  for (int i = 0; i < n; ++i) out |= static_cast<std::uint64_t>(flag(p[i]) ? 1 : 0) << i;
  return out;
}

constexpr std::uint64_t kLsbs = 0x0101010101010101ULL;
// Cell values are 0..3, so bit 1 and bit 0 of each byte decide.
inline std::uint64_t lsbFree(std::uint64_t x) { return (x >> 1) & ~x & kLsbs; }       // == 2
inline std::uint64_t lsbUnknown(std::uint64_t x) { return ~((x >> 1) | x) & kLsbs; }  // == 0
inline bool isFree(std::uint8_t c) { return c == ConservativeGrid::kFree; }
inline bool isUnknownCell(std::uint8_t c) { return c == ConservativeGrid::kUnknown; }

}  // namespace

bool ConservativeGrid::KeepOut::contains(const octomap::point3d& p) const {
  const Eigen::Vector3d d(p.x() - center.x(), p.y() - center.y(), p.z() - center.z());
  const double s = forward_len > 0.0 ? d.dot(forward) : 0.0;
  if (s <= 0.0) return d.squaredNorm() <= radius * radius;  // the ball behind
  // The cylinder ahead: within forward_len along the axis, within radius of it.
  return s <= forward_len && (d - s * forward).squaredNorm() <= radius * radius;
}

double ConservativeGrid::KeepOut::extent() const {
  const bool cyl = forward_len > 0.0 && forward.squaredNorm() > 0.0;
  return cyl ? std::hypot(radius, forward_len) : radius;
}

ConservativeGrid::ConservativeGrid(const octomap::OcTree& raw, const KeepOut& keep_out,
                                   const Eigen::Vector3d& crop_lo, const Eigen::Vector3d& crop_hi,
                                   bool shell, unsigned threads)
    : res_(raw.getResolution()),
      inv_res_(1.0 / raw.getResolution()),  // octomap's resolution_factor, bit for bit
      key_offset_(1 << (raw.getTreeDepth() - 1)) {
  if (threads == 0) threads = std::max(1u, std::thread::hardware_concurrency());
  auto t = Clock::now();

  // The crop in keys, also clamped to octomap's key range (nothing beyond it
  // can be represented or queried).
  const int kmax = 2 * key_offset_ - 1;
  int clo[3], chi[3];
  for (int a = 0; a < 3; ++a) {
    clo[a] = std::max(0, firstKeyAtOrAbove(crop_lo[a], res_, inv_res_, key_offset_));
    chi[a] = std::min(kmax, lastKeyAtOrBelow(crop_hi[a], res_, inv_res_, key_offset_));
    if (clo[a] > chi[a]) {  // the crop holds no cell at all
      stats_.fill_ms = msSince(t);
      return;
    }
  }

  // 1. The tree's box, each bound searched only as far as the crop (grown by
  // the padding cell) can use it.
  int lo[3], hi[3];
  {
    int stop_lo[3], stop_hi[3];
    for (int a = 0; a < 3; ++a) {
      stop_lo[a] = clo[a] + 1;
      stop_hi[a] = chi[a] - 1;
    }
    leafBox(raw, stop_lo, stop_hi, lo, hi);
  }

  // The keep-out's cells (for a ball, the same test as stampUnknownShell),
  // which the box must hold whether or not they fall in the crop.
  std::vector<std::array<int, 3>> ball;
  octomap::OcTreeKey ck;
  const octomap::point3d& center = keep_out.center;
  if (shell && keep_out.radius > 0.0 && std::isfinite(center.x()) && std::isfinite(center.y()) &&
      std::isfinite(center.z()) && keep_out.forward.allFinite() &&
      raw.coordToKeyChecked(center, ck)) {
    const int n = static_cast<int>(std::ceil(keep_out.extent() / res_));
    for (int dz = -n; dz <= n; ++dz)
      for (int dy = -n; dy <= n; ++dy)
        for (int dx = -n; dx <= n; ++dx) {
          const int k[3] = {ck[0] + dx, ck[1] + dy, ck[2] + dz};
          if (k[0] < 0 || k[1] < 0 || k[2] < 0 || k[0] > kmax || k[1] > kmax || k[2] > kmax)
            continue;
          const octomap::OcTreeKey key(static_cast<octomap::key_type>(k[0]),
                                       static_cast<octomap::key_type>(k[1]),
                                       static_cast<octomap::key_type>(k[2]));
          if (!keep_out.contains(raw.keyToCoord(key))) continue;
          ball.push_back({k[0], k[1], k[2]});
          for (int a = 0; a < 3; ++a) {
            lo[a] = std::min(lo[a], k[a]);
            hi[a] = std::max(hi[a], k[a]);
          }
        }
  }

  // The box: grown by one cell, within the key range, cut to the crop.
  if (lo[0] > hi[0]) {  // no leaves and no ball
    stats_.fill_ms = msSince(t);
    return;
  }
  for (int a = 0; a < 3; ++a) {
    lo[a] = std::max(lo[a] - 1, clo[a]);
    hi[a] = std::min(hi[a] + 1, chi[a]);
    if (lo[a] > hi[a]) {  // cropped away entirely
      stats_.fill_ms = msSince(t);
      return;
    }
  }
  kx0_ = lo[0];
  ky0_ = lo[1];
  kz0_ = lo[2];
  nx_ = hi[0] - lo[0] + 1;
  ny_ = hi[1] - lo[1] + 1;
  nz_ = hi[2] - lo[2] + 1;
  const std::size_t sy = static_cast<std::size_t>(nx_);
  const std::size_t sz = sy * static_cast<std::size_t>(ny_);
  const std::size_t rows = static_cast<std::size_t>(ny_) * nz_;

  // 2. Fill. Every cell starts never observed.
  cells_.assign(sz * static_cast<std::size_t>(nz_), kUnknown);
  fillTree(Filler{raw, {kx0_, ky0_, kz0_}, {nx_, ny_, nz_}, cells_.data(), sy, sz}, threads);
  stats_.fill_ms = msSince(t);
  if (!shell) return;
  t = Clock::now();

  // 3. The keep-out: never-observed cells around the drone become free.
  for (const auto& k : ball) {
    const int x = k[0] - kx0_, y = k[1] - ky0_, z = k[2] - kz0_;
    if (x < 0 || y < 0 || z < 0 || x >= nx_ || y >= ny_ || z >= nz_) continue;
    std::uint8_t& c = cells_[static_cast<std::size_t>(z) * sz + static_cast<std::size_t>(y) * sy + x];
    if (c != kUnknown) continue;
    c = kFree;
    ++stats_.ball_freed;
  }
  stats_.ball_ms = msSince(t);
  t = Clock::now();

  // 4. The sweep. Only free cells whose 26 neighbours are all in the grid take
  // part, so a grid under three cells thick on any axis has none.
  if (nx_ < 3 || ny_ < 3 || nz_ < 3) {
    stats_.sweep_ms = msSince(t);
    return;
  }
  const std::size_t wpr = (static_cast<std::size_t>(nx_) + 63) / 64;
  const std::size_t last_word = static_cast<std::size_t>(nx_ - 1) / 64;
  const std::uint64_t last_bit = std::uint64_t{1} << ((nx_ - 1) & 63);
  // The free cells of each row, dilated along x.
  std::vector<std::uint64_t> xdil(wpr * rows, 0);
  constexpr std::size_t kRowChunk = 64;
  const std::size_t row_chunks = (rows + kRowChunk - 1) / kRowChunk;
  const unsigned workers = std::max(1u, threads);
  struct alignas(64) Count { std::size_t n = 0; };
  std::vector<Count> free_count(workers), shell_count(workers);
  std::vector<std::vector<std::uint64_t>> scratch(workers);

  parallelFor(row_chunks, threads, [&](std::size_t c, unsigned w) {
    std::vector<std::uint64_t>& f = scratch[w];
    f.resize(wpr);
    const std::size_t r1 = std::min(rows, (c + 1) * kRowChunk);
    for (std::size_t r = c * kRowChunk; r < r1; ++r) {
      const std::size_t y = r % static_cast<std::size_t>(ny_), z = r / static_cast<std::size_t>(ny_);
      if (y == 0 || z == 0 || y + 1 == static_cast<std::size_t>(ny_) ||
          z + 1 == static_cast<std::size_t>(nz_))
        continue;  // outer layer: stays zero
      const std::uint8_t* row = cells_.data() + r * sy;
      std::size_t count = 0;
      for (std::size_t i = 0; i < wpr; ++i) {
        const int len = static_cast<int>(std::min<std::size_t>(64, sy - 64 * i));
        f[i] = pack(row + 64 * i, len, lsbFree, isFree);
      }
      f[0] &= ~std::uint64_t{1};  // x outer layer
      f[last_word] &= ~last_bit;
      for (std::size_t i = 0; i < wpr; ++i) count += static_cast<std::size_t>(__builtin_popcountll(f[i]));
      free_count[w].n += count;
      std::uint64_t* out = &xdil[r * wpr];
      for (std::size_t i = 0; i < wpr; ++i) {
        std::uint64_t d = f[i] | (f[i] << 1) | (f[i] >> 1);
        if (i > 0) d |= f[i - 1] >> 63;
        if (i + 1 < wpr) d |= f[i + 1] << 63;
        out[i] = d;
      }
    }
  });

  parallelFor(row_chunks, threads, [&](std::size_t c, unsigned w) {
    const std::size_t r1 = std::min(rows, (c + 1) * kRowChunk);
    std::size_t count = 0;
    for (std::size_t r = c * kRowChunk; r < r1; ++r) {
      const int y = static_cast<int>(r % static_cast<std::size_t>(ny_));
      const int z = static_cast<int>(r / static_cast<std::size_t>(ny_));
      const std::uint64_t* nb[9];
      int m = 0;
      for (int dz = -1; dz <= 1; ++dz)
        for (int dy = -1; dy <= 1; ++dy) {
          const int yy = y + dy, zz = z + dz;
          if (yy < 0 || zz < 0 || yy >= ny_ || zz >= nz_) continue;
          nb[m++] = &xdil[(static_cast<std::size_t>(zz) * ny_ + yy) * wpr];
        }
      std::uint8_t* row = cells_.data() + r * sy;
      for (std::size_t i = 0; i < wpr; ++i) {
        std::uint64_t d = 0;
        for (int j = 0; j < m; ++j) d |= nb[j][i];
        if (d == 0) continue;
        const int len = static_cast<int>(std::min<std::size_t>(64, sy - 64 * i));
        std::uint64_t s = d & pack(row + 64 * i, len, lsbUnknown, isUnknownCell);
        count += static_cast<std::size_t>(__builtin_popcountll(s));
        while (s) {
          row[64 * i + static_cast<std::size_t>(__builtin_ctzll(s))] = kShell;
          s &= s - 1;
        }
      }
    }
    shell_count[w].n += count;
  });
  for (unsigned w = 0; w < workers; ++w) {
    stats_.free_cells += free_count[w].n;
    stats_.shell += shell_count[w].n;
  }
  stats_.sweep_ms = msSince(t);
}

ConservativeGrid::Cell ConservativeGrid::at(double x, double y, double z) const {
  // octomap: key = (int)floor(resolution_factor * coord) + tree_max_val. As in
  // DistanceField::getDistance, the box's first key is subtracted as a double,
  // which is exact and keeps NaN and far-away points out of the box.
  const double fx = std::floor(inv_res_ * x) - static_cast<double>(kx0_ - key_offset_);
  const double fy = std::floor(inv_res_ * y) - static_cast<double>(ky0_ - key_offset_);
  const double fz = std::floor(inv_res_ * z) - static_cast<double>(kz0_ - key_offset_);
  if (!(fx >= 0.0 && fx < nx_ && fy >= 0.0 && fy < ny_ && fz >= 0.0 && fz < nz_)) return kUnknown;
  return static_cast<Cell>(
      cells_[static_cast<std::size_t>(fx) +
             static_cast<std::size_t>(nx_) *
                 (static_cast<std::size_t>(fy) +
                  static_cast<std::size_t>(ny_) * static_cast<std::size_t>(fz))]);
}

ConservativeGrid::Cell ConservativeGrid::at(const octomap::point3d& p) const {
  return at(static_cast<double>(p.x()), static_cast<double>(p.y()), static_cast<double>(p.z()));
}

bool ConservativeGrid::isUnknown(double x, double y, double z) const {
  const Cell c = at(x, y, z);
  return c == kUnknown || c == kShell;
}

void ConservativeGrid::obstaclesIn(const Eigen::Vector3d& lo, const Eigen::Vector3d& hi,
                                   std::vector<Eigen::Vector3d>& out) const {
  if (cells_.empty()) return;
  const int k0[3] = {kx0_, ky0_, kz0_}, n[3] = {nx_, ny_, nz_};
  int a0[3], a1[3];
  for (int a = 0; a < 3; ++a) {
    a0[a] = std::max(0, firstKeyAtOrAbove(lo[a], res_, inv_res_, key_offset_) - k0[a]);
    a1[a] = std::min(n[a] - 1, lastKeyAtOrBelow(hi[a], res_, inv_res_, key_offset_) - k0[a]);
    if (a0[a] > a1[a]) return;
  }
  const std::size_t sy = static_cast<std::size_t>(nx_);
  const std::size_t sz = sy * static_cast<std::size_t>(ny_);
  for (int z = a0[2]; z <= a1[2]; ++z) {
    const double cz = cellCentre(kz0_ + z, key_offset_, res_);
    for (int y = a0[1]; y <= a1[1]; ++y) {
      const double cy = cellCentre(ky0_ + y, key_offset_, res_);
      const std::uint8_t* row = cells_.data() + static_cast<std::size_t>(z) * sz +
                                static_cast<std::size_t>(y) * sy;
      int x = a0[0];
      // Obstacles are the odd values (kOccupied 1, kShell 3): skip 8 cells at
      // a time while none of them is.
      while (x <= a1[0]) {
        if (x + 8 <= a1[0] + 1) {
          std::uint64_t w;
          std::memcpy(&w, row + x, 8);
          if ((w & kLsbs) == 0) {
            x += 8;
            continue;
          }
        }
        if (isObstacle(static_cast<Cell>(row[x])))
          out.emplace_back(cellCentre(kx0_ + x, key_offset_, res_), cy, cz);
        ++x;
      }
    }
  }
}

}  // namespace drone_core::planning
