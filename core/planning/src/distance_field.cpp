#include "drone_core/planning/distance_field.hpp"

#include <algorithm>
#include <atomic>
#include <climits>
#include <cmath>
#include <cstring>
#include <exception>
#include <limits>
#include <mutex>
#include <thread>

#ifdef DRONE_CORE_DISTANCE_FIELD_TIMING
#include <chrono>
#include <cstdio>
#endif

// Build outline (all phases parallel over independent work items):
//   1. Walk the octree once: bounding box of all leaves (== getMetricMin/Max in
//      keys), cut to the crop if there is one, and the list of occupied leaves.
//      (From a ConservativeGrid: no walk, the box is the grid's.)
//   2. Stamp the occupied leaves (or the grid's occupied and shell cells) into
//      a bitset (one bit per cell, rows padded to 64-bit words): ~1 MB for 10M cells, so the stamping stays in cache and the
//      big grid is first touched by the x pass instead of by a separate init.
//   3. x pass: per row, squared distance to the nearest occupied bit.
//   4. y and z passes: per column, out[i] = min_j in[j] + (i - j)^2 (the
//      separable step of Felzenszwalb & Huttenlocher). Columns are strided in
//      memory, so each task copies a block of 64 x-adjacent columns (whole cache
//      lines) into a small scratch buffer, transforms it there and copies it back.
//      Two exact ways to do the transform:
//      - windowed brute force over the offsets that can matter (|i - j| < sqrt
//        of the saturation value), on all 64 columns at once in 16-bit SIMD.
//        Used whenever the saturation fits (maxdist up to 128 cells, 6.4 m at
//        5 cm): at 20 cells it is ~5x faster than the envelope below, and still
//        ~3x at 60.
//      - F&H's lower envelope of parabolas per column, O(n) whatever maxdist;
//        the fallback for larger saturation distances.
//
// Truncation: every value is kept exact below far_ (the smallest squared cell
// distance that reads as maxdist) and saturated to far_ above it. Values at
// far_ are left out of the envelope entirely. This is exact below far_: the
// minimising chain of a true distance D < far_ only goes through intermediate
// values <= D, which are therefore exact sites, and every candidate is a real
// (squared) distance >= D; if D >= far_ every candidate is >= far_ too.

namespace drone_core::planning {

namespace {

// Largest saturation value the 16-bit windowed pass handles: in + d^2 < 2 far
// must stay below 2^15.
constexpr std::uint32_t kWindowMaxFar = 16384;

// Runs fn(task, worker) for task in [0, n) on up to `threads` threads (the
// calling thread is one of them), handing tasks out dynamically. An exception
// from a task is rethrown on the calling thread once all threads have stopped.
template <typename Fn>
void parallelFor(std::size_t n, unsigned threads, Fn&& fn) {
  if (n == 0) return;
  const unsigned t = static_cast<unsigned>(std::min<std::size_t>(threads, n));
  if (t <= 1) {
    for (std::size_t i = 0; i < n; ++i) fn(i, 0u);
    return;
  }
  std::atomic<std::size_t> next{0};
  std::exception_ptr error;  // the first exception thrown by any task (bad_alloc)
  std::mutex error_mutex;
  auto work = [&](unsigned worker) {
    try {
      for (;;) {
        const std::size_t i = next.fetch_add(1, std::memory_order_relaxed);
        if (i >= n) break;
        fn(i, worker);
      }
    } catch (...) {
      next.store(n, std::memory_order_relaxed);  // stop handing out tasks
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
      break;  // could not spawn: the threads we have (at least this one) finish the work
    }
  }
  work(0u);
  for (auto& th : pool) th.join();
  if (error) std::rethrow_exception(error);
}

// ---------------------------------------------------------------- octree walk

struct OccLeaf {
  std::uint16_t k[3];  // lowest-corner key
  std::uint16_t level; // log2 of the edge length in cells
};

// One per worker, updated on every leaf: cache-line aligned so that workers do
// not false-share their bounding boxes.
struct alignas(64) WalkResult {
  int lo[3] = {INT_MAX, INT_MAX, INT_MAX};
  int hi[3] = {INT_MIN, INT_MIN, INT_MIN};
  std::vector<OccLeaf> occ;
};

struct Frontier {
  const octomap::OcTreeNode* node;
  unsigned level;
  std::uint16_t k[3];
};

void visitLeaf(const octomap::OcTree& tree, const octomap::OcTreeNode* node, unsigned level,
               const std::uint16_t k[3], WalkResult& out) {
  const int span = 1 << level;
  for (int a = 0; a < 3; ++a) {
    out.lo[a] = std::min(out.lo[a], static_cast<int>(k[a]));
    out.hi[a] = std::max(out.hi[a], static_cast<int>(k[a]) + span - 1);
  }
  if (tree.isNodeOccupied(node))
    out.occ.push_back({{k[0], k[1], k[2]}, static_cast<std::uint16_t>(level)});
}

// Only subtrees reaching into [clip_lo, clip_hi] (keys, inclusive) are walked;
// the full key range walks everything.
struct Clip {
  int lo[3], hi[3];
  bool misses(unsigned level, const std::uint16_t k[3]) const {
    const int span = 1 << level;
    for (int a = 0; a < 3; ++a)
      if (k[a] + span - 1 < lo[a] || k[a] > hi[a]) return true;
    return false;
  }
};

void walk(const octomap::OcTree& tree, const octomap::OcTreeNode* node, unsigned level,
          const std::uint16_t k[3], const Clip& clip, WalkResult& out) {
  if (clip.misses(level, k)) return;
  bool inner = false;
  if (level > 0) {
    const std::uint16_t half = static_cast<std::uint16_t>(1u << (level - 1));
    for (unsigned i = 0; i < 8; ++i) {
      if (!tree.nodeChildExists(node, i)) continue;
      inner = true;
      const std::uint16_t ck[3] = {static_cast<std::uint16_t>(k[0] + ((i & 1) ? half : 0)),
                                   static_cast<std::uint16_t>(k[1] + ((i & 2) ? half : 0)),
                                   static_cast<std::uint16_t>(k[2] + ((i & 4) ? half : 0))};
      walk(tree, tree.getNodeChild(node, i), level - 1, ck, clip, out);
    }
  }
  if (!inner) visitLeaf(tree, node, level, k, out);
}

// Splits the top of the tree into enough independent subtrees for `threads`
// workers and walks them in parallel. Returns one result per worker.
std::vector<WalkResult> walkTree(const octomap::OcTree& tree, unsigned threads, const Clip& clip) {
  std::vector<WalkResult> results(std::max(1u, threads));
  const octomap::OcTreeNode* root = tree.getRoot();
  if (root == nullptr) return results;

  std::vector<Frontier> frontier{{root, tree.getTreeDepth(), {0, 0, 0}}};
  const std::size_t want = static_cast<std::size_t>(threads) * 32;
  for (unsigned round = 0; round < 16 && !frontier.empty() && frontier.size() < want; ++round) {
    std::vector<Frontier> next;
    next.reserve(frontier.size() * 8);
    for (const Frontier& f : frontier) {
      if (clip.misses(f.level, f.k)) continue;
      bool inner = false;
      if (f.level > 0) {
        const std::uint16_t half = static_cast<std::uint16_t>(1u << (f.level - 1));
        for (unsigned i = 0; i < 8; ++i) {
          if (!tree.nodeChildExists(f.node, i)) continue;
          inner = true;
          next.push_back({tree.getNodeChild(f.node, i), f.level - 1,
                          {static_cast<std::uint16_t>(f.k[0] + ((i & 1) ? half : 0)),
                           static_cast<std::uint16_t>(f.k[1] + ((i & 2) ? half : 0)),
                           static_cast<std::uint16_t>(f.k[2] + ((i & 4) ? half : 0))}});
        }
      }
      if (!inner) visitLeaf(tree, f.node, f.level, f.k, results[0]);
    }
    frontier.swap(next);
  }
  parallelFor(frontier.size(), threads, [&](std::size_t i, unsigned w) {
    walk(tree, frontier[i].node, frontier[i].level, frontier[i].k, clip, results[w]);
  });
  return results;
}

// The extent of the leaves along one axis, by branch and bound: the lowest
// leaf key (`upper` false) or the highest (`upper` true), visiting the nearer
// half of each node's children first and skipping any subtree that cannot beat
// the best so far (every octomap node has a leaf below it, so a node whose
// near half has a child never needs its far half). Stops once the answer
// reaches `enough`, where the crop clamps it anyway. (A copy of
// ConservativeGrid's.)
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

// ------------------------------------------------------------------ the passes

struct Geometry {
  int nx, ny, nz;
  int k0[3];
  std::size_t words_per_row;
};

inline void setBits(std::uint64_t* row, int x0, int x1) {  // inclusive range
  int w0 = x0 >> 6, w1 = x1 >> 6;
  const std::uint64_t m0 = ~std::uint64_t{0} << (x0 & 63);
  const std::uint64_t m1 = ~std::uint64_t{0} >> (63 - (x1 & 63));
  if (w0 == w1) {
    __atomic_fetch_or(&row[w0], m0 & m1, __ATOMIC_RELAXED);
    return;
  }
  __atomic_fetch_or(&row[w0], m0, __ATOMIC_RELAXED);
  for (int w = w0 + 1; w < w1; ++w) __atomic_fetch_or(&row[w], ~std::uint64_t{0}, __ATOMIC_RELAXED);
  __atomic_fetch_or(&row[w1], m1, __ATOMIC_RELAXED);
}

// Per-worker scratch for the column passes.
template <typename Cell>
struct Scratch {
  std::vector<Cell> in, out;       // n x B blocks: column b of the block at stride B
  std::vector<int> v;              // envelope: sites
  std::vector<std::int64_t> f;     // envelope: g(v) + v^2
  std::vector<std::int64_t> zn, zd;  // envelope: left boundary of each parabola, as zn / zd
};

// 1D lower envelope of parabolas (Felzenszwalb & Huttenlocher) over one column
// of `n` values read and written at stride `stride`:
//   out[x] = min(far, min_q (x - q)^2 + g[q])   over the sites q with g[q] < far.
// Boundaries are kept as exact fractions (numerator, positive denominator) and
// compared by cross-multiplication, so there is no division and no rounding.
template <typename Cell>
void envelope(const Cell* g, std::size_t stride, int n, Cell* out, std::uint32_t far,
              Scratch<Cell>& s) {
  int* v = s.v.data();
  std::int64_t* f = s.f.data();
  std::int64_t* zn = s.zn.data();
  std::int64_t* zd = s.zd.data();
  int k = -1;
  for (int q = 0; q < n; ++q) {
    const std::uint32_t gq = g[q * stride];
    if (gq >= far) continue;
    const std::int64_t fq = static_cast<std::int64_t>(gq) + static_cast<std::int64_t>(q) * q;
    std::int64_t num = 0, den = 1;
    // Pop parabolas whose whole interval lies right of the intersection with q:
    // intersection s = (fq - f[k]) / (2 (q - v[k])), popped when s <= z[k].
    while (k >= 0) {
      num = fq - f[k];
      den = 2 * static_cast<std::int64_t>(q - v[k]);
      if (k == 0 || num * zd[k] > zn[k] * den) break;  // z[0] = -inf
      --k;
    }
    ++k;
    v[k] = q;
    f[k] = fq;
    zn[k] = num;
    zd[k] = den;
  }
  const Cell farc = static_cast<Cell>(far);
  if (k < 0) {
    for (int x = 0; x < n; ++x) out[x * stride] = farc;
    return;
  }
  int j = 0;
  for (int x = 0; x < n; ++x) {
    while (j < k && zn[j + 1] < static_cast<std::int64_t>(x) * zd[j + 1]) ++j;  // z[j+1] < x
    const std::int64_t d = x - v[j];
    const std::int64_t val = d * d + (f[j] - static_cast<std::int64_t>(v[j]) * v[j]);
    out[x * stride] = val < static_cast<std::int64_t>(far) ? static_cast<Cell>(val) : farc;
  }
}

// The same 1D transform by brute force over the offsets that can matter: a tap
// with d^2 >= far can only give >= far, so R = max d with d^2 < far suffices,
// and the result never exceeds far because the d = 0 tap is <= far. All
// kBlock16 columns of a block at once: the 64-lane accumulator stays in vector
// registers and the inner loops vectorise to 16-bit add + signed min (hence
// int16 and far <= kWindowMaxFar, so in + d^2 < 2^15). On x86-64 GCC also
// emits an AVX2 clone picked at load time (~25% faster than the SSE2 baseline
// the build targets); elsewhere the plain loop is compiled as is.
constexpr int kBlock16 = 64;  // 16-bit columns per block: two cache lines
#if defined(__x86_64__) && defined(__GNUC__) && !defined(__clang__)
__attribute__((target_clones("avx2", "default")))
#endif
void windowBlock(const std::int16_t* in, int n, std::int16_t* out, int R) {
  for (int i = 0; i < n; ++i) {
    const int dlo = std::min(R, i), dhi = std::min(R, n - 1 - i);
    const std::int16_t* c = in + static_cast<std::size_t>(i) * kBlock16;
    std::int16_t acc[kBlock16];
    for (int b = 0; b < kBlock16; ++b) acc[b] = c[b];
    for (int d = 1; d <= dlo; ++d) {
      const std::int16_t* r = c - static_cast<std::size_t>(d) * kBlock16;
      const std::int16_t d2 = static_cast<std::int16_t>(d * d);
      for (int b = 0; b < kBlock16; ++b) {
        const std::int16_t t = static_cast<std::int16_t>(r[b] + d2);
        acc[b] = t < acc[b] ? t : acc[b];
      }
    }
    for (int d = 1; d <= dhi; ++d) {
      const std::int16_t* r = c + static_cast<std::size_t>(d) * kBlock16;
      const std::int16_t d2 = static_cast<std::int16_t>(d * d);
      for (int b = 0; b < kBlock16; ++b) {
        const std::int16_t t = static_cast<std::int16_t>(r[b] + d2);
        acc[b] = t < acc[b] ? t : acc[b];
      }
    }
    std::memcpy(out + static_cast<std::size_t>(i) * kBlock16, acc, sizeof(acc));
  }
}

// Stamps the occupied leaves of an octree walk into the bitset (clipped to the
// box).
void stampLeaves(const Geometry& gm, const std::vector<WalkResult>& walks, unsigned threads,
                 std::uint64_t* bits) {
  const int nx = gm.nx, ny = gm.ny, nz = gm.nz;
  const std::size_t wpr = gm.words_per_row;
  struct Chunk { const OccLeaf* p; std::size_t n; };
  std::vector<Chunk> chunks;
  constexpr std::size_t kChunk = 8192;
  for (const WalkResult& w : walks)
    for (std::size_t i = 0; i < w.occ.size(); i += kChunk)
      chunks.push_back({w.occ.data() + i, std::min(kChunk, w.occ.size() - i)});
  parallelFor(chunks.size(), threads, [&](std::size_t c, unsigned) {
    for (std::size_t i = 0; i < chunks[c].n; ++i) {
      const OccLeaf& L = chunks[c].p[i];
      const int span = 1 << L.level;
      int lo[3], hi[3];
      const int n[3] = {nx, ny, nz};
      bool empty = false;
      for (int a = 0; a < 3; ++a) {
        lo[a] = std::max(0, L.k[a] - gm.k0[a]);
        hi[a] = std::min(n[a] - 1, L.k[a] - gm.k0[a] + span - 1);
        empty |= lo[a] > hi[a];
      }
      if (empty) continue;
      for (int zz = lo[2]; zz <= hi[2]; ++zz)
        for (int yy = lo[1]; yy <= hi[1]; ++yy)
          setBits(&bits[(static_cast<std::size_t>(zz) * ny + yy) * wpr], lo[0], hi[0]);
    }
  });
}

// Stamps a conservative grid's occupied and shell cells (the odd values) into
// the bitset, which covers exactly the grid's box. Rows are independent.
void stampGrid(const Geometry& gm, const std::uint8_t* cells, unsigned threads,
               std::uint64_t* bits) {
  const std::size_t nx = static_cast<std::size_t>(gm.nx);
  const std::size_t rows = static_cast<std::size_t>(gm.ny) * gm.nz;
  const std::size_t wpr = gm.words_per_row;
  constexpr std::size_t kRowChunk = 64;
  parallelFor((rows + kRowChunk - 1) / kRowChunk, threads, [&](std::size_t c, unsigned) {
    const std::size_t r1 = std::min(rows, (c + 1) * kRowChunk);
    for (std::size_t r = c * kRowChunk; r < r1; ++r) {
      const std::uint8_t* row = cells + r * nx;
      std::uint64_t* out = bits + r * wpr;
      for (std::size_t w = 0; w < wpr; ++w) {
        const std::uint8_t* p = row + 64 * w;
        const std::size_t len = std::min<std::size_t>(64, nx - 64 * w);
        std::uint64_t word = 0;
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
        if (len == 64) {
          // Bit 0 of each byte, gathered 8 at a time into the top byte by the
          // multiply (no two partial products share a bit, so nothing carries).
          for (int i = 0; i < 8; ++i) {
            std::uint64_t x;
            std::memcpy(&x, p + 8 * i, 8);
            word |= (((x & 0x0101010101010101ULL) * 0x0102040810204080ULL) >> 56) << (8 * i);
          }
          out[w] = word;
          continue;
        }
#endif
        for (std::size_t i = 0; i < len; ++i)
          word |= static_cast<std::uint64_t>(ConservativeGrid::isObstacle(
                      static_cast<ConservativeGrid::Cell>(p[i])))
                  << i;
        out[w] = word;
      }
    }
  });
}

// The passes; `stamp(bits)` sets the occupied cells' bits first.
template <typename Cell, typename Stamp>
void runPasses(const Geometry& gm, Stamp&& stamp, std::uint32_t far, unsigned threads,
               Cell* grid) {
  const int nx = gm.nx, ny = gm.ny, nz = gm.nz;
  const std::size_t nxy = static_cast<std::size_t>(nx) * ny;
  const std::size_t rows = static_cast<std::size_t>(ny) * nz;
  const std::size_t wpr = gm.words_per_row;

#ifdef DRONE_CORE_DISTANCE_FIELD_TIMING
  using Clock = std::chrono::steady_clock;
  auto t0 = Clock::now();
#endif

  // --- occupancy bitset
  std::vector<std::uint64_t> bits(wpr * rows, 0);
  stamp(bits.data());

#ifdef DRONE_CORE_DISTANCE_FIELD_TIMING
  auto t1 = Clock::now();
#endif

  // sq[d] = d^2 for d < dcap, far from dcap on: dcap is the smallest 1D
  // distance whose square reaches far.
  int dcap = 0;
  while (static_cast<std::uint64_t>(dcap) * dcap < far) ++dcap;
  std::vector<Cell> sq(static_cast<std::size_t>(dcap) + 1);
  for (int d = 0; d < dcap; ++d) sq[d] = static_cast<Cell>(d * d);
  sq[dcap] = static_cast<Cell>(far);
  const Cell farc = static_cast<Cell>(far);

  // --- x pass: rows of the bitset -> squared 1D distance to the nearest set
  // bit. Rows go in chunks, so each page of the grid is first touched by one
  // thread and neighbouring rows stay together.
  constexpr std::size_t kRowChunk = 64;
  parallelFor((rows + kRowChunk - 1) / kRowChunk, threads, [&](std::size_t c, unsigned) {
    const std::size_t r1 = std::min(rows, (c + 1) * kRowChunk);
    for (std::size_t r = c * kRowChunk; r < r1; ++r) {
      const std::uint64_t* rb = &bits[r * wpr];
      Cell* out = grid + r * nx;
      // Fill [a, b) given the occupied cells either side (-1 / nx+... = none).
      auto fill = [&](int a, int b, int prev, int next) {
        for (int x = a; x < b; ++x) {
          int d = dcap;
          if (prev >= 0) d = std::min(d, x - prev);
          if (next >= 0) d = std::min(d, next - x);
          out[x] = sq[d];
        }
      };
      int prev = -1, start = 0;
      for (std::size_t w = 0; w < wpr; ++w) {
        std::uint64_t word = rb[w];
        while (word) {
          const int o = static_cast<int>(w * 64) + __builtin_ctzll(word);
          word &= word - 1;
          fill(start, o, prev, o);
          out[o] = 0;
          prev = o;
          start = o + 1;
        }
      }
      fill(start, nx, prev, -1);
    }
  });
  bits.clear();
  bits.shrink_to_fit();

#ifdef DRONE_CORE_DISTANCE_FIELD_TIMING
  auto t2 = Clock::now();
#endif

  // --- y and z passes: blocks of B x-adjacent columns through scratch.
  constexpr int B = static_cast<int>(128 / sizeof(Cell));  // two cache lines per block row
  static_assert(sizeof(Cell) != 2 || B == kBlock16, "window pass assumes 64-column blocks");
  int R = 0;  // largest 1D offset whose square is below far
  while (static_cast<std::uint64_t>(R + 1) * (R + 1) < far) ++R;
  const bool use_window = sizeof(Cell) == 2 && far <= kWindowMaxFar;

  const int nmax = std::max(ny, nz);
  std::vector<Scratch<Cell>> scratch(std::max(1u, threads));
  const int xblocks = (nx + B - 1) / B;
  auto columnPass = [&](int n, std::size_t col_stride, std::size_t outer_count,
                        std::size_t outer_stride) {
    parallelFor(outer_count * xblocks, threads, [&](std::size_t task, unsigned w) {
      Scratch<Cell>& s = scratch[w];
      if (s.in.size() < static_cast<std::size_t>(nmax) * B) {
        s.in.assign(static_cast<std::size_t>(nmax) * B, farc);
        s.out.resize(static_cast<std::size_t>(nmax) * B);
        s.v.resize(nmax);
        s.f.resize(nmax);
        s.zn.resize(nmax);
        s.zd.resize(nmax);
      }
      const std::size_t outer = task / xblocks;
      const int x0 = static_cast<int>(task % xblocks) * B;
      const int bw = std::min(B, nx - x0);
      Cell* base = grid + outer * outer_stride + x0;
      for (int i = 0; i < n; ++i)
        std::memcpy(&s.in[static_cast<std::size_t>(i) * B], base + i * col_stride,
                    bw * sizeof(Cell));
      if (use_window) {
        if constexpr (sizeof(Cell) == 2)  // lanes >= bw are ignored
          windowBlock(reinterpret_cast<const std::int16_t*>(s.in.data()), n,
                      reinterpret_cast<std::int16_t*>(s.out.data()), R);
      } else {
        for (int b = 0; b < bw; ++b) envelope<Cell>(&s.in[b], B, n, &s.out[b], far, s);
      }
      for (int i = 0; i < n; ++i)
        std::memcpy(base + i * col_stride, &s.out[static_cast<std::size_t>(i) * B],
                    bw * sizeof(Cell));
    });
  };
  columnPass(ny, static_cast<std::size_t>(nx), static_cast<std::size_t>(nz), nxy);  // y
#ifdef DRONE_CORE_DISTANCE_FIELD_TIMING
  auto t3 = Clock::now();
#endif
  columnPass(nz, nxy, static_cast<std::size_t>(ny), static_cast<std::size_t>(nx));  // z

#ifdef DRONE_CORE_DISTANCE_FIELD_TIMING
  auto t4 = Clock::now();
  auto ms = [](auto a, auto b) { return std::chrono::duration<double, std::milli>(b - a).count(); };
  std::fprintf(stderr, "[DistanceField] stamp %.1f ms, x %.1f ms, y %.1f ms, z %.1f ms (%s, R %d)\n",
               ms(t0, t1), ms(t1, t2), ms(t2, t3), ms(t3, t4), use_window ? "window" : "envelope", R);
#endif
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

}  // namespace

DistanceField::DistanceField(const octomap::OcTree& tree, double maxdist, unsigned threads)
    : DistanceField(tree, maxdist, threads,
                    Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity()),
                    Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity())) {}

DistanceField::DistanceField(const octomap::OcTree& tree, double maxdist, unsigned threads,
                             const Eigen::Vector3d& crop_lo, const Eigen::Vector3d& crop_hi)
    : res_(tree.getResolution()),
      inv_res_(1.0 / tree.getResolution()),  // octomap's resolution_factor, bit for bit
      maxdist_(maxdist),
      key_offset_(1 << (tree.getTreeDepth() - 1)) {
  if (threads == 0) threads = std::max(1u, std::thread::hardware_concurrency());
  if (tree.getRoot() == nullptr) return;  // empty tree: an empty box, every query reads -1

  // The crop in keys, within octomap's key range.
  const int kmax = (1 << tree.getTreeDepth()) - 1;
  Clip crop;
  bool cuts = false;
  for (int a = 0; a < 3; ++a) {
    crop.lo[a] = std::max(0, firstKeyAtOrAbove(crop_lo[a], res_, inv_res_, key_offset_));
    crop.hi[a] = std::min(kmax, lastKeyAtOrBelow(crop_hi[a], res_, inv_res_, key_offset_));
    if (crop.lo[a] > crop.hi[a]) return;  // the crop holds no cell: an empty box
    cuts |= crop.lo[a] > 0 || crop.hi[a] < kmax;
  }

#ifdef DRONE_CORE_DISTANCE_FIELD_TIMING
  auto t0 = std::chrono::steady_clock::now();
#endif
  int lo[3] = {INT_MAX, INT_MAX, INT_MAX}, hi[3] = {INT_MIN, INT_MIN, INT_MIN};
  if (cuts) {
    // The box by branch and bound (each bound searched only as far as the
    // crop), so the walk below can skip everything outside the crop.
    const std::uint16_t k0[3] = {0, 0, 0};
    for (int a = 0; a < 3; ++a) {
      Extent lower{tree, a, false, crop.lo[a], INT_MAX};
      lower.search(tree.getRoot(), tree.getTreeDepth(), k0);
      Extent higher{tree, a, true, crop.hi[a], INT_MIN};
      higher.search(tree.getRoot(), tree.getTreeDepth(), k0);
      lo[a] = std::max(lower.best, crop.lo[a]);
      hi[a] = std::min(higher.best, crop.hi[a]);
      if (lo[a] > hi[a]) return;  // nothing left: an empty box
    }
  }
  const Clip clip = cuts ? Clip{{lo[0], lo[1], lo[2]}, {hi[0], hi[1], hi[2]}}
                         : Clip{{0, 0, 0}, {kmax, kmax, kmax}};
  const std::vector<WalkResult> walks = walkTree(tree, threads, clip);
#ifdef DRONE_CORE_DISTANCE_FIELD_TIMING
  std::fprintf(stderr, "[DistanceField] walk %.1f ms\n",
               std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0)
                   .count());
#endif
  if (!cuts) {  // the box from the walk itself
    for (const WalkResult& w : walks)
      for (int a = 0; a < 3; ++a) {
        lo[a] = std::min(lo[a], w.lo[a]);
        hi[a] = std::max(hi[a], w.hi[a]);
      }
    if (lo[0] > hi[0]) return;  // no leaves (cannot happen with a root; kept as a guard)
  }
  // Occupied leaves reaching past the box are clipped by the stamping.
  build(lo, hi, threads, [&](const Geometry& gm, std::uint64_t* bits) {
    stampLeaves(gm, walks, threads, bits);
  });
}

DistanceField::DistanceField(const ConservativeGrid& grid, double maxdist, unsigned threads)
    : res_(grid.resolution()),
      inv_res_(1.0 / grid.resolution()),  // the grid's, and so octomap's, bit for bit
      maxdist_(maxdist),
      key_offset_(grid.keyOffset()) {
  if (threads == 0) threads = std::max(1u, std::thread::hardware_concurrency());
  if (grid.empty()) return;  // an empty box, every query reads -1
  const int lo[3] = {grid.keyX0(), grid.keyY0(), grid.keyZ0()};
  const int hi[3] = {lo[0] + grid.sizeX() - 1, lo[1] + grid.sizeY() - 1, lo[2] + grid.sizeZ() - 1};
  build(lo, hi, threads, [&](const Geometry& gm, std::uint64_t* bits) {
    stampGrid(gm, grid.data(), threads, bits);
  });
}

template <typename Stamp>
void DistanceField::build(const int lo[3], const int hi[3], unsigned threads, Stamp&& stamp) {
  kx0_ = lo[0];
  ky0_ = lo[1];
  kz0_ = lo[2];
  nx_ = hi[0] - lo[0] + 1;
  ny_ = hi[1] - lo[1] + 1;
  nz_ = hi[2] - lo[2] + 1;

  // far_: the smallest squared cell distance that reads as maxdist, i.e. the
  // smallest t with sqrt(t) * res >= maxdist (the same expression the lookup
  // uses, so the saturation boundary is exact). Beyond the largest distance the
  // box can hold, every value is exact anyway, so far_ is capped there.
  const std::uint64_t dmax = static_cast<std::uint64_t>(nx_ - 1) * (nx_ - 1) +
                             static_cast<std::uint64_t>(ny_ - 1) * (ny_ - 1) +
                             static_cast<std::uint64_t>(nz_ - 1) * (nz_ - 1);
  const std::uint64_t cap = std::min<std::uint64_t>(dmax + 1, 0x7fffffffu);
  std::uint64_t far = 0;
  if (maxdist_ > 0) {
    const double rc = maxdist_ / res_;
    if (!(rc * rc < static_cast<double>(cap))) {
      far = cap;
    } else {
      far = static_cast<std::uint64_t>(rc * rc);
      far = far >= 2 ? far - 2 : 0;
      while (far < cap && std::sqrt(static_cast<double>(far)) * res_ < maxdist_) ++far;
    }
  }
  far_ = static_cast<std::uint32_t>(far);

  const Geometry gm{nx_, ny_, nz_, {kx0_, ky0_, kz0_}, (static_cast<std::size_t>(nx_) + 63) / 64};
  const auto stampBits = [&](std::uint64_t* bits) { stamp(gm, bits); };
  const std::size_t cells = static_cast<std::size_t>(nx_) * ny_ * nz_;
  if (far_ <= 0xffffu) {
    sq16_.resize(cells);  // unwritten: the x pass writes every cell
    runPasses<std::uint16_t>(gm, stampBits, far_, threads, sq16_.data());
    lut_.resize(static_cast<std::size_t>(far_) + 1);
    for (std::uint32_t d2 = 0; d2 < far_; ++d2)
      lut_[d2] = static_cast<float>(std::min(std::sqrt(static_cast<double>(d2)) * res_, maxdist_));
    lut_[far_] = static_cast<float>(maxdist_);
  } else {
    sq32_.resize(cells);  // unwritten: the x pass writes every cell
    runPasses<std::uint32_t>(gm, stampBits, far_, threads, sq32_.data());
  }
}

Eigen::Vector3d DistanceField::boxMin() const {
  if (nx_ == 0) return Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  return Eigen::Vector3d(static_cast<double>(kx0_ - key_offset_) * res_,
                         static_cast<double>(ky0_ - key_offset_) * res_,
                         static_cast<double>(kz0_ - key_offset_) * res_);
}

Eigen::Vector3d DistanceField::boxMax() const {
  if (nx_ == 0) return Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
  return Eigen::Vector3d(static_cast<double>(kx0_ + nx_ - key_offset_) * res_,
                         static_cast<double>(ky0_ + ny_ - key_offset_) * res_,
                         static_cast<double>(kz0_ + nz_ - key_offset_) * res_);
}

float DistanceField::getDistance(const octomap::point3d& p) const {
  // octomap: key = (int)floor(resolution_factor * (double)coord) + tree_max_val.
  // Subtracting the box's first key as a double is exact and keeps NaN and
  // far-away points (where octomap's 16-bit key would wrap) out of the box.
  const double fx = std::floor(inv_res_ * static_cast<double>(p.x())) -
                    static_cast<double>(kx0_ - key_offset_);
  const double fy = std::floor(inv_res_ * static_cast<double>(p.y())) -
                    static_cast<double>(ky0_ - key_offset_);
  const double fz = std::floor(inv_res_ * static_cast<double>(p.z())) -
                    static_cast<double>(kz0_ - key_offset_);
  if (!(fx >= 0.0 && fx < nx_ && fy >= 0.0 && fy < ny_ && fz >= 0.0 && fz < nz_)) return -1.0f;
  const std::size_t i =
      static_cast<std::size_t>(fx) +
      static_cast<std::size_t>(nx_) *
          (static_cast<std::size_t>(fy) + static_cast<std::size_t>(ny_) * static_cast<std::size_t>(fz));
  if (!sq16_.empty()) return lut_[sq16_[i]];
  const std::uint32_t d2 = sq32_[i];
  if (d2 >= far_) return static_cast<float>(maxdist_);
  return static_cast<float>(std::min(std::sqrt(static_cast<double>(d2)) * res_, maxdist_));
}

}  // namespace drone_core::planning
