// The unknown-space surcharge: a flat extra cost per metre of path routed
// through space that has never been observed, plus the matching hard stop in
// truncatePath.
//
// The bug this addresses: the clearance field saturates at its threshold, so a
// point further than the threshold from a mapped obstacle scores a proximity
// penalty of exactly zero. That covers the interior of unmapped space and
// everything outside the field's bounding box, so unknown space was the cheapest
// airspace in the problem. With the frontier stamped as an obstacle there is a
// gradient near the shell, but the shell has gaps (the sensor's field of view,
// and the deliberately unstamped ball around the vehicle), and past it the
// penalty returns to zero. Raising CLEARANCE_WEIGHT cannot fix that: it
// multiplies a zero.
//
// Analytic fields and predicates, short solve budgets — fast enough to run on
// every planning edit.

#include "drone_core/planning/conservative_grid.hpp"
#include "drone_core/planning/corridor.hpp"
#include "drone_core/planning/geometric_planner.hpp"
#include "drone_core/planning/unknown_shell.hpp"

#include <algorithm>
#include <array>
#include <climits>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <tuple>
#include <vector>

namespace {

int failures = 0;

void expect(bool ok, const std::string& what) {
  if (!ok) {
    std::cerr << "FAIL: " << what << "\n";
    ++failures;
  }
}

void expectNear(double got, double want, double tol, const std::string& what) {
  if (!(std::abs(got - want) <= tol)) {
    std::cerr << "FAIL: " << what << " (got " << got << ", want " << want << " +/- " << tol
              << ")\n";
    ++failures;
  }
}

// The scene: everything with x > 2 has never been observed. Mapped space is
// otherwise wide open, so the clearance field is saturated everywhere and
// contributes no penalty at all — which is the point. Only the unknown
// surcharge can distinguish the two halves.
constexpr double kFrontierX = 2.0;
bool unknownBeyondFrontier(double x, double, double) { return x > kFrontierX; }

// Tolerance for a charged-cost assertion. OMPL integrates the objective
// trapezoidally over states interpolated at the state-validity resolution, which
// for this state space is a step of roughly half a metre. The surcharge is a
// step function, so the segment straddling the frontier is averaged across and
// the total lands within about (step * weight) of the exact value. Tests
// therefore assert the charge to a few percent over a long path rather than to
// the metre, and lean on the qualitative checks for the rest.
constexpr double kIntegrationSlack = 3.0;


// ------------------------------------------------------------ ConservativeGrid
//
// The references: stampUnknownShell on a copy of the same map (uncropped), and
// a brute-force rebuild of the grid cell by cell through octomap's search()
// (crops, the mirror mode). Neither shares any code with ConservativeGrid.

using drone_core::planning::ConservativeGrid;
using drone_core::planning::stampUnknownShell;
using octomap::OcTree;
using octomap::OcTreeKey;

constexpr double kInf = std::numeric_limits<double>::infinity();
const Eigen::Vector3d kNoCropLo = Eigen::Vector3d::Constant(-kInf);
const Eigen::Vector3d kNoCropHi = Eigen::Vector3d::Constant(kInf);

// A box of cells in key space.
struct KBox {
  int lo[3] = {0, 0, 0};
  int n[3] = {0, 0, 0};
  std::size_t cells() const { return static_cast<std::size_t>(n[0]) * n[1] * n[2]; }
  bool empty() const { return n[0] <= 0 || n[1] <= 0 || n[2] <= 0; }
  std::size_t index(int x, int y, int z) const {
    return static_cast<std::size_t>(x) +
           static_cast<std::size_t>(n[0]) * (static_cast<std::size_t>(y) + static_cast<std::size_t>(n[1]) * z);
  }
  OcTreeKey key(int x, int y, int z) const {
    return OcTreeKey(static_cast<octomap::key_type>(lo[0] + x), static_cast<octomap::key_type>(lo[1] + y),
                     static_cast<octomap::key_type>(lo[2] + z));
  }
};

// Cell centre, octomap's own double arithmetic.
Eigen::Vector3d centreOf(const OcTree& t, const OcTreeKey& k) {
  return {t.keyToCoord(k[0]), t.keyToCoord(k[1]), t.keyToCoord(k[2])};
}

// The ball: the cells stampUnknownShell freed (no node in raw, a free node in
// the stamped copy).
bool isBallCell(const OcTree& raw, const OcTree& stamped, const OcTreeKey& k) {
  if (raw.search(k)) return false;
  const auto* n = stamped.search(k);
  return n && !stamped.isNodeOccupied(n);
}

// The expected box: every leaf of raw (and the ball cells, when `stamped` is
// given) grown by one cell, cut to the cells whose centres lie in [lo, hi].
KBox expectedBox(const OcTree& raw, const OcTree* stamped, const Eigen::Vector3d& lo,
                 const Eigen::Vector3d& hi) {
  int kl[3] = {INT_MAX, INT_MAX, INT_MAX}, kh[3] = {INT_MIN, INT_MIN, INT_MIN};
  const auto add = [&](const OcTreeKey& k, int span) {
    for (int a = 0; a < 3; ++a) {
      kl[a] = std::min(kl[a], static_cast<int>(k[a]));
      kh[a] = std::max(kh[a], static_cast<int>(k[a]) + span - 1);
    }
  };
  const unsigned depth = raw.getTreeDepth();
  for (auto it = raw.begin_leafs(), end = raw.end_leafs(); it != end; ++it)
    add(it.getIndexKey(), 1 << (depth - it.getDepth()));
  if (stamped)
    for (auto it = stamped->begin_leafs(), end = stamped->end_leafs(); it != end; ++it)
      if (it.getDepth() == depth && isBallCell(raw, *stamped, it.getKey())) add(it.getKey(), 1);
  KBox b;
  if (kl[0] > kh[0]) return b;
  for (int a = 0; a < 3; ++a) {
    int l = kl[a] - 1, h = kh[a] + 1;
    while (l <= h && !(raw.keyToCoord(static_cast<octomap::key_type>(l)) >= lo[a])) ++l;
    while (h >= l && !(raw.keyToCoord(static_cast<octomap::key_type>(h)) <= hi[a])) --h;
    b.lo[a] = l;
    b.n[a] = h - l + 1;
  }
  if (b.empty()) return KBox{};
  return b;
}

// The grid, cell by cell: raw's state, the ball (from `stamped`), then shell =
// never-observed cells with a free neighbour whose own 26 neighbours are all
// inside the box.
std::vector<std::uint8_t> expectedCells(const OcTree& raw, const OcTree* stamped, const KBox& box) {
  std::vector<std::uint8_t> v(box.cells(), ConservativeGrid::kUnknown);
  for (int z = 0; z < box.n[2]; ++z)
    for (int y = 0; y < box.n[1]; ++y)
      for (int x = 0; x < box.n[0]; ++x) {
        const OcTreeKey k = box.key(x, y, z);
        const auto* n = raw.search(k);
        std::uint8_t c = ConservativeGrid::kUnknown;
        if (n) c = raw.isNodeOccupied(n) ? ConservativeGrid::kOccupied : ConservativeGrid::kFree;
        else if (stamped && isBallCell(raw, *stamped, k)) c = ConservativeGrid::kFree;
        v[box.index(x, y, z)] = c;
      }
  if (!stamped) return v;
  std::vector<std::uint8_t> out = v;
  for (int z = 0; z < box.n[2]; ++z)
    for (int y = 0; y < box.n[1]; ++y)
      for (int x = 0; x < box.n[0]; ++x) {
        if (v[box.index(x, y, z)] != ConservativeGrid::kUnknown) continue;
        bool shell = false;
        for (int dz = -1; dz <= 1 && !shell; ++dz)
          for (int dy = -1; dy <= 1 && !shell; ++dy)
            for (int dx = -1; dx <= 1 && !shell; ++dx) {
              const int fx = x + dx, fy = y + dy, fz = z + dz;
              if (fx < 1 || fy < 1 || fz < 1 || fx > box.n[0] - 2 || fy > box.n[1] - 2 ||
                  fz > box.n[2] - 2)
                continue;  // not inside, or on the outer layer: not swept
              shell = v[box.index(fx, fy, fz)] == ConservativeGrid::kFree;
            }
        if (shell) out[box.index(x, y, z)] = ConservativeGrid::kShell;
      }
  return out;
}

std::string cellName(const KBox& b, int x, int y, int z) {
  return "(" + std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z) + ") key (" +
         std::to_string(b.lo[0] + x - 32768) + "," + std::to_string(b.lo[1] + y - 32768) + "," +
         std::to_string(b.lo[2] + z - 32768) + ")";
}

// The grid's box, its every cell (read through at() both ways and isUnknown),
// and the cells just outside it, against `want` over `box`.
void checkGrid(const ConservativeGrid& g, const OcTree& raw, const KBox& box,
               const std::vector<std::uint8_t>& want, const std::string& label) {
  if (box.empty()) {
    expect(g.empty() && g.sizeX() == 0 && g.sizeY() == 0 && g.sizeZ() == 0,
           label + ": expected an empty grid");
    return;
  }
  const bool same_box = !g.empty() && g.keyX0() == box.lo[0] && g.keyY0() == box.lo[1] &&
                        g.keyZ0() == box.lo[2] && g.sizeX() == box.n[0] &&
                        g.sizeY() == box.n[1] && g.sizeZ() == box.n[2];
  expect(same_box, label + ": box is key " + std::to_string(g.keyX0() - 32768) + "," +
                       std::to_string(g.keyY0() - 32768) + "," + std::to_string(g.keyZ0() - 32768) +
                       " size " + std::to_string(g.sizeX()) + "x" + std::to_string(g.sizeY()) +
                       "x" + std::to_string(g.sizeZ()) + ", want key " +
                       std::to_string(box.lo[0] - 32768) + "," + std::to_string(box.lo[1] - 32768) +
                       "," + std::to_string(box.lo[2] - 32768) + " size " + std::to_string(box.n[0]) +
                       "x" + std::to_string(box.n[1]) + "x" + std::to_string(box.n[2]));
  if (!same_box) return;
  expect(g.keyOffset() == 32768 && g.resolution() == raw.getResolution(),
         label + ": key offset or resolution wrong");
  int bad = 0, bad_query = 0;
  for (int z = 0; z < box.n[2]; ++z)
    for (int y = 0; y < box.n[1]; ++y)
      for (int x = 0; x < box.n[0]; ++x) {
        const std::size_t i = box.index(x, y, z);
        const std::uint8_t w = want[i];
        if (g.data()[i] != w) {
          if (bad < 5)
            std::cerr << "  " << label << ": cell " << cellName(box, x, y, z) << " is "
                      << int(g.data()[i]) << ", want " << int(w) << "\n";
          ++bad;
        }
        const OcTreeKey k = box.key(x, y, z);
        const Eigen::Vector3d c = centreOf(raw, k);
        const bool unknown = w == ConservativeGrid::kUnknown || w == ConservativeGrid::kShell;
        if (g.at(c.x(), c.y(), c.z()) != w || g.at(raw.keyToCoord(k)) != w ||
            g.isUnknown(c.x(), c.y(), c.z()) != unknown)
          ++bad_query;
      }
  expect(bad == 0, label + ": " + std::to_string(bad) + " of " + std::to_string(box.cells()) +
                       " cells differ from the reference");
  expect(bad_query == 0, label + ": at()/isUnknown() disagree with the reference in " +
                             std::to_string(bad_query) + " cells");

  // Just outside every face, and points a hair either side of each face.
  const double res = raw.getResolution();
  int bad_out = 0;
  for (int a = 0; a < 3; ++a)
    for (int side = 0; side < 2; ++side)
      for (int j = 0; j < 20; ++j) {
        int c[3] = {(j * 7) % box.n[0], (j * 5) % box.n[1], (j * 3) % box.n[2]};
        c[a] = side == 0 ? -1 : box.n[a];
        const Eigen::Vector3d p = centreOf(raw, box.key(c[0], c[1], c[2]));
        if (g.at(p.x(), p.y(), p.z()) != ConservativeGrid::kUnknown) ++bad_out;
        if (!g.isUnknown(p.x(), p.y(), p.z())) ++bad_out;
        c[a] = side == 0 ? 0 : box.n[a] - 1;
        Eigen::Vector3d q = centreOf(raw, box.key(c[0], c[1], c[2]));
        const double face = (side == 0 ? box.lo[a] - 32768 : box.lo[a] + box.n[a] - 32768) * res;
        const double eps = (side == 0 ? -1.0 : 1.0) * 0.01 * res;
        q[a] = face + eps;
        if (g.at(q.x(), q.y(), q.z()) != ConservativeGrid::kUnknown) ++bad_out;
        q[a] = face - eps;
        if (g.at(q.x(), q.y(), q.z()) != want[box.index(c[0], c[1], c[2])]) ++bad_out;
      }
  // NaN, infinite, far away, and a whole key range away (octomap's key wraps).
  const Eigen::Vector3d mid = centreOf(raw, box.key(box.n[0] / 2, box.n[1] / 2, box.n[2] / 2));
  for (int a = 0; a < 3; ++a)
    for (double v : {std::numeric_limits<double>::quiet_NaN(), kInf, -kInf, 1e9, -1e30,
                     mid[a] + 65536.0 * res, mid[a] - 65536.0 * res}) {
      Eigen::Vector3d p = mid;
      p[a] = v;
      if (g.at(p.x(), p.y(), p.z()) != ConservativeGrid::kUnknown) ++bad_out;
    }
  expect(bad_out == 0, label + ": " + std::to_string(bad_out) + " queries outside the box read wrong");

  std::size_t shell = 0;
  for (auto c : want) shell += c == ConservativeGrid::kShell;
  expect(g.stats().shell == shell, label + ": stats.shell " + std::to_string(g.stats().shell) +
                                       ", want " + std::to_string(shell));
}

// obstaclesIn against a scan of the grid's own cells, for a few windows.
void checkObstaclesIn(const ConservativeGrid& g, const OcTree& raw, const KBox& box,
                      const std::string& label, std::mt19937& rng) {
  using V = std::tuple<double, double, double>;
  const double res = raw.getResolution();
  const Eigen::Vector3d bmin((box.lo[0] - 32768) * res, (box.lo[1] - 32768) * res,
                             (box.lo[2] - 32768) * res);
  const Eigen::Vector3d bmax((box.lo[0] + box.n[0] - 32768) * res, (box.lo[1] + box.n[1] - 32768) * res,
                             (box.lo[2] + box.n[2] - 32768) * res);
  std::uniform_real_distribution<double> u(-0.2, 1.2);
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> windows = {
      {kNoCropLo, kNoCropHi},              // everything
      {bmin, bmax},                        // exactly the box
      {bmax + Eigen::Vector3d::Constant(0.1), bmax + Eigen::Vector3d::Constant(1.0)},  // outside
      {bmax, bmin},                        // inverted
  };
  // Windows whose bounds sit exactly on cell centres (inclusive).
  if (!box.empty()) {
    const Eigen::Vector3d c0 = centreOf(raw, box.key(1, 1, 1));
    const Eigen::Vector3d c1 = centreOf(raw, box.key(box.n[0] / 2, box.n[1] - 2, box.n[2] / 2));
    windows.push_back({c0, c1});
  }
  for (int i = 0; i < 12; ++i) {
    Eigen::Vector3d a, b;
    for (int k = 0; k < 3; ++k) {
      a[k] = bmin[k] + u(rng) * (bmax[k] - bmin[k]);
      b[k] = bmin[k] + u(rng) * (bmax[k] - bmin[k]);
      if (a[k] > b[k]) std::swap(a[k], b[k]);
    }
    windows.push_back({a, b});
  }
  int bad = 0;
  for (const auto& [lo, hi] : windows) {
    std::vector<V> want;
    for (int z = 0; z < box.n[2]; ++z)
      for (int y = 0; y < box.n[1]; ++y)
        for (int x = 0; x < box.n[0]; ++x) {
          if (!ConservativeGrid::isObstacle(static_cast<ConservativeGrid::Cell>(g.data()[box.index(x, y, z)])))
            continue;
          const Eigen::Vector3d c = centreOf(raw, box.key(x, y, z));
          if ((c.array() >= lo.array()).all() && (c.array() <= hi.array()).all())
            want.emplace_back(c.x(), c.y(), c.z());
        }
    std::vector<Eigen::Vector3d> out = {Eigen::Vector3d(1, 2, 3)};  // appended to, not cleared
    g.obstaclesIn(lo, hi, out);
    std::vector<V> got;
    for (std::size_t i = 1; i < out.size(); ++i) got.emplace_back(out[i].x(), out[i].y(), out[i].z());
    std::sort(want.begin(), want.end());
    std::sort(got.begin(), got.end());
    if (out.empty() || out[0] != Eigen::Vector3d(1, 2, 3) || got != want) ++bad;
  }
  expect(bad == 0, label + ": obstaclesIn wrong in " + std::to_string(bad) + " of " +
                       std::to_string(windows.size()) + " windows");
}

// A room-like map with a ragged carved free region (a union of spheres with
// noisy surfaces, so the shell is big and irregular), clutter inside it, a
// wall slab, stray occupied voxels out in unknown space, pruned so free blocks
// merge.
std::unique_ptr<OcTree> raggedMap(std::mt19937& rng) {
  auto tree = std::make_unique<OcTree>(0.05);
  const int n[3] = {150, 40, 26}, lo[3] = {32768 - 70, 32768 - 17, 32768 - 5};  // rows span 3 words
  std::uniform_real_distribution<double> u(0.0, 1.0);
  struct Ball { double c[3], r; };
  std::vector<Ball> balls;
  for (int i = 0; i < 12; ++i)
    balls.push_back({{8 + u(rng) * 134, 8 + u(rng) * 24, 6 + u(rng) * 14}, 5 + 5 * u(rng)});
  for (int z = 0; z < n[2]; ++z)
    for (int y = 0; y < n[1]; ++y)
      for (int x = 0; x < n[0]; ++x) {
        bool free = false;
        for (const Ball& b : balls) {
          const double d = std::sqrt((x - b.c[0]) * (x - b.c[0]) + (y - b.c[1]) * (y - b.c[1]) +
                                     (z - b.c[2]) * (z - b.c[2]));
          free |= d < b.r + 1.5 * (u(rng) - 0.5);
        }
        const OcTreeKey k(static_cast<octomap::key_type>(lo[0] + x),
                          static_cast<octomap::key_type>(lo[1] + y),
                          static_cast<octomap::key_type>(lo[2] + z));
        const bool wall = (x == 30 || x == 100) && y > 10 && z < 20;
        if (free) {
          tree->updateNode(k, wall || u(rng) < 0.01);
        } else if (u(rng) < 0.002) {
          tree->updateNode(k, true);
        }
      }
  tree->prune();
  return tree;
}

void checkConservativeGrid() {
  std::mt19937 rng(777);

  // --- the cube of case 8, and the ragged map: same classification as
  //     stampUnknownShell, voxel for voxel.
  OcTree cube(0.1);
  for (double x = 0.05; x < 1.0; x += 0.1)
    for (double y = 0.05; y < 1.0; y += 0.1)
      for (double z = 0.05; z < 1.0; z += 0.1) cube.updateNode(octomap::point3d(x, y, z), false);
  cube.updateNode(octomap::point3d(0.55, 0.55, 0.05), true);
  cube.updateNode(octomap::point3d(0.55, 0.55, 0.05), true);
  cube.prune();
  const std::unique_ptr<OcTree> ragged = raggedMap(rng);
  {
    bool merged = false;
    for (auto it = ragged->begin_leafs(), end = ragged->end_leafs(); it != end; ++it)
      merged |= !ragged->isNodeOccupied(*it) && it.getDepth() < ragged->getTreeDepth();
    expect(merged, "ragged map: no merged free blocks (test setup)");
  }

  struct MapCase {
    const OcTree* raw;
    octomap::point3d drone;
    double radius;
    std::string name;
  };
  // The ragged map's drone sits at the edge of its free region so the ball
  // reaches into unknown space.
  octomap::point3d edge;
  {
    const double res = ragged->getResolution();
    for (auto it = ragged->begin_leafs(), end = ragged->end_leafs(); it != end; ++it) {
      if (ragged->isNodeOccupied(*it) || it.getDepth() != ragged->getTreeDepth()) continue;
      const octomap::point3d p = it.getCoordinate();
      if (!ragged->search(p + octomap::point3d(static_cast<float>(res), 0, 0))) {
        edge = p;
        break;
      }
    }
  }
  const std::vector<MapCase> maps = {
      {&cube, octomap::point3d(0.05f, 0.05f, 0.55f), 0.32, "cube"},
      {ragged.get(), edge, 0.3, "ragged map"},
  };
  for (const MapCase& m : maps) {
    OcTree stamped(*m.raw);
    const auto ref = stampUnknownShell(stamped, m.drone, m.radius);
    const KBox box = expectedBox(*m.raw, &stamped, kNoCropLo, kNoCropHi);
    std::vector<std::uint8_t> want(box.cells());
    // Straight from the stamped copy: occupied in it but not in raw is shell.
    std::size_t stamped_cells = 0;
    for (int z = 0; z < box.n[2]; ++z)
      for (int y = 0; y < box.n[1]; ++y)
        for (int x = 0; x < box.n[0]; ++x) {
          const OcTreeKey k = box.key(x, y, z);
          const auto* r = m.raw->search(k);
          const auto* s = stamped.search(k);
          std::uint8_t c = ConservativeGrid::kUnknown;
          if (r) c = m.raw->isNodeOccupied(r) ? ConservativeGrid::kOccupied : ConservativeGrid::kFree;
          else if (s) c = stamped.isNodeOccupied(s) ? ConservativeGrid::kShell : ConservativeGrid::kFree;
          stamped_cells += c == ConservativeGrid::kShell;
          want[box.index(x, y, z)] = c;
        }
    expect(stamped_cells == ref.stamped,
           m.name + ": stampUnknownShell stamped cells outside the expected box (test setup)");
    expect(ref.ball_freed > 0, m.name + ": the ball freed nothing (test setup)");
    // The brute-force rebuild agrees with the stamped copy (checks the reference).
    expect(expectedCells(*m.raw, &stamped, box) == want,
           m.name + ": brute-force grid differs from stampUnknownShell (test reference)");

    std::vector<std::uint8_t> first;
    for (unsigned threads : {1u, 2u, 8u, 0u}) {
      const ConservativeGrid g(*m.raw, m.drone, m.radius, kNoCropLo, kNoCropHi, true, threads);
      const std::string label = m.name + " (" + std::to_string(threads) + " threads)";
      checkGrid(g, *m.raw, box, want, label);
      expect(g.stats().ball_freed == ref.ball_freed, label + ": ball_freed " +
                                                         std::to_string(g.stats().ball_freed) + ", want " +
                                                         std::to_string(ref.ball_freed));
      if (first.empty()) {
        first.assign(g.data(), g.data() + box.cells());
      } else {
        expect(!g.empty() && std::memcmp(first.data(), g.data(), first.size()) == 0,
               label + ": differs from the 1-thread grid");
      }
    }
    const ConservativeGrid g(*m.raw, m.drone, m.radius, kNoCropLo, kNoCropHi, true, 4);
    checkObstaclesIn(g, *m.raw, box, m.name, rng);

    // The mirror: raw's box + 1, raw's states, no ball, no shell.
    {
      const KBox mbox = expectedBox(*m.raw, nullptr, kNoCropLo, kNoCropHi);
      const ConservativeGrid mirror(*m.raw, m.drone, m.radius, kNoCropLo, kNoCropHi, false, 3);
      checkGrid(mirror, *m.raw, mbox, expectedCells(*m.raw, nullptr, mbox), m.name + " mirror");
      expect(mirror.stats().ball_freed == 0 && mirror.stats().shell == 0,
             m.name + " mirror: stats report a ball or a shell");
      checkObstaclesIn(mirror, *m.raw, mbox, m.name + " mirror", rng);
    }

    // Crops: through the middle on each axis (the cut layer's free cells are
    // not swept, so there is no shell beyond it), a window inside the map,
    // bounds exactly on cell centres, and ones that miss the map altogether.
    const double res = m.raw->getResolution();
    const Eigen::Vector3d mid = centreOf(*m.raw, box.key(box.n[0] / 2, box.n[1] / 2, box.n[2] / 2));
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> crops;
    for (int a = 0; a < 3; ++a) {
      Eigen::Vector3d hi = kNoCropHi, lo = kNoCropLo;
      hi[a] = mid[a] + 0.3 * res;
      crops.push_back({kNoCropLo, hi});
      lo[a] = mid[a] - 0.5 * res;  // exactly on a cell boundary: that cell's centre is inside
      crops.push_back({lo, kNoCropHi});
    }
    crops.push_back({mid - Eigen::Vector3d(0.21, 0.17, 0.13), mid + Eigen::Vector3d(0.19, 0.23, 0.11)});
    crops.push_back({mid - Eigen::Vector3d::Constant(2 * res), mid});  // bounds on centres
    const std::size_t kInnerCrops = crops.size();
    // Bounds on the centres of the map's own first and last few cells, where
    // the padding cell and the crop meet.
    {
      const KBox tb = expectedBox(*m.raw, &stamped, kNoCropLo, kNoCropHi);  // tree box + 1
      for (int a = 0; a < 3; ++a)
        for (int off = 1; off <= 2; ++off) {
          Eigen::Vector3d lo = kNoCropLo, hi = kNoCropHi;
          lo[a] = m.raw->keyToCoord(static_cast<octomap::key_type>(tb.lo[a] + off));
          crops.push_back({lo, kNoCropHi});
          hi[a] = m.raw->keyToCoord(static_cast<octomap::key_type>(tb.lo[a] + tb.n[a] - 1 - off));
          crops.push_back({kNoCropLo, hi});
        }
    }
    std::size_t full_shell = g.stats().shell;
    for (std::size_t ci = 0; ci < crops.size(); ++ci) {
      const auto& [lo, hi] = crops[ci];
      const KBox cbox = expectedBox(*m.raw, &stamped, lo, hi);
      const ConservativeGrid cg(*m.raw, m.drone, m.radius, lo, hi, true, 4);
      const std::string label = m.name + " cropped to [" + std::to_string(lo.x()) + "," +
                                std::to_string(lo.y()) + "," + std::to_string(lo.z()) + "]..[" +
                                std::to_string(hi.x()) + "," + std::to_string(hi.y()) + "," +
                                std::to_string(hi.z()) + "]";
      expect(!cbox.empty() && cbox.cells() <= box.cells(), label + ": empty crop (test setup)");
      checkGrid(cg, *m.raw, cbox, expectedCells(*m.raw, &stamped, cbox), label);
      expect(cg.stats().shell <= full_shell, label + ": more shell than the uncropped grid");
      if (ci >= kInnerCrops) continue;  // the boundary crops: the grid is what they test
      checkObstaclesIn(cg, *m.raw, cbox, label, rng);
      const ConservativeGrid cm(*m.raw, m.drone, m.radius, lo, hi, false, 4);
      const KBox mbox = expectedBox(*m.raw, nullptr, lo, hi);
      checkGrid(cm, *m.raw, mbox, expectedCells(*m.raw, nullptr, mbox), label + " mirror");
    }
    const Eigen::Vector3d far_away = mid + Eigen::Vector3d(0, 0, 50);
    const Eigen::Vector3d nan = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    for (const auto& [lo, hi] : std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>{
             {far_away, far_away + Eigen::Vector3d::Constant(1)},  // misses the map
             {mid + Eigen::Vector3d::Constant(0.3 * res), mid + Eigen::Vector3d::Constant(0.4 * res)},  // between centres
             {mid, mid - Eigen::Vector3d::Constant(res)},          // inverted
             {nan, kNoCropHi}}) {
      const ConservativeGrid e(*m.raw, m.drone, m.radius, lo, hi, true, 4);
      expect(e.empty() && e.sizeX() == 0 && e.sizeY() == 0 && e.sizeZ() == 0,
             m.name + ": a crop that misses the map gave a non-empty grid");
      expect(e.at(mid.x(), mid.y(), mid.z()) == ConservativeGrid::kUnknown &&
                 e.isUnknown(mid.x(), mid.y(), mid.z()),
             m.name + ": an empty grid did not read unknown");
      std::vector<Eigen::Vector3d> out;
      e.obstaclesIn(kNoCropLo, kNoCropHi, out);
      expect(out.empty() && e.stats().shell == 0 && e.stats().free_cells == 0,
             m.name + ": an empty grid reported obstacles or cells");
    }
  }

  // --- an empty tree: the grid holds the ball (+1), wrapped in shell, as
  //     stampUnknownShell does; with no ball there is no grid at all.
  {
    OcTree empty(0.05);
    OcTree stamped(empty);
    const octomap::point3d drone(0.31f, -0.52f, 1.07f);
    const auto ref = stampUnknownShell(stamped, drone, 0.25);
    const KBox box = expectedBox(empty, &stamped, kNoCropLo, kNoCropHi);
    const ConservativeGrid g(empty, drone, 0.25, kNoCropLo, kNoCropHi, true, 2);
    checkGrid(g, empty, box, expectedCells(empty, &stamped, box), "empty tree with a ball");
    expect(g.stats().ball_freed == ref.ball_freed && g.stats().shell == ref.stamped,
           "empty tree with a ball: counts differ from stampUnknownShell");
    expect(g.at(drone) == ConservativeGrid::kFree, "empty tree with a ball: the drone's cell is not free");
    const ConservativeGrid none(empty, drone, 0.0, kNoCropLo, kNoCropHi, true, 2);
    expect(none.empty(), "empty tree without a ball: non-empty grid");
    const ConservativeGrid nan_centre(
        empty, octomap::point3d(std::numeric_limits<float>::quiet_NaN(), 0, 0), 0.25, kNoCropLo,
        kNoCropHi, true, 2);
    expect(nan_centre.empty(), "empty tree with a NaN centre: non-empty grid");
  }

  // --- the keep-out shape: a ball behind the heading, a cylinder of the same
  //     radius ahead of it, wrapped in shell; a zero heading is the plain ball.
  {
    OcTree empty(0.1);
    const octomap::point3d c(0.05f, 0.05f, 0.05f);  // a cell centre
    for (const Eigen::Vector3d& fwd : {Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 1, 0)}) {
      ConservativeGrid::KeepOut ko;
      ko.center = c;
      ko.radius = 0.6;
      ko.forward = fwd;
      ko.forward_len = 0.6;
      const ConservativeGrid g(empty, ko, kNoCropLo, kNoCropHi, true, 2);
      // Offsets along the heading (a) and across it (b), level.
      const Eigen::Vector3d side(-fwd.y(), fwd.x(), 0.0);
      const auto at = [&](double a, double b) {
        const Eigen::Vector3d p = Eigen::Vector3d(c.x(), c.y(), c.z()) + a * fwd + b * side;
        return g.at(p.x(), p.y(), p.z());
      };
      const std::string name = "keep-out along (" + std::to_string(fwd.x()) + ", " +
                               std::to_string(fwd.y()) + ")";
      expect(at(0.5, 0.0) == ConservativeGrid::kFree, name + ": ahead on the axis not free");
      expect(at(0.5, 0.5) == ConservativeGrid::kFree,
             name + ": the cylinder's corner (outside the ball) not free");
      expect(at(-0.5, 0.0) == ConservativeGrid::kFree, name + ": behind on the axis not free");
      expect(at(-0.5, 0.5) == ConservativeGrid::kShell,
             name + ": behind, outside the ball, not shell");
      // (Not asserted as shell: their free neighbours sit exactly on the edge.)
      expect(at(0.7, 0.0) != ConservativeGrid::kFree, name + ": past the cylinder's end is free");
      expect(at(0.3, 0.7) != ConservativeGrid::kFree, name + ": beside the cylinder is free");
    }
    ConservativeGrid::KeepOut ball;
    ball.center = c;
    ball.radius = 0.6;
    ball.forward_len = 0.6;  // no heading: ignored
    const ConservativeGrid gb(empty, ball, kNoCropLo, kNoCropHi, true, 2);
    const ConservativeGrid gr(empty, c, 0.6, kNoCropLo, kNoCropHi, true, 2);
    expect(gb.stats().ball_freed == gr.stats().ball_freed && gb.stats().shell == gr.stats().shell,
           "keep-out without a heading differs from the plain ball");
  }
}

}  // namespace

int main() {
  using namespace drone_core::planning;

  auto empty = std::make_shared<octomap::OcTree>(0.1);
  const auto wideOpen = [](double, double, double) { return 1.0; };

  // 1. The core claim: with a saturated clearance field, cost is pure length —
  //    the two halves of the world are indistinguishable — until the surcharge
  //    is applied. A 14 m path with 12 m of it beyond the frontier must then
  //    cost 14 (length) + 12 * weight.
  {
    const std::vector<std::vector<double>> path = {{0, 0, 1}, {14, 0, 1}};

    GeometricPlanner plain(empty, /*planning_time=*/0.1);
    plain.setClearance(wideOpen, /*weight=*/100.0, /*threshold=*/1.0);
    expectNear(plain.pathCost(path), 14.0, 1e-6,
               "a saturated clearance field should charge nothing, even at weight 100");

    GeometricPlanner charged(empty, /*planning_time=*/0.1);
    charged.setClearance(wideOpen, /*weight=*/100.0, /*threshold=*/1.0);
    charged.setUnknownPenalty(unknownBeyondFrontier, /*weight=*/10.0);
    expectNear(charged.pathCost(path), 14.0 + 12.0 * 10.0, kIntegrationSlack,
               "unknown surcharge not charged per metre beyond the frontier");

    // The breakdown must attribute it to the unknown term, not to clearance:
    // the two call for opposite fixes, so a log that confuses them misleads.
    const auto cb = charged.costBreakdown(path);
    expectNear(cb.length, 14.0, 1e-6, "breakdown length wrong");
    expectNear(cb.clearance, 0.0, 1e-6, "breakdown blamed clearance for the unknown surcharge");
    expectNear(cb.unknown, 120.0, kIntegrationSlack, "breakdown unknown term wrong");
    expectNear(cb.total, cb.length + cb.clearance + cb.unknown, 1e-9,
               "breakdown terms do not sum to the total");
  }

  // 1b. The frontier's share of the proximity term: with a separate cost field,
  //     the penalty that field adds over the validity field. Validity 0.8 m
  //     from something along the whole path, the cost field 0.3 m: at weight
  //     2 / threshold 1 the proximity term is 2 * 0.7 * 10 = 14 in all, of which
  //     2 * 0.2 * 10 = 4 is mapped obstacles and 10 the frontier.
  {
    const std::vector<std::vector<double>> path = {{0, 0, 1}, {10, 0, 1}};
    GeometricPlanner p(empty, /*planning_time=*/0.1);
    p.setClearance([](double, double, double) { return 0.8; }, /*weight=*/2.0,
                   /*threshold=*/1.0);
    const auto one = p.costBreakdown(path);
    expect(!one.split && one.frontier == 0.0, "a single field reported a frontier share");
    p.setCostClearance([](double, double, double) { return 0.3; }, /*frontier_weight=*/2.0);
    const auto cb = p.costBreakdown(path);
    expect(cb.split, "a separate cost field was not split");
    expectNear(cb.clearance, 14.0, 1e-6, "split: proximity term wrong");
    expectNear(cb.frontier, 10.0, 1e-6, "split: frontier share wrong");
    expectNear(cb.total, cb.length + cb.clearance + cb.unknown, 1e-9,
               "split: terms do not sum to the total");
    // Its own weight: at 5 the frontier's extra 0.5 costs 5 * 0.5 * 10 = 25,
    // the obstacle share stays 4, and the total follows.
    p.setCostClearance([](double, double, double) { return 0.3; }, /*frontier_weight=*/5.0);
    const auto w5 = p.costBreakdown(path);
    expectNear(w5.frontier, 25.0, 1e-6, "frontier weight: frontier share wrong");
    expectNear(w5.clearance - w5.frontier, 4.0, 1e-6, "frontier weight changed the obstacle share");
    expectNear(p.pathCost(path), 10.0 + 29.0, 1e-6, "frontier weight not in the path cost");
    // A mapped obstacle nearer than the frontier: the frontier adds nothing.
    p.setClearance([](double, double, double) { return 0.3; }, /*weight=*/2.0, /*threshold=*/1.0);
    p.setCostClearance([](double, double, double) { return 0.3; }, /*frontier_weight=*/5.0);
    expectNear(p.costBreakdown(path).frontier, 0.0, 1e-9,
               "frontier charged where a mapped obstacle is the nearest hazard");
  }

  // 2. Flat, not a ramp: cost must keep accruing the further in you go. Twice
  //    the depth into unknown space, twice the surcharge. A distance-based
  //    penalty would saturate and stop charging, which is exactly how the old
  //    behaviour let a route dive deep for free.
  {
    GeometricPlanner planner(empty, /*planning_time=*/0.1);
    planner.setClearance(wideOpen, /*weight=*/1.0, /*threshold=*/1.0);
    planner.setUnknownPenalty(unknownBeyondFrontier, /*weight=*/10.0);
    const double shallow = planner.costBreakdown({{0, 0, 1}, {8, 0, 1}}).unknown;   // 6 m
    const double deep = planner.costBreakdown({{0, 0, 1}, {14, 0, 1}}).unknown;     // 12 m
    expectNear(deep, 2.0 * shallow, 2.0 * kIntegrationSlack,
               "unknown surcharge does not accrue linearly with depth");
  }

  // 3. A weight of 0, or no predicate, must leave the cost exactly as it was.
  {
    const std::vector<std::vector<double>> path = {{0, 0, 1}, {4, 0, 1}};
    GeometricPlanner off(empty, /*planning_time=*/0.1);
    off.setClearance(wideOpen, /*weight=*/1.0, /*threshold=*/1.0);
    off.setUnknownPenalty(unknownBeyondFrontier, /*weight=*/0.0);
    expectNear(off.pathCost(path), 4.0, 1e-6, "zero weight still charged for unknown space");
    expectNear(off.costBreakdown(path).unknown, 0.0, 1e-9,
               "zero weight reported a non-zero unknown term");
  }

  // 4. The surcharge must not make unknown space INVALID. A goal beyond the
  //    frontier has to stay reachable, or the optimistic search loses the whole
  //    reason it exists: EIT* would discard the goal state and abort, and
  //    best-effort could no longer chase a goal as the map grows.
  {
    GeometricPlanner planner(empty, /*planning_time=*/0.5);
    planner.setClearance(wideOpen, /*weight=*/1.0, /*threshold=*/1.0);
    planner.setUnknownPenalty(unknownBeyondFrontier, /*weight=*/50.0);
    std::vector<std::vector<double>> path;
    expect(planner.planPath({0, 0, 1}, {5, 0, 1}, path) && !path.empty(),
           "an expensive-unknown goal beyond the frontier became unreachable");
    expect(planner.lastGoalProjection() == 0.0,
           "a goal in unknown space was projected, so the surcharge leaked into validity");
    if (!path.empty()) {
      expect(path.back()[0] > kFrontierX,
             "the path stopped at the frontier instead of reaching the goal beyond it");
    }
  }

  // 5. The routing fix, which is the actual bug. Mapped space is a corridor
  //    -1 <= y <= 1 for x <= 2; beyond x = 2 everything is unknown. The goal
  //    sits at (4, 0), so any path must end in unknown space. But a detour that
  //    swings wide through unknown space to get there must lose to the direct
  //    route, which enters as late as possible. Scored directly rather than via
  //    a solve, so the assertion does not depend on planner randomness.
  {
    GeometricPlanner planner(empty, /*planning_time=*/0.1);
    planner.setClearance(wideOpen, /*weight=*/1.0, /*threshold=*/1.0);
    planner.setUnknownPenalty(unknownBeyondFrontier, /*weight=*/10.0);

    // Straight in: 2 m of unknown.
    const double direct = planner.pathCost({{0, 0, 1}, {4, 0, 1}});
    // Out through a gap and around: shorter in mapped space but 4+ m of unknown.
    const double around =
        planner.pathCost({{0, 0, 1}, {1, 0, 1}, {3, 3, 1}, {4, 0, 1}});
    expect(around > direct,
           "a wide detour through unknown space still beats the direct route");
  }

  // 6. Truncation's hard stop. The clearance oracle says everything is fine —
  //    which is what a gap in the stamped shell looks like — so only the
  //    predicate can cut the path. Without it the whole 4 m is committed.
  {
    const std::vector<Eigen::Vector3d> path = {{0, 0, 1}, {4, 0, 1}};
    const CorridorClearanceFn clear = [](double, double, double) { return 1.0; };

    const auto uncut = truncatePath(clear, path, /*margin=*/0.5, /*escape_ramp=*/1.0);
    expectNear(uncut.back().x(), 4.0, 1e-6,
               "a clearance-only truncation should commit the whole path here");

    const auto cut = truncatePath(clear, path, /*margin=*/0.5, /*escape_ramp=*/1.0,
                                  /*sample_step=*/0.05, unknownBeyondFrontier);
    expect(cut.size() >= 2, "unknown-aware truncation produced no committable prefix");
    expectNear(cut.back().x(), kFrontierX, 0.06,
               "truncation did not stop at the edge of observed space");
  }

  // 7. The stop is exempt from the escape ramp. The ramp trades margin for the
  //    ability to move at all, which is defensible against a hazard whose
  //    distance we can measure. It is not defensible against space we have never
  //    looked at, so even a point close to the drone must be cut.
  {
    const CorridorClearanceFn clear = [](double, double, double) { return 1.0; };
    // Frontier at x = 0.2, well inside a 1 m escape ramp.
    const auto nearUnknown = [](double x, double, double) { return x > 0.2; };
    const auto cut = truncatePath(clear, {{0, 0, 1}, {4, 0, 1}}, /*margin=*/0.5,
                                  /*escape_ramp=*/1.0, /*sample_step=*/0.05, nearUnknown);
    expect(cut.back().x() <= 0.25,
           "the escape ramp let truncation commit into unknown space near the drone");
  }

  // 8. The unknown shell. A known-free 1 m cube with one occupied voxel on its
  //    floor, the drone at one corner: the shell closes the cube on every side
  //    with no hole, the ball around the drone is freed and wrapped, and nothing
  //    known is changed.
  {
    const double res = 0.1;
    octomap::OcTree tree(res);
    for (double x = 0.05; x < 1.0; x += res)
      for (double y = 0.05; y < 1.0; y += res)
        for (double z = 0.05; z < 1.0; z += res)
          tree.updateNode(octomap::point3d(x, y, z), false);
    tree.updateNode(octomap::point3d(0.55, 0.55, 0.05), true);
    tree.updateNode(octomap::point3d(0.55, 0.55, 0.05), true);
    const octomap::point3d drone(0.05, 0.05, 0.55);
    const auto stats = stampUnknownShell(tree, drone, 0.32);
    const auto occupied = [&](double x, double y, double z) {
      const auto* n = tree.search(octomap::point3d(x, y, z));
      return n && tree.isNodeOccupied(n);
    };
    const auto freeKnown = [&](double x, double y, double z) {
      const auto* n = tree.search(octomap::point3d(x, y, z));
      return n && !tree.isNodeOccupied(n);
    };
    expect(stats.ball_freed > 0, "the ball freed nothing beside the drone");
    expect(occupied(1.05, 0.55, 0.55), "shell missing on the +x face");
    expect(occupied(0.55, 1.05, 0.55), "shell missing on the +y face");
    expect(occupied(0.55, 0.55, 1.05), "shell missing on the top face");
    expect(occupied(0.55, 0.55, -0.05), "shell missing on the bottom face");
    expect(occupied(1.05, 1.05, 1.05), "shell missing on a corner (26-neighbourhood)");
    expect(freeKnown(-0.15, 0.05, 0.55), "the ball did not free unknown space behind the drone");
    expect(occupied(-0.35, 0.05, 0.55), "the shell does not wrap around the ball");
    expect(tree.search(octomap::point3d(-0.55, 0.05, 0.55)) == nullptr,
           "space beyond the shell was touched");
    expect(occupied(0.55, 0.55, 0.05), "a known obstacle was changed");
    expect(freeKnown(0.55, 0.55, 0.55), "known free space was changed");
    // A closed shell: every voxel just outside the cube is either the ball
    // (free) or stamped.
    int holes = 0;
    for (double a = -0.05; a < 1.1; a += res)
      for (double b = -0.05; b < 1.1; b += res) {
        for (const auto& p : {octomap::point3d(-0.05, a, b), octomap::point3d(1.05, a, b),
                              octomap::point3d(a, -0.05, b), octomap::point3d(a, 1.05, b),
                              octomap::point3d(a, b, -0.05), octomap::point3d(a, b, 1.05)}) {
          if (!tree.search(p)) ++holes;
        }
      }
    expect(holes == 0, "the shell has " + std::to_string(holes) + " holes");
  }

  // 10. Keeping the path where the camera can see it (setUnknownSlopeLimit).
  //     Hard: no point of a path in unknown space steeper than 20 deg from the
  //     start, for every planner (EIT* included — it is a point check). Soft:
  //     an edge steeper than 20 deg through unknown space costs weight x the
  //     vertical metres beyond it. Through explored space neither applies.
  {
    const auto slopeFrom = [](const std::vector<double>& a, const std::vector<double>& b) {
      return std::atan2(std::abs(b[2] - a[2]), std::hypot(b[0] - a[0], b[1] - a[1])) * 180.0 / M_PI;
    };
    const auto allUnknown = [](double, double, double) { return true; };
    const auto allKnown = [](double, double, double) { return false; };
    for (const PlannerType type : {PlannerType::ABITstar, PlannerType::EITstar}) {
      const std::string name = toString(type);
      // A reachable goal (9.5 deg): every waypoint inside the cone.
      GeometricPlanner planner(empty, /*planning_time=*/0.5);
      planner.setPlannerType(type);
      planner.setClearance(wideOpen, /*weight=*/1.0, /*threshold=*/1.0);
      planner.setUnknownSlopeLimit(allUnknown, 20.0, 5.0);
      std::vector<std::vector<double>> path;
      expect(planner.planPath({0, 0, 1}, {6, 0, 2}, path) && path.size() >= 2,
             name + " slope cone: no path to a 9.5 deg goal");
      double worst = 0.0;
      for (const auto& w : path) worst = std::max(worst, slopeFrom(path.front(), w));
      expect(worst <= 20.0 + 1e-6,
             name + " slope cone: a waypoint at " + std::to_string(worst) + " deg from the start");
      // A goal outside the cone (34 deg): best effort ends inside it.
      GeometricPlanner be(empty, /*planning_time=*/0.5);
      be.setPlannerType(type);
      be.setBestEffort(true);
      be.setClearance(wideOpen, /*weight=*/1.0, /*threshold=*/1.0);
      be.setUnknownSlopeLimit(allUnknown, 20.0, 5.0);
      std::vector<std::vector<double>> p2;
      if (be.planPath({0, 0, 1}, {3, 0, 3}, p2) && p2.size() >= 2) {
        expect(slopeFrom(p2.front(), p2.back()) <= 20.0 + 1e-6,
               name + " slope cone: best effort ended outside the cone");
      }
    }
    {
      GeometricPlanner planner(empty, /*planning_time=*/0.5);
      planner.setClearance(wideOpen, /*weight=*/1.0, /*threshold=*/1.0);
      planner.setUnknownSlopeLimit(allUnknown, 20.0, 5.0);
      expect(!planner.isPathValid({{0, 0, 1}, {1, 0, 2}}),
             "slope cone: isPathValid accepted a 45 deg point in unknown space");
      // The cost: 2 m up over 1 m, 20 deg allows 0.364 m, so 1.636 m beyond, x5.
      const auto cb = planner.costBreakdown({{0, 0, 1}, {1, 0, 3}});
      expectNear(cb.steep, 5.0 * (2.0 - std::tan(20.0 * M_PI / 180.0)), 1e-6,
                 "slope cost: wrong steep term");
      expectNear(cb.total, cb.length + cb.clearance + cb.unknown + cb.steep, 1e-9,
                 "slope cost: terms do not sum to the total");
      expectNear(planner.costBreakdown({{0, 0, 1}, {6, 0, 2}}).steep, 0.0, 1e-12,
                 "slope cost: charged a shallow edge");
    }
    {
      GeometricPlanner planner(empty, /*planning_time=*/0.5);
      planner.setClearance(wideOpen, /*weight=*/1.0, /*threshold=*/1.0);
      planner.setUnknownSlopeLimit(allKnown, 20.0, 5.0);
      expect(planner.isPathValid({{0, 0, 1}, {1, 0, 2}}),
             "slope cone: a steep point in explored space was rejected");
      expectNear(planner.costBreakdown({{0, 0, 1}, {1, 0, 3}}).steep, 0.0, 1e-12,
                 "slope cost: charged a steep edge through explored space");
    }
  }

  // 11. Early stop (setEarlyStop): with a 1 s budget and a 0.2 s early stop, an
  //     easy path returns soon after 0.2 s; without it RRT* uses the full second.
  {
    const auto timed = [&](double early) {
      GeometricPlanner planner(empty, /*planning_time=*/1.0);
      planner.setClearance(wideOpen, /*weight=*/1.0, /*threshold=*/1.0);
      planner.setEarlyStop(early);
      std::vector<std::vector<double>> path;
      const auto t0 = std::chrono::steady_clock::now();
      const bool ok = planner.planPath({0, 0, 1}, {3, 0, 1}, path);
      const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      expect(ok, "early stop: no path on an empty map");
      return t;
    };
    const double with = timed(0.2), without = timed(0.0);
    expect(with < 0.5, "early stop: an easy search took " + std::to_string(with) + " s, not ~0.2 s");
    expect(without > 0.8, "early stop: without it the search ended at " + std::to_string(without) + " s");
  }

  // 9. ConservativeGrid, the stamp-free replacement for 8: the same shell,
  //    ball and free classification as stampUnknownShell voxel for voxel, on
  //    the cube and on a ragged map with merged blocks, at any thread count;
  //    crops, the mirror mode, empty grids and obstaclesIn.
  checkConservativeGrid();

  if (failures == 0) {
    std::cout << "unknown_cost: all checks passed\n";
    return 0;
  }
  return 1;
}
