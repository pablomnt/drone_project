// Exploration helpers: the exit point along a path and viewpoint candidates
// around it, on analytic predicates.

#include "drone_core/planning/exploration.hpp"

#include <cmath>
#include <iostream>
#include <string>

namespace {
int failures = 0;
void expect(bool ok, const std::string& what) {
  if (!ok) {
    std::cerr << "FAIL: " << what << "\n";
    ++failures;
  }
}
}  // namespace

int main() {
  using namespace drone_core::planning;
  using V = Eigen::Vector3d;

  // Explored space is x < 3; everything beyond is unknown.
  const PointTest unknown = [](const V& p) { return p.x() >= 3.0; };

  // 1. Exit point: the first unknown sample along the path.
  {
    const auto e = findExitPoint({V(0, 0, 1), V(2, 0, 1), V(6, 0, 1)}, unknown, 0.05);
    expect(e.has_value() && std::abs(e->x() - 3.0) < 0.06, "exit point not at the frontier");
    expect(!findExitPoint({V(0, 0, 1), V(2, 0, 1)}, unknown).has_value(),
           "an all-observed path reported an exit point");
  }

  // 2. Viewpoints around an exit at (3, 0, 1): valid only in explored space at
  //    least 0.3 m from the frontier and above the floor; a wall at x in
  //    [1.4, 1.6] for y > 0 blocks the sight line from that side.
  {
    const V exit(3.0, 0.0, 1.0), drone(0.0, 0.0, 1.0);
    const PointTest valid = [](const V& p) { return p.x() < 2.7 && p.z() > 0.5 && p.z() < 2.5; };
    const PointTest see = [](const V& p) { return !(p.x() > 1.4 && p.x() < 1.6 && p.y() > 0.0); };
    const auto vps = viewpointCandidates(exit, drone, valid, see);
    expect(!vps.empty(), "no viewpoint found");
    for (const auto& v : vps) {
      expect(valid(v.pos), "a viewpoint fails the validity test");
      const double d = (exit - v.pos).norm();
      expect(d >= 1.5 - 1e-9 && d <= 4.0 + 1e-9, "a viewpoint outside the distance band");
      const double elev = std::atan2(std::abs(exit.z() - v.pos.z()),
                                     std::hypot(exit.x() - v.pos.x(), exit.y() - v.pos.y()));
      expect(elev <= 20.0 * M_PI / 180.0 + 1e-9, "a viewpoint looks too steeply at the exit");
      const double facing = std::atan2(exit.y() - v.pos.y(), exit.x() - v.pos.x());
      expect(std::abs(std::remainder(v.yaw - facing, 2.0 * M_PI)) < 1e-9, "yaw does not face the exit");
      // The sight line must avoid the wall, to within the 5 cm sampling (the
      // map's resolution): checked against the wall shrunk by 3 cm, so a line
      // grazing its corner between samples does not count.
      const PointTest core = [](const V& q) {
        return !(q.x() > 1.43 && q.x() < 1.57 && q.y() > 0.03);
      };
      bool blocked = false;
      for (int k = 1; k < 200; ++k) blocked |= !core(v.pos + (exit - v.pos) * (k / 200.0));
      expect(!blocked, "a viewpoint's sight line crosses the wall");
    }
    if (!vps.empty()) {
      expect(std::abs((exit - vps.front().pos).norm() - 3.0) < 0.01,
             "the best viewpoint is not at the ideal 3 m");
      for (std::size_t i = 1; i < vps.size(); ++i) {
        expect(vps[i - 1].score <= vps[i].score, "viewpoints not sorted best first");
      }
    }
  }

  // 3. No valid position at all: no candidates.
  {
    const auto vps = viewpointCandidates(V(3, 0, 1), V(0, 0, 1),
                                         [](const V&) { return false; },
                                         [](const V&) { return true; });
    expect(vps.empty(), "candidates where nothing is valid");
  }

  if (failures == 0) {
    std::cout << "exploration: all checks passed\n";
    return 0;
  }
  std::cerr << "exploration: " << failures << " failure(s)\n";
  return 1;
}
