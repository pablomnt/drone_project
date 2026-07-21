#include "drone_core/planning/corridor.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Dense>

namespace drone_core::planning {

namespace {

// Sample coordinates along one axis of an axis-aligned region [lo_i, hi_i].
// A positive extent is split into the fewest `step`-sized intervals that cover
// it (ceil), giving n+1 evenly spaced samples that always include both faces.
// A zero (or degenerate) extent collapses to a single sample at the coordinate
// so a thin slab or a degenerate axis-aligned segment never divides by zero.
std::vector<double> axisSamples(double lo_i, double hi_i, double step) {
  const double extent = hi_i - lo_i;
  if (extent <= 0.0) return {lo_i};  // zero-thickness axis => one sample (faces coincide)

  const int n = std::max(1, static_cast<int>(std::ceil(extent / step)));
  std::vector<double> s(n + 1);
  for (int j = 0; j <= n; ++j) s[j] = lo_i + (extent * j) / n;  // both faces included
  return s;
}

// FREE TEST: an axis-aligned region [rlo, rhi] is free iff every grid sample
// (spacing `step`, both faces included on each axis) has clearance >= margin.
// This is the sole safety oracle: growBox only commits a slab that passes here,
// so the whole grown box is guaranteed >= margin by construction.
bool regionFree(const CorridorClearanceFn& clearance, const Eigen::Vector3d& rlo,
                const Eigen::Vector3d& rhi, double step, double margin) {
  const std::vector<double> xs = axisSamples(rlo.x(), rhi.x(), step);
  const std::vector<double> ys = axisSamples(rlo.y(), rhi.y(), step);
  const std::vector<double> zs = axisSamples(rlo.z(), rhi.z(), step);

  for (double x : xs) {
    for (double y : ys) {
      for (double z : zs) {
        if (clearance(x, y, z) < margin) return false;
      }
    }
  }
  return true;
}

}  // namespace

std::vector<Eigen::Vector3d> resamplePath(const std::vector<Eigen::Vector3d>& path,
                                          double max_segment_len) {
  // Nothing to subdivide: too few points or a nonsensical cap => pass through.
  if (path.size() < 2 || max_segment_len <= 0.0) return path;

  std::vector<Eigen::Vector3d> out;
  out.reserve(path.size());

  // Push a point only if it differs from the current tail, so shared segment
  // endpoints (b of segment i == a of segment i+1) and degenerate zero-length
  // segments never produce consecutive duplicates.
  const auto pushUnique = [&out](const Eigen::Vector3d& q) {
    if (out.empty() || (out.back().array() != q.array()).any()) out.push_back(q);
  };

  pushUnique(path.front());  // seed the start once; the loop only adds interiors + ends

  for (size_t i = 0; i + 1 < path.size(); ++i) {
    const Eigen::Vector3d& a = path[i];
    const Eigen::Vector3d& b = path[i + 1];
    const double L = (b - a).norm();

    // Fewest equal pieces that keep every piece within the cap (>=1 so a and b
    // stay connected even for a zero-length or already-short segment).
    const int n = std::max(1, static_cast<int>(std::ceil(L / max_segment_len)));

    // Interior split points; the shared endpoints are handled by pushUnique
    // (a was pushed as the previous end / start, b is pushed just below).
    for (int k = 1; k < n; ++k) {
      const double t = static_cast<double>(k) / n;
      pushUnique(a + t * (b - a));
    }
    pushUnique(b);
  }

  // Guarantee the last point is exactly the original endpoint (no rounding on b).
  out.back() = path.back();
  return out;
}

std::vector<Eigen::Vector3d> truncatePath(const CorridorClearanceFn& conservative_clearance,
                                          const std::vector<Eigen::Vector3d>& path,
                                          double margin, double sample_step) {
  if (path.size() < 2) return path;
  const Eigen::Vector3d& start = path.front();

  // A sampled point is safe when its conservative clearance covers the required
  // margin, which RAMPS with distance from the start: required = min(margin,
  // |q - start|). This serves the same purpose as the planner's start-escape
  // sphere (a drone parked near the mapped floor, or inside the frontier
  // keep-out, must be able to root a path) but without the sphere's cliff — a
  // hard sphere leaves a dead band just outside it where the full margin
  // applies at once, cutting even a path that heads directly away from the
  // hazard. The ramp instead demands exactly the clearance the vehicle can
  // have gained by moving that far from where it already, physically, is: a
  // climb-out from a low hover survives (clearance grows with distance), while
  // a crawl alongside the hazard is cut. Clearance must additionally be
  // strictly positive everywhere (never inside an occupied/unknown voxel).
  const auto safe = [&](const Eigen::Vector3d& q) {
    const double d = conservative_clearance(q.x(), q.y(), q.z());
    return d > 0.0 && d >= std::min(margin, (q - start).norm());
  };

  std::vector<Eigen::Vector3d> out;
  out.push_back(start);
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    const Eigen::Vector3d& a = path[i];
    const Eigen::Vector3d& b = path[i + 1];
    const double L = (b - a).norm();
    const int n = std::max(1, static_cast<int>(std::ceil(L / sample_step)));
    for (int k = 1; k <= n; ++k) {
      const Eigen::Vector3d q = a + (static_cast<double>(k) / n) * (b - a);
      if (!safe(q)) {
        // Cut just before the first unsafe sample. The previous sample is the
        // committed endpoint (unless it duplicates the tail, e.g. an unsafe
        // first sample of a segment cutting at the shared waypoint).
        const Eigen::Vector3d last = a + (static_cast<double>(k - 1) / n) * (b - a);
        if ((last - out.back()).norm() > 1e-9) out.push_back(last);
        return out;
      }
    }
    out.push_back(b);
  }
  return out;  // whole path safe
}

bool growBox(const CorridorClearanceFn& clearance, const Eigen::Vector3d& a,
             const Eigen::Vector3d& b, const CorridorParams& p, Box& box_out) {
  // Seed at the tight AABB of the two endpoints. For an axis-aligned segment this
  // is a thin slab (or zero-thickness on some axes); the free test handles that.
  Eigen::Vector3d lo = a.cwiseMin(b);
  Eigen::Vector3d hi = a.cwiseMax(b);

  // If the segment's own AABB already clips an obstacle we cannot seed a free box.
  if (!regionFree(clearance, lo, hi, p.step, p.margin)) return false;

  // Six faces, each pushed outward in `step` increments while the newly swept
  // slab stays free and its accumulated growth stays within the cap. Testing
  // only the thin new slab each step keeps this O(box surface), not O(volume).
  // Order: -x, +x, -y, +y, -z, +z.
  bool open[6] = {true, true, true, true, true, true};
  double growth[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const double eps = 1e-9;  // absorb step accumulation so the last valid step isn't lost

  bool any_open = true;
  while (any_open) {
    any_open = false;
    for (int f = 0; f < 6; ++f) {
      if (!open[f]) continue;

      // Cap reached: this face can grow no further.
      if (growth[f] + p.step > p.max_box_growth + eps) {
        open[f] = false;
        continue;
      }

      // Candidate slab = the thin region newly swept by pushing face f out by step,
      // spanning the box's current cross-section on the other two axes.
      Eigen::Vector3d slo = lo;
      Eigen::Vector3d shi = hi;
      switch (f) {
        case 0: slo.x() = lo.x() - p.step; shi.x() = lo.x(); break;  // -x
        case 1: slo.x() = hi.x();          shi.x() = hi.x() + p.step; break;  // +x
        case 2: slo.y() = lo.y() - p.step; shi.y() = lo.y(); break;  // -y
        case 3: slo.y() = hi.y();          shi.y() = hi.y() + p.step; break;  // +y
        case 4: slo.z() = lo.z() - p.step; shi.z() = lo.z(); break;  // -z
        case 5: slo.z() = hi.z();          shi.z() = hi.z() + p.step; break;  // +z
      }

      if (regionFree(clearance, slo, shi, p.step, p.margin)) {
        // Commit: move the face out and record its growth.
        switch (f) {
          case 0: lo.x() -= p.step; break;
          case 1: hi.x() += p.step; break;
          case 2: lo.y() -= p.step; break;
          case 3: hi.y() += p.step; break;
          case 4: lo.z() -= p.step; break;
          case 5: hi.z() += p.step; break;
        }
        growth[f] += p.step;
        any_open = true;  // this face may still have room; keep the outer loop alive
      } else {
        open[f] = false;  // slab blocked => this face is final
      }
    }
  }

  // Growth is outward only, so [lo, hi] still contains a and b by construction.
  box_out.lo = lo;
  box_out.hi = hi;
  return true;
}

bool buildCorridor(const CorridorClearanceFn& clearance,
                   const std::vector<Eigen::Vector3d>& path,
                   const CorridorParams& p,
                   std::vector<Eigen::Vector3d>& resampled_out,
                   std::vector<Box>& boxes_out) {
  resampled_out.clear();
  boxes_out.clear();
  if (path.size() < 2) return false;

  resampled_out = resamplePath(path, p.max_segment_len);
  if (resampled_out.size() < 2) {
    resampled_out.clear();
    return false;
  }

  boxes_out.reserve(resampled_out.size() - 1);
  for (size_t i = 0; i + 1 < resampled_out.size(); ++i) {
    Box box;
    if (!growBox(clearance, resampled_out[i], resampled_out[i + 1], p, box)) {
      // One un-growable segment aborts the whole corridor; the caller falls back
      // to plain min-snap, so leave nothing half-built.
      resampled_out.clear();
      boxes_out.clear();
      return false;
    }
    // Each box contains both its endpoints, so it shares the junction waypoint
    // with its neighbour; growing outward from a seed spanning that shared point
    // makes consecutive boxes overlap with positive volume there. No extra
    // overlap-forcing code is needed -- adjacency falls out of the construction.
    boxes_out.push_back(box);
  }

  return true;  // boxes_out.size() == resampled_out.size() - 1
}

}  // namespace drone_core::planning
