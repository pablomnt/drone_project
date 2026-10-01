#pragma once

#include <functional>
#include <optional>
#include <vector>

#include <Eigen/Core>

namespace drone_core::planning {

// Helpers for the "uncover" phase of goal-directed exploration (see
// AutonomyCore's mission): find where an optimistic path leaves explored space,
// and a place from which the camera can look at that spot.

using PointTest = std::function<bool(const Eigen::Vector3d&)>;

// The first point along `path` (sampled every `step` metres from its start) in
// never-observed space, or nothing if the whole path is observed.
std::optional<Eigen::Vector3d> findExitPoint(const std::vector<Eigen::Vector3d>& path,
                                             const PointTest& is_unknown, double step = 0.05);

struct ViewpointParams {
  double distance = 3.0;           // ideal distance from the exit point [m]
  double min_distance = 1.5;       // [m]
  double max_distance = 4.0;       // [m]
  // The camera looks forward and roughly level: the exit point may sit at most
  // this far above or below the viewpoint's horizontal [deg].
  double max_elevation_deg = 20.0;
  int azimuth_steps = 24;          // directions tried around the exit point
  // Score = |distance - ideal| + drone_weight x distance from the drone: near the
  // ideal distance first, then the one closest to where the drone is.
  double drone_weight = 0.2;
  double los_step = 0.05;          // line-of-sight sampling step [m]
};

struct Viewpoint {
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  double yaw = 0.0;      // facing the exit point [rad], map frame
  double score = 0.0;    // lower is better
};

// Candidate viewpoints around `exit`, best first: positions between
// min_distance and max_distance from it (rings at several distances and
// heights, `azimuth_steps` directions each), within max_elevation_deg of level
// as seen from the exit point, where `valid` accepts the position (the caller's
// conservative check: observed, clear of obstacles and of unknown space) and
// `see_through` accepts every point of the straight sight line from it to the
// exit point (the caller's "not inside a mapped obstacle"). Reachability is left
// to the caller, which plans to them in this order.
std::vector<Viewpoint> viewpointCandidates(const Eigen::Vector3d& exit,
                                           const Eigen::Vector3d& drone,
                                           const PointTest& valid,
                                           const PointTest& see_through,
                                           const ViewpointParams& params = {});

}  // namespace drone_core::planning
