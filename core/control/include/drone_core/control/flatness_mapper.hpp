#pragma once

#include "drone_core/common/types.hpp"

namespace drone_core::control {

// Maps a minimum-snap trajectory to the instantaneous reference the controller
// tracks. A quadrotor is differentially flat in position and yaw, so evaluating
// the position polynomials and their first two derivatives yields the velocity
// and acceleration feed-forward directly. Yaw is not part of the trajectory; it
// is derived here from the direction of travel.
class FlatnessMapper {
public:
  struct Params {
    // Below this horizontal speed the heading from velocity is unreliable.
    double speed_min{0.3};
    // Treat the heading as "spinning" above this turn rate [rad/s].
    double heading_rate_max{1.5};
    // Speeds below this are treated as a standstill (heading undefined).
    double speed_eps{0.05};
  };

  FlatnessMapper() = default;
  explicit FlatnessMapper(const Params& params) : params_(params) {}

  // Seed the held yaw, e.g. to the vehicle's current heading when a new
  // trajectory is engaged, so the commanded yaw does not jump.
  void reset(double initial_yaw);

  // Evaluate the trajectory at wall-clock time `now`. Time is clamped to the
  // trajectory span, so sampling past the end holds the final state.
  common::Reference sample(const common::Trajectory& traj, double now);

private:
  Params params_;
  double last_yaw_{0.0};
};

}  // namespace drone_core::control
