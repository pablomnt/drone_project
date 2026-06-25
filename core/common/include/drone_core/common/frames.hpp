#pragma once

#include <Eigen/Dense>

// Coordinate-frame plumbing between the autonomy core (world ENU, body FLU) and
// the PX4 flight controller (world NED, body FRD). Every transform the wrapper
// node used to carry inline now lives here so it can be unit-tested without ROS
// and reused the day the autopilot changes. OKVIS reports its world frame with a
// 90-degree yaw offset relative to our ENU convention, which is folded into the
// yaw helpers below.
namespace drone_core::frames {

// Wrap an angle into (-pi, pi].
double wrapPi(double angle);

// Swap a PX4 NED vector (north, east, down) into our ENU convention
// (east, north, up). Works for both position and velocity.
Eigen::Vector3d pxNedToEnu(const Eigen::Vector3d& ned);

// Extract an ENU yaw from a PX4 attitude quaternion (which describes a NED/FRD
// rotation).
double pxAttitudeToEnuYaw(const Eigen::Quaterniond& q_ned);

// Extract an ENU yaw from an OKVIS odometry quaternion, applying the fixed
// 90-degree offset between the OKVIS world frame and our ENU convention.
double okvisAttitudeToEnuYaw(const Eigen::Quaterniond& q);

// Rotate a body-frame velocity into the world frame using the body orientation.
Eigen::Vector3d bodyVelToWorld(const Eigen::Quaterniond& q_world_body,
                               const Eigen::Vector3d& vel_body);

// Convert a desired ENU/FLU attitude into the PX4 NED/FRD quaternion the
// autopilot expects. yaw_drift corrects the controller's VIO-derived heading
// toward the autopilot's own (generally more reliable) yaw estimate before the
// frame swap.
Eigen::Quaterniond enuAttitudeToPxNed(const Eigen::Matrix3d& R_enu, double yaw_drift);

}  // namespace drone_core::frames
