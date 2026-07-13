#include "drone_core/common/frames.hpp"

#include <cmath>

namespace drone_core::frames {

double wrapPi(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

Eigen::Vector3d pxNedToEnu(const Eigen::Vector3d& ned) {
  return Eigen::Vector3d(ned.y(), ned.x(), -ned.z());
}

double pxAttitudeToEnuYaw(const Eigen::Quaterniond& q_ned) {
  const double siny_cosp = 2.0 * (q_ned.w() * q_ned.z() + q_ned.x() * q_ned.y());
  const double cosy_cosp = 1.0 - 2.0 * (q_ned.y() * q_ned.y() + q_ned.z() * q_ned.z());
  const double yaw_ned = std::atan2(siny_cosp, cosy_cosp);

  // NED yaw is clockwise-from-north; ENU yaw is counter-clockwise-from-east.
  return wrapPi(-yaw_ned + M_PI_2);
}

double okvisAttitudeToEnuYaw(const Eigen::Quaterniond& q) {
  const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
  const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  return wrapPi(std::atan2(siny_cosp, cosy_cosp) + M_PI_2);
}

Eigen::Vector3d bodyVelToWorld(const Eigen::Quaterniond& q_world_body,
                               const Eigen::Vector3d& vel_body) {
  return q_world_body.toRotationMatrix() * vel_body;
}

Eigen::Quaterniond enuAttitudeToPxNed(const Eigen::Matrix3d& R_enu, double yaw_drift) {
  const Eigen::Matrix3d R_yaw_correction =
      Eigen::AngleAxisd(yaw_drift, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Matrix3d R_enu_aligned = R_yaw_correction * R_enu;

  Eigen::Matrix3d R_world_enu_to_ned;
  R_world_enu_to_ned << 0, 1, 0,
                        1, 0, 0,
                        0, 0, -1;

  Eigen::Matrix3d R_body_flu_to_frd;
  R_body_flu_to_frd << 1, 0, 0,
                       0, -1, 0,
                       0, 0, -1;

  const Eigen::Matrix3d R_ned = R_world_enu_to_ned * R_enu_aligned * R_body_flu_to_frd;

  Eigen::Quaterniond q_ned(R_ned);
  q_ned.normalize();
  return q_ned;
}

}  // namespace drone_core::frames
