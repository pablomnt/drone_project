// Standalone (no test framework) checks for the frame conversions. Returns
// non-zero on the first failure so CTest reports it.

#include "drone_core/common/frames.hpp"

#include <cmath>
#include <iostream>

namespace {

int g_failures = 0;

void expectNear(double a, double b, double tol, const char* what) {
  if (std::abs(a - b) > tol) {
    std::cerr << "FAIL: " << what << " expected " << b << " got " << a << "\n";
    ++g_failures;
  }
}

}  // namespace

int main() {
  using namespace drone_core::frames;

  // wrapPi keeps angles inside [-pi, pi]. Avoid the exact +/-pi boundary, whose
  // sign is ambiguous under floating point.
  expectNear(wrapPi(0.0), 0.0, 1e-9, "wrapPi(0)");
  expectNear(wrapPi(2.5 * M_PI), 0.5 * M_PI, 1e-9, "wrapPi(2.5pi)");
  expectNear(wrapPi(-2.5 * M_PI), -0.5 * M_PI, 1e-9, "wrapPi(-2.5pi)");
  expectNear(std::abs(wrapPi(3.0 * M_PI)), M_PI, 1e-9, "wrapPi(3pi) magnitude");

  // NED <-> ENU is an axis swap with a flipped vertical.
  const Eigen::Vector3d ned(1.0, 2.0, 3.0);
  const Eigen::Vector3d enu = pxNedToEnu(ned);
  expectNear(enu.x(), 2.0, 1e-9, "pxNedToEnu.x (east=north_idx1)");
  expectNear(enu.y(), 1.0, 1e-9, "pxNedToEnu.y (north=north_idx0)");
  expectNear(enu.z(), -3.0, 1e-9, "pxNedToEnu.z (up=-down)");

  // An identity attitude maps to an ENU yaw of +pi/2 for both estimators
  // because of the world-frame offset baked into the helpers.
  const Eigen::Quaterniond identity = Eigen::Quaterniond::Identity();
  expectNear(pxAttitudeToEnuYaw(identity), M_PI_2, 1e-9, "pxAttitudeToEnuYaw(identity)");
  expectNear(okvisAttitudeToEnuYaw(identity), M_PI_2, 1e-9, "okvisAttitudeToEnuYaw(identity)");

  // Rotating a body velocity by the identity leaves it untouched.
  const Eigen::Vector3d v = bodyVelToWorld(identity, Eigen::Vector3d(0.5, -0.25, 1.0));
  expectNear(v.x(), 0.5, 1e-9, "bodyVelToWorld.x");
  expectNear(v.y(), -0.25, 1e-9, "bodyVelToWorld.y");
  expectNear(v.z(), 1.0, 1e-9, "bodyVelToWorld.z");

  // A level hover with no yaw drift must still yield a valid unit quaternion.
  const Eigen::Quaterniond q = enuAttitudeToPxNed(Eigen::Matrix3d::Identity(), 0.0);
  expectNear(q.norm(), 1.0, 1e-9, "enuAttitudeToPxNed unit norm");

  if (g_failures == 0) {
    std::cout << "frames: all checks passed\n";
    return 0;
  }
  std::cerr << "frames: " << g_failures << " failure(s)\n";
  return 1;
}
