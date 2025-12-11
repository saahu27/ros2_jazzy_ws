/**
 * @file quaternion_utils.cpp
 * @brief Implementation of quaternion utilities
 */

#include "puma560_cpp/quaternion_utils.hpp"
#include <cmath>
#include <algorithm>

namespace puma560_cpp
{

Eigen::Quaterniond normalize(const Eigen::Quaterniond& q)
{
  return q.normalized();
}

Eigen::Quaterniond slerp(
  const Eigen::Quaterniond& q0,
  const Eigen::Quaterniond& q1,
  double t)
{
  // Clamp t to [0, 1]
  t = std::clamp(t, 0.0, 1.0);

  // Handle edge cases
  if (t <= 1e-9) {
    return q0;
  }
  if (t >= 1.0 - 1e-9) {
    return q1;
  }

  // Normalize input quaternions
  Eigen::Quaterniond v0 = q0.normalized();
  Eigen::Quaterniond v1 = q1.normalized();

  // Compute dot product (cosine of angle between quaternions)
  double dot = v0.dot(v1);

  // If dot is negative, negate one quaternion to take shorter path
  // (quaternions q and -q represent the same rotation)
  if (dot < 0.0) {
    v1 = Eigen::Quaterniond(-v1.w(), -v1.x(), -v1.y(), -v1.z());
    dot = -dot;
  }

  // Clamp dot to avoid numerical issues with acos
  dot = std::clamp(dot, -1.0, 1.0);

  // If quaternions are very close, use linear interpolation
  // to avoid division by zero in slerp formula
  constexpr double DOT_THRESHOLD = 0.9995;
  if (dot > DOT_THRESHOLD) {
    // Linear interpolation
    Eigen::Quaterniond result(
      v0.w() + t * (v1.w() - v0.w()),
      v0.x() + t * (v1.x() - v0.x()),
      v0.y() + t * (v1.y() - v0.y()),
      v0.z() + t * (v1.z() - v0.z())
    );
    return result.normalized();
  }

  // Standard SLERP formula:
  // q(t) = (sin((1-t)*theta) * q0 + sin(t*theta) * q1) / sin(theta)
  double theta_0 = std::acos(dot);    // angle between quaternions
  double theta = theta_0 * t;          // interpolated angle
  double sin_theta = std::sin(theta);
  double sin_theta_0 = std::sin(theta_0);

  double s0 = std::cos(theta) - dot * sin_theta / sin_theta_0;
  double s1 = sin_theta / sin_theta_0;

  Eigen::Quaterniond result(
    s0 * v0.w() + s1 * v1.w(),
    s0 * v0.x() + s1 * v1.x(),
    s0 * v0.y() + s1 * v1.y(),
    s0 * v0.z() + s1 * v1.z()
  );

  return result.normalized();
}

Eigen::Quaterniond fromMsg(const geometry_msgs::msg::Quaternion& msg)
{
  // Eigen uses (w, x, y, z) constructor order
  return Eigen::Quaterniond(msg.w, msg.x, msg.y, msg.z).normalized();
}

geometry_msgs::msg::Quaternion toMsg(const Eigen::Quaterniond& q)
{
  geometry_msgs::msg::Quaternion msg;
  Eigen::Quaterniond normalized = q.normalized();
  msg.x = normalized.x();
  msg.y = normalized.y();
  msg.z = normalized.z();
  msg.w = normalized.w();
  return msg;
}

geometry_msgs::msg::Quaternion slerpMsg(
  const geometry_msgs::msg::Quaternion& q0,
  const geometry_msgs::msg::Quaternion& q1,
  double t)
{
  return toMsg(slerp(fromMsg(q0), fromMsg(q1), t));
}

bool isApprox(const Eigen::Quaterniond& q0, const Eigen::Quaterniond& q1, double tolerance)
{
  // Check both q and -q due to double-cover property
  double dot = std::abs(q0.normalized().dot(q1.normalized()));
  double angle = 2.0 * std::acos(std::clamp(dot, 0.0, 1.0));
  return angle < tolerance;
}

}  // namespace puma560_cpp

