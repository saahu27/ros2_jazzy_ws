/**
 * @file quaternion_utils.hpp
 * @brief Quaternion utilities for orientation interpolation
 * 
 * Implements Spherical Linear Interpolation (SLERP) between quaternions
 * using the geometric formula:
 *   q(t) = (sin((1-t)*theta) * q0 + sin(t*theta) * q1) / sin(theta)
 * where theta = arccos(q0 · q1)
 * 
 * Reference: Shoemake, "Animating Rotation with Quaternion Curves", SIGGRAPH 1985
 * 
 * @author Sahruday Patti
 */

#ifndef PUMA560_CPP__QUATERNION_UTILS_HPP_
#define PUMA560_CPP__QUATERNION_UTILS_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/quaternion.hpp>

namespace puma560_cpp
{

/**
 * @brief Spherical Linear Interpolation between two quaternions
 * 
 * @param q0 Starting quaternion (Eigen format: x, y, z, w)
 * @param q1 Ending quaternion
 * @param t Interpolation parameter [0, 1]
 * @return Interpolated quaternion
 */
Eigen::Quaterniond slerp(
  const Eigen::Quaterniond& q0,
  const Eigen::Quaterniond& q1,
  double t);

/**
 * @brief Convert ROS Quaternion message to Eigen Quaterniond
 * 
 * @param msg ROS Quaternion message
 * @return Eigen Quaterniond
 */
Eigen::Quaterniond fromMsg(const geometry_msgs::msg::Quaternion& msg);

/**
 * @brief Convert Eigen Quaterniond to ROS Quaternion message
 * 
 * @param q Eigen Quaterniond
 * @return ROS Quaternion message
 */
geometry_msgs::msg::Quaternion toMsg(const Eigen::Quaterniond& q);

/**
 * @brief SLERP between two ROS Quaternion messages
 * 
 * Convenience function that handles message conversion internally.
 * 
 * @param q0 Starting quaternion message
 * @param q1 Ending quaternion message
 * @param t Interpolation parameter [0, 1]
 * @return Interpolated quaternion message
 */
geometry_msgs::msg::Quaternion slerpMsg(
  const geometry_msgs::msg::Quaternion& q0,
  const geometry_msgs::msg::Quaternion& q1,
  double t);

/**
 * @brief Normalize a quaternion
 * 
 * @param q Input quaternion
 * @return Normalized quaternion
 */
Eigen::Quaterniond normalize(const Eigen::Quaterniond& q);

/**
 * @brief Check if two quaternions are approximately equal
 * 
 * Handles the double-cover property (q and -q represent same rotation)
 * 
 * @param q0 First quaternion
 * @param q1 Second quaternion
 * @param tolerance Angular tolerance in radians
 * @return true if approximately equal
 */
bool isApprox(const Eigen::Quaterniond& q0, const Eigen::Quaterniond& q1, double tolerance = 1e-6);

}  // namespace puma560_cpp

#endif  // PUMA560_CPP__QUATERNION_UTILS_HPP_

