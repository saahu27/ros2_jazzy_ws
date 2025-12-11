/**
 * @file synchronized_profiles.hpp
 * @brief Synchronized trapezoidal velocity profiles for multi-joint motion
 * 
 * Generates independent trapezoidal profiles for each joint, synchronized
 * to finish at the same time (determined by the slowest joint).
 * 
 * This is the CORRECT approach for joint-space motion because:
 * 1. Each joint moves independently with its own velocity/acceleration
 * 2. Units are not mixed (meters vs radians)
 * 3. Per-joint limits are respected
 * 4. All joints start and stop together
 * 
 * Mathematical approach:
 * - For each joint i with displacement delta_i:
 *   1. Compute unconstrained time T_i = time_for_trapezoidal(|delta_i|, v_max_i, a_max_i)
 *   2. Find T_total = max(T_1, ..., T_n)
 *   3. Scale each joint's profile to complete in T_total
 * 
 * @author Sahruday Patti
 */

#ifndef PUMA560_CPP__SYNCHRONIZED_PROFILES_HPP_
#define PUMA560_CPP__SYNCHRONIZED_PROFILES_HPP_

#include <vector>
#include <string>
#include <map>
#include <memory>
#include <tuple>
#include <Eigen/Dense>
#include "puma560_cpp/trapezoidal_profile.hpp"

namespace puma560_cpp
{

/**
 * @class SynchronizedProfiles
 * @brief Generates synchronized trapezoidal profiles for multiple joints
 */
class SynchronizedProfiles
{
public:
  /**
   * @brief Default per-joint velocity limits
   * lift_joint is prismatic (m/s), others are revolute (rad/s)
   */
  static const std::map<std::string, double> DEFAULT_V_MAX;
  
  /**
   * @brief Default per-joint acceleration limits
   */
  static const std::map<std::string, double> DEFAULT_A_MAX;

  /**
   * @brief Construct synchronized profiles for multiple joints
   * 
   * @param joint_names Names of the joints
   * @param start_positions Starting positions for each joint
   * @param end_positions Ending positions for each joint
   * @param v_max_dict Maximum velocities per joint (uses defaults if empty)
   * @param a_max_dict Maximum accelerations per joint (uses defaults if empty)
   */
  SynchronizedProfiles(
    const std::vector<std::string>& joint_names,
    const Eigen::VectorXd& start_positions,
    const Eigen::VectorXd& end_positions,
    const std::map<std::string, double>& v_max_dict = {},
    const std::map<std::string, double>& a_max_dict = {});

  /**
   * @brief Evaluate all joint profiles at time t
   * 
   * @param t Time since start of motion (seconds)
   * @return tuple<positions, velocities, accelerations> for all joints
   */
  std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd> evaluate(double t) const;

  /**
   * @brief Get the total synchronized time for the motion
   * @return Total duration in seconds (determined by slowest joint)
   */
  double getTotalTime() const { return total_time_; }

  /**
   * @brief Get the number of joints
   */
  size_t getNumJoints() const { return joint_names_.size(); }

  /**
   * @brief Get joint names
   */
  const std::vector<std::string>& getJointNames() const { return joint_names_; }

  /**
   * @brief Get individual joint profile (nullptr if joint doesn't move)
   */
  const TrapezoidalProfile* getJointProfile(size_t index) const;

private:
  /**
   * @brief Compute synchronized profiles for all joints
   */
  void computeSynchronizedProfiles();

  /**
   * @brief Get velocity limit for a joint
   */
  double getVMax(const std::string& name) const;

  /**
   * @brief Get acceleration limit for a joint
   */
  double getAMax(const std::string& name) const;

  std::vector<std::string> joint_names_;
  Eigen::VectorXd start_;
  Eigen::VectorXd end_;
  Eigen::VectorXd deltas_;
  std::map<std::string, double> v_max_dict_;
  std::map<std::string, double> a_max_dict_;
  
  std::vector<std::unique_ptr<TrapezoidalProfile>> profiles_;
  double total_time_;
};

}  // namespace puma560_cpp

#endif  // PUMA560_CPP__SYNCHRONIZED_PROFILES_HPP_

