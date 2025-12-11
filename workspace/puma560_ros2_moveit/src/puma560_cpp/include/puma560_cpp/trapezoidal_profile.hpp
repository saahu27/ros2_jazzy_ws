/**
 * @file trapezoidal_profile.hpp
 * @brief Trapezoidal velocity profile generator for smooth motion control
 * 
 * Implements the standard three-phase trapezoidal velocity profile:
 * - Phase 1 (Acceleration): v(t) = a_max * t
 * - Phase 2 (Cruise): v(t) = v_peak (constant)
 * - Phase 3 (Deceleration): v(t) = v_peak - a_max * (t - t2)
 * 
 * Automatically switches to triangular profile when distance is too short
 * to reach v_max.
 * 
 * Theory Reference:
 * - Craig, "Introduction to Robotics", Chapter 7: Trajectory Generation
 * - Siciliano et al., "Robotics: Modelling, Planning and Control", Chapter 3
 * 
 * @author Sahruday Patti
 */

#ifndef PUMA560_CPP__TRAPEZOIDAL_PROFILE_HPP_
#define PUMA560_CPP__TRAPEZOIDAL_PROFILE_HPP_

#include <tuple>
#include <cmath>
#include <string>

namespace puma560_cpp
{

/**
 * @class TrapezoidalProfile
 * @brief Generates trapezoidal/triangular velocity profile for single-axis motion
 */
class TrapezoidalProfile
{
public:
  /**
   * @brief Construct a trapezoidal velocity profile
   * 
   * @param distance Total displacement (can be negative for reverse motion)
   * @param v_max Maximum velocity limit
   * @param a_max Maximum acceleration limit
   */
  TrapezoidalProfile(double distance, double v_max, double a_max);

  /**
   * @brief Evaluate the profile at time t
   * 
   * @param t Time since start of motion (seconds)
   * @return tuple<position, velocity, acceleration> at time t
   *         Position is relative displacement from start
   */
  std::tuple<double, double, double> evaluate(double t) const;

  /**
   * @brief Get the total time for the motion
   * @return Total duration in seconds
   */
  double getTotalTime() const { return total_time_; }

  /**
   * @brief Get the peak velocity achieved
   * @return Peak velocity (may be less than v_max for triangular profiles)
   */
  double getPeakVelocity() const { return v_peak_; }

  /**
   * @brief Check if this is a triangular profile (didn't reach v_max)
   * @return true if triangular, false if trapezoidal
   */
  bool isTriangular() const { return t_const_ < 1e-9; }

  /**
   * @brief Get string representation for debugging
   */
  std::string toString() const;

private:
  double distance_;     ///< Absolute distance to travel
  double sign_;         ///< Direction of motion (+1 or -1)
  double v_max_;        ///< Maximum velocity constraint
  double a_max_;        ///< Maximum acceleration constraint
  double v_peak_;       ///< Actual peak velocity achieved
  double t_accel_;      ///< Duration of acceleration phase
  double t_const_;      ///< Duration of constant velocity phase
  double t_decel_;      ///< Duration of deceleration phase
  double t1_;           ///< End time of acceleration phase
  double t2_;           ///< End time of constant velocity phase
  double total_time_;   ///< Total motion duration
};

}  // namespace puma560_cpp

#endif  // PUMA560_CPP__TRAPEZOIDAL_PROFILE_HPP_

