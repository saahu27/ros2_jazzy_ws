/**
 * @file trapezoidal_profile.cpp
 * @brief Implementation of TrapezoidalProfile class
 */

#include "puma560_cpp/trapezoidal_profile.hpp"
#include <sstream>
#include <iomanip>
#include <algorithm>

namespace puma560_cpp
{

TrapezoidalProfile::TrapezoidalProfile(double distance, double v_max, double a_max)
  : v_max_(std::abs(v_max)), a_max_(std::abs(a_max))
{
  // Handle direction
  sign_ = (distance >= 0) ? 1.0 : -1.0;
  distance_ = std::abs(distance);

  // Handle zero or near-zero distance
  if (distance_ < 1e-9) {
    t_accel_ = 0.0;
    t_const_ = 0.0;
    t_decel_ = 0.0;
    t1_ = 0.0;
    t2_ = 0.0;
    total_time_ = 0.0;
    v_peak_ = 0.0;
    return;
  }

  // Time to accelerate to v_max
  double t_accel_full = v_max_ / a_max_;
  
  // Distance covered during full acceleration phase
  double d_accel_full = 0.5 * a_max_ * t_accel_full * t_accel_full;

  // Check if we can reach v_max (trapezoidal) or not (triangular)
  if (2.0 * d_accel_full >= distance_) {
    // TRIANGULAR PROFILE: Cannot reach v_max
    // Time to accelerate: t = sqrt(d / a)
    t_accel_ = std::sqrt(distance_ / a_max_);
    t_const_ = 0.0;
    t_decel_ = t_accel_;
    v_peak_ = a_max_ * t_accel_;
  } else {
    // TRAPEZOIDAL PROFILE: Can reach v_max
    t_accel_ = t_accel_full;
    t_decel_ = t_accel_full;
    double d_const = distance_ - 2.0 * d_accel_full;
    t_const_ = d_const / v_max_;
    v_peak_ = v_max_;
  }

  // Compute phase transition times
  t1_ = t_accel_;
  t2_ = t_accel_ + t_const_;
  total_time_ = t_accel_ + t_const_ + t_decel_;
}

std::tuple<double, double, double> TrapezoidalProfile::evaluate(double t) const
{
  double pos, vel, acc;

  // Clamp time to valid range
  t = std::clamp(t, 0.0, total_time_);

  if (total_time_ < 1e-9) {
    // No motion
    return {0.0, 0.0, 0.0};
  }

  if (t <= t1_) {
    // PHASE 1: Acceleration
    // p(t) = 0.5 * a * t²
    // v(t) = a * t
    // a(t) = a
    acc = a_max_;
    vel = a_max_ * t;
    pos = 0.5 * a_max_ * t * t;
  } else if (t <= t2_) {
    // PHASE 2: Constant velocity (cruise)
    // p(t) = d_accel + v_peak * (t - t1)
    // v(t) = v_peak
    // a(t) = 0
    double dt = t - t1_;
    acc = 0.0;
    vel = v_peak_;
    double d_accel = 0.5 * a_max_ * t1_ * t1_;
    pos = d_accel + v_peak_ * dt;
  } else {
    // PHASE 3: Deceleration
    // p(t) = d_accel + d_const + v_peak*(t-t2) - 0.5*a*(t-t2)²
    // v(t) = v_peak - a * (t - t2)
    // a(t) = -a
    double dt = t - t2_;
    acc = -a_max_;
    vel = v_peak_ - a_max_ * dt;
    double d_accel = 0.5 * a_max_ * t1_ * t1_;
    double d_const = v_peak_ * t_const_;
    pos = d_accel + d_const + v_peak_ * dt - 0.5 * a_max_ * dt * dt;
  }

  // Apply direction
  return {pos * sign_, vel * sign_, acc * sign_};
}

std::string TrapezoidalProfile::toString() const
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3);
  oss << "TrapezoidalProfile(d=" << distance_ * sign_
      << ", v_max=" << v_max_
      << ", a_max=" << a_max_
      << ", T=" << total_time_ << "s"
      << ", type=" << (isTriangular() ? "triangular" : "trapezoidal")
      << ")";
  return oss.str();
}

}  // namespace puma560_cpp

