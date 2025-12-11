/**
 * @file synchronized_profiles.cpp
 * @brief Implementation of SynchronizedProfiles class
 * 
 * Uses VELOCITY SCALING (not time-scaling) to synchronize joints.
 * This matches the correct Python implementation.
 */

#include "puma560_cpp/synchronized_profiles.hpp"
#include <algorithm>
#include <cmath>

namespace puma560_cpp
{

// Default per-joint velocity limits
const std::map<std::string, double> SynchronizedProfiles::DEFAULT_V_MAX = {
  {"lift_joint", 0.5},  // m/s (prismatic)
  {"j1", 2.0},          // rad/s (revolute)
  {"j2", 2.0},
  {"j3", 2.0},
  {"j4", 2.0},
  {"j5", 2.0},
  {"j6", 2.0}
};

// Default per-joint acceleration limits
const std::map<std::string, double> SynchronizedProfiles::DEFAULT_A_MAX = {
  {"lift_joint", 1.0},  // m/s² (prismatic)
  {"j1", 5.0},          // rad/s² (revolute)
  {"j2", 5.0},
  {"j3", 5.0},
  {"j4", 5.0},
  {"j5", 5.0},
  {"j6", 5.0}
};

SynchronizedProfiles::SynchronizedProfiles(
  const std::vector<std::string>& joint_names,
  const Eigen::VectorXd& start_positions,
  const Eigen::VectorXd& end_positions,
  const std::map<std::string, double>& v_max_dict,
  const std::map<std::string, double>& a_max_dict)
  : joint_names_(joint_names),
    start_(start_positions),
    end_(end_positions),
    v_max_dict_(v_max_dict),
    a_max_dict_(a_max_dict),
    total_time_(0.0)
{
  deltas_ = end_ - start_;
  computeSynchronizedProfiles();
}

double SynchronizedProfiles::getVMax(const std::string& name) const
{
  // First check user-provided limits
  auto it = v_max_dict_.find(name);
  if (it != v_max_dict_.end()) {
    return it->second;
  }
  // Fall back to defaults
  auto default_it = DEFAULT_V_MAX.find(name);
  if (default_it != DEFAULT_V_MAX.end()) {
    return default_it->second;
  }
  // Ultimate fallback
  return 1.0;
}

double SynchronizedProfiles::getAMax(const std::string& name) const
{
  // First check user-provided limits
  auto it = a_max_dict_.find(name);
  if (it != a_max_dict_.end()) {
    return it->second;
  }
  // Fall back to defaults
  auto default_it = DEFAULT_A_MAX.find(name);
  if (default_it != DEFAULT_A_MAX.end()) {
    return default_it->second;
  }
  // Ultimate fallback
  return 1.0;
}

void SynchronizedProfiles::computeSynchronizedProfiles()
{
  size_t n_joints = joint_names_.size();
  profiles_.resize(n_joints);

  // Step 1: Compute unconstrained times for each joint
  std::vector<double> unconstrained_times(n_joints, 0.0);
  
  for (size_t i = 0; i < n_joints; ++i) {
    double abs_delta = std::abs(deltas_(i));
    if (abs_delta < 1e-9) {
      unconstrained_times[i] = 0.0;
      profiles_[i] = nullptr;
      continue;
    }

    double v_max = getVMax(joint_names_[i]);
    double a_max = getAMax(joint_names_[i]);

    // Create test profile with absolute delta to get timing
    TrapezoidalProfile test_profile(abs_delta, v_max, a_max);
    unconstrained_times[i] = test_profile.getTotalTime();
  }

  // Step 2: Find slowest joint (determines total time)
  total_time_ = *std::max_element(unconstrained_times.begin(), unconstrained_times.end());

  if (total_time_ < 1e-9) {
    // No motion needed
    return;
  }

  // Step 3: Create profiles with SCALED VELOCITY to match total_time
  // This is the CORRECT approach - scale velocity, not time!
  for (size_t i = 0; i < n_joints; ++i) {
    double delta = deltas_(i);  // Keep the sign for direction!
    double abs_delta = std::abs(delta);
    
    if (abs_delta < 1e-9) {
      profiles_[i] = nullptr;
      continue;
    }
    
    double v_max_orig = getVMax(joint_names_[i]);
    double a_max_orig = getAMax(joint_names_[i]);
    
    // Check if this joint is the slowest (or very close)
    if (std::abs(unconstrained_times[i] - total_time_) < 1e-6) {
      // This is the slowest joint - use original limits with SIGNED delta
      profiles_[i] = std::make_unique<TrapezoidalProfile>(delta, v_max_orig, a_max_orig);
      continue;
    }
    
    // This joint would finish early - scale DOWN velocity to match total_time
    // For trapezoidal profile: T = v/a + d/v
    // Solving for v given T and d:
    //   v² - T*a*v + a*d = 0
    //   v = (T*a - sqrt(T²*a² - 4*a*d)) / 2
    
    double T = total_time_;
    double a = a_max_orig;
    double d = abs_delta;
    
    double discriminant = T * T * a * a - 4.0 * a * d;
    
    double v_scaled;
    double a_scaled = a_max_orig;
    
    if (discriminant >= 0) {
      // Trapezoidal or triangular profile possible with this acceleration
      v_scaled = (T * a - std::sqrt(discriminant)) / 2.0;
      v_scaled = std::min(v_scaled, v_max_orig);  // Don't exceed original limit
      v_scaled = std::max(v_scaled, 1e-6);        // Avoid zero velocity
    } else {
      // Discriminant < 0: Need triangular profile with lower acceleration
      // For triangular: T = 2*sqrt(d/a) => a = 4*d/T²
      a_scaled = 4.0 * d / (T * T);
      a_scaled = std::min(a_scaled, a_max_orig);  // Don't exceed original limit
      v_scaled = v_max_orig;  // Will become triangular anyway
    }
    
    // Create profile with SIGNED delta to preserve direction
    profiles_[i] = std::make_unique<TrapezoidalProfile>(delta, v_scaled, a_scaled);
  }
}

std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>
SynchronizedProfiles::evaluate(double t) const
{
  size_t n_joints = joint_names_.size();
  Eigen::VectorXd positions(n_joints);
  Eigen::VectorXd velocities(n_joints);
  Eigen::VectorXd accelerations(n_joints);

  for (size_t i = 0; i < n_joints; ++i) {
    if (!profiles_[i]) {
      // Joint doesn't move - stay at start position
      positions(i) = start_(i);
      velocities(i) = 0.0;
      accelerations(i) = 0.0;
    } else {
      // Evaluate the profile directly - no time scaling needed!
      // TrapezoidalProfile already handles the sign internally
      auto [pos, vel, acc] = profiles_[i]->evaluate(t);
      
      // pos is the displacement from start, add to start position
      positions(i) = start_(i) + pos;
      velocities(i) = vel;
      accelerations(i) = acc;
    }
  }

  return {positions, velocities, accelerations};
}

const TrapezoidalProfile* SynchronizedProfiles::getJointProfile(size_t index) const
{
  if (index < profiles_.size()) {
    return profiles_[index].get();
  }
  return nullptr;
}

}  // namespace puma560_cpp
