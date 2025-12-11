/**
 * @file compliance_control.hpp
 * @brief Position-Based Admittance Control for Constant Force Regulation
 * 
 * Implements position-based admittance control:
 *   x_cmd = x_eq + Kf * (F_d - F_meas)
 * 
 * Where:
 *   x_eq   = equilibrium position (found during contact establishment)
 *   Kf     = compliance gain (m/N)
 *   F_d    = desired contact force
 *   F_meas = measured contact force
 * 
 * Features:
 * - Two-zone deadband for steady state behavior
 * - Gain scheduling for different error magnitudes
 * - Low-pass filtering of force measurements
 * - Force sensor calibration
 * 
 * Filter Design and Stability:
 * ============================
 * The low-pass filter for force measurements is a first-order exponential filter:
 *     y[k] = alpha * x[k] + (1 - alpha) * y[k-1]
 * 
 * The time constant tau relates to alpha and sampling frequency f as:
 *     tau = 1 / (alpha * f)
 * 
 * @author Sahruday Patti
 */

#ifndef PUMA560_CPP__COMPLIANCE_CONTROL_HPP_
#define PUMA560_CPP__COMPLIANCE_CONTROL_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <moveit_msgs/srv/get_position_fk.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "puma560_cpp/quaternion_utils.hpp"

namespace puma560_cpp
{

/**
 * @class ComplianceControl
 * @brief ROS 2 node for position-based admittance control
 * 
 * Uses MutuallyExclusive callback groups for action client (ROS2 best practice)
 * and Reentrant callback group for subscribers.
 */
class ComplianceControl : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
  using GetPositionIK = moveit_msgs::srv::GetPositionIK;
  using GetPositionFK = moveit_msgs::srv::GetPositionFK;

  /**
   * @brief Constructor
   */
  explicit ComplianceControl(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /**
   * @brief Destructor
   */
  ~ComplianceControl();

  /**
   * @brief Execute compliance control demo
   */
  bool executeDemo();

private:
  // Joint configuration - matches Python exactly
  static const std::vector<std::string> ALL_JOINT_NAMES;
  static const std::string ACTION_NAME;
  static const std::string RESULTS_DIR;
  static const std::map<std::string, double> PRE_CONTACT_JOINTS;

  // Control parameters (matching Python defaults)
  double f_desired_ = 100.0;        // Target contact force (N)
  double kf_compliance_ = 0.00003;  // Compliance gain (m/N) = 0.03mm per Newton error
  double control_hz_ = 50.0;        // Control rate (Hz)
  double filter_alpha_ = 0.015;     // Smoother filtering (tau ≈ 1.3s at 50Hz)
  
  // Two-zone deadband parameters (matching Python)
  double deadband_inner_ = 8.0;     // +/-8N: True steady state (92-108N)
  double deadband_outer_ = 20.0;    // 8-20N below target: Slow correction zone
  double deadband_high_ = 25.0;     // Only retract if F > 125N
  
  // Two-level correction rates (gain scheduling)
  double correction_fast_ = 0.0002;   // Fast: 0.2mm/s for large errors (F < 80N)
  double correction_slow_ = 0.00008;  // Slow: 0.08mm/s for small errors (80-92N)
  
  // Approach parameters
  double approach_vel_ = 0.002;     // Approach velocity (m/s) = 2mm/s
  double contact_threshold_ = 80.0; // Wait until force is near target
  double wall_x_ = 0.87;            // Wall surface position (m)
  double duration_ = 30.0;          // Total control duration (s)

  // Callback groups for thread safety
  rclcpp::CallbackGroup::SharedPtr action_callback_group_;
  rclcpp::CallbackGroup::SharedPtr sub_callback_group_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr ft_sensor_sub_;

  // Service clients
  rclcpp::Client<GetPositionIK>::SharedPtr ik_client_;
  rclcpp::Client<GetPositionFK>::SharedPtr fk_client_;

  // Action client (with MutuallyExclusive callback group)
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;

  // Timer for control loop
  rclcpp::TimerBase::SharedPtr control_timer_;

  // State
  std::mutex state_mutex_;
  Eigen::VectorXd current_positions_;
  bool has_joint_state_ = false;
  double current_lift_position_ = 0.0;
  
  // Force sensor state
  std::atomic<double> force_raw_{0.0};
  double force_filtered_ = 0.0;
  double force_offset_ = 0.0;  // For calibration (like Python)
  
  // Control state
  double x_equilibrium_ = 0.0;
  double x_actual_ = 0.0;
  double y_fixed_ = 0.0;
  double z_fixed_ = 0.0;
  Eigen::Quaterniond current_orientation_;
  bool in_contact_ = false;
  bool control_active_ = false;
  std::string phase_ = "APPROACH";

  // Recording - matches Python structure exactly
  bool recording_ = false;
  double record_start_time_ = 0.0;
  std::vector<double> recorded_times_;
  std::vector<double> recorded_forces_raw_;
  std::vector<double> recorded_forces_filtered_;
  std::vector<double> recorded_contact_force_;
  std::vector<double> recorded_x_actual_;
  std::vector<double> recorded_x_cmd_;
  std::vector<double> recorded_x_eq_;
  std::vector<double> recorded_force_error_;

  /**
   * @brief Callback for joint state messages
   */
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  /**
   * @brief Callback for force/torque sensor messages
   * Note: Subscribes to /ft_sensor (matching Python), not /ft_sensor/wrench
   */
  void ftSensorCallback(const geometry_msgs::msg::Wrench::SharedPtr msg);

  /**
   * @brief Control loop callback (called by timer)
   */
  void controlLoop();

  /**
   * @brief Wait for valid joint state
   */
  bool waitForJointState(double timeout_sec = 10.0);

  /**
   * @brief Wait for services
   */
  bool waitForServices(double timeout_sec = 30.0);

  /**
   * @brief Calibrate force sensor (zero offset in free space)
   * Matches Python's calibrate_force() method
   */
  void calibrateForce();

  /**
   * @brief Get calibrated contact force
   */
  double getContactForce();

  /**
   * @brief Compute IK (all 7 joints)
   */
  std::optional<Eigen::VectorXd> computeIK(
    const Eigen::Vector3d& position,
    const Eigen::Quaterniond& orientation,
    const Eigen::VectorXd& seed_state);

  /**
   * @brief Compute FK (all 7 joints)
   */
  std::optional<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> computeFK(
    const Eigen::VectorXd& joint_positions);

  /**
   * @brief Move to pre-contact position
   */
  bool moveToPreContact();

  /**
   * @brief Run the main control loop (approach + regulate phases)
   */
  void runControl();

  /**
   * @brief Send position command to robot
   */
  bool sendJointCommand(const std::map<std::string, double>& joints, double duration = 0.05);

  /**
   * @brief Start recording
   */
  void startRecording();

  /**
   * @brief Stop recording
   */
  void stopRecording();

  /**
   * @brief Save data to CSV
   */
  void saveToCSV(const std::string& filename);

  /**
   * @brief Log configuration
   */
  void logConfig();

  /**
   * @brief Get current time (wall clock)
   */
  double now() const;
};

}  // namespace puma560_cpp

#endif  // PUMA560_CPP__COMPLIANCE_CONTROL_HPP_
