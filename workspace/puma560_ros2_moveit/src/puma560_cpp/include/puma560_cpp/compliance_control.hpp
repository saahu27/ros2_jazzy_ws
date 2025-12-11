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
  // Joint configuration
  static const std::vector<std::string> ARM_JOINT_NAMES;
  static const std::vector<std::string> ALL_JOINT_NAMES;
  static const std::string ACTION_NAME;
  static const std::string RESULTS_DIR;

  // Control parameters
  double f_desired_ = 100.0;        // Target contact force (N)
  double kf_compliance_ = 0.00003;  // Compliance gain (m/N)
  double control_hz_ = 50.0;        // Control rate (Hz)
  double filter_alpha_ = 0.005;     // Force filter alpha
  
  // Deadband parameters
  double deadband_inner_ = 8.0;     // Inner deadband (N)
  double deadband_outer_ = 20.0;    // Outer deadband (N)
  double deadband_high_ = 25.0;     // High force threshold (N)
  
  // Correction rates
  double correction_fast_ = 0.0001; // Fast correction (m/s)
  double correction_slow_ = 0.00005; // Slow correction (m/s)
  
  // Approach parameters
  double approach_vel_ = 0.002;     // Approach velocity (m/s)
  double contact_threshold_ = 80.0; // Contact detection threshold (N)
  double wall_x_ = 0.87;            // Wall position (m)
  double duration_ = 30.0;          // Total control duration (s)

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr ft_sensor_sub_;

  // Service clients
  rclcpp::Client<GetPositionIK>::SharedPtr ik_client_;
  rclcpp::Client<GetPositionFK>::SharedPtr fk_client_;

  // Action client
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
  
  // Control state
  double x_equilibrium_ = 0.0;
  double x_actual_ = 0.0;
  Eigen::Quaterniond current_orientation_;
  bool in_contact_ = false;
  bool control_active_ = false;

  // Recording
  bool recording_ = false;
  double record_start_time_ = 0.0;
  std::vector<double> recorded_times_;
  std::vector<double> recorded_forces_raw_;
  std::vector<double> recorded_forces_filtered_;
  std::vector<double> recorded_x_positions_;
  std::vector<double> recorded_x_commands_;

  /**
   * @brief Callback for joint state messages
   */
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  /**
   * @brief Callback for force/torque sensor messages
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
   * @brief Compute IK
   */
  std::optional<Eigen::VectorXd> computeIK(
    const Eigen::Vector3d& position,
    const Eigen::Quaterniond& orientation,
    const Eigen::VectorXd& seed_state);

  /**
   * @brief Compute FK
   */
  std::optional<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> computeFK(
    const Eigen::VectorXd& joint_positions);

  /**
   * @brief Move to pre-contact position
   */
  bool moveToPreContact();

  /**
   * @brief Approach wall until contact
   */
  bool approachWall();

  /**
   * @brief Execute single control step
   */
  void executeControlStep(double dt);

  /**
   * @brief Send position command to robot
   */
  bool sendPositionCommand(const Eigen::Vector3d& position);

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
   * @brief Get current time
   */
  double now() const;
};

}  // namespace puma560_cpp

#endif  // PUMA560_CPP__COMPLIANCE_CONTROL_HPP_

