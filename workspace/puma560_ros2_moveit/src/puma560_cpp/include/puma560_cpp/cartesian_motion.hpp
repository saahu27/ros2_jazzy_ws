/**
 * @file cartesian_motion.hpp
 * @brief Cartesian Motion Controller with trapezoidal velocity profiles
 * 
 * Implements Cartesian-space motion control using:
 * - Trapezoidal velocity profiles for path parameter s(t)
 * - Linear interpolation for position: x(t) = x_start + (x_end - x_start) * s(t)
 * - SLERP for orientation interpolation
 * - MoveIt IK service for joint solution
 * 
 * Theory Reference:
 * - Craig, "Introduction to Robotics", Chapter 7: Trajectory Generation
 * - Siciliano et al., "Robotics: Modelling, Planning and Control", Chapter 3
 * 
 * @author Sahruday Patti
 */

#ifndef PUMA560_CPP__CARTESIAN_MOTION_HPP_
#define PUMA560_CPP__CARTESIAN_MOTION_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <moveit_msgs/srv/get_position_fk.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "puma560_cpp/trapezoidal_profile.hpp"
#include "puma560_cpp/quaternion_utils.hpp"

namespace puma560_cpp
{

/**
 * @class CartesianMotion
 * @brief ROS 2 node for Cartesian space motion control
 */
class CartesianMotion : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
  using GetPositionIK = moveit_msgs::srv::GetPositionIK;
  using GetPositionFK = moveit_msgs::srv::GetPositionFK;

  /**
   * @brief Constructor
   */
  explicit CartesianMotion(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /**
   * @brief Destructor
   */
  ~CartesianMotion();

  /**
   * @brief Execute a Cartesian motion demo pattern
   */
  bool executeDemo();

private:
  // Joint configuration
  static const std::vector<std::string> ARM_JOINT_NAMES;
  static const std::vector<std::string> ALL_JOINT_NAMES;
  static const std::string ACTION_NAME;
  static const std::string RESULTS_DIR;

  // Parameters
  double v_max_ = 0.05;       // Max Cartesian velocity (m/s)
  double a_max_ = 0.5;        // Max Cartesian acceleration (m/s²)
  double lift_v_max_ = 0.08;  // Max lift velocity (m/s)
  double dt_ = 0.005;         // Trajectory sampling period (s)

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  // Service clients
  rclcpp::Client<GetPositionIK>::SharedPtr ik_client_;
  rclcpp::Client<GetPositionFK>::SharedPtr fk_client_;

  // Action client
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;

  // State
  std::mutex state_mutex_;
  Eigen::VectorXd current_positions_;
  bool has_joint_state_ = false;
  double current_lift_position_ = 0.0;

  // Recording
  bool recording_ = false;
  double record_start_time_ = 0.0;
  std::vector<double> recorded_times_;
  std::map<std::string, std::vector<double>> recorded_positions_;
  std::map<std::string, std::vector<double>> recorded_velocities_;
  std::vector<Eigen::Vector3d> recorded_ee_positions_;
  std::vector<Eigen::Vector3d> recorded_ee_velocities_;

  /**
   * @brief Callback for joint state messages
   */
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  /**
   * @brief Wait for valid joint state
   */
  bool waitForJointState(double timeout_sec = 10.0);

  /**
   * @brief Wait for services to become available
   */
  bool waitForServices(double timeout_sec = 30.0);

  /**
   * @brief Compute inverse kinematics
   * @param position Desired EE position
   * @param orientation Desired EE orientation
   * @param seed_state Current joint state for IK seed
   * @return Joint positions if IK succeeded
   */
  std::optional<Eigen::VectorXd> computeIK(
    const Eigen::Vector3d& position,
    const Eigen::Quaterniond& orientation,
    const Eigen::VectorXd& seed_state);

  /**
   * @brief Compute forward kinematics
   * @param joint_positions Current joint positions
   * @return Pair of (position, orientation) if FK succeeded
   */
  std::optional<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> computeFK(
    const Eigen::VectorXd& joint_positions);

  /**
   * @brief Generate Cartesian trajectory with trapezoidal profile
   * @param start_pos Starting Cartesian position
   * @param end_pos Target Cartesian position
   * @param orientation Fixed orientation during motion
   * @param seed_joints Initial joint configuration for IK
   * @return Generated trajectory or nullopt if IK failed
   */
  std::optional<trajectory_msgs::msg::JointTrajectory> generateCartesianTrajectory(
    const Eigen::Vector3d& start_pos,
    const Eigen::Vector3d& end_pos,
    const Eigen::Quaterniond& orientation,
    const Eigen::VectorXd& seed_joints);

  /**
   * @brief Generate lift trajectory (prismatic joint motion)
   * @param start_height Starting height
   * @param end_height Target height
   * @param hold_arm_joints Arm joints to hold constant during lift
   * @return Generated trajectory
   */
  trajectory_msgs::msg::JointTrajectory generateLiftTrajectory(
    double start_height,
    double end_height,
    const Eigen::VectorXd& hold_arm_joints);

  /**
   * @brief Execute a trajectory
   */
  bool executeTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory);

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

#endif  // PUMA560_CPP__CARTESIAN_MOTION_HPP_

