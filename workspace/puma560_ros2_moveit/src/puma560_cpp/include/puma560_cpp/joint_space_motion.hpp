/**
 * @file joint_space_motion.hpp
 * @brief Joint Space Motion Controller with synchronized trapezoidal profiles
 * 
 * Implements joint-space motion control using:
 * - Independent per-joint trapezoidal velocity profiles
 * - Synchronized timing (all joints finish together)
 * - FollowJointTrajectory action client for execution
 * 
 * @author Sahruday Patti
 */

#ifndef PUMA560_CPP__JOINT_SPACE_MOTION_HPP_
#define PUMA560_CPP__JOINT_SPACE_MOTION_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <Eigen/Dense>

#include "puma560_cpp/synchronized_profiles.hpp"

namespace puma560_cpp
{

/**
 * @class JointSpaceMotion
 * @brief ROS 2 node for joint space motion control
 */
class JointSpaceMotion : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  /**
   * @brief Constructor
   * @param options Node options
   */
  explicit JointSpaceMotion(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /**
   * @brief Destructor
   */
  ~JointSpaceMotion();

  /**
   * @brief Execute a series of waypoints
   * @return true if all waypoints executed successfully
   */
  bool executeWaypoints();

private:
  // Joint configuration
  static const std::vector<std::string> JOINT_NAMES;
  static const std::string ACTION_NAME;
  static const std::string RESULTS_DIR;

  // Callback groups (must be stored as member to keep alive)
  rclcpp::CallbackGroup::SharedPtr action_callback_group_;
  rclcpp::CallbackGroup::SharedPtr sub_callback_group_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  // Action client with dedicated callback group
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;

  // State
  std::mutex state_mutex_;
  Eigen::VectorXd current_positions_;
  bool has_joint_state_ = false;

  // Recording - uses wall clock for monotonic timestamps (like Python version)
  bool recording_ = false;
  double record_start_time_wall_ = 0.0;  // Wall clock for data recording
  std::vector<double> recorded_times_;
  std::map<std::string, std::vector<double>> recorded_positions_;
  std::map<std::string, std::vector<double>> recorded_velocities_;
  std::vector<double> commanded_times_;
  std::map<std::string, std::vector<double>> commanded_positions_;
  std::map<std::string, std::vector<double>> commanded_velocities_;

  /**
   * @brief Callback for joint state messages
   */
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  /**
   * @brief Wait for valid joint state
   * @param timeout_sec Timeout in seconds
   * @return true if joint state received
   */
  bool waitForJointState(double timeout_sec = 10.0);

  /**
   * @brief Generate trajectory with synchronized trapezoidal profiles
   * @param start_joints Starting joint positions
   * @param end_joints Target joint positions
   * @param v_max_scale Velocity scaling factor (0-1)
   * @param a_max_scale Acceleration scaling factor (0-1)
   * @param dt Trajectory point spacing
   * @return Generated trajectory message
   */
  trajectory_msgs::msg::JointTrajectory generateTrajectory(
    const Eigen::VectorXd& start_joints,
    const Eigen::VectorXd& end_joints,
    double v_max_scale = 1.0,
    double a_max_scale = 1.0,
    double dt = 0.02);

  /**
   * @brief Execute a trajectory
   * @param trajectory Trajectory to execute
   * @return true if execution successful
   */
  bool executeTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory);

  /**
   * @brief Start recording joint data
   */
  void startRecording();

  /**
   * @brief Stop recording and return data
   */
  void stopRecording();

  /**
   * @brief Save recorded data to CSV file
   * @param filename Output filename (without path)
   */
  void saveToCSV(const std::string& filename);

  /**
   * @brief Get current time in seconds
   */
  double now() const;
};

}  // namespace puma560_cpp

#endif  // PUMA560_CPP__JOINT_SPACE_MOTION_HPP_

