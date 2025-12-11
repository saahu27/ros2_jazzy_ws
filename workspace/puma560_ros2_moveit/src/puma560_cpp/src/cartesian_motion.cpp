/**
 * @file cartesian_motion.cpp
 * @brief Implementation of CartesianMotion node
 */

#include "puma560_cpp/cartesian_motion.hpp"
#include "puma560_cpp/synchronized_profiles.hpp"

#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <filesystem>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace puma560_cpp
{

// Static member definitions
const std::vector<std::string> CartesianMotion::ARM_JOINT_NAMES = {
  "j1", "j2", "j3", "j4", "j5", "j6"
};
const std::vector<std::string> CartesianMotion::ALL_JOINT_NAMES = {
  "lift_joint", "j1", "j2", "j3", "j4", "j5", "j6"
};
const std::string CartesianMotion::ACTION_NAME = "/arm_controller/follow_joint_trajectory";
const std::string CartesianMotion::RESULTS_DIR = "/root/ros2_ws/src/puma560_ros2_moveit/results";

CartesianMotion::CartesianMotion(const rclcpp::NodeOptions& options)
  : Node("cartesian_motion", options),
    current_positions_(Eigen::VectorXd::Zero(ALL_JOINT_NAMES.size()))
{
  RCLCPP_INFO(get_logger(), "Initializing CartesianMotion node");

  // Declare parameters
  v_max_ = declare_parameter("v_max", 0.05);
  a_max_ = declare_parameter("a_max", 0.5);
  lift_v_max_ = declare_parameter("lift_v_max", 0.08);
  dt_ = declare_parameter("dt", 0.005);

  // Create callback group for concurrent execution
  auto callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // Create joint state subscriber
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group;
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    std::bind(&CartesianMotion::jointStateCallback, this, _1),
    sub_options);

  // Create service clients
  ik_client_ = create_client<GetPositionIK>("/compute_ik");
  fk_client_ = create_client<GetPositionFK>("/compute_fk");

  // Create action client
  action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
    this, ACTION_NAME);

  // Ensure results directory exists
  std::filesystem::create_directories(RESULTS_DIR);

  RCLCPP_INFO(get_logger(), "CartesianMotion node initialized");
}

CartesianMotion::~CartesianMotion()
{
  RCLCPP_INFO(get_logger(), "CartesianMotion node shutting down");
}

void CartesianMotion::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  bool has_velocity = msg->velocity.size() == msg->name.size();

  for (size_t i = 0; i < msg->name.size(); ++i) {
    auto it = std::find(ALL_JOINT_NAMES.begin(), ALL_JOINT_NAMES.end(), msg->name[i]);
    if (it != ALL_JOINT_NAMES.end()) {
      size_t idx = std::distance(ALL_JOINT_NAMES.begin(), it);
      current_positions_(idx) = msg->position[i];
      
      if (msg->name[i] == "lift_joint") {
        current_lift_position_ = msg->position[i];
      }
    }
  }

  // Check if we have all joints
  size_t count = 0;
  for (const auto& name : ALL_JOINT_NAMES) {
    if (std::find(msg->name.begin(), msg->name.end(), name) != msg->name.end()) {
      count++;
    }
  }
  if (count == ALL_JOINT_NAMES.size()) {
    has_joint_state_ = true;
  }

  // Recording
  if (recording_) {
    double t = now() - record_start_time_;
    recorded_times_.push_back(t);

    for (size_t i = 0; i < ALL_JOINT_NAMES.size(); ++i) {
      const auto& name = ALL_JOINT_NAMES[i];
      recorded_positions_[name].push_back(current_positions_(i));

      // Find velocity from message
      auto msg_it = std::find(msg->name.begin(), msg->name.end(), name);
      if (msg_it != msg->name.end() && has_velocity) {
        size_t msg_idx = std::distance(msg->name.begin(), msg_it);
        recorded_velocities_[name].push_back(msg->velocity[msg_idx]);
      } else {
        recorded_velocities_[name].push_back(0.0);
      }
    }
  }
}

bool CartesianMotion::waitForJointState(double timeout_sec)
{
  // Executor is already spinning in another thread, just wait for the flag
  auto start = std::chrono::steady_clock::now();
  while (!has_joint_state_) {
    auto elapsed = std::chrono::steady_clock::now() - start;
    if (std::chrono::duration<double>(elapsed).count() > timeout_sec) {
      RCLCPP_ERROR(get_logger(), "Timeout waiting for joint state");
      return false;
    }
    std::this_thread::sleep_for(50ms);
  }
  return true;
}

bool CartesianMotion::waitForServices(double timeout_sec)
{
  RCLCPP_INFO(get_logger(), "Waiting for IK/FK services...");
  
  if (!ik_client_->wait_for_service(std::chrono::duration<double>(timeout_sec))) {
    RCLCPP_ERROR(get_logger(), "IK service not available");
    return false;
  }
  
  if (!fk_client_->wait_for_service(std::chrono::duration<double>(timeout_sec))) {
    RCLCPP_ERROR(get_logger(), "FK service not available");
    return false;
  }
  
  RCLCPP_INFO(get_logger(), "IK/FK services available");
  return true;
}

std::optional<Eigen::VectorXd> CartesianMotion::computeIK(
  const Eigen::Vector3d& position,
  const Eigen::Quaterniond& orientation,
  const Eigen::VectorXd& seed_state)
{
  auto request = std::make_shared<GetPositionIK::Request>();
  
  // Set up IK request
  request->ik_request.group_name = "arm";
  request->ik_request.avoid_collisions = false;
  
  // Set target pose - use world frame (matching Python)
  request->ik_request.pose_stamped.header.frame_id = "world";
  request->ik_request.pose_stamped.pose.position.x = position.x();
  request->ik_request.pose_stamped.pose.position.y = position.y();
  request->ik_request.pose_stamped.pose.position.z = position.z();
  request->ik_request.pose_stamped.pose.orientation = toMsg(orientation);
  
  // Set seed state (arm joints only)
  request->ik_request.robot_state.joint_state.name = ARM_JOINT_NAMES;
  request->ik_request.robot_state.joint_state.position.resize(ARM_JOINT_NAMES.size());
  for (size_t i = 0; i < ARM_JOINT_NAMES.size(); ++i) {
    // Find this joint in the full state (skip lift_joint at index 0)
    request->ik_request.robot_state.joint_state.position[i] = seed_state(i + 1);
  }

  // Call service - executor is already spinning, use future.wait_for directly
  auto future = ik_client_->async_send_request(request);
  if (future.wait_for(5s) != std::future_status::ready) {
    RCLCPP_WARN(get_logger(), "IK service call timeout");
    return std::nullopt;
  }

  auto response = future.get();
  if (response->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_WARN(get_logger(), "IK failed with error code: %d", response->error_code.val);
    return std::nullopt;
  }

  // Extract joint positions
  Eigen::VectorXd result(ARM_JOINT_NAMES.size());
  const auto& joint_state = response->solution.joint_state;
  for (size_t i = 0; i < ARM_JOINT_NAMES.size(); ++i) {
    auto it = std::find(joint_state.name.begin(), joint_state.name.end(), ARM_JOINT_NAMES[i]);
    if (it != joint_state.name.end()) {
      size_t idx = std::distance(joint_state.name.begin(), it);
      result(i) = joint_state.position[idx];
    }
  }

  return result;
}

std::optional<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> CartesianMotion::computeFK(
  const Eigen::VectorXd& joint_positions)
{
  auto request = std::make_shared<GetPositionFK::Request>();
  
  request->header.frame_id = "world";  // Base frame
  request->fk_link_names = {"link7"};  // End effector link (matching Python)
  
  // Set joint state (arm joints only)
  request->robot_state.joint_state.name = ARM_JOINT_NAMES;
  request->robot_state.joint_state.position.resize(ARM_JOINT_NAMES.size());
  for (size_t i = 0; i < ARM_JOINT_NAMES.size(); ++i) {
    request->robot_state.joint_state.position[i] = joint_positions(i + 1);  // Skip lift_joint
  }

  // Call service - executor is already spinning, use future.wait_for directly
  auto future = fk_client_->async_send_request(request);
  if (future.wait_for(5s) != std::future_status::ready) {
    RCLCPP_WARN(get_logger(), "FK service call timeout");
    return std::nullopt;
  }

  auto response = future.get();
  if (response->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_WARN(get_logger(), "FK failed with error code: %d", response->error_code.val);
    return std::nullopt;
  }

  if (response->pose_stamped.empty()) {
    RCLCPP_WARN(get_logger(), "FK returned no poses");
    return std::nullopt;
  }

  const auto& pose = response->pose_stamped[0].pose;
  Eigen::Vector3d position(pose.position.x, pose.position.y, pose.position.z);
  Eigen::Quaterniond orientation = fromMsg(pose.orientation);

  return std::make_pair(position, orientation);
}

std::optional<trajectory_msgs::msg::JointTrajectory> CartesianMotion::generateCartesianTrajectory(
  const Eigen::Vector3d& start_pos,
  const Eigen::Vector3d& end_pos,
  const Eigen::Quaterniond& orientation,
  const Eigen::VectorXd& seed_joints)
{
  // Compute distance
  double distance = (end_pos - start_pos).norm();
  if (distance < 1e-6) {
    RCLCPP_INFO(get_logger(), "Start and end positions are the same");
    return trajectory_msgs::msg::JointTrajectory();
  }

  // Create trapezoidal profile for path parameter s(t)
  TrapezoidalProfile profile(distance, v_max_, a_max_);
  double total_time = profile.getTotalTime();

  RCLCPP_INFO(get_logger(), "Generating Cartesian trajectory: d=%.3fm, T=%.3fs", 
              distance, total_time);

  // Generate trajectory points
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = ALL_JOINT_NAMES;

  int n_points = static_cast<int>(std::ceil(total_time / dt_)) + 1;
  Eigen::VectorXd current_seed = seed_joints;
  int ik_failures = 0;
  const int max_ik_failures = 5;

  for (int i = 0; i < n_points; ++i) {
    double t = std::min(i * dt_, total_time);
    auto [s, s_dot, s_ddot] = profile.evaluate(t);

    // Interpolate position
    Eigen::Vector3d pos = start_pos + (end_pos - start_pos) * (s / distance);

    // Compute IK
    auto ik_result = computeIK(pos, orientation, current_seed);
    if (!ik_result) {
      ik_failures++;
      if (ik_failures > max_ik_failures) {
        RCLCPP_ERROR(get_logger(), "Too many IK failures");
        return std::nullopt;
      }
      continue;
    }

    ik_failures = 0;
    Eigen::VectorXd arm_joints = *ik_result;

    // Create trajectory point
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.resize(ALL_JOINT_NAMES.size());
    point.velocities.resize(ALL_JOINT_NAMES.size());
    point.accelerations.resize(ALL_JOINT_NAMES.size());

    // Lift joint stays constant
    point.positions[0] = current_lift_position_;
    point.velocities[0] = 0.0;
    point.accelerations[0] = 0.0;

    // Arm joints from IK
    for (size_t j = 0; j < ARM_JOINT_NAMES.size(); ++j) {
      point.positions[j + 1] = arm_joints(j);
      // Velocity estimation from Jacobian would be more accurate
      // For now, use numerical differentiation if we have previous point
      if (!trajectory.points.empty()) {
        double dt_actual = dt_;
        point.velocities[j + 1] = (arm_joints(j) - trajectory.points.back().positions[j + 1]) / dt_actual;
      } else {
        point.velocities[j + 1] = 0.0;
      }
      point.accelerations[j + 1] = 0.0;
    }

    point.time_from_start.sec = static_cast<int32_t>(t);
    point.time_from_start.nanosec = static_cast<uint32_t>((t - point.time_from_start.sec) * 1e9);

    trajectory.points.push_back(point);

    // Update seed for next IK
    current_seed(0) = current_lift_position_;
    for (size_t j = 0; j < ARM_JOINT_NAMES.size(); ++j) {
      current_seed(j + 1) = arm_joints(j);
    }
  }

  // Ensure the last point has zero velocity (JTC requirement)
  if (!trajectory.points.empty()) {
    auto& last_point = trajectory.points.back();
    for (size_t j = 0; j < last_point.velocities.size(); ++j) {
      last_point.velocities[j] = 0.0;
      last_point.accelerations[j] = 0.0;
    }
  }

  return trajectory;
}

trajectory_msgs::msg::JointTrajectory CartesianMotion::generateLiftTrajectory(
  double start_height,
  double end_height,
  const Eigen::VectorXd& hold_arm_joints)
{
  TrapezoidalProfile profile(end_height - start_height, lift_v_max_, a_max_);
  double total_time = profile.getTotalTime();

  RCLCPP_INFO(get_logger(), "Generating lift trajectory: h=%.3f->%.3f, T=%.3fs",
              start_height, end_height, total_time);

  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = ALL_JOINT_NAMES;

  int n_points = static_cast<int>(std::ceil(total_time / dt_)) + 1;
  for (int i = 0; i < n_points; ++i) {
    double t = std::min(i * dt_, total_time);
    auto [pos, vel, acc] = profile.evaluate(t);

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.resize(ALL_JOINT_NAMES.size());
    point.velocities.resize(ALL_JOINT_NAMES.size());
    point.accelerations.resize(ALL_JOINT_NAMES.size());

    // Lift joint
    point.positions[0] = start_height + pos;
    point.velocities[0] = vel;
    point.accelerations[0] = acc;

    // Arm joints held constant
    for (size_t j = 0; j < ARM_JOINT_NAMES.size(); ++j) {
      point.positions[j + 1] = hold_arm_joints(j + 1);
      point.velocities[j + 1] = 0.0;
      point.accelerations[j + 1] = 0.0;
    }

    point.time_from_start.sec = static_cast<int32_t>(t);
    point.time_from_start.nanosec = static_cast<uint32_t>((t - point.time_from_start.sec) * 1e9);

    trajectory.points.push_back(point);
  }

  return trajectory;
}

bool CartesianMotion::executeTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory)
{
  if (trajectory.points.empty()) {
    RCLCPP_INFO(get_logger(), "Empty trajectory, skipping");
    return true;
  }

  if (!action_client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(get_logger(), "Action server not available");
    return false;
  }

  auto goal = FollowJointTrajectory::Goal();
  goal.trajectory = trajectory;

  RCLCPP_INFO(get_logger(), "Sending trajectory with %zu points", trajectory.points.size());

  // Track result via callback since executor is already running
  std::atomic<bool> goal_accepted{false};
  std::atomic<bool> goal_done{false};
  std::atomic<bool> goal_succeeded{false};

  auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
  
  send_goal_options.goal_response_callback = 
    [this, &goal_accepted](const GoalHandleFJT::SharedPtr& goal_handle) {
      if (!goal_handle) {
        RCLCPP_ERROR(get_logger(), "Goal was rejected");
        goal_accepted.store(false);
      } else {
        RCLCPP_INFO(get_logger(), "Goal accepted");
        goal_accepted.store(true);
      }
    };
  
  send_goal_options.result_callback = 
    [this, &goal_done, &goal_succeeded](const GoalHandleFJT::WrappedResult& result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(get_logger(), "Trajectory execution succeeded");
        goal_succeeded.store(true);
      } else {
        RCLCPP_WARN(get_logger(), "Trajectory execution failed with code: %d", 
                    static_cast<int>(result.code));
        goal_succeeded.store(false);
      }
      goal_done.store(true);
    };

  // Send goal
  action_client_->async_send_goal(goal, send_goal_options);

  // Wait for goal acceptance
  auto start = std::chrono::steady_clock::now();
  while (!goal_accepted.load() && !goal_done.load()) {
    auto elapsed = std::chrono::steady_clock::now() - start;
    if (std::chrono::duration<double>(elapsed).count() > 10.0) {
      RCLCPP_ERROR(get_logger(), "Timeout waiting for goal acceptance");
      return false;
    }
    std::this_thread::sleep_for(10ms);
  }

  if (!goal_accepted.load()) {
    return false;
  }

  // Calculate expected duration
  double expected_duration = 0.0;
  if (!trajectory.points.empty()) {
    const auto& last_point = trajectory.points.back();
    expected_duration = last_point.time_from_start.sec + 
                        last_point.time_from_start.nanosec * 1e-9;
  }
  double timeout_sec = expected_duration + 10.0;

  // Wait for result
  start = std::chrono::steady_clock::now();
  while (!goal_done.load()) {
    auto elapsed = std::chrono::steady_clock::now() - start;
    if (std::chrono::duration<double>(elapsed).count() > timeout_sec) {
      RCLCPP_ERROR(get_logger(), "Timeout waiting for result");
      return false;
    }
    std::this_thread::sleep_for(50ms);
  }

  return goal_succeeded.load();
}

void CartesianMotion::startRecording()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  recorded_times_.clear();
  recorded_positions_.clear();
  recorded_velocities_.clear();
  recorded_ee_positions_.clear();
  recorded_ee_velocities_.clear();

  for (const auto& name : ALL_JOINT_NAMES) {
    recorded_positions_[name] = {};
    recorded_velocities_[name] = {};
  }

  record_start_time_ = now();
  recording_ = true;
  RCLCPP_INFO(get_logger(), "Started recording");
}

void CartesianMotion::stopRecording()
{
  recording_ = false;
  RCLCPP_INFO(get_logger(), "Stopped recording. Recorded %zu samples", recorded_times_.size());
}

void CartesianMotion::saveToCSV(const std::string& filename)
{
  std::string filepath = RESULTS_DIR + "/" + filename;
  std::ofstream file(filepath);
  if (!file.is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open file: %s", filepath.c_str());
    return;
  }

  // Write header
  file << "time";
  for (const auto& name : ALL_JOINT_NAMES) {
    file << ",pos_" << name << ",vel_" << name;
  }
  file << "\n";

  // Write data
  for (size_t i = 0; i < recorded_times_.size(); ++i) {
    file << recorded_times_[i];
    for (const auto& name : ALL_JOINT_NAMES) {
      file << "," << recorded_positions_[name][i] 
           << "," << recorded_velocities_[name][i];
    }
    file << "\n";
  }

  file.close();
  RCLCPP_INFO(get_logger(), "Saved data to: %s", filepath.c_str());
}

bool CartesianMotion::executeDemo()
{
  RCLCPP_INFO(get_logger(), "Starting Cartesian motion demo");

  // Wait for services and joint state
  if (!waitForServices()) {
    return false;
  }
  if (!waitForJointState()) {
    return false;
  }

  RCLCPP_INFO(get_logger(), "Services and joint state ready");

  // Get current pose
  auto fk_result = computeFK(current_positions_);
  if (!fk_result) {
    RCLCPP_ERROR(get_logger(), "Failed to compute initial FK");
    return false;
  }

  auto [current_pos, current_orient] = *fk_result;
  RCLCPP_INFO(get_logger(), "Current EE position: (%.3f, %.3f, %.3f)",
              current_pos.x(), current_pos.y(), current_pos.z());

  // Start recording
  startRecording();

  // Demo pattern: square at current height
  double square_size = 0.15;
  std::vector<Eigen::Vector3d> corners = {
    current_pos,
    current_pos + Eigen::Vector3d(square_size, 0, 0),
    current_pos + Eigen::Vector3d(square_size, square_size, 0),
    current_pos + Eigen::Vector3d(0, square_size, 0),
    current_pos
  };

  Eigen::VectorXd current_joints = current_positions_;

  // Execute square pattern
  for (size_t i = 1; i < corners.size(); ++i) {
    RCLCPP_INFO(get_logger(), "Moving to corner %zu/%zu", i, corners.size() - 1);

    auto traj = generateCartesianTrajectory(
      corners[i - 1], corners[i], current_orient, current_joints);

    if (!traj || !executeTrajectory(*traj)) {
      RCLCPP_ERROR(get_logger(), "Failed to execute trajectory");
      stopRecording();
      return false;
    }

    current_joints = current_positions_;
    std::this_thread::sleep_for(200ms);
  }

  // Lift motion
  RCLCPP_INFO(get_logger(), "Executing lift motion");
  double lift_start = current_lift_position_;
  double lift_end = lift_start + 0.2;

  auto lift_traj = generateLiftTrajectory(lift_start, lift_end, current_joints);
  if (!executeTrajectory(lift_traj)) {
    RCLCPP_ERROR(get_logger(), "Failed to execute lift trajectory");
    stopRecording();
    return false;
  }

  // Stop recording and save
  stopRecording();

  auto now_time = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now_time);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&time_t_now), "%Y%m%d_%H%M%S");
  std::string csv_filename = "cartesian_motion_cpp_" + ss.str() + ".csv";
  
  saveToCSV(csv_filename);

  RCLCPP_INFO(get_logger(), "Cartesian motion demo completed successfully");
  return true;
}

double CartesianMotion::now() const
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace puma560_cpp

// Main function
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<puma560_cpp::CartesianMotion>();

  // Use multi-threaded executor
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  std::thread spin_thread([&executor]() {
    executor.spin();
  });

  std::this_thread::sleep_for(std::chrono::seconds(2));

  bool success = node->executeDemo();

  rclcpp::shutdown();
  spin_thread.join();

  return success ? 0 : 1;
}

