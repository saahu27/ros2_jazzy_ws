/**
 * @file compliance_control.cpp
 * @brief Implementation of ComplianceControl node
 */

#include "puma560_cpp/compliance_control.hpp"
#include "puma560_cpp/trapezoidal_profile.hpp"
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
const std::vector<std::string> ComplianceControl::ARM_JOINT_NAMES = {
  "j1", "j2", "j3", "j4", "j5", "j6"
};
const std::vector<std::string> ComplianceControl::ALL_JOINT_NAMES = {
  "lift_joint", "j1", "j2", "j3", "j4", "j5", "j6"
};
const std::string ComplianceControl::ACTION_NAME = "/arm_controller/follow_joint_trajectory";
const std::string ComplianceControl::RESULTS_DIR = "/root/ros2_ws/src/puma560_ros2_moveit/results";

// Pre-contact joint configuration
const std::map<std::string, double> PRE_CONTACT_JOINTS = {
  {"lift_joint", 0.3456},
  {"j1", 0.0},
  {"j2", 0.4298},
  {"j3", 0.8268},
  {"j4", 0.0},
  {"j5", -0.3077},
  {"j6", 0.0}
};

ComplianceControl::ComplianceControl(const rclcpp::NodeOptions& options)
  : Node("compliance_control", options),
    current_positions_(Eigen::VectorXd::Zero(ALL_JOINT_NAMES.size()))
{
  RCLCPP_INFO(get_logger(), "Initializing ComplianceControl node");

  // Declare parameters
  f_desired_ = declare_parameter("f_desired", 100.0);
  kf_compliance_ = declare_parameter("kf_compliance", 0.00003);
  control_hz_ = declare_parameter("control_hz", 50.0);
  filter_alpha_ = declare_parameter("filter_alpha", 0.005);
  deadband_inner_ = declare_parameter("deadband_inner", 8.0);
  deadband_outer_ = declare_parameter("deadband_outer", 20.0);
  deadband_high_ = declare_parameter("deadband_high", 25.0);
  correction_fast_ = declare_parameter("correction_fast", 0.0001);
  correction_slow_ = declare_parameter("correction_slow", 0.00005);
  approach_vel_ = declare_parameter("approach_vel", 0.002);
  contact_threshold_ = declare_parameter("contact_threshold", 80.0);
  wall_x_ = declare_parameter("wall_x", 0.87);
  duration_ = declare_parameter("duration", 30.0);

  // Create callback group
  auto callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // Create joint state subscriber
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group;
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    std::bind(&ComplianceControl::jointStateCallback, this, _1),
    sub_options);

  // Create FT sensor subscriber
  ft_sensor_sub_ = create_subscription<geometry_msgs::msg::Wrench>(
    "/ft_sensor/wrench", 10,
    std::bind(&ComplianceControl::ftSensorCallback, this, _1),
    sub_options);

  // Create service clients
  ik_client_ = create_client<GetPositionIK>("/compute_ik");
  fk_client_ = create_client<GetPositionFK>("/compute_fk");

  // Create action client
  action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
    this, ACTION_NAME);

  // Ensure results directory exists
  std::filesystem::create_directories(RESULTS_DIR);

  RCLCPP_INFO(get_logger(), "ComplianceControl node initialized");
}

ComplianceControl::~ComplianceControl()
{
  control_active_ = false;
  if (control_timer_) {
    control_timer_->cancel();
  }
  RCLCPP_INFO(get_logger(), "ComplianceControl node shutting down");
}

void ComplianceControl::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

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

  size_t count = 0;
  for (const auto& name : ALL_JOINT_NAMES) {
    if (std::find(msg->name.begin(), msg->name.end(), name) != msg->name.end()) {
      count++;
    }
  }
  if (count == ALL_JOINT_NAMES.size()) {
    has_joint_state_ = true;
  }
}

void ComplianceControl::ftSensorCallback(const geometry_msgs::msg::Wrench::SharedPtr msg)
{
  // Store raw force (we use X component for contact with wall)
  force_raw_.store(-msg->force.x);  // Negative because force is reaction
}

void ComplianceControl::controlLoop()
{
  if (!control_active_) {
    return;
  }

  double dt = 1.0 / control_hz_;
  double force_raw = force_raw_.load();
  
  // Apply low-pass filter to force
  force_filtered_ = filter_alpha_ * force_raw + (1.0 - filter_alpha_) * force_filtered_;
  
  // Calculate force error (positive = need more force)
  double force_error = f_desired_ - force_filtered_;
  
  // Two-zone deadband control logic
  double correction = 0.0;
  double x_cmd = x_actual_;
  
  if (std::abs(force_error) < deadband_inner_) {
    // INNER ZONE: True steady state - do nothing
    correction = 0.0;
    x_cmd = x_actual_;
  } else if (force_error > deadband_outer_) {
    // LARGE ERROR (F < target - outer): Fast correction
    x_equilibrium_ += correction_fast_ * dt;
    correction = correction_fast_ * dt;
    x_cmd = x_equilibrium_;
  } else if (force_error > deadband_inner_) {
    // OUTER ZONE: Slow correction
    x_equilibrium_ += correction_slow_ * dt;
    correction = correction_slow_ * dt;
    x_cmd = x_equilibrium_;
  } else if (force_error < -deadband_high_) {
    // ABOVE TARGET (F > target + high): Retract
    correction = kf_compliance_ * force_error;
    x_cmd = x_equilibrium_ + correction;
  }
  
  // Record data
  if (recording_) {
    double t = now() - record_start_time_;
    recorded_times_.push_back(t);
    recorded_forces_raw_.push_back(force_raw);
    recorded_forces_filtered_.push_back(force_filtered_);
    recorded_x_positions_.push_back(x_actual_);
    recorded_x_commands_.push_back(x_cmd);
  }
  
  // Send command if changed
  if (std::abs(x_cmd - x_actual_) > 1e-6) {
    Eigen::Vector3d target_pos(x_cmd, 0.0, 0.0);  // Will be combined with current y, z
    sendPositionCommand(target_pos);
    x_actual_ = x_cmd;
  }
}

bool ComplianceControl::waitForJointState(double timeout_sec)
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

bool ComplianceControl::waitForServices(double timeout_sec)
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

std::optional<Eigen::VectorXd> ComplianceControl::computeIK(
  const Eigen::Vector3d& position,
  const Eigen::Quaterniond& orientation,
  const Eigen::VectorXd& seed_state)
{
  auto request = std::make_shared<GetPositionIK::Request>();
  
  request->ik_request.group_name = "arm";
  request->ik_request.avoid_collisions = false;
  
  request->ik_request.pose_stamped.header.frame_id = "world";
  request->ik_request.pose_stamped.pose.position.x = position.x();
  request->ik_request.pose_stamped.pose.position.y = position.y();
  request->ik_request.pose_stamped.pose.position.z = position.z();
  request->ik_request.pose_stamped.pose.orientation = toMsg(orientation);
  
  request->ik_request.robot_state.joint_state.name = ARM_JOINT_NAMES;
  request->ik_request.robot_state.joint_state.position.resize(ARM_JOINT_NAMES.size());
  for (size_t i = 0; i < ARM_JOINT_NAMES.size(); ++i) {
    request->ik_request.robot_state.joint_state.position[i] = seed_state(i + 1);
  }

  // Call service - executor is already spinning, use future.wait_for directly
  auto future = ik_client_->async_send_request(request);
  if (future.wait_for(5s) != std::future_status::ready) {
    return std::nullopt;
  }

  auto response = future.get();
  if (response->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    return std::nullopt;
  }

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

std::optional<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> ComplianceControl::computeFK(
  const Eigen::VectorXd& joint_positions)
{
  auto request = std::make_shared<GetPositionFK::Request>();
  
  request->header.frame_id = "world";
  request->fk_link_names = {"link7"};
  
  request->robot_state.joint_state.name = ARM_JOINT_NAMES;
  request->robot_state.joint_state.position.resize(ARM_JOINT_NAMES.size());
  for (size_t i = 0; i < ARM_JOINT_NAMES.size(); ++i) {
    request->robot_state.joint_state.position[i] = joint_positions(i + 1);
  }

  // Call service - executor is already spinning, use future.wait_for directly
  auto future = fk_client_->async_send_request(request);
  if (future.wait_for(5s) != std::future_status::ready) {
    return std::nullopt;
  }

  auto response = future.get();
  if (response->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS ||
      response->pose_stamped.empty()) {
    return std::nullopt;
  }

  const auto& pose = response->pose_stamped[0].pose;
  Eigen::Vector3d position(pose.position.x, pose.position.y, pose.position.z);
  Eigen::Quaterniond orientation = fromMsg(pose.orientation);

  return std::make_pair(position, orientation);
}

bool ComplianceControl::moveToPreContact()
{
  RCLCPP_INFO(get_logger(), "Moving to pre-contact position");

  if (!action_client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(get_logger(), "Action server not available");
    return false;
  }

  // Create trajectory to pre-contact position
  Eigen::VectorXd start = current_positions_;
  Eigen::VectorXd end(ALL_JOINT_NAMES.size());
  for (size_t i = 0; i < ALL_JOINT_NAMES.size(); ++i) {
    end(i) = PRE_CONTACT_JOINTS.at(ALL_JOINT_NAMES[i]);
  }

  SynchronizedProfiles profiles(ALL_JOINT_NAMES, start, end);
  double total_time = profiles.getTotalTime();
  double dt = 0.02;

  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = ALL_JOINT_NAMES;

  int n_points = static_cast<int>(std::ceil(total_time / dt)) + 1;
  for (int i = 0; i < n_points; ++i) {
    double t = std::min(i * dt, total_time);
    auto [pos, vel, acc] = profiles.evaluate(t);

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.resize(ALL_JOINT_NAMES.size());
    point.velocities.resize(ALL_JOINT_NAMES.size());

    for (size_t j = 0; j < ALL_JOINT_NAMES.size(); ++j) {
      point.positions[j] = pos(j);
      point.velocities[j] = vel(j);
    }

    point.time_from_start.sec = static_cast<int32_t>(t);
    point.time_from_start.nanosec = static_cast<uint32_t>((t - point.time_from_start.sec) * 1e9);

    trajectory.points.push_back(point);
  }

  // Execute trajectory with callback-based waiting
  auto goal = FollowJointTrajectory::Goal();
  goal.trajectory = trajectory;

  std::atomic<bool> goal_accepted{false};
  std::atomic<bool> goal_done{false};
  std::atomic<bool> goal_succeeded{false};

  auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
  send_goal_options.goal_response_callback = 
    [&goal_accepted](const GoalHandleFJT::SharedPtr& goal_handle) {
      goal_accepted.store(goal_handle != nullptr);
    };
  send_goal_options.result_callback = 
    [&goal_done, &goal_succeeded](const GoalHandleFJT::WrappedResult& result) {
      goal_succeeded.store(result.code == rclcpp_action::ResultCode::SUCCEEDED);
      goal_done.store(true);
    };

  action_client_->async_send_goal(goal, send_goal_options);

  // Wait for goal acceptance
  auto wait_start = std::chrono::steady_clock::now();
  while (!goal_accepted.load() && !goal_done.load()) {
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - wait_start).count() > 10.0) {
      return false;
    }
    std::this_thread::sleep_for(10ms);
  }

  if (!goal_accepted.load()) {
    return false;
  }

  // Wait for result
  double timeout_sec = total_time + 10.0;
  wait_start = std::chrono::steady_clock::now();
  while (!goal_done.load()) {
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - wait_start).count() > timeout_sec) {
      return false;
    }
    std::this_thread::sleep_for(50ms);
  }

  return goal_succeeded.load();
}

bool ComplianceControl::approachWall()
{
  RCLCPP_INFO(get_logger(), "Approaching wall...");

  // Get current position via FK
  auto fk_result = computeFK(current_positions_);
  if (!fk_result) {
    RCLCPP_ERROR(get_logger(), "FK failed");
    return false;
  }

  auto [current_pos, current_orient] = *fk_result;
  current_orientation_ = current_orient;
  x_actual_ = current_pos.x();

  RCLCPP_INFO(get_logger(), "Current X position: %.4f, wall at: %.4f", x_actual_, wall_x_);

  // Slowly approach until contact
  double approach_step = approach_vel_ / control_hz_;
  while (x_actual_ < wall_x_ && force_filtered_ < contact_threshold_) {
    x_actual_ += approach_step;
    
    Eigen::Vector3d target(x_actual_, current_pos.y(), current_pos.z());
    sendPositionCommand(target);
    
    // Update force filter
    double force_raw = force_raw_.load();
    force_filtered_ = filter_alpha_ * force_raw + (1.0 - filter_alpha_) * force_filtered_;
    
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Approaching: x=%.4f, F=%.1fN", x_actual_, force_filtered_);
    
    // Executor is already spinning, just sleep
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000.0 / control_hz_)));
  }

  if (force_filtered_ >= contact_threshold_) {
    RCLCPP_INFO(get_logger(), "Contact established at x=%.4f with F=%.1fN", 
                x_actual_, force_filtered_);
    x_equilibrium_ = x_actual_;
    in_contact_ = true;
    return true;
  }

  RCLCPP_WARN(get_logger(), "Reached wall position without sufficient contact force");
  return false;
}

bool ComplianceControl::sendPositionCommand(const Eigen::Vector3d& position)
{
  // Get current full position
  auto fk_result = computeFK(current_positions_);
  if (!fk_result) {
    return false;
  }

  auto [current_pos, _] = *fk_result;
  
  // Create target with only X changed
  Eigen::Vector3d target(position.x(), current_pos.y(), current_pos.z());
  
  // Compute IK
  auto ik_result = computeIK(target, current_orientation_, current_positions_);
  if (!ik_result) {
    return false;
  }

  // Send as single-point trajectory
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = ALL_JOINT_NAMES;

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions.resize(ALL_JOINT_NAMES.size());
  point.positions[0] = current_lift_position_;  // Keep lift constant
  for (size_t i = 0; i < ARM_JOINT_NAMES.size(); ++i) {
    point.positions[i + 1] = (*ik_result)(i);
  }
  point.time_from_start.sec = 0;
  point.time_from_start.nanosec = static_cast<uint32_t>(1e9 / control_hz_);

  trajectory.points.push_back(point);

  auto goal = FollowJointTrajectory::Goal();
  goal.trajectory = trajectory;

  // Fire and forget (don't wait for result in control loop)
  action_client_->async_send_goal(goal);
  return true;
}

void ComplianceControl::startRecording()
{
  recorded_times_.clear();
  recorded_forces_raw_.clear();
  recorded_forces_filtered_.clear();
  recorded_x_positions_.clear();
  recorded_x_commands_.clear();
  
  record_start_time_ = now();
  recording_ = true;
  RCLCPP_INFO(get_logger(), "Started recording");
}

void ComplianceControl::stopRecording()
{
  recording_ = false;
  RCLCPP_INFO(get_logger(), "Stopped recording. Recorded %zu samples", recorded_times_.size());
}

void ComplianceControl::saveToCSV(const std::string& filename)
{
  std::string filepath = RESULTS_DIR + "/" + filename;
  std::ofstream file(filepath);
  if (!file.is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open file: %s", filepath.c_str());
    return;
  }

  file << "time,force_raw,force_filtered,x_position,x_command\n";
  for (size_t i = 0; i < recorded_times_.size(); ++i) {
    file << recorded_times_[i] << ","
         << recorded_forces_raw_[i] << ","
         << recorded_forces_filtered_[i] << ","
         << recorded_x_positions_[i] << ","
         << recorded_x_commands_[i] << "\n";
  }

  file.close();
  RCLCPP_INFO(get_logger(), "Saved data to: %s", filepath.c_str());
}

bool ComplianceControl::executeDemo()
{
  RCLCPP_INFO(get_logger(), "Starting compliance control demo");

  // Wait for services and joint state
  if (!waitForServices()) {
    return false;
  }
  if (!waitForJointState()) {
    return false;
  }

  // Move to pre-contact position
  if (!moveToPreContact()) {
    RCLCPP_ERROR(get_logger(), "Failed to move to pre-contact position");
    return false;
  }

  std::this_thread::sleep_for(1s);

  // Approach wall
  if (!approachWall()) {
    RCLCPP_ERROR(get_logger(), "Failed to establish contact");
    return false;
  }

  // Start recording and control
  startRecording();
  control_active_ = true;

  // Create control timer
  auto timer_period = std::chrono::duration<double>(1.0 / control_hz_);
  control_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
    std::bind(&ComplianceControl::controlLoop, this));

  RCLCPP_INFO(get_logger(), "Compliance control active for %.1f seconds", duration_);

  // Run for specified duration - executor is already spinning in another thread
  auto start_time = std::chrono::steady_clock::now();
  while (rclcpp::ok()) {
    auto elapsed = std::chrono::steady_clock::now() - start_time;
    if (std::chrono::duration<double>(elapsed).count() >= duration_) {
      break;
    }
    std::this_thread::sleep_for(50ms);
  }

  // Stop control
  control_active_ = false;
  control_timer_->cancel();
  stopRecording();

  // Save results
  auto now_time = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now_time);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&time_t_now), "%Y%m%d_%H%M%S");
  std::string filename = "compliance_control_cpp_" + ss.str() + ".csv";
  
  saveToCSV(filename);

  RCLCPP_INFO(get_logger(), "Compliance control demo completed");
  return true;
}

double ComplianceControl::now() const
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace puma560_cpp

// Main function
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<puma560_cpp::ComplianceControl>();

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

