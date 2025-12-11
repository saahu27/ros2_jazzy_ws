/**
 * @file compliance_control.cpp
 * @brief Implementation of ComplianceControl node
 * 
 * Position-Based Admittance Control for Constant Force Regulation
 * 
 * Based on the direct position-compliance formulation:
 *     x_cmd = x_eq + Kf * (F_d - F_meas)
 * 
 * Where:
 *     x_eq   = equilibrium position (found during contact establishment)
 *     Kf     = compliance gain (m/N)
 *     F_d    = desired contact force
 *     F_meas = measured contact force
 */

#include "puma560_cpp/compliance_control.hpp"
#include "puma560_cpp/synchronized_profiles.hpp"

#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <future>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace puma560_cpp
{

// Static member definitions - matches Python exactly
const std::vector<std::string> ComplianceControl::ALL_JOINT_NAMES = {
  "lift_joint", "j1", "j2", "j3", "j4", "j5", "j6"
};
const std::string ComplianceControl::ACTION_NAME = "/arm_controller/follow_joint_trajectory";
const std::string ComplianceControl::RESULTS_DIR = "/root/ros2_ws/src/puma560_ros2_moveit/results";

// Pre-contact joint configuration (matches Python exactly)
const std::map<std::string, double> ComplianceControl::PRE_CONTACT_JOINTS = {
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

  // Declare parameters (matching Python defaults)
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

  // Create MutuallyExclusive callback group for action client
  action_callback_group_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  // Create Reentrant callback group for subscribers
  sub_callback_group_ = create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);

  // Create joint state subscriber
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = sub_callback_group_;
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    std::bind(&ComplianceControl::jointStateCallback, this, _1),
    sub_options);

  // Create FT sensor subscriber - CORRECT TOPIC (matching Python: /ft_sensor)
  ft_sensor_sub_ = create_subscription<geometry_msgs::msg::Wrench>(
    "/ft_sensor", 10,
    std::bind(&ComplianceControl::ftSensorCallback, this, _1),
    sub_options);

  // Create service clients
  ik_client_ = create_client<GetPositionIK>("/compute_ik");
  fk_client_ = create_client<GetPositionFK>("/compute_fk");

  // Create action client with MutuallyExclusive callback group
  action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
    this, ACTION_NAME, action_callback_group_);

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

void ComplianceControl::logConfig()
{
  // Calculate filter time constant for reference
  double tau = (filter_alpha_ > 0) ? 1.0 / (filter_alpha_ * control_hz_) : INFINITY;
  
  RCLCPP_INFO(get_logger(), "============================================================");
  RCLCPP_INFO(get_logger(), "POSITION-BASED ADMITTANCE CONTROL");
  RCLCPP_INFO(get_logger(), "============================================================");
  RCLCPP_INFO(get_logger(), "  Control law: x_cmd = x_eq + Kf*(F_d - F)");
  RCLCPP_INFO(get_logger(), "  F_desired = %.0f N", f_desired_);
  RCLCPP_INFO(get_logger(), "  Kf = %.6f m/N = %.4f mm/N", kf_compliance_, kf_compliance_ * 1000);
  RCLCPP_INFO(get_logger(), "  Filter alpha = %.3f (tau ≈ %.2fs at %.0fHz)", 
              filter_alpha_, tau, control_hz_);
  RCLCPP_INFO(get_logger(), "  Deadbands: inner=+/-%.0fN, outer=%.0fN", 
              deadband_inner_, deadband_outer_);
  RCLCPP_INFO(get_logger(), "============================================================");
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
  // Store raw force (like Python: self.raw_force = msg.force.x)
  force_raw_.store(msg->force.x);
  
  // Apply low-pass filter (like Python)
  double alpha = filter_alpha_;
  force_filtered_ = alpha * msg->force.x + (1.0 - alpha) * force_filtered_;
}

double ComplianceControl::getContactForce()
{
  // Get calibrated contact force (like Python)
  return force_filtered_ - force_offset_;
}

void ComplianceControl::calibrateForce()
{
  // Calibrate force sensor in free space (like Python's calibrate_force())
  RCLCPP_INFO(get_logger(), "Calibrating force sensor...");
  std::this_thread::sleep_for(500ms);
  
  std::vector<double> readings;
  for (int i = 0; i < 50; ++i) {
    readings.push_back(force_filtered_);
    std::this_thread::sleep_for(40ms);
  }
  
  // Compute mean
  double sum = 0.0;
  for (double r : readings) {
    sum += r;
  }
  force_offset_ = sum / readings.size();
  
  RCLCPP_INFO(get_logger(), "  Offset = %.2f N", force_offset_);
}

bool ComplianceControl::waitForJointState(double timeout_sec)
{
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
  
  if (!action_client_->wait_for_action_server(std::chrono::duration<double>(timeout_sec))) {
    RCLCPP_ERROR(get_logger(), "Action server not available");
    return false;
  }
  
  RCLCPP_INFO(get_logger(), "Services available");
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
  
  // Set seed state - ALL 7 JOINTS (matching Python)
  request->ik_request.robot_state.joint_state.name = ALL_JOINT_NAMES;
  request->ik_request.robot_state.joint_state.position.resize(ALL_JOINT_NAMES.size());
  for (size_t i = 0; i < ALL_JOINT_NAMES.size(); ++i) {
    request->ik_request.robot_state.joint_state.position[i] = seed_state(i);
  }
  
  request->ik_request.timeout.sec = 0;
  request->ik_request.timeout.nanosec = 100000000;  // 0.1s

  auto future = ik_client_->async_send_request(request);
  if (future.wait_for(1s) != std::future_status::ready) {
    return std::nullopt;
  }

  auto response = future.get();
  if (response->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    return std::nullopt;
  }

  // Extract result in correct order
  Eigen::VectorXd result = Eigen::VectorXd::Zero(ALL_JOINT_NAMES.size());
  const auto& joint_state = response->solution.joint_state;
  for (size_t i = 0; i < ALL_JOINT_NAMES.size(); ++i) {
    auto it = std::find(joint_state.name.begin(), joint_state.name.end(), ALL_JOINT_NAMES[i]);
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
  
  // Set joint state - ALL 7 JOINTS (matching Python)
  request->robot_state.joint_state.name = ALL_JOINT_NAMES;
  request->robot_state.joint_state.position.resize(ALL_JOINT_NAMES.size());
  for (size_t i = 0; i < ALL_JOINT_NAMES.size(); ++i) {
    request->robot_state.joint_state.position[i] = joint_positions(i);
  }

  auto future = fk_client_->async_send_request(request);
  if (future.wait_for(2s) != std::future_status::ready) {
    return std::nullopt;
  }

  auto response = future.get();
  if (response->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS ||
      response->pose_stamped.empty()) {
    return std::nullopt;
  }

  const auto& pose = response->pose_stamped[0].pose;
  Eigen::Vector3d position(pose.position.x, pose.position.y, pose.position.z);
  Eigen::Quaterniond quat = fromMsg(pose.orientation);

  return std::make_pair(position, quat);
}

bool ComplianceControl::moveToPreContact()
{
  RCLCPP_INFO(get_logger(), "Moving to pre-contact position...");

  // Create trajectory to pre-contact position (like Python)
  Eigen::VectorXd start;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    start = current_positions_;
  }
  
  Eigen::VectorXd end(ALL_JOINT_NAMES.size());
  for (size_t i = 0; i < ALL_JOINT_NAMES.size(); ++i) {
    end(i) = PRE_CONTACT_JOINTS.at(ALL_JOINT_NAMES[i]);
  }

  SynchronizedProfiles profiles(ALL_JOINT_NAMES, start, end);
  double total_time = profiles.getTotalTime();
  double dt = 0.02;

  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.header.stamp = this->get_clock()->now();
  trajectory.header.frame_id = "";  // Empty for joint-space trajectory
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

  // Ensure final point is exactly at goal with zero velocity
  if (!trajectory.points.empty()) {
    auto& last_point = trajectory.points.back();
    for (size_t j = 0; j < ALL_JOINT_NAMES.size(); ++j) {
      last_point.positions[j] = end(j);
      last_point.velocities[j] = 0.0;
    }
    last_point.time_from_start.sec = static_cast<int32_t>(total_time);
    last_point.time_from_start.nanosec = static_cast<uint32_t>((total_time - last_point.time_from_start.sec) * 1e9);
  }

  // Execute trajectory with promise/future pattern
  auto goal = FollowJointTrajectory::Goal();
  goal.trajectory = trajectory;

  auto goal_response_promise = std::make_shared<std::promise<GoalHandleFJT::SharedPtr>>();
  auto goal_response_future = goal_response_promise->get_future();

  auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
  send_goal_options.goal_response_callback = 
    [goal_response_promise](const GoalHandleFJT::SharedPtr& goal_handle) {
      goal_response_promise->set_value(goal_handle);
    };

  action_client_->async_send_goal(goal, send_goal_options);

  if (goal_response_future.wait_for(10s) == std::future_status::timeout) {
    return false;
  }

  auto goal_handle = goal_response_future.get();
  if (!goal_handle) {
    return false;
  }

  // Wait for result
  auto result_future = action_client_->async_get_result(goal_handle);
  auto timeout = std::chrono::duration<double>(total_time + 10.0);
  if (result_future.wait_for(std::chrono::duration_cast<std::chrono::milliseconds>(timeout)) 
      == std::future_status::timeout) {
    return false;
  }

  auto wrapped_result = result_future.get();
  bool success = (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED);

  if (success) {
    std::this_thread::sleep_for(1s);
    RCLCPP_INFO(get_logger(), "At pre-contact position");
  }

  return success;
}

bool ComplianceControl::sendJointCommand(const std::map<std::string, double>& joints, double duration)
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.header.stamp = this->get_clock()->now();
  trajectory.header.frame_id = "";  // Empty for joint-space trajectory
  trajectory.joint_names = ALL_JOINT_NAMES;

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions.resize(ALL_JOINT_NAMES.size());
  point.velocities.resize(ALL_JOINT_NAMES.size(), 0.0);  // Zero velocity at target (final point)
  for (size_t i = 0; i < ALL_JOINT_NAMES.size(); ++i) {
    auto it = joints.find(ALL_JOINT_NAMES[i]);
    if (it != joints.end()) {
      point.positions[i] = it->second;
    }
  }
  point.time_from_start.sec = 0;
  point.time_from_start.nanosec = static_cast<uint32_t>(duration * 1e9);

  trajectory.points.push_back(point);

  auto goal = FollowJointTrajectory::Goal();
  goal.trajectory = trajectory;

  // Fire and forget (don't wait for result in control loop - like Python)
  action_client_->async_send_goal(goal);
  return true;
}

void ComplianceControl::runControl()
{
  // Get starting pose via FK (like Python)
  Eigen::VectorXd current_joints;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_joints = current_positions_;
  }
  
  auto fk_result = computeFK(current_joints);
  if (!fk_result) {
    RCLCPP_ERROR(get_logger(), "FK failed");
    return;
  }

  auto [current_pos, current_orient] = *fk_result;
  current_orientation_ = current_orient;
  x_actual_ = current_pos.x();
  y_fixed_ = current_pos.y();
  z_fixed_ = current_pos.z();

  RCLCPP_INFO(get_logger(), " ");
  RCLCPP_INFO(get_logger(), "Start X: %.1fmm, Wall: %.0fmm", x_actual_ * 1000, wall_x_ * 1000);
  RCLCPP_INFO(get_logger(), "Gap: %.1fmm", (wall_x_ - x_actual_) * 1000);
  RCLCPP_INFO(get_logger(), " ");

  double dt = 1.0 / control_hz_;
  double x_cmd = x_actual_;
  double x_equilibrium = 0.0;  // Will be set when contact is established
  
  // Position limits
  double x_min = x_actual_ - 0.01;
  double x_max = wall_x_ + 0.02;
  
  // Seed for IK
  Eigen::VectorXd seed = current_joints;
  
  double t_start = now();
  double last_log = -1.0;
  phase_ = "APPROACH";
  double correction = 0.0;

  while ((now() - t_start) < duration_ && rclcpp::ok()) {
    double t = now() - t_start;
    double loop_start = now();

    // Get actual position via FK
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_joints = current_positions_;
    }
    auto p = computeFK(current_joints);
    if (p) {
      x_actual_ = p->first.x();
    }

    // Get force
    double F = getContactForce();
    double force_error = f_desired_ - F;

    // ========================================
    // PHASE 1: APPROACH (until contact)
    // ========================================
    if (phase_ == "APPROACH") {
      if (F > contact_threshold_) {
        // Contact established! Record equilibrium position
        x_equilibrium = x_actual_;
        phase_ = "REGULATE";
        RCLCPP_INFO(get_logger(), " ");
        RCLCPP_INFO(get_logger(), "========================================");
        RCLCPP_INFO(get_logger(), "CONTACT! x_eq = %.2fmm", x_equilibrium * 1000);
        RCLCPP_INFO(get_logger(), "Starting force regulation...");
        RCLCPP_INFO(get_logger(), "========================================");
      } else {
        // Slowly approach wall
        x_cmd = x_actual_ + approach_vel_ * dt;
      }
    }
    // ========================================
    // PHASE 2: REGULATE (two-zone control with gain scheduling)
    // ========================================
    else if (phase_ == "REGULATE") {
      if (std::abs(force_error) < deadband_inner_) {
        // INNER ZONE: True steady state - do nothing
        correction = 0.0;
        x_cmd = x_actual_;
      } else if (force_error > deadband_outer_) {
        // LARGE ERROR (F < target - outer): Fast correction
        x_equilibrium += correction_fast_ * dt;
        correction = correction_fast_ * dt;
        x_cmd = x_equilibrium;
      } else if (force_error > deadband_inner_) {
        // OUTER ZONE: Slow correction
        x_equilibrium += correction_slow_ * dt;
        correction = correction_slow_ * dt;
        x_cmd = x_equilibrium;
      } else if (force_error < -deadband_high_) {
        // ABOVE TARGET (F > target + high): Retract
        correction = kf_compliance_ * force_error;
        x_cmd = x_equilibrium + correction;
      } else {
        // Between inner and high above target: Hold (conservative)
        correction = 0.0;
        x_cmd = x_actual_;
      }
    }

    // Clamp position
    x_cmd = std::clamp(x_cmd, x_min, x_max);

    // Send command via IK
    Eigen::Vector3d target_pos(x_cmd, y_fixed_, z_fixed_);
    auto ik_result = computeIK(target_pos, current_orientation_, seed);
    if (ik_result) {
      std::map<std::string, double> joints;
      for (size_t i = 0; i < ALL_JOINT_NAMES.size(); ++i) {
        joints[ALL_JOINT_NAMES[i]] = (*ik_result)(i);
      }
      sendJointCommand(joints, dt * 0.9);
      seed = *ik_result;
    }

    // Record data (matching Python)
    if (recording_) {
      recorded_times_.push_back(t);
      recorded_forces_raw_.push_back(force_raw_.load());
      recorded_forces_filtered_.push_back(force_filtered_);
      recorded_contact_force_.push_back(F);
      recorded_x_actual_.push_back(x_actual_);
      recorded_x_cmd_.push_back(x_cmd);
      recorded_x_eq_.push_back(x_equilibrium);
      recorded_force_error_.push_back(force_error);
    }

    // Log (like Python)
    if (t - last_log >= 1.0) {
      if (phase_ == "APPROACH") {
        RCLCPP_INFO(get_logger(), "t=%5.1fs [APPROACH] F=%5.1fN X=%.2fmm", 
                    t, F, x_actual_ * 1000);
      } else {
        RCLCPP_INFO(get_logger(), "t=%5.1fs [REGULATE] F=%6.1fN (err=%+5.1f) X=%.2fmm corr=%+.2fmm",
                    t, F, force_error, x_actual_ * 1000, correction * 1000);
      }
      last_log = t;
    }

    // Timing
    double elapsed = now() - loop_start;
    if (elapsed < dt) {
      std::this_thread::sleep_for(std::chrono::duration<double>(dt - elapsed));
    }
  }

  RCLCPP_INFO(get_logger(), " ");
  RCLCPP_INFO(get_logger(), "Control complete");
}

void ComplianceControl::startRecording()
{
  recorded_times_.clear();
  recorded_forces_raw_.clear();
  recorded_forces_filtered_.clear();
  recorded_contact_force_.clear();
  recorded_x_actual_.clear();
  recorded_x_cmd_.clear();
  recorded_x_eq_.clear();
  recorded_force_error_.clear();
  
  record_start_time_ = now();
  recording_ = true;
  RCLCPP_INFO(get_logger(), "Started recording");
}

void ComplianceControl::stopRecording()
{
  recording_ = false;
  RCLCPP_INFO(get_logger(), "Stopped recording. %zu samples", recorded_times_.size());
}

void ComplianceControl::saveToCSV(const std::string& filename)
{
  std::string filepath = RESULTS_DIR + "/" + filename;
  std::ofstream file(filepath);
  if (!file.is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open file: %s", filepath.c_str());
    return;
  }

  // Write header (matching Python data structure)
  file << "time,raw_force,filtered_force,contact_force,x_actual,x_cmd,x_eq,force_error\n";
  
  for (size_t i = 0; i < recorded_times_.size(); ++i) {
    file << std::fixed << std::setprecision(6)
         << recorded_times_[i] << ","
         << recorded_forces_raw_[i] << ","
         << recorded_forces_filtered_[i] << ","
         << recorded_contact_force_[i] << ","
         << recorded_x_actual_[i] << ","
         << recorded_x_cmd_[i] << ","
         << recorded_x_eq_[i] << ","
         << recorded_force_error_[i] << "\n";
  }

  file.close();
  RCLCPP_INFO(get_logger(), "Saved data to: %s", filepath.c_str());
}

bool ComplianceControl::executeDemo()
{
  RCLCPP_INFO(get_logger(), "============================================================");
  RCLCPP_INFO(get_logger(), " POSITION-BASED ADMITTANCE CONTROL");
  RCLCPP_INFO(get_logger(), " Maintain constant %.0fN force against wall", f_desired_);
  RCLCPP_INFO(get_logger(), "============================================================");

  logConfig();

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

  // Calibrate force sensor (like Python)
  calibrateForce();

  std::this_thread::sleep_for(500ms);

  // Start recording and run control
  startRecording();
  runControl();
  stopRecording();

  // Save results
  auto now_time = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now_time);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&time_t_now), "%Y%m%d_%H%M%S");
  std::string filename = "admittance_cpp_" + ss.str() + ".csv";
  
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
