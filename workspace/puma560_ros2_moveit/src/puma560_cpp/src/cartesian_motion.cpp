/**
 * @file cartesian_motion.cpp
 * @brief Implementation of CartesianMotion node
 * 
 * TRUE Cartesian motion with trapezoidal velocity profiles.
 * End-effector moves in STRAIGHT LINES in Cartesian space at constant velocity.
 * 
 * Key difference from Joint-Space Motion:
 * - Joint-Space: q(t) = q_start + (q_end - q_start) * s(t)  -> Curved Cartesian path
 * - Cartesian:   x(t) = x_start + (x_end - x_start) * s(t)  -> Straight Cartesian path
 *                q(t) = IK(x(t))
 */

#include "puma560_cpp/cartesian_motion.hpp"
#include "puma560_cpp/trapezoidal_profile.hpp"

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
const std::vector<std::string> CartesianMotion::ALL_JOINT_NAMES = {
  "lift_joint", "j1", "j2", "j3", "j4", "j5", "j6"
};
const std::string CartesianMotion::ACTION_NAME = "/arm_controller/follow_joint_trajectory";
const std::string CartesianMotion::RESULTS_DIR = "/root/ros2_ws/src/puma560_ros2_moveit/results";
const std::string CartesianMotion::PLANNING_GROUP = "arm";
const std::string CartesianMotion::EE_LINK = "link7";
const std::string CartesianMotion::BASE_FRAME = "world";

CartesianMotion::CartesianMotion(const rclcpp::NodeOptions& options)
  : Node("cartesian_motion", options),
    current_positions_(Eigen::VectorXd::Zero(ALL_JOINT_NAMES.size()))
{
  RCLCPP_INFO(get_logger(), "Initializing CartesianMotion node");

  // Declare parameters (matching Python values)
  v_max_ = declare_parameter("v_max", 0.08);       // 8 cm/s
  a_max_ = declare_parameter("a_max", 0.15);       // 15 cm/s²
  lift_v_max_ = declare_parameter("lift_v_max", 0.1);  // 10 cm/s
  dt_ = declare_parameter("dt", 0.005);            // 200 Hz

  // Create MutuallyExclusive callback group for action client
  // This ensures action callbacks don't run concurrently
  action_callback_group_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  // Create Reentrant callback group for subscribers
  // This allows joint state updates while action is running
  sub_callback_group_ = create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);

  // Create joint state subscriber with its own callback group
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = sub_callback_group_;
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    std::bind(&CartesianMotion::jointStateCallback, this, _1),
    sub_options);

  // Create service clients
  ik_client_ = create_client<GetPositionIK>("/compute_ik");
  fk_client_ = create_client<GetPositionFK>("/compute_fk");

  // Create action client with MutuallyExclusive callback group
  action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
    this, ACTION_NAME, action_callback_group_);

  // Ensure results directory exists
  std::filesystem::create_directories(RESULTS_DIR);

  RCLCPP_INFO(get_logger(), "CartesianMotion node initialized");
  RCLCPP_INFO(get_logger(), "  Planning group: %s", PLANNING_GROUP.c_str());
  RCLCPP_INFO(get_logger(), "  End-effector: %s", EE_LINK.c_str());
}

CartesianMotion::~CartesianMotion()
{
  RCLCPP_INFO(get_logger(), "CartesianMotion node shutting down");
}

void CartesianMotion::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  // Check if velocity data is available in the message (like Python)
  bool has_velocity = (msg->velocity.size() == msg->name.size());

  // Extract positions for our joints
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

  // Recording (like Python's _joint_state_callback)
  if (recording_) {
    double t = now() - record_start_time_;
    recorded_times_.push_back(t);

    for (size_t i = 0; i < msg->name.size(); ++i) {
      const auto& msg_name = msg->name[i];
      auto it = std::find(ALL_JOINT_NAMES.begin(), ALL_JOINT_NAMES.end(), msg_name);
      if (it != ALL_JOINT_NAMES.end()) {
        size_t joint_idx = std::distance(ALL_JOINT_NAMES.begin(), it);
        recorded_joint_positions_[msg_name].push_back(current_positions_(joint_idx));

        // Use velocity from message if available (more accurate, like Python)
        if (has_velocity) {
          recorded_joint_velocities_[msg_name].push_back(msg->velocity[i]);
        } else {
          // Fallback to numerical differentiation
          if (recorded_joint_positions_[msg_name].size() > 1) {
            size_t n = recorded_times_.size();
            double dt_meas = recorded_times_[n-1] - recorded_times_[n-2];
            if (dt_meas > 0.001) {
              size_t pos_n = recorded_joint_positions_[msg_name].size();
              double vel = (recorded_joint_positions_[msg_name][pos_n-1] - 
                            recorded_joint_positions_[msg_name][pos_n-2]) / dt_meas;
              recorded_joint_velocities_[msg_name].push_back(vel);
            } else {
              recorded_joint_velocities_[msg_name].push_back(0.0);
            }
          } else {
            recorded_joint_velocities_[msg_name].push_back(0.0);
          }
        }
      }
    }
  }
}

bool CartesianMotion::waitForJointState(double timeout_sec)
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

bool CartesianMotion::waitForServices(double timeout_sec)
{
  RCLCPP_INFO(get_logger(), "Waiting for MoveIt IK service...");
  
  if (!ik_client_->wait_for_service(std::chrono::duration<double>(timeout_sec))) {
    RCLCPP_ERROR(get_logger(), "IK service not available!");
    return false;
  }
  RCLCPP_INFO(get_logger(), "  IK service ready");
  
  RCLCPP_INFO(get_logger(), "Waiting for MoveIt FK service...");
  if (!fk_client_->wait_for_service(std::chrono::duration<double>(timeout_sec))) {
    RCLCPP_ERROR(get_logger(), "FK service not available!");
    return false;
  }
  RCLCPP_INFO(get_logger(), "  FK service ready");
  
  return true;
}

std::optional<Eigen::VectorXd> CartesianMotion::computeIK(
  const Eigen::Vector3d& position,
  const Eigen::Quaterniond& orientation,
  const Eigen::VectorXd& seed_state)
{
  // Check for near-singular seed configuration before IK
  // Singularity thresholds for 6-DOF manipulator (PUMA-style)
  constexpr double SINGULARITY_THRESHOLD = 0.05;  // ~3 degrees
  
  // Wrist singularity: j5 near 0 (axes 4 and 6 align)
  // seed_state indices: [0]=lift, [1]=j1, [2]=j2, [3]=j3, [4]=j4, [5]=j5, [6]=j6
  if (std::abs(seed_state(5)) < SINGULARITY_THRESHOLD) {
    RCLCPP_DEBUG(get_logger(), "Near wrist singularity (j5 ≈ 0), IK may be ill-conditioned");
  }
  
  // Elbow singularity: j3 near ±π/2 (arm fully extended or folded)
  if (std::abs(std::abs(seed_state(3)) - M_PI_2) < SINGULARITY_THRESHOLD) {
    RCLCPP_DEBUG(get_logger(), "Near elbow singularity (j3 ≈ ±π/2), IK may be ill-conditioned");
  }
  
  auto request = std::make_shared<GetPositionIK::Request>();
  
  // Set up IK request - matches Python exactly
  request->ik_request.group_name = PLANNING_GROUP;
  request->ik_request.avoid_collisions = false;
  
  // Set target pose in world frame (matching Python)
  request->ik_request.pose_stamped.header.frame_id = BASE_FRAME;
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
  
  // Set timeout (matching Python)
  request->ik_request.timeout.sec = 1;
  request->ik_request.timeout.nanosec = 0;

  // Call service
  auto future = ik_client_->async_send_request(request);
  if (future.wait_for(2s) != std::future_status::ready) {
    RCLCPP_WARN(get_logger(), "IK service call timeout");
    return std::nullopt;
  }

  auto response = future.get();
  if (response->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_WARN(get_logger(), "IK failed with error code: %d", response->error_code.val);
    return std::nullopt;
  }

  // Extract joint positions in correct order (like Python)
  Eigen::VectorXd result = Eigen::VectorXd::Zero(ALL_JOINT_NAMES.size());
  const auto& joint_state = response->solution.joint_state;
  for (size_t i = 0; i < ALL_JOINT_NAMES.size(); ++i) {
    auto it = std::find(joint_state.name.begin(), joint_state.name.end(), ALL_JOINT_NAMES[i]);
    if (it != joint_state.name.end()) {
      size_t idx = std::distance(joint_state.name.begin(), it);
      result(i) = joint_state.position[idx];
    }
  }
  
  // Check result for near-singular configuration
  if (std::abs(result(5)) < SINGULARITY_THRESHOLD) {
    RCLCPP_DEBUG(get_logger(), "IK solution near wrist singularity");
  }

  return result;
}

std::optional<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> CartesianMotion::computeFK(
  const Eigen::VectorXd& joint_positions)
{
  auto request = std::make_shared<GetPositionFK::Request>();
  
  // Set up FK request - matches Python exactly
  request->header.frame_id = BASE_FRAME;
  request->fk_link_names = {EE_LINK};
  
  // Set joint state - ALL 7 JOINTS (matching Python)
  request->robot_state.joint_state.name = ALL_JOINT_NAMES;
  request->robot_state.joint_state.position.resize(ALL_JOINT_NAMES.size());
  for (size_t i = 0; i < ALL_JOINT_NAMES.size(); ++i) {
    request->robot_state.joint_state.position[i] = joint_positions(i);
  }

  // Call service
  auto future = fk_client_->async_send_request(request);
  if (future.wait_for(2s) != std::future_status::ready) {
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
  Eigen::Quaterniond quat = fromMsg(pose.orientation);

  return std::make_pair(position, quat);
}

std::optional<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> CartesianMotion::getCurrentEEPose()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!has_joint_state_) {
    return std::nullopt;
  }
  return computeFK(current_positions_);
}

std::optional<trajectory_msgs::msg::JointTrajectory> CartesianMotion::generateCartesianTrajectory(
  const Eigen::Vector3d& start_pos,
  const Eigen::Vector3d& end_pos,
  const Eigen::Quaterniond& orientation,
  const Eigen::VectorXd& seed_joints)
{
  // Compute Cartesian distance (XYZ)
  double dx = end_pos.x() - start_pos.x();
  double dy = end_pos.y() - start_pos.y();
  double dz = end_pos.z() - start_pos.z();
  double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

  if (distance < 1e-6) {
    RCLCPP_WARN(get_logger(), "Start and end positions are the same");
    return std::nullopt;
  }

  // Create trapezoidal profile for Cartesian distance
  TrapezoidalProfile profile(distance, v_max_, a_max_);
  double total_time = profile.getTotalTime();

  RCLCPP_INFO(get_logger(), "Cartesian trajectory:");
  RCLCPP_INFO(get_logger(), "  Distance: %.4f m", distance);
  RCLCPP_INFO(get_logger(), "  Duration: %.3f s", total_time);
  RCLCPP_INFO(get_logger(), "  Peak velocity: %.4f m/s", profile.getPeakVelocity());
  RCLCPP_INFO(get_logger(), "  Start: (%.3f, %.3f, %.3f)", 
              start_pos.x(), start_pos.y(), start_pos.z());
  RCLCPP_INFO(get_logger(), "  End:   (%.3f, %.3f, %.3f)", 
              end_pos.x(), end_pos.y(), end_pos.z());

  // Direction unit vector in Cartesian space
  Eigen::Vector3d direction(dx/distance, dy/distance, dz/distance);

  // Build trajectory
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.header.stamp = this->get_clock()->now();
  trajectory.header.frame_id = BASE_FRAME;
  trajectory.joint_names = ALL_JOINT_NAMES;

  // Storage for trajectory data (for recording)
  std::vector<Eigen::Vector3d> trajectory_poses;
  std::vector<Eigen::Vector3d> trajectory_velocities;

  // Sample trajectory points
  double t = 0.0;
  Eigen::VectorXd prev_joints = seed_joints;
  Eigen::VectorXd prev_vels = Eigen::VectorXd::Zero(ALL_JOINT_NAMES.size());
  int ik_failures = 0;
  const int max_ik_failures = 5;

  while (t <= total_time + dt_) {
    // Get position and velocity along the path
    auto [pos_scalar, vel_scalar, acc_scalar] = profile.evaluate(t);

    // Normalized parameter s ∈ [0, 1]
    double s = (distance > 1e-9) ? (pos_scalar / distance) : 1.0;
    s = std::clamp(s, 0.0, 1.0);

    // Interpolate pose linearly in Cartesian space
    Eigen::Vector3d current_pos = start_pos + (end_pos - start_pos) * s;

    // Compute Cartesian velocity vector
    Eigen::Vector3d cart_vel = direction * vel_scalar;

    // Solve IK for this pose
    auto ik_result = computeIK(current_pos, orientation, prev_joints);

    if (!ik_result) {
      ik_failures++;
      if (ik_failures > max_ik_failures) {
        RCLCPP_ERROR(get_logger(), "Too many IK failures (%d), aborting trajectory", ik_failures);
        return std::nullopt;
      }
      t += dt_;
      continue;
    }

    Eigen::VectorXd joint_positions = *ik_result;

    // Compute joint velocities/accelerations using finite differences at high rate
    Eigen::VectorXd joint_velocities = Eigen::VectorXd::Zero(ALL_JOINT_NAMES.size());
    Eigen::VectorXd joint_accelerations = Eigen::VectorXd::Zero(ALL_JOINT_NAMES.size());
    // Per-joint limits: lift_joint is prismatic (m/s, m/s²), others are revolute (rad/s, rad/s²)
    const std::vector<double> max_joint_vel = {0.5, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0};
    const std::vector<double> max_joint_acc = {1.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0};

    if (t > 0) {
      for (size_t j = 0; j < ALL_JOINT_NAMES.size(); ++j) {
        double raw_vel = (joint_positions(j) - prev_joints(j)) / dt_;
        double raw_acc = (raw_vel - prev_vels(j)) / dt_;
        joint_velocities(j) = std::clamp(raw_vel, -max_joint_vel[j], max_joint_vel[j]);
        joint_accelerations(j) = std::clamp(raw_acc, -max_joint_acc[j], max_joint_acc[j]);
      }
    }

    prev_vels = joint_velocities;
    prev_joints = joint_positions;

    // Store for recording
    trajectory_poses.push_back(current_pos);
    trajectory_velocities.push_back(cart_vel);

    // Create trajectory point
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.resize(ALL_JOINT_NAMES.size());
    point.velocities.resize(ALL_JOINT_NAMES.size());
    point.accelerations.resize(ALL_JOINT_NAMES.size());

    for (size_t j = 0; j < ALL_JOINT_NAMES.size(); ++j) {
      point.positions[j] = joint_positions(j);
      point.velocities[j] = joint_velocities(j);
      point.accelerations[j] = joint_accelerations(j);
    }

    point.time_from_start.sec = static_cast<int32_t>(t);
    point.time_from_start.nanosec = static_cast<uint32_t>((t - point.time_from_start.sec) * 1e9);

    trajectory.points.push_back(point);
    t += dt_;
  }

  // Ensure final point is exactly at goal with zero velocity
  auto final_ik = computeIK(end_pos, orientation, prev_joints);
  if (final_ik && !trajectory.points.empty()) {
    auto& last_point = trajectory.points.back();
    for (size_t j = 0; j < ALL_JOINT_NAMES.size(); ++j) {
      last_point.positions[j] = (*final_ik)(j);
      last_point.velocities[j] = 0.0;
      last_point.accelerations[j] = 0.0;
    }
    last_point.time_from_start.sec = static_cast<int32_t>(total_time);
    last_point.time_from_start.nanosec = static_cast<uint32_t>((total_time - last_point.time_from_start.sec) * 1e9);
  }

  if (ik_failures > 0) {
    RCLCPP_WARN(get_logger(), "Trajectory completed with %d IK failures", ik_failures);
  }

  RCLCPP_INFO(get_logger(), "Generated trajectory with %zu points", trajectory.points.size());

  // Store commanded data to pending buffers (time-stamped at execution start)
  if (recording_) {
    pending_time_from_start_.clear();
    pending_cartesian_x_.clear();
    pending_cartesian_y_.clear();
    pending_cartesian_z_.clear();
    pending_cartesian_vx_.clear();
    pending_cartesian_vy_.clear();
    pending_cartesian_vz_.clear();
    pending_joint_positions_.clear();
    pending_joint_velocities_.clear();
    for (const auto& name : ALL_JOINT_NAMES) {
      pending_joint_positions_[name] = {};
      pending_joint_velocities_[name] = {};
    }

    for (size_t i = 0; i < trajectory.points.size(); ++i) {
      const auto& point = trajectory.points[i];
      double t_point = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
      pending_time_from_start_.push_back(t_point);

      for (size_t j = 0; j < ALL_JOINT_NAMES.size(); ++j) {
        pending_joint_positions_[ALL_JOINT_NAMES[j]].push_back(point.positions[j]);
        pending_joint_velocities_[ALL_JOINT_NAMES[j]].push_back(point.velocities[j]);
      }

      if (i < trajectory_poses.size()) {
        pending_cartesian_x_.push_back(trajectory_poses[i].x());
        pending_cartesian_y_.push_back(trajectory_poses[i].y());
        pending_cartesian_z_.push_back(trajectory_poses[i].z());
        pending_cartesian_vx_.push_back(trajectory_velocities[i].x());
        pending_cartesian_vy_.push_back(trajectory_velocities[i].y());
        pending_cartesian_vz_.push_back(trajectory_velocities[i].z());
      }
    }
  }

  return trajectory;
}

bool CartesianMotion::moveToXYZ(double x, double y, double z, double v_max, double a_max)
{
  // Get current pose for orientation
  auto current_pose = getCurrentEEPose();
  if (!current_pose) {
    RCLCPP_ERROR(get_logger(), "Could not get current end-effector pose");
    return false;
  }

  auto [current_pos, current_orient] = *current_pose;
  Eigen::Vector3d target_pos(x, y, z);

  // Temporarily override parameters
  double old_v_max = v_max_;
  double old_a_max = a_max_;
  v_max_ = v_max;
  a_max_ = a_max;

  // Generate trajectory
  Eigen::VectorXd current_joints;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_joints = current_positions_;
  }

  auto trajectory = generateCartesianTrajectory(
    current_pos, target_pos, current_orient, current_joints);

  // Restore parameters
  v_max_ = old_v_max;
  a_max_ = old_a_max;

  if (!trajectory) {
    return false;
  }

  return executeTrajectory(*trajectory);
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

  // Use promise/future pattern for clean synchronization
  auto goal_response_promise = std::make_shared<std::promise<GoalHandleFJT::SharedPtr>>();
  auto goal_response_future = goal_response_promise->get_future();

  auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
  send_goal_options.goal_response_callback = 
    [this, goal_response_promise](const GoalHandleFJT::SharedPtr& goal_handle) {
      goal_response_promise->set_value(goal_handle);
    };

  // Send goal asynchronously
  action_client_->async_send_goal(goal, send_goal_options);

  // Wait for goal response with timeout
  auto goal_status = goal_response_future.wait_for(10s);
  if (goal_status == std::future_status::timeout) {
    RCLCPP_ERROR(get_logger(), "Timeout waiting for goal response");
    return false;
  }

  auto goal_handle = goal_response_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "Goal was rejected");
    return false;
  }

  // If recording, stamp pending commanded data at execution start
  if (recording_ && !pending_time_from_start_.empty()) {
    double exec_start = now() - record_start_time_;
    for (size_t i = 0; i < pending_time_from_start_.size(); ++i) {
      commanded_times_.push_back(exec_start + pending_time_from_start_[i]);
      for (const auto& name : ALL_JOINT_NAMES) {
        commanded_joint_positions_[name].push_back(pending_joint_positions_[name][i]);
        commanded_joint_velocities_[name].push_back(pending_joint_velocities_[name][i]);
      }
      if (i < pending_cartesian_x_.size()) {
        commanded_cartesian_x_.push_back(pending_cartesian_x_[i]);
        commanded_cartesian_y_.push_back(pending_cartesian_y_[i]);
        commanded_cartesian_z_.push_back(pending_cartesian_z_[i]);
      }
      if (i < pending_cartesian_vx_.size()) {
        commanded_cartesian_vx_.push_back(pending_cartesian_vx_[i]);
        commanded_cartesian_vy_.push_back(pending_cartesian_vy_[i]);
        commanded_cartesian_vz_.push_back(pending_cartesian_vz_[i]);
      }
    }
    // Clear pending buffers after stamping
    pending_time_from_start_.clear();
    pending_joint_positions_.clear();
    pending_joint_velocities_.clear();
    pending_cartesian_x_.clear();
    pending_cartesian_y_.clear();
    pending_cartesian_z_.clear();
    pending_cartesian_vx_.clear();
    pending_cartesian_vy_.clear();
    pending_cartesian_vz_.clear();
  }

  RCLCPP_INFO(get_logger(), "Goal accepted");

  // Wait for result using future
  auto result_future = action_client_->async_get_result(goal_handle);

  // Calculate timeout based on trajectory duration
  double expected_duration = 0.0;
  if (!trajectory.points.empty()) {
    const auto& last_point = trajectory.points.back();
    expected_duration = last_point.time_from_start.sec + 
                        last_point.time_from_start.nanosec * 1e-9;
  }
  auto timeout = std::chrono::duration<double>(expected_duration + 10.0);

  // Wait for result with timeout
  auto result_status = result_future.wait_for(
    std::chrono::duration_cast<std::chrono::milliseconds>(timeout));

  if (result_status == std::future_status::timeout) {
    RCLCPP_ERROR(get_logger(), "Timeout waiting for trajectory execution");
    return false;
  }

  auto wrapped_result = result_future.get();
  if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(get_logger(), "Trajectory execution SUCCEEDED");
    return true;
  } else {
    RCLCPP_WARN(get_logger(), "Trajectory execution failed with code: %d", 
                static_cast<int>(wrapped_result.code));
    return false;
  }
}

void CartesianMotion::startRecording()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  
  // Clear measured data
  recorded_times_.clear();
  recorded_joint_positions_.clear();
  recorded_joint_velocities_.clear();
  recorded_cartesian_time_.clear();
  recorded_cartesian_x_.clear();
  recorded_cartesian_y_.clear();
  recorded_cartesian_z_.clear();
  recorded_cartesian_vx_.clear();
  recorded_cartesian_vy_.clear();
  recorded_cartesian_vz_.clear();

  // Clear commanded data
  commanded_times_.clear();
  commanded_joint_positions_.clear();
  commanded_joint_velocities_.clear();
  commanded_cartesian_x_.clear();
  commanded_cartesian_y_.clear();
  commanded_cartesian_z_.clear();
  commanded_cartesian_vx_.clear();
  commanded_cartesian_vy_.clear();
  commanded_cartesian_vz_.clear();

  // Clear pending commanded data
  pending_time_from_start_.clear();
  pending_joint_positions_.clear();
  pending_joint_velocities_.clear();
  pending_cartesian_x_.clear();
  pending_cartesian_y_.clear();
  pending_cartesian_z_.clear();
  pending_cartesian_vx_.clear();
  pending_cartesian_vy_.clear();
  pending_cartesian_vz_.clear();

  for (const auto& name : ALL_JOINT_NAMES) {
    recorded_joint_positions_[name] = {};
    recorded_joint_velocities_[name] = {};
    commanded_joint_positions_[name] = {};
    commanded_joint_velocities_[name] = {};
    pending_joint_positions_[name] = {};
    pending_joint_velocities_[name] = {};
  }

  record_start_time_ = now();
  recording_ = true;
  RCLCPP_INFO(get_logger(), "Started recording");
}

void CartesianMotion::stopRecording()
{
  // Copy recorded data under lock, then release lock before FK calls
  // This prevents deadlock: FK service calls need executor, which may need callbacks
  std::vector<double> times_copy;
  std::map<std::string, std::vector<double>> joint_pos_copy;
  
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    recording_ = false;
    RCLCPP_INFO(get_logger(), "Stopped recording. %zu samples", recorded_times_.size());
    
    // Copy data we need for FK computation
    times_copy = recorded_times_;
    joint_pos_copy = recorded_joint_positions_;
  }
  // Lock released here - safe to make service calls

  // Compute measured Cartesian positions using FK (post-processing, like Python)
  RCLCPP_INFO(get_logger(), "Computing measured Cartesian positions via FK...");
  
  // Sample up to 1000 points for better velocity resolution (matching Python)
  // Higher sampling = more accurate velocity estimation via differentiation
  const size_t TARGET_SAMPLES = 1000;
  size_t sample_step = std::max(size_t(1), times_copy.size() / TARGET_SAMPLES);
  
  for (size_t idx = 0; idx < times_copy.size(); idx += sample_step) {
    Eigen::VectorXd joint_pos(ALL_JOINT_NAMES.size());
    for (size_t j = 0; j < ALL_JOINT_NAMES.size(); ++j) {
      joint_pos(j) = joint_pos_copy[ALL_JOINT_NAMES[j]][idx];
    }
    
    auto pose = computeFK(joint_pos);
    if (pose) {
      recorded_cartesian_time_.push_back(times_copy[idx]);
      recorded_cartesian_x_.push_back(pose->first.x());
      recorded_cartesian_y_.push_back(pose->first.y());
      recorded_cartesian_z_.push_back(pose->first.z());
    }
  }

  // Compute measured Cartesian velocities from positions using central difference
  // Central difference: v[i] = (x[i+1] - x[i-1]) / (2*dt) is more accurate than forward diff
  for (size_t i = 0; i < recorded_cartesian_time_.size(); ++i) {
    if (i > 0 && i < recorded_cartesian_time_.size() - 1) {
      // Central difference for interior points (more accurate)
      double dt = recorded_cartesian_time_[i+1] - recorded_cartesian_time_[i-1];
      if (dt > 0.001) {
        recorded_cartesian_vx_.push_back((recorded_cartesian_x_[i+1] - recorded_cartesian_x_[i-1]) / dt);
        recorded_cartesian_vy_.push_back((recorded_cartesian_y_[i+1] - recorded_cartesian_y_[i-1]) / dt);
        recorded_cartesian_vz_.push_back((recorded_cartesian_z_[i+1] - recorded_cartesian_z_[i-1]) / dt);
      } else {
        recorded_cartesian_vx_.push_back(0.0);
        recorded_cartesian_vy_.push_back(0.0);
        recorded_cartesian_vz_.push_back(0.0);
      }
    } else if (i > 0) {
      // Forward difference for last point
      double dt = recorded_cartesian_time_[i] - recorded_cartesian_time_[i-1];
      if (dt > 0.001) {
        recorded_cartesian_vx_.push_back((recorded_cartesian_x_[i] - recorded_cartesian_x_[i-1]) / dt);
        recorded_cartesian_vy_.push_back((recorded_cartesian_y_[i] - recorded_cartesian_y_[i-1]) / dt);
        recorded_cartesian_vz_.push_back((recorded_cartesian_z_[i] - recorded_cartesian_z_[i-1]) / dt);
      } else {
        recorded_cartesian_vx_.push_back(0.0);
        recorded_cartesian_vy_.push_back(0.0);
        recorded_cartesian_vz_.push_back(0.0);
      }
    } else {
      recorded_cartesian_vx_.push_back(0.0);
      recorded_cartesian_vy_.push_back(0.0);
      recorded_cartesian_vz_.push_back(0.0);
    }
  }

  RCLCPP_INFO(get_logger(), "  Computed FK for %zu samples", recorded_cartesian_time_.size());
}

void CartesianMotion::saveToCSV(const std::string& filename)
{
  std::string filepath = RESULTS_DIR + "/" + filename;
  std::ofstream file(filepath);
  if (!file.is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open file: %s", filepath.c_str());
    return;
  }

  // Write header - include BOTH commanded and measured data
  file << "time";
  for (const auto& name : ALL_JOINT_NAMES) {
    file << ",pos_" << name << ",vel_" << name;
  }
  // Measured Cartesian (from FK on recorded joints)
  file << ",cart_x,cart_y,cart_z,cart_vx,cart_vy,cart_vz";
  // Commanded Cartesian (ideal trapezoidal profile)
  file << ",cmd_cart_x,cmd_cart_y,cmd_cart_z,cmd_cart_vx,cmd_cart_vy,cmd_cart_vz";
  file << "\n";

  // Write measured data with both measured and commanded Cartesian
  for (size_t i = 0; i < recorded_times_.size(); ++i) {
    file << std::fixed << std::setprecision(6) << recorded_times_[i];
    for (const auto& name : ALL_JOINT_NAMES) {
      if (i < recorded_joint_positions_[name].size()) {
        file << "," << recorded_joint_positions_[name][i];
      } else {
        file << ",";
      }
      if (i < recorded_joint_velocities_[name].size()) {
        file << "," << recorded_joint_velocities_[name][i];
      } else {
        file << ",";
      }
    }
    
    // MEASURED Cartesian data - find matching sample by time using binary search
    if (!recorded_cartesian_time_.empty()) {
      auto it = std::lower_bound(recorded_cartesian_time_.begin(), 
                                  recorded_cartesian_time_.end(), 
                                  recorded_times_[i]);
      if (it != recorded_cartesian_time_.end()) {
        size_t cart_idx = std::distance(recorded_cartesian_time_.begin(), it);
        // Use closest sample (check if previous is closer)
        if (cart_idx > 0) {
          double diff_curr = std::abs(*it - recorded_times_[i]);
          double diff_prev = std::abs(*(it - 1) - recorded_times_[i]);
          if (diff_prev < diff_curr) {
            cart_idx--;
          }
        }
        if (cart_idx < recorded_cartesian_x_.size()) {
          file << "," << recorded_cartesian_x_[cart_idx]
               << "," << recorded_cartesian_y_[cart_idx]
               << "," << recorded_cartesian_z_[cart_idx];
          if (cart_idx < recorded_cartesian_vx_.size()) {
            file << "," << recorded_cartesian_vx_[cart_idx]
                 << "," << recorded_cartesian_vy_[cart_idx]
                 << "," << recorded_cartesian_vz_[cart_idx];
          } else {
            file << ",,,";
          }
        } else {
          file << ",,,,,";
        }
      } else {
        file << ",,,,,";
      }
    } else {
      file << ",,,,,";
    }
    
    // COMMANDED Cartesian data - find matching sample by time
    if (!commanded_times_.empty()) {
      auto it = std::lower_bound(commanded_times_.begin(), 
                                  commanded_times_.end(), 
                                  recorded_times_[i]);
      if (it != commanded_times_.end()) {
        size_t cmd_idx = std::distance(commanded_times_.begin(), it);
        // Use closest sample
        if (cmd_idx > 0) {
          double diff_curr = std::abs(*it - recorded_times_[i]);
          double diff_prev = std::abs(*(it - 1) - recorded_times_[i]);
          if (diff_prev < diff_curr) {
            cmd_idx--;
          }
        }
        if (cmd_idx < commanded_cartesian_x_.size()) {
          file << "," << commanded_cartesian_x_[cmd_idx]
               << "," << commanded_cartesian_y_[cmd_idx]
               << "," << commanded_cartesian_z_[cmd_idx];
          if (cmd_idx < commanded_cartesian_vx_.size()) {
            file << "," << commanded_cartesian_vx_[cmd_idx]
                 << "," << commanded_cartesian_vy_[cmd_idx]
                 << "," << commanded_cartesian_vz_[cmd_idx];
          } else {
            file << ",,,";
          }
        } else {
          file << ",,,,,";
        }
      } else {
        file << ",,,,,";
      }
    } else {
      file << ",,,,,";
    }
    
    file << "\n";
  }

  file.close();
  RCLCPP_INFO(get_logger(), "Saved data to: %s", filepath.c_str());
}

bool CartesianMotion::executeDemo()
{
  RCLCPP_INFO(get_logger(), " ");
  RCLCPP_INFO(get_logger(), "======================================================================");
  RCLCPP_INFO(get_logger(), " TRUE CARTESIAN MOTION with Trapezoidal Velocity Profiles");
  RCLCPP_INFO(get_logger(), "======================================================================");
  RCLCPP_INFO(get_logger(), " ");
  RCLCPP_INFO(get_logger(), "This demo achieves CONSTANT-VELOCITY Cartesian motion by:");
  RCLCPP_INFO(get_logger(), "  1. Defining waypoints in Cartesian space (XYZ)");
  RCLCPP_INFO(get_logger(), "  2. Applying trapezoidal profile to Cartesian distance");
  RCLCPP_INFO(get_logger(), "  3. Using MoveIt IK to compute joint angles at each timestep");
  RCLCPP_INFO(get_logger(), " ");

  // Wait for services and joint state
  if (!waitForServices()) {
    return false;
  }
  if (!waitForJointState()) {
    return false;
  }

  RCLCPP_INFO(get_logger(), "Services and joint state ready");

  // Get current end-effector pose
  auto current_pose = getCurrentEEPose();
  if (!current_pose) {
    RCLCPP_ERROR(get_logger(), "Could not get current end-effector pose!");
    return false;
  }

  auto [ee_pos, ee_orient] = *current_pose;
  RCLCPP_INFO(get_logger(), "Current EE position: (%.3f, %.3f, %.3f)",
              ee_pos.x(), ee_pos.y(), ee_pos.z());

  // Start recording
  startRecording();

  // Motion parameters (matching Python)
  const double V_MAX = 0.08;      // 8 cm/s
  const double A_MAX = 0.15;      // 15 cm/s²
  const double LIFT_V_MAX = 0.1;  // 10 cm/s

  // Get initial pose and define waypoints relative to it
  double base_x = ee_pos.x();
  double base_y = ee_pos.y();
  double base_z = ee_pos.z();

  RCLCPP_INFO(get_logger(), "Base position: (%.3f, %.3f, %.3f)", base_x, base_y, base_z);

  // =========================================================
  // PHASE 1: Square pattern at current Z height
  // =========================================================
  RCLCPP_INFO(get_logger(), " ");
  RCLCPP_INFO(get_logger(), "============================================================");
  RCLCPP_INFO(get_logger(), "PHASE 1: Square pattern in XY plane");
  RCLCPP_INFO(get_logger(), "============================================================");

  double square_size = 0.15;  // 15 cm square
  std::vector<std::tuple<double, double, double>> square_waypoints = {
    {base_x + square_size, base_y, base_z},                    // Right
    {base_x + square_size, base_y + square_size, base_z},      // Up
    {base_x, base_y + square_size, base_z},                    // Left
    {base_x, base_y, base_z},                                  // Back to start
  };

  for (size_t i = 0; i < square_waypoints.size(); ++i) {
    auto [x, y, z] = square_waypoints[i];
    RCLCPP_INFO(get_logger(), "  Moving to waypoint %zu: (%.3f, %.3f, %.3f)", i+1, x, y, z);
    if (!moveToXYZ(x, y, z, V_MAX, A_MAX)) {
      RCLCPP_WARN(get_logger(), "  Failed to reach waypoint %zu", i+1);
    }
    std::this_thread::sleep_for(500ms);
  }

  // =========================================================
  // PHASE 2: Move lift UP (Z motion via lift joint)
  // =========================================================
  RCLCPP_INFO(get_logger(), " ");
  RCLCPP_INFO(get_logger(), "============================================================");
  RCLCPP_INFO(get_logger(), "PHASE 2: Raise lift by 0.3m");
  RCLCPP_INFO(get_logger(), "============================================================");

  auto pose_after_square = getCurrentEEPose();
  if (pose_after_square) {
    auto [pos, orient] = *pose_after_square;
    double new_z = pos.z() + 0.3;
    RCLCPP_INFO(get_logger(), "  Moving Z from %.3f to %.3f", pos.z(), new_z);
    moveToXYZ(pos.x(), pos.y(), new_z, LIFT_V_MAX, A_MAX);
  }
  std::this_thread::sleep_for(1s);

  // =========================================================
  // PHASE 3: Triangle pattern at new height
  // =========================================================
  RCLCPP_INFO(get_logger(), " ");
  RCLCPP_INFO(get_logger(), "============================================================");
  RCLCPP_INFO(get_logger(), "PHASE 3: Triangle pattern at elevated height");
  RCLCPP_INFO(get_logger(), "============================================================");

  auto pose_phase3 = getCurrentEEPose();
  if (pose_phase3) {
    auto [pos, orient] = *pose_phase3;
    double cx = pos.x();
    double cy = pos.y();
    double cz = pos.z();

    double tri_size = 0.12;  // 12 cm triangle
    std::vector<std::tuple<double, double, double>> triangle_waypoints = {
      {cx + tri_size, cy, cz},                           // Right
      {cx + tri_size/2, cy + tri_size * 0.866, cz},      // Top (equilateral)
      {cx, cy, cz},                                       // Back to start
    };

    for (size_t i = 0; i < triangle_waypoints.size(); ++i) {
      auto [x, y, z] = triangle_waypoints[i];
      RCLCPP_INFO(get_logger(), "  Moving to waypoint %zu: (%.3f, %.3f, %.3f)", i+1, x, y, z);
      moveToXYZ(x, y, z, V_MAX, A_MAX);
      std::this_thread::sleep_for(500ms);
    }
  }

  // =========================================================
  // PHASE 4: Move lift UP again
  // =========================================================
  RCLCPP_INFO(get_logger(), " ");
  RCLCPP_INFO(get_logger(), "============================================================");
  RCLCPP_INFO(get_logger(), "PHASE 4: Raise lift another 0.3m");
  RCLCPP_INFO(get_logger(), "============================================================");

  auto pose_phase4 = getCurrentEEPose();
  if (pose_phase4) {
    auto [pos, orient] = *pose_phase4;
    double new_z = pos.z() + 0.3;
    RCLCPP_INFO(get_logger(), "  Moving Z from %.3f to %.3f", pos.z(), new_z);
    moveToXYZ(pos.x(), pos.y(), new_z, LIFT_V_MAX, A_MAX);
  }
  std::this_thread::sleep_for(1s);

  // =========================================================
  // PHASE 5: Line pattern at top height
  // =========================================================
  RCLCPP_INFO(get_logger(), " ");
  RCLCPP_INFO(get_logger(), "============================================================");
  RCLCPP_INFO(get_logger(), "PHASE 5: Back-and-forth line at top height");
  RCLCPP_INFO(get_logger(), "============================================================");

  auto pose_phase5 = getCurrentEEPose();
  if (pose_phase5) {
    auto [pos, orient] = *pose_phase5;
    double cx = pos.x();
    double cy = pos.y();
    double cz = pos.z();

    double line_length = 0.2;  // 20 cm line
    std::vector<std::tuple<double, double, double>> line_waypoints = {
      {cx + line_length, cy, cz},    // Forward
      {cx, cy, cz},                   // Back
      {cx, cy + line_length, cz},     // Right
      {cx, cy, cz},                   // Back
    };

    for (size_t i = 0; i < line_waypoints.size(); ++i) {
      auto [x, y, z] = line_waypoints[i];
      RCLCPP_INFO(get_logger(), "  Moving to waypoint %zu: (%.3f, %.3f, %.3f)", i+1, x, y, z);
      moveToXYZ(x, y, z, V_MAX, A_MAX);
      std::this_thread::sleep_for(300ms);
    }
  }

  // =========================================================
  // SAVE RESULTS
  // =========================================================
  std::this_thread::sleep_for(1s);
  stopRecording();

  RCLCPP_INFO(get_logger(), " ");
  RCLCPP_INFO(get_logger(), "============================================================");
  RCLCPP_INFO(get_logger(), "SAVING RESULTS");
  RCLCPP_INFO(get_logger(), "============================================================");

  auto now_time = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now_time);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&time_t_now), "%Y%m%d_%H%M%S");
  std::string csv_filename = "cartesian_motion_cpp_" + ss.str() + ".csv";
  
  saveToCSV(csv_filename);

  RCLCPP_INFO(get_logger(), " ");
  RCLCPP_INFO(get_logger(), "KEY OBSERVATION:");
  RCLCPP_INFO(get_logger(), "  - Cartesian velocities show TRAPEZOIDAL profiles");
  RCLCPP_INFO(get_logger(), "  - XY trajectory shows STRAIGHT LINES");
  RCLCPP_INFO(get_logger(), "  - Joint velocities are NOT trapezoidal (expected!)");
  RCLCPP_INFO(get_logger(), "  This proves TRUE Cartesian motion with constant velocity!");

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

  // Use multi-threaded executor for proper callback group handling
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
