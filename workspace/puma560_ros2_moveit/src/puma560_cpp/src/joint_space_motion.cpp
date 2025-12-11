/**
 * @file joint_space_motion.cpp
 * @brief Implementation of JointSpaceMotion node
 */

#include "puma560_cpp/joint_space_motion.hpp"

#include <chrono>
#include <fstream>
#include <future>
#include <iomanip>
#include <sstream>
#include <filesystem>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace puma560_cpp
{

// Static member definitions
const std::vector<std::string> JointSpaceMotion::JOINT_NAMES = {
  "lift_joint", "j1", "j2", "j3", "j4", "j5", "j6"
};
const std::string JointSpaceMotion::ACTION_NAME = "/arm_controller/follow_joint_trajectory";
const std::string JointSpaceMotion::RESULTS_DIR = "/root/ros2_ws/src/puma560_ros2_moveit/results";

JointSpaceMotion::JointSpaceMotion(const rclcpp::NodeOptions& options)
  : Node("joint_space_motion", options),
    current_positions_(Eigen::VectorXd::Zero(JOINT_NAMES.size()))
{
  RCLCPP_INFO(get_logger(), "Initializing JointSpaceMotion node");

  // Create MutuallyExclusive callback group for action client
  // This ensures action callbacks don't run concurrently with each other
  action_callback_group_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  // Create Reentrant callback group for joint state subscriber
  // This allows joint state updates while action is running
  // IMPORTANT: Store as member variable to prevent going out of scope
  sub_callback_group_ = create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);

  // Create joint state subscriber with its own callback group
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = sub_callback_group_;
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    std::bind(&JointSpaceMotion::jointStateCallback, this, _1),
    sub_options);

  // Create action client with MutuallyExclusive callback group
  action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
    this, ACTION_NAME, action_callback_group_);

  // Ensure results directory exists
  std::filesystem::create_directories(RESULTS_DIR);

  RCLCPP_INFO(get_logger(), "JointSpaceMotion node initialized");
}

JointSpaceMotion::~JointSpaceMotion()
{
  RCLCPP_INFO(get_logger(), "JointSpaceMotion node shutting down");
}

void JointSpaceMotion::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  // Extract positions for our joints
  std::map<std::string, double> positions;
  std::map<std::string, double> velocities;
  bool has_velocity = !msg->velocity.empty() && msg->velocity.size() == msg->name.size();

  // Log diagnostic info once
  static bool first_msg = true;
  if (first_msg) {
    RCLCPP_INFO(get_logger(), "JointState: %zu joints, has_velocity=%s",
                msg->name.size(), has_velocity ? "true" : "false");
    first_msg = false;
  }

  for (size_t i = 0; i < msg->name.size(); ++i) {
    auto it = std::find(JOINT_NAMES.begin(), JOINT_NAMES.end(), msg->name[i]);
    if (it != JOINT_NAMES.end()) {
      size_t idx = std::distance(JOINT_NAMES.begin(), it);
      positions[msg->name[i]] = msg->position[i];
      current_positions_(idx) = msg->position[i];
      if (has_velocity) {
        velocities[msg->name[i]] = msg->velocity[i];
      }
    }
  }

  if (positions.size() == JOINT_NAMES.size()) {
    has_joint_state_ = true;
  }

  // Recording with proper time and velocity handling
  if (recording_) {
    // Use wall clock for monotonic timestamps (like Python version)
    double t = now() - record_start_time_wall_;
    
    // Store time and data
    recorded_times_.push_back(t);

    for (size_t i = 0; i < JOINT_NAMES.size(); ++i) {
      const auto& name = JOINT_NAMES[i];
      recorded_positions_[name].push_back(current_positions_(i));

      // Use velocity from message if available (preferred - smoother)
      if (has_velocity && velocities.count(name)) {
        recorded_velocities_[name].push_back(velocities[name]);
      } else if (recorded_times_.size() > 1) {
        // Fallback to numerical differentiation
        size_t n = recorded_times_.size();
        double dt = recorded_times_[n-1] - recorded_times_[n-2];
        if (dt > 0.001) {
          size_t pos_n = recorded_positions_[name].size();
          double vel = (recorded_positions_[name][pos_n-1] - 
                        recorded_positions_[name][pos_n-2]) / dt;
          recorded_velocities_[name].push_back(vel);
        } else {
          recorded_velocities_[name].push_back(0.0);
        }
      } else {
        recorded_velocities_[name].push_back(0.0);
      }
    }
  }
}

bool JointSpaceMotion::waitForJointState(double timeout_sec)
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

trajectory_msgs::msg::JointTrajectory JointSpaceMotion::generateTrajectory(
  const Eigen::VectorXd& start_joints,
  const Eigen::VectorXd& end_joints,
  double v_max_scale,
  double a_max_scale,
  double dt)
{
  // Scale velocity and acceleration limits
  std::map<std::string, double> v_max_scaled;
  std::map<std::string, double> a_max_scaled;
  for (const auto& name : JOINT_NAMES) {
    v_max_scaled[name] = SynchronizedProfiles::DEFAULT_V_MAX.at(name) * v_max_scale;
    a_max_scaled[name] = SynchronizedProfiles::DEFAULT_A_MAX.at(name) * a_max_scale;
  }

  // Create synchronized profiles
  SynchronizedProfiles profiles(
    JOINT_NAMES, start_joints, end_joints, v_max_scaled, a_max_scaled);

  double total_time = profiles.getTotalTime();
  RCLCPP_INFO(get_logger(), "Generating trajectory: T=%.3fs, dt=%.3fs", total_time, dt);

  // Generate trajectory points
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.header.stamp = this->get_clock()->now();
  trajectory.header.frame_id = "";  // Empty for joint-space trajectory
  trajectory.joint_names = JOINT_NAMES;

  int n_points = static_cast<int>(std::ceil(total_time / dt)) + 1;
  for (int i = 0; i < n_points; ++i) {
    double t = std::min(i * dt, total_time);
    auto [pos, vel, acc] = profiles.evaluate(t);

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.resize(JOINT_NAMES.size());
    point.velocities.resize(JOINT_NAMES.size());
    point.accelerations.resize(JOINT_NAMES.size());

    for (size_t j = 0; j < JOINT_NAMES.size(); ++j) {
      point.positions[j] = pos(j);
      point.velocities[j] = vel(j);
      point.accelerations[j] = acc(j);
    }

    point.time_from_start.sec = static_cast<int32_t>(t);
    point.time_from_start.nanosec = static_cast<uint32_t>((t - point.time_from_start.sec) * 1e9);

    trajectory.points.push_back(point);
  }

  // Ensure final point is exactly at goal with zero velocity (JTC requirement)
  if (!trajectory.points.empty()) {
    auto& last_point = trajectory.points.back();
    auto [final_pos, _, __] = profiles.evaluate(total_time);
    for (size_t j = 0; j < JOINT_NAMES.size(); ++j) {
      last_point.positions[j] = end_joints(j);  // Exact goal position
      last_point.velocities[j] = 0.0;           // Zero velocity at end
      last_point.accelerations[j] = 0.0;        // Zero acceleration at end
    }
    last_point.time_from_start.sec = static_cast<int32_t>(total_time);
    last_point.time_from_start.nanosec = static_cast<uint32_t>((total_time - last_point.time_from_start.sec) * 1e9);
  }

  // Store commanded values with time offset from recording start
  // This ensures commanded data aligns with measured data timeline
  if (recording_) {
    double time_offset = now() - record_start_time_wall_;
    for (const auto& point : trajectory.points) {
      double t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
      commanded_times_.push_back(time_offset + t);
      for (size_t j = 0; j < JOINT_NAMES.size(); ++j) {
        commanded_positions_[JOINT_NAMES[j]].push_back(point.positions[j]);
        commanded_velocities_[JOINT_NAMES[j]].push_back(point.velocities[j]);
      }
    }
  }

  return trajectory;
}

bool JointSpaceMotion::executeTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory)
{
  const int MAX_RETRIES = 5;
  const double RETRY_DELAY = 1.0;  // seconds
  
  for (int attempt = 1; attempt <= MAX_RETRIES; ++attempt) {
    if (!action_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(get_logger(), "Action server not available");
      return false;
    }

    // Create goal
    auto goal = FollowJointTrajectory::Goal();
    goal.trajectory = trajectory;

    if (attempt == 1) {
      RCLCPP_INFO(get_logger(), "Sending trajectory with %zu points", trajectory.points.size());
    } else {
      RCLCPP_INFO(get_logger(), "Retry %d: Sending trajectory", attempt);
    }

    // Send goal and get future for response
    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    
    // Use promise/future pattern for clean synchronization
    auto goal_response_promise = std::make_shared<std::promise<GoalHandleFJT::SharedPtr>>();
    auto goal_response_future = goal_response_promise->get_future();
    
    send_goal_options.goal_response_callback = 
      [this, goal_response_promise](const GoalHandleFJT::SharedPtr& goal_handle) {
        goal_response_promise->set_value(goal_handle);
      };

    // Send goal asynchronously
    auto goal_future = action_client_->async_send_goal(goal, send_goal_options);

    // Wait for goal response with timeout
    auto goal_status = goal_response_future.wait_for(std::chrono::seconds(10));
    if (goal_status == std::future_status::timeout) {
      RCLCPP_ERROR(get_logger(), "Timeout waiting for goal response");
      return false;
    }

    auto goal_handle = goal_response_future.get();
    if (!goal_handle) {
      // Goal was rejected
      RCLCPP_WARN(get_logger(), "Goal was rejected (controller may not be active yet)");
      if (attempt < MAX_RETRIES) {
        RCLCPP_INFO(get_logger(), "Waiting %.1fs before retry...", RETRY_DELAY);
        std::this_thread::sleep_for(std::chrono::duration<double>(RETRY_DELAY));
        continue;
      } else {
        RCLCPP_ERROR(get_logger(), "Max retries exceeded, goal still rejected");
        return false;
      }
    }

    RCLCPP_INFO(get_logger(), "Goal accepted");

    // Goal was accepted, now wait for result using future
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

    // Get the result
    auto wrapped_result = result_future.get();
    if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(get_logger(), "Trajectory execution succeeded");
      return true;
    } else {
      RCLCPP_WARN(get_logger(), "Trajectory execution failed with code: %d", 
                  static_cast<int>(wrapped_result.code));
      return false;
    }
  }  // end retry loop
  
  return false;  // Should not reach here
}

void JointSpaceMotion::startRecording()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  recorded_times_.clear();
  recorded_positions_.clear();
  recorded_velocities_.clear();
  commanded_times_.clear();
  commanded_positions_.clear();
  commanded_velocities_.clear();

  for (const auto& name : JOINT_NAMES) {
    recorded_positions_[name] = {};
    recorded_velocities_[name] = {};
    commanded_positions_[name] = {};
    commanded_velocities_[name] = {};
  }

  record_start_time_wall_ = now();  // Wall clock for monotonic recording
  recording_ = true;
  RCLCPP_INFO(get_logger(), "Started recording");
}

void JointSpaceMotion::stopRecording()
{
  // Acquire lock to safely stop recording and access recorded data
  std::lock_guard<std::mutex> lock(state_mutex_);
  
  recording_ = false;
  RCLCPP_INFO(get_logger(), "Stopped recording. Recorded %zu samples", recorded_times_.size());
}

void JointSpaceMotion::saveToCSV(const std::string& filename)
{
  // Acquire lock to safely copy recorded data
  std::lock_guard<std::mutex> lock(state_mutex_);
  
  std::string filepath = RESULTS_DIR + "/" + filename;
  std::ofstream file(filepath);
  if (!file.is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open file: %s", filepath.c_str());
    return;
  }

  // Write header - include cmd_time for proper alignment
  file << "time";
  for (const auto& name : JOINT_NAMES) {
    file << ",pos_" << name << ",vel_" << name;
  }
  file << ",cmd_time";
  for (const auto& name : JOINT_NAMES) {
    file << ",cmd_pos_" << name << ",cmd_vel_" << name;
  }
  file << "\n";

  // Write recorded data
  size_t n_recorded = recorded_times_.size();
  size_t n_commanded = commanded_times_.size();

  for (size_t i = 0; i < std::max(n_recorded, n_commanded); ++i) {
    // Measured time
    if (i < n_recorded) {
      file << std::fixed << std::setprecision(6) << recorded_times_[i];
    } else {
      file << "";
    }

    // Recorded positions and velocities
    for (const auto& name : JOINT_NAMES) {
      if (i < recorded_positions_[name].size()) {
        file << "," << std::fixed << std::setprecision(6) 
             << recorded_positions_[name][i] 
             << "," << recorded_velocities_[name][i];
      } else {
        file << ",,";
      }
    }

    // Commanded time
    if (i < n_commanded) {
      file << "," << std::fixed << std::setprecision(6) << commanded_times_[i];
    } else {
      file << ",";
    }

    // Commanded positions and velocities
    for (const auto& name : JOINT_NAMES) {
      if (i < commanded_positions_[name].size()) {
        file << "," << std::fixed << std::setprecision(6)
             << commanded_positions_[name][i] 
             << "," << commanded_velocities_[name][i];
      } else {
        file << ",,";
      }
    }

    file << "\n";
  }

  file.close();
  RCLCPP_INFO(get_logger(), "Saved data to: %s", filepath.c_str());
}

bool JointSpaceMotion::executeWaypoints()
{
  RCLCPP_INFO(get_logger(), "Starting joint space motion demo");

  // Wait for joint state
  if (!waitForJointState()) {
    return false;
  }

  RCLCPP_INFO(get_logger(), "Current joint positions received");

  // Velocity and acceleration scaling
  // Lower V_SCALE ensures we can reach cruise phase for shorter motions
  // Min distance for trapezoidal = (v_max*V_SCALE)² / (a_max*A_SCALE)
  // With V_SCALE=0.25, A_SCALE=0.8: min_dist = (2.0*0.25)² / (5.0*0.8) = 0.125 rad ≈ 7°
  const double V_SCALE = 0.25;  // Lower velocity to ensure trapezoidal profiles
  const double A_SCALE = 0.8;   // Higher acceleration for snappier motion

  // Helper lambda to create waypoint vector
  // Order: [lift_joint, j1, j2, j3, j4, j5, j6]
  auto make_waypoint = [](double lift, double j1, double j2, double j3, 
                          double j4, double j5, double j6) -> Eigen::VectorXd {
    Eigen::VectorXd wp(7);
    wp << lift, j1, j2, j3, j4, j5, j6;
    return wp;
  };

  // Start recording
  startRecording();

  // Get current position with mutex protection
  Eigen::VectorXd current;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current = current_positions_;
  }

  // =========================================================
  // PHASE 1: Move to home position (Z=0)
  // =========================================================
  RCLCPP_INFO(get_logger(), "PHASE 1: Move to home position");
  {
    Eigen::VectorXd target = make_waypoint(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    auto trajectory = generateTrajectory(current, target, V_SCALE, A_SCALE, 0.02);
    if (!executeTrajectory(trajectory)) {
      RCLCPP_ERROR(get_logger(), "Failed in Phase 1");
      stopRecording();
      return false;
    }
    // Use actual measured position for next trajectory (more accurate than commanded)
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current = current_positions_;
    }
    std::this_thread::sleep_for(500ms);
  }

  // =========================================================
  // PHASE 2: XY motion at Z=0.0m (lift at bottom)
  // =========================================================
  RCLCPP_INFO(get_logger(), "PHASE 2: Joint motion at Z=0.0m");
  std::vector<Eigen::VectorXd> waypoints_z0 = {
    make_waypoint(0.0, 0.3, 0.3, 0.2, 0.2, 0.2, 0.2),
    make_waypoint(0.0, 0.8, 0.3, 0.5, 0.2, 0.2, 0.2),
    make_waypoint(0.0, 0.8, 0.8, 0.5, 0.5, 0.2, 0.2),
    make_waypoint(0.0, 0.3, 0.8, 0.2, 0.5, 0.5, 0.5)
  };
  for (size_t i = 0; i < waypoints_z0.size(); ++i) {
    RCLCPP_INFO(get_logger(), "  Waypoint %zu/%zu at Z=0.0m", i + 1, waypoints_z0.size());
    auto trajectory = generateTrajectory(current, waypoints_z0[i], V_SCALE, A_SCALE, 0.02);
    if (!executeTrajectory(trajectory)) {
      RCLCPP_ERROR(get_logger(), "Failed in Phase 2");
      stopRecording();
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current = current_positions_;
    }
    std::this_thread::sleep_for(500ms);
  }

  // =========================================================
  // PHASE 3: Lift to Z=0.5m with trapezoidal profile
  // =========================================================
  RCLCPP_INFO(get_logger(), "PHASE 3: Lift to Z=0.5m");
  {
    // Only change lift joint, keep arm joints same
    Eigen::VectorXd target = current;
    target(0) = 0.5;  // lift_joint
    auto trajectory = generateTrajectory(current, target, V_SCALE, A_SCALE, 0.02);
    if (!executeTrajectory(trajectory)) {
      RCLCPP_ERROR(get_logger(), "Failed in Phase 3");
      stopRecording();
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current = current_positions_;
    }
    std::this_thread::sleep_for(500ms);
  }

  // =========================================================
  // PHASE 4: Joint motion at Z=0.5m
  // =========================================================
  RCLCPP_INFO(get_logger(), "PHASE 4: Joint motion at Z=0.5m");
  std::vector<Eigen::VectorXd> waypoints_z05 = {
    make_waypoint(0.5, 0.2, 0.2, 0.3, 0.3, 0.3, 0.3),
    make_waypoint(0.5, 0.7, 0.2, 0.0, 0.3, 0.0, 0.3),
    make_waypoint(0.5, 0.7, 0.7, 0.0, 0.0, 0.0, 0.0),
    make_waypoint(0.5, 0.2, 0.7, 0.3, 0.3, 0.3, 0.3)
  };
  for (size_t i = 0; i < waypoints_z05.size(); ++i) {
    RCLCPP_INFO(get_logger(), "  Waypoint %zu/%zu at Z=0.5m", i + 1, waypoints_z05.size());
    auto trajectory = generateTrajectory(current, waypoints_z05[i], V_SCALE, A_SCALE, 0.02);
    if (!executeTrajectory(trajectory)) {
      RCLCPP_ERROR(get_logger(), "Failed in Phase 4");
      stopRecording();
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current = current_positions_;
    }
    std::this_thread::sleep_for(500ms);
  }

  // =========================================================
  // PHASE 5: Lift to Z=1.0m
  // =========================================================
  RCLCPP_INFO(get_logger(), "PHASE 5: Lift to Z=1.0m");
  {
    Eigen::VectorXd target = current;
    target(0) = 1.0;  // lift_joint
    auto trajectory = generateTrajectory(current, target, V_SCALE, A_SCALE, 0.02);
    if (!executeTrajectory(trajectory)) {
      RCLCPP_ERROR(get_logger(), "Failed in Phase 5");
      stopRecording();
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current = current_positions_;
    }
    std::this_thread::sleep_for(500ms);
  }

  // =========================================================
  // PHASE 6: Joint motion at Z=1.0m (top)
  // =========================================================
  RCLCPP_INFO(get_logger(), "PHASE 6: Joint motion at Z=1.0m");
  std::vector<Eigen::VectorXd> waypoints_z1 = {
    make_waypoint(1.0, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2),
    make_waypoint(1.0, 0.7, 0.2, -0.2, 0.2, -0.2, 0.2),
    make_waypoint(1.0, 0.7, -0.3, -0.2, -0.2, -0.2, -0.2),
    make_waypoint(1.0, 0.2, -0.3, 0.2, -0.2, 0.2, -0.2)
  };
  for (size_t i = 0; i < waypoints_z1.size(); ++i) {
    RCLCPP_INFO(get_logger(), "  Waypoint %zu/%zu at Z=1.0m", i + 1, waypoints_z1.size());
    auto trajectory = generateTrajectory(current, waypoints_z1[i], V_SCALE, A_SCALE, 0.02);
    if (!executeTrajectory(trajectory)) {
      RCLCPP_ERROR(get_logger(), "Failed in Phase 6");
      stopRecording();
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current = current_positions_;
    }
    std::this_thread::sleep_for(500ms);
  }

  // =========================================================
  // PHASE 7: Return to home
  // =========================================================
  RCLCPP_INFO(get_logger(), "PHASE 7: Return to home");
  {
    Eigen::VectorXd target = make_waypoint(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    auto trajectory = generateTrajectory(current, target, V_SCALE, A_SCALE, 0.02);
    if (!executeTrajectory(trajectory)) {
      RCLCPP_ERROR(get_logger(), "Failed in Phase 7");
      stopRecording();
      return false;
    }
  }

  // Stop recording and save
  stopRecording();

  // Generate timestamp for filename
  auto now_time = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now_time);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&time_t_now), "%Y%m%d_%H%M%S");
  std::string filename = "joint_space_motion_cpp_" + ss.str() + ".csv";
  
  saveToCSV(filename);

  RCLCPP_INFO(get_logger(), "Joint space motion demo completed successfully");
  return true;
}

double JointSpaceMotion::now() const
{
  // Use wall clock for monotonic timestamps (like Python's time.time())
  // This ensures consistent data recording even if simulation time jumps
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace puma560_cpp

// Main function
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<puma560_cpp::JointSpaceMotion>();

  // Use multi-threaded executor
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  // Run in separate thread
  std::thread spin_thread([&executor]() {
    executor.spin();
  });

  // Wait a bit for everything to initialize
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // Execute the demo
  bool success = node->executeWaypoints();

  // Shutdown
  rclcpp::shutdown();
  spin_thread.join();

  return success ? 0 : 1;
}

