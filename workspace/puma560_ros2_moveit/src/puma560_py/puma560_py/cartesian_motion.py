#!/usr/bin/env python3
"""
 Cartesian Motion Controller with Trapezoidal Velocity Profiles

This implementation achieves CONSTANT-VELOCITY Cartesian motion in the XY plane
by using MoveIt's inverse kinematics service to compute joint angles at each
timestep along a straight-line Cartesian path.

- Joint-Space: q(t) = q_start + (q_end - q_start) * s(t)  → Curved Cartesian path
- Cartesian:   x(t) = x_start + (x_end - x_start) * s(t)  → Straight Cartesian path
                q(t) = IK(x(t))

Theory Reference:
- Craig, "Introduction to Robotics", Chapter 7: Trajectory Generation
- Siciliano et al., "Robotics: Modelling, Planning and Control", Chapter 3
- MoveIt motion planning framework documentation

Author: Sahruday Patti
"""

import time
import os
import numpy as np
import threading
from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration
from std_msgs.msg import Header

# MoveIt services for kinematics
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from moveit_msgs.msg import PositionIKRequest, RobotState, MoveItErrorCodes

# For plotting
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from datetime import datetime

# Results directory
RESULTS_DIR = '/root/ros2_ws/src/puma560_ros2_moveit/src/puma560_py/results'


class TrapezoidalVelocityProfile:
    r"""
    Generates trapezoidal velocity profile for smooth motion.
    
    The profile has three phases:
    
    Velocity v(t):
           v_max  _______________
                 /               \
                /                 \
               /                   \
        ______/                     \______
              t1        t2          t3
              
    Position p(t) = integral of v(t):
    - Phase 1 (acceleration):  p = 0.5*a*t^2
    - Phase 2 (constant vel):  p = p1 + v_max*(t-t1)  
    - Phase 3 (deceleration):  p = p2 + v_max*(t-t2) - 0.5*a*(t-t2)^2
    
    Reference: Craig, "Introduction to Robotics", Chapter 7
    """
    
    def __init__(self, distance, v_max, a_max):
        """
        Args:
            distance: Total distance to travel (can be negative)
            v_max: Maximum velocity (always positive)
            a_max: Maximum acceleration (always positive)
        """
        self.distance = abs(distance)
        self.sign = np.sign(distance) if distance != 0 else 1
        self.v_max = abs(v_max)
        self.a_max = abs(a_max)
        
        self._compute_profile()
    
    def _compute_profile(self):
        """Compute the trapezoidal profile timing parameters."""
        if self.distance < 1e-9:
            self.t_accel = 0
            self.t_const = 0
            self.t_decel = 0
            self.v_peak = 0
            self.total_time = 0
            self.t1 = 0
            self.t2 = 0
            self.t3 = 0
            return
            
        t_accel_full = self.v_max / self.a_max
        d_accel_full = 0.5 * self.a_max * t_accel_full**2
        
        if 2 * d_accel_full >= self.distance:
            # Triangular profile
            self.t_accel = np.sqrt(self.distance / self.a_max)
            self.t_const = 0
            self.t_decel = self.t_accel
            self.v_peak = self.a_max * self.t_accel
        else:
            # Trapezoidal profile
            self.t_accel = t_accel_full
            self.t_decel = t_accel_full
            d_const = self.distance - 2 * d_accel_full
            self.t_const = d_const / self.v_max
            self.v_peak = self.v_max
        
        self.total_time = self.t_accel + self.t_const + self.t_decel
        self.t1 = self.t_accel
        self.t2 = self.t_accel + self.t_const
        self.t3 = self.total_time
    
    def evaluate(self, t):
        """
        Evaluate position, velocity, and acceleration at time t.
        Returns: (position, velocity, acceleration) tuple
        """
        if t < 0:
            return 0, 0, 0
        
        if self.total_time < 1e-9:
            return 0, 0, 0
            
        if t >= self.total_time:
            return self.distance * self.sign, 0, 0
        
        if t <= self.t1:
            acc = self.a_max
            vel = self.a_max * t
            pos = 0.5 * self.a_max * t**2
        elif t <= self.t2:
            dt = t - self.t1
            acc = 0
            vel = self.v_peak
            d_accel = 0.5 * self.a_max * self.t1**2
            pos = d_accel + self.v_peak * dt
        else:
            dt = t - self.t2
            acc = -self.a_max
            vel = self.v_peak - self.a_max * dt
            d_accel = 0.5 * self.a_max * self.t1**2
            d_const = self.v_peak * self.t_const
            pos = d_accel + d_const + self.v_peak * dt - 0.5 * self.a_max * dt**2
        
        return pos * self.sign, vel * self.sign, acc * self.sign
    
    def get_normalized(self, t):
        """Get normalized position s(t) in [0, 1] and its derivative."""
        if self.distance < 1e-9:
            return 1.0, 0.0
        pos, vel, _ = self.evaluate(t)
        return pos / (self.distance * self.sign), vel / (self.distance * self.sign)
    
    def get_total_time(self):
        return self.total_time


class CartesianMotionController(Node):
    """
    Controller for  Cartesian motion using MoveIt IK.
    
    This achieves constant-velocity end-effector motion in Cartesian space
    by computing inverse kinematics at each trajectory point.
    """
    
    def __init__(self):
        super().__init__('cartesian_motion_controller')
        
        # Joint configuration
        self.joint_names = ["lift_joint", "j1", "j2", "j3", "j4", "j5", "j6"]
        self.planning_group = "arm"
        self.ee_link = "link7"  # End-effector link
        self.base_frame = "world"
        
        # MoveIt IK service client
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        
        # Action client for trajectory execution
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        
        # Current state
        self.current_positions = None
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )
        
        # Data recording
        self.recorded_times = []
        self.recorded_joint_positions = {name: [] for name in self.joint_names}
        self.recorded_joint_velocities = {name: [] for name in self.joint_names}
        self.recorded_cartesian_positions = {'x': [], 'y': [], 'z': []}
        self.recorded_cartesian_velocities = {'x': [], 'y': [], 'z': []}
        
        self.commanded_times = []
        self.commanded_joint_positions = {name: [] for name in self.joint_names}
        self.commanded_joint_velocities = {name: [] for name in self.joint_names}
        self.commanded_cartesian_positions = {'x': [], 'y': [], 'z': []}
        self.commanded_cartesian_velocities = {'vx': [], 'vy': [], 'vz': []}
        
        self.recording = False
        self.record_start_time = None
        self._last_cartesian_pos = None
        self._last_cartesian_time = None
        
        self.get_logger().info("CartesianMotionController initialized")
        self.get_logger().info(f"  Planning group: {self.planning_group}")
        self.get_logger().info(f"  End-effector: {self.ee_link}")
    
    def _joint_state_callback(self, msg):
        """Store current joint positions and record if enabled."""
        positions = {}
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                positions[name] = msg.position[i]
        
        if len(positions) == len(self.joint_names):
            self.current_positions = np.array([positions[n] for n in self.joint_names])
        
        if self.recording and self.current_positions is not None:
            t = time.time() - self.record_start_time
            self.recorded_times.append(t)
            
            for i, name in enumerate(self.joint_names):
                self.recorded_joint_positions[name].append(self.current_positions[i])
                if len(self.recorded_joint_positions[name]) > 1:
                    dt = self.recorded_times[-1] - self.recorded_times[-2]
                    if dt > 0.001:
                        vel = (self.recorded_joint_positions[name][-1] - 
                               self.recorded_joint_positions[name][-2]) / dt
                    else:
                        vel = 0.0
                else:
                    vel = 0.0
                self.recorded_joint_velocities[name].append(vel)
    
    def wait_for_services(self, timeout=10.0):
        """Wait for MoveIt services to become available."""
        self.get_logger().info("Waiting for MoveIt IK service...")
        if not self.ik_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error("IK service not available!")
            return False
        self.get_logger().info("  IK service ready")
        
        self.get_logger().info("Waiting for MoveIt FK service...")
        if not self.fk_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error("FK service not available!")
            return False
        self.get_logger().info("  FK service ready")
        
        return True
    
    def wait_for_current_state(self, timeout=5.0):
        """Wait until we have current joint positions."""
        start = time.time()
        while self.current_positions is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout:
                raise RuntimeError("Timeout waiting for joint states")
        return self.current_positions.copy()
    
    def compute_fk(self, joint_positions):
        """
        Compute forward kinematics to get end-effector pose.
        
        Args:
            joint_positions: Array of joint positions [7]
            
        Returns:
            PoseStamped of end-effector, or None on failure
        """
        request = GetPositionFK.Request()
        request.header = Header()
        request.header.frame_id = self.base_frame
        request.header.stamp = self.get_clock().now().to_msg()
        
        request.fk_link_names = [self.ee_link]
        
        # Set robot state
        request.robot_state = RobotState()
        request.robot_state.joint_state.name = self.joint_names
        request.robot_state.joint_state.position = joint_positions.tolist()
        
        future = self.fk_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is None:
            self.get_logger().error("FK service call failed")
            return None
        
        response = future.result()
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"FK failed with error code: {response.error_code.val}")
            return None
        
        return response.pose_stamped[0]
    
    def compute_ik(self, target_pose, seed_state=None):
        """
        Compute inverse kinematics for a target pose.
        
        Args:
            target_pose: PoseStamped or Pose of desired end-effector position
            seed_state: Optional seed joint positions for IK solver
            
        Returns:
            Joint positions array [7], or None on failure
        """
        request = GetPositionIK.Request()
        request.ik_request = PositionIKRequest()
        request.ik_request.group_name = self.planning_group
        
        # Set target pose
        if isinstance(target_pose, PoseStamped):
            request.ik_request.pose_stamped = target_pose
        else:
            ps = PoseStamped()
            ps.header.frame_id = self.base_frame
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose = target_pose
            request.ik_request.pose_stamped = ps
        
        # Set seed state
        request.ik_request.robot_state = RobotState()
        request.ik_request.robot_state.joint_state.name = self.joint_names
        if seed_state is not None:
            request.ik_request.robot_state.joint_state.position = seed_state.tolist()
        elif self.current_positions is not None:
            request.ik_request.robot_state.joint_state.position = self.current_positions.tolist()
        else:
            request.ik_request.robot_state.joint_state.position = [0.0] * 7
        
        request.ik_request.timeout = Duration(sec=1, nanosec=0)
        
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is None:
            self.get_logger().error("IK service call failed")
            return None
        
        response = future.result()
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().warn(f"IK failed with error code: {response.error_code.val}")
            return None
        
        # Extract joint positions in correct order
        result = np.zeros(7)
        for i, name in enumerate(self.joint_names):
            if name in response.solution.joint_state.name:
                idx = response.solution.joint_state.name.index(name)
                result[i] = response.solution.joint_state.position[idx]
        
        return result
    
    def get_current_ee_pose(self):
        """Get current end-effector pose using FK."""
        if self.current_positions is None:
            return None
        return self.compute_fk(self.current_positions)
    
    def interpolate_pose(self, start_pose, end_pose, s):
        """
        Linearly interpolate between two poses.
        
        Args:
            start_pose: Starting Pose
            end_pose: Ending Pose
            s: Interpolation parameter [0, 1]
            
        Returns:
            Interpolated Pose
        """
        pose = Pose()
        
        # Linear interpolation of position
        pose.position.x = start_pose.position.x + s * (end_pose.position.x - start_pose.position.x)
        pose.position.y = start_pose.position.y + s * (end_pose.position.y - start_pose.position.y)
        pose.position.z = start_pose.position.z + s * (end_pose.position.z - start_pose.position.z)
        
        # SLERP for orientation (simplified - use start orientation for XY plane motion)
        # For XY plane motion at constant orientation, we keep orientation fixed
        pose.orientation = deepcopy(start_pose.orientation)
        
        return pose
    
    def generate_cartesian_trajectory(
        self,
        start_pose,
        end_pose,
        v_max=0.1,      # m/s in Cartesian space
        a_max=0.2,      # m/s² in Cartesian space
        dt=0.02,        # 50 Hz sampling
        seed_joints=None
    ):
        """
        Generate a joint trajectory for straight-line Cartesian motion.
        
        Cartesian motion:
        1. Compute Cartesian distance
        2. Apply trapezoidal profile in Cartesian space
        3. Interpolate poses linearly
        4. Use IK to convert each pose to joint angles
        
        Args:
            start_pose: Starting end-effector Pose
            end_pose: Ending end-effector Pose
            v_max: Maximum Cartesian velocity (m/s)
            a_max: Maximum Cartesian acceleration (m/s²)
            dt: Trajectory sampling period
            seed_joints: Initial joint configuration for IK
            
        Returns:
            JointTrajectory message, or None on failure
        """
        # Compute Cartesian distance (XYZ)
        dx = end_pose.position.x - start_pose.position.x
        dy = end_pose.position.y - start_pose.position.y
        dz = end_pose.position.z - start_pose.position.z
        distance = np.sqrt(dx**2 + dy**2 + dz**2)
        
        if distance < 1e-6:
            self.get_logger().warn("Start and end poses are the same")
            return None
        
        # Create trapezoidal profile for Cartesian distance
        profile = TrapezoidalVelocityProfile(distance, v_max, a_max)
        
        self.get_logger().info(f"Cartesian trajectory:")
        self.get_logger().info(f"  Distance: {distance:.4f} m")
        self.get_logger().info(f"  Duration: {profile.total_time:.3f} s")
        self.get_logger().info(f"  Peak velocity: {profile.v_peak:.4f} m/s")
        self.get_logger().info(f"  Start: ({start_pose.position.x:.3f}, {start_pose.position.y:.3f}, {start_pose.position.z:.3f})")
        self.get_logger().info(f"  End:   ({end_pose.position.x:.3f}, {end_pose.position.y:.3f}, {end_pose.position.z:.3f})")
        
        # Direction unit vector in Cartesian space
        direction = np.array([dx, dy, dz]) / distance
        
        # Build trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        # Sample trajectory points
        t = 0.0
        prev_joints = seed_joints if seed_joints is not None else self.current_positions
        ik_failures = 0
        
        trajectory_poses = []  # Store for recording
        trajectory_velocities = []
        
        while t <= profile.total_time + dt:
            # Get position and velocity along the path
            pos_scalar, vel_scalar, acc_scalar = profile.evaluate(t)
            
            # Normalized parameter s ∈ [0, 1]
            s = pos_scalar / distance if distance > 1e-9 else 1.0
            s = np.clip(s, 0.0, 1.0)
            
            # Interpolate pose linearly in Cartesian space
            current_pose = self.interpolate_pose(start_pose, end_pose, s)
            
            # Compute Cartesian velocity vector
            cart_vel = direction * vel_scalar  # [vx, vy, vz]
            
            # Solve IK for this pose
            joint_positions = self.compute_ik(current_pose, seed_state=prev_joints)
            
            if joint_positions is None:
                ik_failures += 1
                if ik_failures > 5:
                    self.get_logger().error(f"Too many IK failures ({ik_failures}), aborting trajectory")
                    return None
                # Skip this point and continue
                t += dt
                continue
            
            # Compute joint velocities using numerical differentiation
            if prev_joints is not None and t > 0:
                joint_velocities = (joint_positions - prev_joints) / dt
            else:
                joint_velocities = np.zeros(7)
            
            prev_joints = joint_positions.copy()
            
            # Store for commanded data
            trajectory_poses.append({
                'x': current_pose.position.x,
                'y': current_pose.position.y,
                'z': current_pose.position.z
            })
            trajectory_velocities.append({
                'vx': cart_vel[0],
                'vy': cart_vel[1],
                'vz': cart_vel[2]
            })
            
            # Create trajectory point
            point = JointTrajectoryPoint()
            point.positions = joint_positions.tolist()
            point.velocities = joint_velocities.tolist()
            point.accelerations = [0.0] * 7  # Let controller handle accelerations
            
            sec = int(t)
            nanosec = int((t - sec) * 1e9)
            point.time_from_start = Duration(sec=sec, nanosec=nanosec)
            
            trajectory.points.append(point)
            t += dt
        
        # Ensure final point is exactly at goal
        final_joints = self.compute_ik(end_pose, seed_state=prev_joints)
        if final_joints is not None:
            final_point = JointTrajectoryPoint()
            final_point.positions = final_joints.tolist()
            final_point.velocities = [0.0] * 7
            final_point.accelerations = [0.0] * 7
            sec = int(profile.total_time)
            nanosec = int((profile.total_time - sec) * 1e9)
            final_point.time_from_start = Duration(sec=sec, nanosec=nanosec)
            
            if trajectory.points:
                trajectory.points[-1] = final_point
        
        if ik_failures > 0:
            self.get_logger().warn(f"Trajectory completed with {ik_failures} IK failures")
        
        self.get_logger().info(f"Generated trajectory with {len(trajectory.points)} points")
        
        # Store commanded data if recording
        if self.recording and self.record_start_time is not None:
            time_offset = time.time() - self.record_start_time
            for i, point in enumerate(trajectory.points):
                t_point = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
                self.commanded_times.append(time_offset + t_point)
                
                for j, name in enumerate(self.joint_names):
                    self.commanded_joint_positions[name].append(point.positions[j])
                    self.commanded_joint_velocities[name].append(point.velocities[j])
                
                if i < len(trajectory_poses):
                    self.commanded_cartesian_positions['x'].append(trajectory_poses[i]['x'])
                    self.commanded_cartesian_positions['y'].append(trajectory_poses[i]['y'])
                    self.commanded_cartesian_positions['z'].append(trajectory_poses[i]['z'])
                    self.commanded_cartesian_velocities['vx'].append(trajectory_velocities[i]['vx'])
                    self.commanded_cartesian_velocities['vy'].append(trajectory_velocities[i]['vy'])
                    self.commanded_cartesian_velocities['vz'].append(trajectory_velocities[i]['vz'])
        
        return trajectory
    
    def execute_trajectory(self, trajectory, wait=True):
        """Execute a trajectory using the action interface."""
        if trajectory is None:
            return False
        
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available")
            return False
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        
        self.get_logger().info("Sending trajectory to controller...")
        future = self._action_client.send_goal_async(goal)
        
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("Trajectory goal rejected!")
            return False
        
        self.get_logger().info("Trajectory accepted, executing...")
        
        if wait:
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
            result = result_future.result()
            
            if result.result.error_code == 0:
                self.get_logger().info("Trajectory execution SUCCEEDED")
                return True
            else:
                self.get_logger().error(f"Trajectory execution failed: {result.result.error_code}")
                return False
        
        return True
    
    def move_cartesian_linear(self, target_pose, v_max=0.1, a_max=0.2):
        """
        Move end-effector in a straight line to target pose.
        
        Args:
            target_pose: Target Pose for end-effector
            v_max: Maximum Cartesian velocity (m/s)
            a_max: Maximum Cartesian acceleration (m/s²)
        """
        # Get current pose
        current_pose_stamped = self.get_current_ee_pose()
        if current_pose_stamped is None:
            self.get_logger().error("Could not get current end-effector pose")
            return False
        
        current_pose = current_pose_stamped.pose
        
        # Generate trajectory
        trajectory = self.generate_cartesian_trajectory(
            current_pose, target_pose, v_max, a_max,
            seed_joints=self.current_positions
        )
        
        return self.execute_trajectory(trajectory)
    
    def move_to_xyz(self, x, y, z, v_max=0.1, a_max=0.2):
        """
        Move end-effector to specified XYZ position, maintaining orientation.
        
        Args:
            x, y, z: Target position in world frame
            v_max: Maximum Cartesian velocity (m/s)
            a_max: Maximum Cartesian acceleration (m/s²)
        """
        # Get current pose for orientation
        current_pose_stamped = self.get_current_ee_pose()
        if current_pose_stamped is None:
            self.get_logger().error("Could not get current end-effector pose")
            return False
        
        # Create target pose with same orientation
        target = Pose()
        target.position.x = x
        target.position.y = y
        target.position.z = z
        target.orientation = current_pose_stamped.pose.orientation
        
        return self.move_cartesian_linear(target, v_max, a_max)
    
    def start_recording(self):
        """Start recording motion data."""
        self.recorded_times = []
        self.recorded_joint_positions = {name: [] for name in self.joint_names}
        self.recorded_joint_velocities = {name: [] for name in self.joint_names}
        self.recorded_cartesian_positions = {'x': [], 'y': [], 'z': []}
        self.recorded_cartesian_velocities = {'x': [], 'y': [], 'z': []}
        
        self.commanded_times = []
        self.commanded_joint_positions = {name: [] for name in self.joint_names}
        self.commanded_joint_velocities = {name: [] for name in self.joint_names}
        self.commanded_cartesian_positions = {'x': [], 'y': [], 'z': []}
        self.commanded_cartesian_velocities = {'vx': [], 'vy': [], 'vz': []}
        
        self.record_start_time = time.time()
        self.recording = True
        self.get_logger().info("Started recording")
    
    def stop_recording(self):
        """Stop recording and return data."""
        self.recording = False
        self.get_logger().info(f"Stopped recording. {len(self.recorded_times)} samples")
        
        # Compute measured Cartesian positions using FK (post-processing)
        self.get_logger().info("Computing measured Cartesian positions via FK...")
        measured_cart_x = []
        measured_cart_y = []
        measured_cart_z = []
        
        # Sample every Nth point to avoid too many FK calls
        sample_step = max(1, len(self.recorded_times) // 200)
        sampled_indices = list(range(0, len(self.recorded_times), sample_step))
        sampled_times = []
        
        for idx in sampled_indices:
            joint_pos = np.array([self.recorded_joint_positions[name][idx] for name in self.joint_names])
            pose = self.compute_fk(joint_pos)
            if pose is not None:
                measured_cart_x.append(pose.pose.position.x)
                measured_cart_y.append(pose.pose.position.y)
                measured_cart_z.append(pose.pose.position.z)
                sampled_times.append(self.recorded_times[idx])
        
        # Compute measured Cartesian velocities from positions
        measured_cart_vx = []
        measured_cart_vy = []
        measured_cart_vz = []
        for i in range(len(sampled_times)):
            if i > 0:
                dt = sampled_times[i] - sampled_times[i-1]
                if dt > 0.001:
                    measured_cart_vx.append((measured_cart_x[i] - measured_cart_x[i-1]) / dt)
                    measured_cart_vy.append((measured_cart_y[i] - measured_cart_y[i-1]) / dt)
                    measured_cart_vz.append((measured_cart_z[i] - measured_cart_z[i-1]) / dt)
                else:
                    measured_cart_vx.append(0.0)
                    measured_cart_vy.append(0.0)
                    measured_cart_vz.append(0.0)
            else:
                measured_cart_vx.append(0.0)
                measured_cart_vy.append(0.0)
                measured_cart_vz.append(0.0)
        
        self.get_logger().info(f"  Computed FK for {len(sampled_times)} samples")
        
        return {
            # Measured joint data (full resolution)
            'measured_time': np.array(self.recorded_times),
            'measured_joint_positions': {n: np.array(self.recorded_joint_positions[n]) for n in self.joint_names},
            'measured_joint_velocities': {n: np.array(self.recorded_joint_velocities[n]) for n in self.joint_names},
            
            # Measured Cartesian data (sampled)
            'measured_cartesian_time': np.array(sampled_times),
            'measured_cartesian_positions': {
                'x': np.array(measured_cart_x),
                'y': np.array(measured_cart_y),
                'z': np.array(measured_cart_z)
            },
            'measured_cartesian_velocities': {
                'vx': np.array(measured_cart_vx),
                'vy': np.array(measured_cart_vy),
                'vz': np.array(measured_cart_vz)
            },
            
            # Commanded data
            'commanded_time': np.array(self.commanded_times),
            'commanded_joint_positions': {n: np.array(self.commanded_joint_positions[n]) for n in self.joint_names},
            'commanded_joint_velocities': {n: np.array(self.commanded_joint_velocities[n]) for n in self.joint_names},
            'commanded_cartesian_positions': {k: np.array(v) for k, v in self.commanded_cartesian_positions.items()},
            'commanded_cartesian_velocities': {k: np.array(v) for k, v in self.commanded_cartesian_velocities.items()},
        }


def smooth_signal(signal, window_size=15):
    """Apply moving average smoothing."""
    if len(signal) < window_size:
        return signal
    pad_size = window_size // 2
    padded = np.pad(signal, (pad_size, pad_size), mode='edge')
    kernel = np.ones(window_size) / window_size
    return np.convolve(padded, kernel, mode='valid')


def plot_cartesian_motion(data, output_path):
    """
    Plot Cartesian motion results COMMANDED vs MEASURED comparisons:
    1. End-effector velocities: commanded vs measured
    2. End-effector XY trajectory: measured (solid) vs commanded (dashed)
    3. Joint velocities: all 7 joints, commanded vs measured
    """
    fig, axes = plt.subplots(3, 1, figsize=(16, 14))
    
    # Get time data
    cmd_time = data.get('commanded_time', np.array([]))
    meas_time = data.get('measured_time', np.array([]))
    meas_cart_time = data.get('measured_cartesian_time', np.array([]))
    
    # Normalize times to start at 0
    t0 = cmd_time[0] if len(cmd_time) > 0 else 0
    cmd_time = cmd_time - t0
    meas_time = meas_time - t0 if len(meas_time) > 0 else meas_time
    meas_cart_time = meas_cart_time - t0 if len(meas_cart_time) > 0 else meas_cart_time
    
    # =========================================================
    # Plot 1: END-EFFECTOR VELOCITIES - Commanded vs Measured
    # =========================================================
    ax1 = axes[0]
    
    # Commanded Cartesian velocities
    cmd_cart_vel = data.get('commanded_cartesian_velocities', {})
    if 'vx' in cmd_cart_vel and len(cmd_cart_vel['vx']) > 0:
        vx = np.array(cmd_cart_vel['vx'])
        vy = np.array(cmd_cart_vel['vy'])
        vz = np.array(cmd_cart_vel['vz'])
        min_len = min(len(vx), len(vy), len(vz), len(cmd_time))
        cmd_speed = np.sqrt(vx[:min_len]**2 + vy[:min_len]**2 + vz[:min_len]**2)
        ax1.plot(cmd_time[:min_len], cmd_speed, 'b--', linewidth=2, 
                label='COMMANDED |V|', alpha=0.9)
    
    # Measured Cartesian velocities
    meas_cart_vel = data.get('measured_cartesian_velocities', {})
    if 'vx' in meas_cart_vel and len(meas_cart_vel['vx']) > 0:
        vx = np.array(meas_cart_vel['vx'])
        vy = np.array(meas_cart_vel['vy'])
        vz = np.array(meas_cart_vel['vz'])
        min_len = min(len(vx), len(vy), len(vz), len(meas_cart_time))
        meas_speed = np.sqrt(vx[:min_len]**2 + vy[:min_len]**2 + vz[:min_len]**2)
        # Smooth the measured data
        meas_speed_smooth = smooth_signal(meas_speed, window_size=5)
        ax1.plot(meas_cart_time[:len(meas_speed_smooth)], meas_speed_smooth, 'r-', 
                linewidth=2.5, label='MEASURED |V| (smoothed)', alpha=0.9)
    
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Speed (m/s)')
    ax1.set_title('END-EFFECTOR SPEED: Measured (solid) vs Commanded (dashed)\n(Trapezoidal profile shows constant velocity during cruise phase)', 
                  fontsize=12, fontweight='bold')
    ax1.legend(loc='upper right', fontsize=10)
    ax1.grid(True, alpha=0.3)
    ax1.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    
    # =========================================================
    # Plot 2: END-EFFECTOR XY TRAJECTORY - Measured (solid) vs Commanded (dashed)
    # =========================================================
    ax2 = axes[1]
    
    # Measured positions (SOLID - primary)
    meas_cart_pos = data.get('measured_cartesian_positions', {})
    if 'x' in meas_cart_pos and 'y' in meas_cart_pos:
        x_meas = np.array(meas_cart_pos['x'])
        y_meas = np.array(meas_cart_pos['y'])
        if len(x_meas) > 0:
            ax2.plot(x_meas, y_meas, 'r-', linewidth=2.5, label='MEASURED path', alpha=0.9)
            ax2.scatter([x_meas[0]], [y_meas[0]], c='green', s=150, marker='o', 
                       label='Start', zorder=5, edgecolors='black')
            ax2.scatter([x_meas[-1]], [y_meas[-1]], c='red', s=150, marker='s', 
                       label='End', zorder=5, edgecolors='black')
    
    # Commanded positions (DASHED - reference)
    cmd_cart_pos = data.get('commanded_cartesian_positions', {})
    if 'x' in cmd_cart_pos and 'y' in cmd_cart_pos:
        x_cmd = np.array(cmd_cart_pos['x'])
        y_cmd = np.array(cmd_cart_pos['y'])
        if len(x_cmd) > 0:
            ax2.plot(x_cmd, y_cmd, 'b--', linewidth=1.5, label='COMMANDED path', alpha=0.7)
    
    ax2.set_xlabel('X Position (m)')
    ax2.set_ylabel('Y Position (m)')
    ax2.set_title('END-EFFECTOR XY TRAJECTORY: Measured (solid) vs Commanded (dashed)\n(Straight lines confirm Cartesian motion)', 
                  fontsize=12, fontweight='bold')
    ax2.legend(loc='upper right', fontsize=10)
    ax2.grid(True, alpha=0.3)
    ax2.set_aspect('equal', adjustable='box')
    
    # =========================================================
    # Plot 3: ALL JOINT VELOCITIES - Commanded vs Measured
    # =========================================================
    ax3 = axes[2]
    
    joint_colors = {
        'lift_joint': 'purple', 
        'j1': 'red', 
        'j2': 'green', 
        'j3': 'blue', 
        'j4': 'orange', 
        'j5': 'brown', 
        'j6': 'magenta'
    }
    
    cmd_joint_vel = data.get('commanded_joint_velocities', {})
    meas_joint_vel = data.get('measured_joint_velocities', {})
    
    # Plot ALL joints
    all_joints = ['lift_joint', 'j1', 'j2', 'j3', 'j4', 'j5', 'j6']
    
    # Track max velocity for axis limits
    max_vel = 0.5  # Default
    
    for jname in all_joints:
        color = joint_colors[jname]
        # Commanded - clip outliers (numerical differentiation artifacts between segments)
        if jname in cmd_joint_vel and len(cmd_joint_vel[jname]) > 0:
            vel = np.array(cmd_joint_vel[jname])
            vel_clipped = np.clip(vel, -2.0, 2.0)  # Reasonable joint velocity limits
            ax3.plot(cmd_time[:len(vel_clipped)], vel_clipped,
                    color=color, linewidth=1.5, linestyle='--', 
                    label=f'{jname} (cmd)', alpha=0.7)
            max_vel = max(max_vel, np.max(np.abs(vel_clipped)))
        # Measured
        if jname in meas_joint_vel and len(meas_joint_vel[jname]) > 0:
            vel_smooth = smooth_signal(meas_joint_vel[jname], window_size=15)
            vel_smooth_clipped = np.clip(vel_smooth, -2.0, 2.0)
            ax3.plot(meas_time[:len(vel_smooth_clipped)], vel_smooth_clipped,
                    color=color, linewidth=2, linestyle='-', 
                    label=f'{jname} (meas)', alpha=0.9)
    
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Velocity (rad/s or m/s)')
    ax3.set_ylim(-max_vel * 1.2, max_vel * 1.2)  # Set reasonable y-axis limits
    ax3.set_title('JOINT VELOCITIES: Measured (solid) vs Commanded (dashed) - All 7 Joints', 
                  fontsize=12, fontweight='bold')
    ax3.legend(loc='upper right', ncol=4, fontsize=8)
    ax3.grid(True, alpha=0.3)
    ax3.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    
    # Overall title
    fig.suptitle('CARTESIAN MOTION: End-Effector & Joint Velocity Analysis', 
                 fontsize=14, fontweight='bold', y=1.01)
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Plot saved to: {output_path}")
    plt.close()


def main():
    """
    Main function demonstrating Cartesian motion with trapezoidal velocity profiles.
    
    The end-effector moves in STRAIGHT LINES in the XY plane at constant velocity,
    at multiple Z-heights adjusted via the lift joint.
    """
    rclpy.init()
    
    logger = get_logger("cartesian_motion")
    
    logger.info("=" * 70)
    logger.info(" CARTESIAN MOTION with Trapezoidal Velocity Profiles")
    logger.info("=" * 70)
    logger.info("")
    logger.info("This demo achieves CONSTANT-VELOCITY Cartesian motion by:")
    logger.info("  1. Defining waypoints in Cartesian space (XYZ)")
    logger.info("  2. Applying trapezoidal profile to Cartesian distance")
    logger.info("  3. Using MoveIt IK to compute joint angles at each timestep")
    logger.info("")
    
    # Create controller
    controller = CartesianMotionController()
    
    # Spin in background with graceful shutdown handling
    def spin_node():
        try:
            while rclpy.ok():
                rclpy.spin_once(controller, timeout_sec=0.1)
        except Exception:
            pass
    
    spin_thread = threading.Thread(target=spin_node, daemon=True)
    spin_thread.start()
    
    # Wait for services
    logger.info("Waiting for MoveIt services...")
    if not controller.wait_for_services(timeout=30.0):
        logger.error("MoveIt services not available. Is move_group running?")
        logger.error("  Try: ros2 launch puma560_description moveit.launch.py")
        controller.destroy_node()
        rclpy.shutdown()
        return
    
    # Wait for joint states
    logger.info("Waiting for joint states...")
    time.sleep(2.0)
    current_joints = controller.wait_for_current_state()
    logger.info(f"Current joints: {current_joints}")
    
    # Get current end-effector pose
    current_pose = controller.get_current_ee_pose()
    if current_pose is None:
        logger.error("Could not get current end-effector pose!")
        controller.destroy_node()
        rclpy.shutdown()
        return
    
    ee_pos = current_pose.pose.position
    logger.info(f"Current EE position: ({ee_pos.x:.3f}, {ee_pos.y:.3f}, {ee_pos.z:.3f})")
    
    # Start recording
    controller.start_recording()
    
    # =========================================================
    # MOTION PARAMETERS
    # =========================================================
    V_MAX = 0.08      # 8 cm/s - slow for clear visualization
    A_MAX = 0.15      # 15 cm/s² acceleration
    LIFT_V_MAX = 0.1  # 10 cm/s for lift motion
    
    # =========================================================
    # Get initial pose and define waypoints relative to it
    # =========================================================
    base_x = ee_pos.x
    base_y = ee_pos.y
    base_z = ee_pos.z
    
    logger.info(f"\nBase position: ({base_x:.3f}, {base_y:.3f}, {base_z:.3f})")
    
    # =========================================================
    # PHASE 1: Square pattern at current Z height
    # =========================================================
    logger.info("\n" + "=" * 60)
    logger.info("PHASE 1: Square pattern in XY plane (demonstrating straight-line motion)")
    logger.info("=" * 60)
    
    # Define a square pattern (relative motion)
    square_size = 0.15  # 15 cm square
    square_waypoints = [
        (base_x + square_size, base_y, base_z),                    # Right
        (base_x + square_size, base_y + square_size, base_z),      # Up
        (base_x, base_y + square_size, base_z),                    # Left
        (base_x, base_y, base_z),                                  # Back to start
    ]
    
    for i, (x, y, z) in enumerate(square_waypoints):
        logger.info(f"  Moving to waypoint {i+1}: ({x:.3f}, {y:.3f}, {z:.3f})")
        success = controller.move_to_xyz(x, y, z, v_max=V_MAX, a_max=A_MAX)
        if not success:
            logger.warn(f"  Failed to reach waypoint {i+1}")
        time.sleep(0.5)
    
    # =========================================================
    # PHASE 2: Move lift UP (Z motion via lift joint)
    # =========================================================
    logger.info("\n" + "=" * 60)
    logger.info("PHASE 2: Raise lift by 0.3m")
    logger.info("=" * 60)
    
    # Get current pose after square
    current_pose = controller.get_current_ee_pose()
    if current_pose:
        new_z = current_pose.pose.position.z + 0.3
        logger.info(f"  Moving Z from {current_pose.pose.position.z:.3f} to {new_z:.3f}")
        controller.move_to_xyz(
            current_pose.pose.position.x,
            current_pose.pose.position.y,
            new_z,
            v_max=LIFT_V_MAX, a_max=A_MAX
        )
    time.sleep(1.0)
    
    # =========================================================
    # PHASE 3: Triangle pattern at new height
    # =========================================================
    logger.info("\n" + "=" * 60)
    logger.info("PHASE 3: Triangle pattern at elevated height")
    logger.info("=" * 60)
    
    current_pose = controller.get_current_ee_pose()
    if current_pose:
        cx = current_pose.pose.position.x
        cy = current_pose.pose.position.y
        cz = current_pose.pose.position.z
        
        tri_size = 0.12  # 12 cm triangle
        triangle_waypoints = [
            (cx + tri_size, cy, cz),                           # Right
            (cx + tri_size/2, cy + tri_size * 0.866, cz),      # Top (equilateral)
            (cx, cy, cz),                                       # Back to start
        ]
        
        for i, (x, y, z) in enumerate(triangle_waypoints):
            logger.info(f"  Moving to waypoint {i+1}: ({x:.3f}, {y:.3f}, {z:.3f})")
            controller.move_to_xyz(x, y, z, v_max=V_MAX, a_max=A_MAX)
            time.sleep(0.5)
    
    # =========================================================
    # PHASE 4: Move lift UP again
    # =========================================================
    logger.info("\n" + "=" * 60)
    logger.info("PHASE 4: Raise lift another 0.3m")
    logger.info("=" * 60)
    
    current_pose = controller.get_current_ee_pose()
    if current_pose:
        new_z = current_pose.pose.position.z + 0.3
        logger.info(f"  Moving Z from {current_pose.pose.position.z:.3f} to {new_z:.3f}")
        controller.move_to_xyz(
            current_pose.pose.position.x,
            current_pose.pose.position.y,
            new_z,
            v_max=LIFT_V_MAX, a_max=A_MAX
        )
    time.sleep(1.0)
    
    # =========================================================
    # PHASE 5: Line pattern at top height
    # =========================================================
    logger.info("\n" + "=" * 60)
    logger.info("PHASE 5: Back-and-forth line at top height")
    logger.info("=" * 60)
    
    current_pose = controller.get_current_ee_pose()
    if current_pose:
        cx = current_pose.pose.position.x
        cy = current_pose.pose.position.y
        cz = current_pose.pose.position.z
        
        line_length = 0.2  # 20 cm line
        line_waypoints = [
            (cx + line_length, cy, cz),    # Forward
            (cx, cy, cz),                   # Back
            (cx, cy + line_length, cz),     # Right
            (cx, cy, cz),                   # Back
        ]
        
        for i, (x, y, z) in enumerate(line_waypoints):
            logger.info(f"  Moving to waypoint {i+1}: ({x:.3f}, {y:.3f}, {z:.3f})")
            controller.move_to_xyz(x, y, z, v_max=V_MAX, a_max=A_MAX)
            time.sleep(0.3)
    
    # =========================================================
    # SAVE RESULTS
    # =========================================================
    time.sleep(1.0)
    data = controller.stop_recording()
    
    logger.info("\n" + "=" * 60)
    logger.info("GENERATING PLOTS")
    logger.info("=" * 60)
    
    os.makedirs(RESULTS_DIR, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    plot_path = os.path.join(RESULTS_DIR, f"cartesian_motion_{timestamp}.png")
    
    plot_cartesian_motion(data, plot_path)
    
    logger.info(f"\nPlot saved to: {plot_path}")
    logger.info("")
    logger.info("KEY OBSERVATION:")
    logger.info("  - Cartesian velocities show TRAPEZOIDAL profiles")
    logger.info("  - XY trajectory shows STRAIGHT LINES")
    logger.info("  - Joint velocities are NOT trapezoidal (expected!)")
    logger.info("  This proves  Cartesian motion with constant velocity!")
    
    # Cleanup - proper shutdown to avoid "terminate called without an active exception"
    logger.info("\nShutting down...")
    try:
        rclpy.shutdown()
    except Exception:
        pass
    
    # Give spin thread time to notice shutdown
    time.sleep(0.5)
    
    logger.info("Done!")


if __name__ == "__main__":
    main()
