#!/usr/bin/env python3
"""
Joint Space Motion with Synchronized Trapezoidal Velocity Profiles

This implementation uses SYNCHRONIZED per-joint trapezoidal velocity profiles:
1. Each joint has its own velocity and acceleration limits
2. The slowest joint determines the total motion time
3. Faster joints are scaled to finish at the same time

This is the CORRECT approach for multi-joint motion because:
- Different joints have different units (lift: meters, arm: radians)
- Different joints have different physical limits (motor size, inertia)
- All joints start and stop together (synchronized motion)

Theory Reference:
- Craig, "Introduction to Robotics", Chapter 7: Trajectory Generation
- Modern Robotics by Kevin Lynch & Frank Park, Chapter 9

Author: Sahruday Patti
"""

import time
import os
import numpy as np
import threading
from typing import Dict, List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration

# For plotting
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from datetime import datetime

# Results directory
RESULTS_DIR = '/root/ros2_ws/src/puma560_ros2_moveit/results'

# Default per-joint velocity limits (matching C++)
# lift_joint is prismatic (m/s), others are revolute (rad/s)
DEFAULT_V_MAX = {
    'lift_joint': 0.5,  # m/s
    'j1': 2.0,          # rad/s
    'j2': 2.0,
    'j3': 2.0,
    'j4': 2.0,
    'j5': 2.0,
    'j6': 2.0,
}

# Default per-joint acceleration limits (matching C++)
DEFAULT_A_MAX = {
    'lift_joint': 1.0,  # m/s²
    'j1': 5.0,          # rad/s²
    'j2': 5.0,
    'j3': 5.0,
    'j4': 5.0,
    'j5': 5.0,
    'j6': 5.0,
}


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
    
    def __init__(self, distance: float, v_max: float, a_max: float):
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
        
        # Time to accelerate from 0 to v_max
        t_accel_full = self.v_max / self.a_max
        
        # Distance covered during full acceleration phase
        d_accel_full = 0.5 * self.a_max * t_accel_full**2
        
        # Check if we can reach v_max (triangular vs trapezoidal)
        if 2 * d_accel_full >= self.distance:
            # TRIANGULAR profile - can't reach v_max
            self.t_accel = np.sqrt(self.distance / self.a_max)
            self.t_const = 0
            self.t_decel = self.t_accel
            self.v_peak = self.a_max * self.t_accel
        else:
            # TRAPEZOIDAL profile - full v_max reached
            self.t_accel = t_accel_full
            self.t_decel = t_accel_full
            d_const = self.distance - 2 * d_accel_full
            self.t_const = d_const / self.v_max
            self.v_peak = self.v_max
        
        self.total_time = self.t_accel + self.t_const + self.t_decel
        self.t1 = self.t_accel
        self.t2 = self.t_accel + self.t_const
        self.t3 = self.total_time
    
    def evaluate(self, t: float) -> Tuple[float, float, float]:
        """
        Evaluate position, velocity, and acceleration at time t.
        
        Returns:
            (position, velocity, acceleration) tuple
        """
        if t < 0:
            return 0, 0, 0
        
        if self.total_time < 1e-9:
            return 0, 0, 0
        
        if t >= self.total_time:
            return self.distance * self.sign, 0, 0
        
        if t <= self.t1:
            # PHASE 1: Acceleration
            acc = self.a_max
            vel = self.a_max * t
            pos = 0.5 * self.a_max * t**2
        elif t <= self.t2:
            # PHASE 2: Constant velocity
            dt = t - self.t1
            acc = 0
            vel = self.v_peak
            d_accel = 0.5 * self.a_max * self.t1**2
            pos = d_accel + self.v_peak * dt
        else:
            # PHASE 3: Deceleration
            dt = t - self.t2
            acc = -self.a_max
            vel = self.v_peak - self.a_max * dt
            d_accel = 0.5 * self.a_max * self.t1**2
            d_const = self.v_peak * self.t_const
            pos = d_accel + d_const + self.v_peak * dt - 0.5 * self.a_max * dt**2
        
        return pos * self.sign, vel * self.sign, acc * self.sign
    
    def get_total_time(self) -> float:
        return self.total_time
    
    def __repr__(self):
        return (f"TrapezoidalProfile(d={self.distance:.3f}, v_max={self.v_max:.3f}, "
                f"a_max={self.a_max:.3f}, T={self.total_time:.3f}s)")


class SynchronizedProfiles:
    """
    Generates synchronized trapezoidal profiles for multiple joints.
    
    Uses VELOCITY SCALING to synchronize joints:
    1. Compute unconstrained time for each joint with its own limits
    2. Find the slowest joint (determines total time)
    3. Scale velocities for faster joints to match total time
    
    This is the CORRECT approach for multi-joint motion because:
    - Units are not mixed (meters vs radians)
    - Per-joint limits are respected
    - All joints start and stop together
    
    Mathematical approach:
    - For each joint i with displacement delta_i:
      1. Compute unconstrained time T_i with (v_max_i, a_max_i)
      2. Find T_total = max(T_1, ..., T_n)
      3. Scale each joint's velocity so it takes T_total
    """
    
    def __init__(
        self,
        joint_names: List[str],
        start_positions: np.ndarray,
        end_positions: np.ndarray,
        v_max_dict: Optional[Dict[str, float]] = None,
        a_max_dict: Optional[Dict[str, float]] = None
    ):
        """
        Args:
            joint_names: List of joint names
            start_positions: Starting positions for each joint
            end_positions: Ending positions for each joint
            v_max_dict: Maximum velocities per joint (uses defaults if None)
            a_max_dict: Maximum accelerations per joint (uses defaults if None)
        """
        self.joint_names = joint_names
        self.start = np.array(start_positions)
        self.end = np.array(end_positions)
        self.deltas = self.end - self.start
        self.v_max_dict = v_max_dict or {}
        self.a_max_dict = a_max_dict or {}
        
        self.profiles: List[Optional[TrapezoidalVelocityProfile]] = []
        self.total_time = 0.0
        
        self._compute_synchronized_profiles()
    
    def _get_v_max(self, name: str) -> float:
        """Get velocity limit for a joint."""
        if name in self.v_max_dict:
            return self.v_max_dict[name]
        return DEFAULT_V_MAX.get(name, 1.0)
    
    def _get_a_max(self, name: str) -> float:
        """Get acceleration limit for a joint."""
        if name in self.a_max_dict:
            return self.a_max_dict[name]
        return DEFAULT_A_MAX.get(name, 1.0)
    
    def _compute_synchronized_profiles(self):
        """Compute synchronized profiles for all joints using velocity scaling."""
        n_joints = len(self.joint_names)
        
        # Step 1: Compute unconstrained times for each joint
        unconstrained_times = []
        for i in range(n_joints):
            abs_delta = abs(self.deltas[i])
            if abs_delta < 1e-9:
                unconstrained_times.append(0.0)
                continue
            
            v_max = self._get_v_max(self.joint_names[i])
            a_max = self._get_a_max(self.joint_names[i])
            
            # Create test profile with absolute delta to get timing
            test_profile = TrapezoidalVelocityProfile(abs_delta, v_max, a_max)
            unconstrained_times.append(test_profile.get_total_time())
        
        # Step 2: Find slowest joint (determines total time)
        self.total_time = max(unconstrained_times) if unconstrained_times else 0.0
        
        if self.total_time < 1e-9:
            # No motion needed
            self.profiles = [None] * n_joints
            return
        
        # Step 3: Create profiles with SCALED VELOCITY to match total_time
        self.profiles = []
        for i in range(n_joints):
            delta = self.deltas[i]  # Keep sign for direction!
            abs_delta = abs(delta)
            
            if abs_delta < 1e-9:
                self.profiles.append(None)
                continue
            
            v_max_orig = self._get_v_max(self.joint_names[i])
            a_max_orig = self._get_a_max(self.joint_names[i])
            
            # Check if this joint is the slowest
            if abs(unconstrained_times[i] - self.total_time) < 1e-6:
                # Slowest joint - use original limits
                self.profiles.append(
                    TrapezoidalVelocityProfile(delta, v_max_orig, a_max_orig)
                )
                continue
            
            # This joint would finish early - scale DOWN velocity to match total_time
            # For trapezoidal profile: T = v/a + d/v
            # Solving for v given T and d:
            #   v² - T*a*v + a*d = 0
            #   v = (T*a - sqrt(T²*a² - 4*a*d)) / 2
            T = self.total_time
            a = a_max_orig
            d = abs_delta
            
            discriminant = T * T * a * a - 4.0 * a * d
            
            if discriminant >= 0:
                # Trapezoidal or triangular profile possible with this acceleration
                v_scaled = (T * a - np.sqrt(discriminant)) / 2.0
                v_scaled = min(v_scaled, v_max_orig)  # Don't exceed original limit
                v_scaled = max(v_scaled, 1e-6)        # Avoid zero velocity
                a_scaled = a_max_orig
            else:
                # Need triangular profile with lower acceleration
                # For triangular: T = 2*sqrt(d/a) => a = 4*d/T²
                a_scaled = 4.0 * d / (T * T)
                a_scaled = min(a_scaled, a_max_orig)  # Don't exceed original limit
                v_scaled = v_max_orig  # Will become triangular anyway
            
            # Create profile with SIGNED delta to preserve direction
            self.profiles.append(
                TrapezoidalVelocityProfile(delta, v_scaled, a_scaled)
            )
    
    def evaluate(self, t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Evaluate all joint profiles at time t.
        
        Returns:
            (positions, velocities, accelerations) for all joints
        """
        n_joints = len(self.joint_names)
        positions = np.zeros(n_joints)
        velocities = np.zeros(n_joints)
        accelerations = np.zeros(n_joints)
        
        for i in range(n_joints):
            if self.profiles[i] is None:
                # Joint doesn't move - stay at start position
                positions[i] = self.start[i]
                velocities[i] = 0.0
                accelerations[i] = 0.0
            else:
                # Evaluate the profile directly
                pos, vel, acc = self.profiles[i].evaluate(t)
                positions[i] = self.start[i] + pos
                velocities[i] = vel
                accelerations[i] = acc
        
        return positions, velocities, accelerations
    
    def get_total_time(self) -> float:
        return self.total_time


class JointTrajectoryExecutor(Node):
    """
    Executes joint trajectories using synchronized trapezoidal profiles.
    
    Uses MutuallyExclusive callback groups for action client (ROS2 best practice).
    """
    
    def __init__(self):
        super().__init__('trajectory_executor')
        
        self.joint_names = ["lift_joint", "j1", "j2", "j3", "j4", "j5", "j6"]
        
        # Create MutuallyExclusive callback group for action client
        self.action_callback_group = MutuallyExclusiveCallbackGroup()
        
        # Create Reentrant callback group for subscribers
        self.sub_callback_group = ReentrantCallbackGroup()
        
        # Action client for trajectory execution
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            callback_group=self.action_callback_group
        )
        
        # Subscribe to joint states for current position
        self.current_positions = None
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10,
            callback_group=self.sub_callback_group
        )
        
        # Data recording - MEASURED from robot
        self.recorded_times = []
        self.recorded_positions = {name: [] for name in self.joint_names}
        self.recorded_velocities = {name: [] for name in self.joint_names}
        self.recording = False
        self.record_start_time = None
        
        # COMMANDED data - what we send to controller (ideal trapezoidal)
        self.commanded_times = []
        self.commanded_velocities = {name: [] for name in self.joint_names}
        self.commanded_positions = {name: [] for name in self.joint_names}
        
        self.get_logger().info("JointTrajectoryExecutor initialized")
        self.get_logger().info("  Using synchronized per-joint trapezoidal profiles")
    
    def _joint_state_callback(self, msg):
        """Store current joint positions and record if enabled."""
        positions = {}
        velocities = {}
        has_velocity = len(msg.velocity) == len(msg.name)
        
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                positions[name] = msg.position[i]
                if has_velocity:
                    velocities[name] = msg.velocity[i]
        
        if len(positions) == len(self.joint_names):
            self.current_positions = np.array([positions[n] for n in self.joint_names])
        
        # Recording
        if self.recording and self.current_positions is not None:
            t = time.time() - self.record_start_time
            self.recorded_times.append(t)
            
            for i, name in enumerate(self.joint_names):
                self.recorded_positions[name].append(self.current_positions[i])
                
                # Use velocity from message if available
                if has_velocity and name in velocities:
                    self.recorded_velocities[name].append(velocities[name])
                elif len(self.recorded_positions[name]) > 1:
                    # Fallback to numerical differentiation
                    dt = self.recorded_times[-1] - self.recorded_times[-2]
                    if dt > 0.001:
                        vel = (self.recorded_positions[name][-1] - 
                               self.recorded_positions[name][-2]) / dt
                    else:
                        vel = 0.0
                    self.recorded_velocities[name].append(vel)
                else:
                    self.recorded_velocities[name].append(0.0)
    
    def start_recording(self):
        """Start recording joint data."""
        self.recorded_times = []
        self.recorded_positions = {name: [] for name in self.joint_names}
        self.recorded_velocities = {name: [] for name in self.joint_names}
        self.commanded_times = []
        self.commanded_velocities = {name: [] for name in self.joint_names}
        self.commanded_positions = {name: [] for name in self.joint_names}
        self.record_start_time = time.time()
        self.recording = True
        self.get_logger().info("Started recording")
    
    def stop_recording(self):
        """Stop recording and return data."""
        self.recording = False
        self.get_logger().info(f"Stopped recording. {len(self.recorded_times)} measured samples")
        self.get_logger().info(f"  Commanded trajectory: {len(self.commanded_times)} points")
        return {
            # Measured data
            'time': np.array(self.recorded_times),
            'positions': {n: np.array(self.recorded_positions[n]) for n in self.joint_names},
            'velocities': {n: np.array(self.recorded_velocities[n]) for n in self.joint_names},
            # Commanded data 
            'commanded_time': np.array(self.commanded_times),
            'commanded_velocities': {n: np.array(self.commanded_velocities[n]) for n in self.joint_names},
            'commanded_positions': {n: np.array(self.commanded_positions[n]) for n in self.joint_names},
        }
    
    def _store_commanded_trajectory(self, trajectory, time_offset):
        """Store the commanded trajectory data for plotting."""
        for point in trajectory.points:
            t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            self.commanded_times.append(time_offset + t)
            for i, name in enumerate(self.joint_names):
                self.commanded_positions[name].append(point.positions[i])
                self.commanded_velocities[name].append(point.velocities[i])
    
    def wait_for_current_state(self, timeout=5.0):
        """Wait until we have current joint positions."""
        start = time.time()
        while self.current_positions is None:
            time.sleep(0.1)
            if time.time() - start > timeout:
                raise RuntimeError("Timeout waiting for joint states")
        return self.current_positions.copy()
    
    def generate_trajectory_with_synchronized_profiles(
        self, 
        start_joints, 
        end_joints, 
        v_scale=1.0,
        a_scale=1.0,
        dt=0.02
    ):
        """
        Generate a JointTrajectory using SYNCHRONIZED per-joint trapezoidal profiles.
        
        This is the CORRECT approach:
        - Each joint has its own velocity and acceleration limits
        - The slowest joint determines the total motion time
        - Faster joints are scaled to finish at the same time
        
        Args:
            start_joints: Starting joint positions
            end_joints: Ending joint positions
            v_scale: Velocity scaling factor (0-1)
            a_scale: Acceleration scaling factor (0-1)
            dt: Time step for trajectory sampling
            
        Returns:
            JointTrajectory message ready to execute
        """
        start = np.array(start_joints)
        end = np.array(end_joints)
        
        # Scale velocity and acceleration limits
        v_max_scaled = {name: DEFAULT_V_MAX[name] * v_scale for name in self.joint_names}
        a_max_scaled = {name: DEFAULT_A_MAX[name] * a_scale for name in self.joint_names}
        
        # Create synchronized profiles
        profiles = SynchronizedProfiles(
            self.joint_names, start, end, v_max_scaled, a_max_scaled
        )
        
        total_time = profiles.get_total_time()
        
        if total_time < 1e-6:
            self.get_logger().warn("Start and end positions are the same")
            return None
        
        self.get_logger().info(f"Generating trajectory: T={total_time:.3f}s, dt={dt:.3f}s")
        
        # Build trajectory message
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        # Sample the profile at regular intervals
        t = 0.0
        while t <= total_time + dt:
            t_eval = min(t, total_time)
            positions, velocities, accelerations = profiles.evaluate(t_eval)
            
            # Create trajectory point
            point = JointTrajectoryPoint()
            point.positions = positions.tolist()
            point.velocities = velocities.tolist()
            point.accelerations = accelerations.tolist()
            
            # Set timestamp
            sec = int(t_eval)
            nanosec = int((t_eval - sec) * 1e9)
            point.time_from_start = Duration(sec=sec, nanosec=nanosec)
            
            trajectory.points.append(point)
            t += dt
        
        # Ensure final point is exactly at goal with zero velocity
        if trajectory.points:
            final_point = JointTrajectoryPoint()
            final_point.positions = end.tolist()
            final_point.velocities = [0.0] * len(self.joint_names)
            final_point.accelerations = [0.0] * len(self.joint_names)
            sec = int(total_time)
            nanosec = int((total_time - sec) * 1e9)
            final_point.time_from_start = Duration(sec=sec, nanosec=nanosec)
            trajectory.points[-1] = final_point
        
        self.get_logger().info(f"Generated trajectory with {len(trajectory.points)} points")
        
        return trajectory
    
    def execute_trajectory(self, trajectory, wait=True):
        """Execute a trajectory using the action interface."""
        if trajectory is None:
            return False
        
        # Wait for action server
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available")
            return False
        
        # Store commanded trajectory for plotting (with current time offset)
        if self.recording and self.record_start_time is not None:
            time_offset = time.time() - self.record_start_time
            self._store_commanded_trajectory(trajectory, time_offset)
        
        # Create goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        
        # Send goal
        self.get_logger().info("Sending trajectory to controller...")
        future = self._action_client.send_goal_async(goal)
        
        # Wait for goal acceptance
        while not future.done():
            time.sleep(0.01)
        
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("Trajectory goal rejected!")
            return False
        
        self.get_logger().info("Trajectory accepted, executing...")
        
        if wait:
            # Wait for result
            result_future = goal_handle.get_result_async()
            while not result_future.done():
                time.sleep(0.05)
            result = result_future.result()
            
            if result.result.error_code == 0:
                self.get_logger().info("Trajectory execution SUCCEEDED")
                return True
            else:
                self.get_logger().error(f"Trajectory execution failed: {result.result.error_code}")
                return False
        
        return True
    
    def move_with_trapezoidal_profile(self, target_joints, v_scale=0.5, a_scale=0.5):
        """
        High-level function to move to target using synchronized trapezoidal profiles.
        
        Args:
            target_joints: Target joint positions [7]
            v_scale: Velocity scaling factor (0-1)
            a_scale: Acceleration scaling factor (0-1)
            
        Returns:
            True if successful
        """
        # Get current position
        current = self.wait_for_current_state()
        
        # Generate trajectory with synchronized profiles
        trajectory = self.generate_trajectory_with_synchronized_profiles(
            current, target_joints, v_scale, a_scale
        )
        
        # Execute
        return self.execute_trajectory(trajectory)


# Standardized smoothing window size for all velocity signals (matching cartesian_motion.py)
SMOOTHING_WINDOW_SIZE = 11


def smooth_signal(signal, window_size=None):
    """Apply moving average smoothing to reduce noise.
    
    Args:
        signal: Input signal array
        window_size: Window size for moving average. If None, uses SMOOTHING_WINDOW_SIZE.
    
    Returns:
        Smoothed signal
    """
    if window_size is None:
        window_size = SMOOTHING_WINDOW_SIZE
    if len(signal) < window_size:
        return signal
    
    pad_size = window_size // 2
    padded = np.pad(signal, (pad_size, pad_size), mode='edge')
    kernel = np.ones(window_size) / window_size
    smoothed = np.convolve(padded, kernel, mode='valid')
    
    return smoothed


def plot_velocities_corrected(data, output_path):
    """Plot joint velocities comparing COMMANDED vs MEASURED."""
    fig, axes = plt.subplots(4, 1, figsize=(16, 14))
    
    measured_time = data['time']
    commanded_time = data.get('commanded_time', np.array([]))
    
    if len(measured_time) == 0:
        print("No data to plot")
        return
    
    # Normalize times to start at 0
    t0 = measured_time[0] if len(measured_time) > 0 else 0
    measured_time = measured_time - t0
    if len(commanded_time) > 0:
        commanded_time = commanded_time - t0
    
    # Plot 1: LIFT JOINT
    ax1 = axes[0]
    if 'commanded_velocities' in data and 'lift_joint' in data['commanded_velocities']:
        cmd_vel = data['commanded_velocities']['lift_joint']
        if len(cmd_vel) > 0:
            ax1.plot(commanded_time[:len(cmd_vel)], cmd_vel, 
                    'b-', linewidth=3, label='COMMANDED (ideal)', alpha=0.9)
    if 'lift_joint' in data['velocities']:
        vel = data['velocities']['lift_joint']
        vel_smooth = smooth_signal(vel)
        ax1.plot(measured_time[:len(vel_smooth)], vel_smooth, 
                'r-', linewidth=1.5, label='MEASURED (smoothed)', alpha=0.7)
    ax1.set_ylabel('Velocity (m/s)')
    ax1.set_title('LIFT JOINT: Commanded Trapezoidal vs Measured', fontsize=12, fontweight='bold')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    ax1.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    
    # Plot 2: ARM JOINTS - Commanded
    ax2 = axes[1]
    colors = ['red', 'green', 'blue', 'cyan', 'magenta', 'orange']
    for i, jname in enumerate(['j1', 'j2', 'j3', 'j4', 'j5', 'j6']):
        if 'commanded_velocities' in data and jname in data['commanded_velocities']:
            cmd_vel = data['commanded_velocities'][jname]
            if len(cmd_vel) > 0:
                ax2.plot(commanded_time[:len(cmd_vel)], cmd_vel,
                        color=colors[i], linewidth=2, label=jname, alpha=0.8)
    ax2.set_ylabel('Velocity (rad/s)')
    ax2.set_title('ARM JOINTS: COMMANDED Velocities (Synchronized Trapezoidal Profiles)', 
                  fontsize=12, fontweight='bold')
    ax2.legend(loc='upper right', ncol=3)
    ax2.grid(True, alpha=0.3)
    ax2.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    
    # Plot 3: ARM JOINTS - Measured
    ax3 = axes[2]
    for i, jname in enumerate(['j1', 'j2', 'j3', 'j4', 'j5', 'j6']):
        if jname in data['velocities']:
            vel = data['velocities'][jname]
            vel_smooth = smooth_signal(vel)
            ax3.plot(measured_time[:len(vel_smooth)], vel_smooth,
                    color=colors[i], linewidth=1.5, label=jname, alpha=0.8)
    ax3.set_ylabel('Velocity (rad/s)')
    ax3.set_title('ARM JOINTS: MEASURED Velocities (Smoothed)', fontsize=12, fontweight='bold')
    ax3.legend(loc='upper right', ncol=3)
    ax3.grid(True, alpha=0.3)
    ax3.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    
    # Plot 4: Joint Positions
    ax4 = axes[3]
    if 'commanded_positions' in data and 'lift_joint' in data['commanded_positions']:
        cmd_pos = data['commanded_positions']['lift_joint']
        if len(cmd_pos) > 0:
            ax4.plot(commanded_time[:len(cmd_pos)], cmd_pos,
                    'b-', linewidth=3, label='lift (cmd)', alpha=0.8)
    if 'lift_joint' in data['positions']:
        pos = data['positions']['lift_joint']
        ax4.plot(measured_time[:len(pos)], pos,
                'b--', linewidth=1, label='lift (meas)', alpha=0.5)
    for i, jname in enumerate(['j1', 'j2', 'j3', 'j4', 'j5', 'j6']):
        if 'commanded_positions' in data and jname in data['commanded_positions']:
            cmd_pos = data['commanded_positions'][jname]
            if len(cmd_pos) > 0:
                ax4.plot(commanded_time[:len(cmd_pos)], cmd_pos,
                        color=colors[i], linewidth=2, label=f'{jname} (cmd)', alpha=0.8)
        if jname in data['positions']:
            meas_pos = data['positions'][jname]
            if len(meas_pos) > 0:
                ax4.plot(measured_time[:len(meas_pos)], meas_pos,
                        color=colors[i], linewidth=1, linestyle='--', 
                        label=f'{jname} (meas)', alpha=0.5)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Position (m or rad)')
    ax4.set_title('Joint Positions: Commanded (solid) vs Measured (dashed)', fontsize=12, fontweight='bold')
    ax4.legend(loc='upper right', ncol=4, fontsize=8)
    ax4.grid(True, alpha=0.3)
    
    fig.suptitle('SYNCHRONIZED TRAPEZOIDAL VELOCITY PROFILES: Per-Joint with Time Synchronization', 
                 fontsize=14, fontweight='bold', y=1.02)
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Plot saved to: {output_path}")
    plt.close()


def main():
    """Main function demonstrating synchronized trapezoidal velocity profile execution."""
    rclpy.init()
    
    logger = get_logger("joint_space_motion")
    
    logger.info("=" * 70)
    logger.info(" Synchronized Trapezoidal Velocity Profile Motion Control")
    logger.info("=" * 70)
    logger.info(" ")
    logger.info("Using SYNCHRONIZED per-joint profiles (correct approach):")
    logger.info("  - Each joint has its own velocity/acceleration limits")
    logger.info("  - Slowest joint determines total time")
    logger.info("  - All joints start and stop together")
    logger.info(" ")
    
    # Create executor node
    executor_node = JointTrajectoryExecutor()
    
    # Use MultiThreadedExecutor for proper callback group handling
    executor = MultiThreadedExecutor()
    executor.add_node(executor_node)
    
    # Spin in background thread
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    # Wait for joint states
    logger.info("Waiting for joint states...")
    time.sleep(2.0)
    current = executor_node.wait_for_current_state()
    logger.info(f"Current joints: {current}")
    
    # Start recording
    executor_node.start_recording()
    
    # Velocity and acceleration scaling
    # Lower V_SCALE ensures we can reach cruise phase for shorter motions
    # Min distance for trapezoidal = (v_max*V_SCALE)² / (a_max*A_SCALE)
    # With V_SCALE=0.25, A_SCALE=0.8: min_dist = (2.0*0.25)² / (5.0*0.8) = 0.125 rad ≈ 7°
    V_SCALE = 0.25  # Lower velocity to ensure trapezoidal profiles
    A_SCALE = 0.8   # Higher acceleration for snappier motion
    
    # Helper to create waypoint vector
    def make_waypoint(lift, j1, j2, j3, j4, j5, j6):
        return np.array([lift, j1, j2, j3, j4, j5, j6])
    
    # PHASE 1: Move to home
    logger.info("\nPHASE 1: Move to home position")
    target = make_waypoint(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    executor_node.move_with_trapezoidal_profile(target, V_SCALE, A_SCALE)
    time.sleep(0.5)
    
    # PHASE 2: XY motion at Z=0.0m
    logger.info("\nPHASE 2: Joint motion at Z=0.0m")
    waypoints_z0 = [
        make_waypoint(0.0, 0.3, 0.3, 0.2, 0.2, 0.2, 0.2),
        make_waypoint(0.0, 0.8, 0.3, 0.5, 0.2, 0.2, 0.2),
        make_waypoint(0.0, 0.8, 0.8, 0.5, 0.5, 0.2, 0.2),
        make_waypoint(0.0, 0.3, 0.8, 0.2, 0.5, 0.5, 0.5),
    ]
    for i, wp in enumerate(waypoints_z0):
        logger.info(f"  Waypoint {i+1}/{len(waypoints_z0)} at Z=0.0m")
        executor_node.move_with_trapezoidal_profile(wp, V_SCALE, A_SCALE)
        time.sleep(0.5)
    
    # PHASE 3: Lift to Z=0.5m
    logger.info("\nPHASE 3: Lift to Z=0.5m")
    current = executor_node.wait_for_current_state()
    target = current.copy()
    target[0] = 0.5
    executor_node.move_with_trapezoidal_profile(target, V_SCALE, A_SCALE)
    time.sleep(0.5)
    
    # PHASE 4: Joint motion at Z=0.5m
    logger.info("\nPHASE 4: Joint motion at Z=0.5m")
    waypoints_z05 = [
        make_waypoint(0.5, 0.2, 0.2, 0.3, 0.3, 0.3, 0.3),
        make_waypoint(0.5, 0.7, 0.2, 0.0, 0.3, 0.0, 0.3),
        make_waypoint(0.5, 0.7, 0.7, 0.0, 0.0, 0.0, 0.0),
        make_waypoint(0.5, 0.2, 0.7, 0.3, 0.3, 0.3, 0.3),
    ]
    for i, wp in enumerate(waypoints_z05):
        logger.info(f"  Waypoint {i+1}/{len(waypoints_z05)} at Z=0.5m")
        executor_node.move_with_trapezoidal_profile(wp, V_SCALE, A_SCALE)
        time.sleep(0.5)
    
    # PHASE 5: Lift to Z=1.0m
    logger.info("\nPHASE 5: Lift to Z=1.0m")
    current = executor_node.wait_for_current_state()
    target = current.copy()
    target[0] = 1.0
    executor_node.move_with_trapezoidal_profile(target, V_SCALE, A_SCALE)
    time.sleep(0.5)
    
    # PHASE 6: Joint motion at Z=1.0m
    logger.info("\nPHASE 6: Joint motion at Z=1.0m")
    waypoints_z1 = [
        make_waypoint(1.0, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2),
        make_waypoint(1.0, 0.7, 0.2, -0.2, 0.2, -0.2, 0.2),
        make_waypoint(1.0, 0.7, -0.3, -0.2, -0.2, -0.2, -0.2),
        make_waypoint(1.0, 0.2, -0.3, 0.2, -0.2, 0.2, -0.2),
    ]
    for i, wp in enumerate(waypoints_z1):
        logger.info(f"  Waypoint {i+1}/{len(waypoints_z1)} at Z=1.0m")
        executor_node.move_with_trapezoidal_profile(wp, V_SCALE, A_SCALE)
        time.sleep(0.5)
    
    # PHASE 7: Return to home
    logger.info("\nPHASE 7: Return to home")
    home = make_waypoint(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    executor_node.move_with_trapezoidal_profile(home, V_SCALE, A_SCALE)
    
    # Save results
    time.sleep(1.0)
    data = executor_node.stop_recording()
    
    logger.info("\nGenerating velocity plots...")
    
    os.makedirs(RESULTS_DIR, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    plot_path = os.path.join(RESULTS_DIR, f"joint_space_motion_{timestamp}.png")
    
    plot_velocities_corrected(data, plot_path)
    
    logger.info(f"\nPlot saved to: {plot_path}")
    
    # Cleanup
    try:
        executor.shutdown()
        executor_node.destroy_node()
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
