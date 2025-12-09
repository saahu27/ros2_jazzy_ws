#!/usr/bin/env python3
"""

This implementation uses trapezoidal velocity profiles by:
1. Generating trajectory points
2. Publishing JointTrajectory messages directly to the controller
3. Computing positions from our velocity profile at each timestep

Theory Reference:
- Craig, "Introduction to Robotics", Chapter 7: Trajectory Generation
- Modern Robotics by Kevin Lynch & Frank Park, Chapter 9

Author: Sahruday Patti
"""

import time
import os
import numpy as np
import threading

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from rclpy.action import ActionClient

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
        """
        Compute the trapezoidal profile timing parameters.
        
        Note: If distance is too short to reach v_max,
        we get a triangular profile instead.
        """
        # Time to accelerate from 0 to v_max
        t_accel_full = self.v_max / self.a_max
        
        # Distance covered during full acceleration phase
        d_accel_full = 0.5 * self.a_max * t_accel_full**2
        
        # Check if we can reach v_max (triangular vs trapezoidal)
        if 2 * d_accel_full >= self.distance:
            # TRIANGULAR profile - can't reach v_max
            # Solve: d = 2 * (0.5 * a * t²) => t = sqrt(d/a)
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
        
        # Store phase boundaries
        self.t1 = self.t_accel
        self.t2 = self.t_accel + self.t_const
        self.t3 = self.total_time
    
    def evaluate(self, t):
        """
        Evaluate position, velocity, and acceleration at time t.
        
        This is the key function that generates our trajectory!
        
        Returns:
            (position, velocity, acceleration) tuple
        """
        if t < 0:
            return 0, 0, 0
        
        if t >= self.total_time:
            return self.distance * self.sign, 0, 0
        
        if t <= self.t1:
            # PHASE 1: Acceleration
            # v(t) = a*t
            # p(t) = 0.5*a*t²
            acc = self.a_max
            vel = self.a_max * t
            pos = 0.5 * self.a_max * t**2
            
        elif t <= self.t2:
            # PHASE 2: Constant velocity
            # v(t) = v_peak
            # p(t) = d_accel + v_peak*(t - t1)
            dt = t - self.t1
            acc = 0
            vel = self.v_peak
            d_accel = 0.5 * self.a_max * self.t1**2
            pos = d_accel + self.v_peak * dt
            
        else:
            # PHASE 3: Deceleration
            # v(t) = v_peak - a*(t - t2)
            # p(t) = d_accel + d_const + v_peak*(t-t2) - 0.5*a*(t-t2)²
            dt = t - self.t2
            acc = -self.a_max
            vel = self.v_peak - self.a_max * dt
            d_accel = 0.5 * self.a_max * self.t1**2
            d_const = self.v_peak * self.t_const
            pos = d_accel + d_const + self.v_peak * dt - 0.5 * self.a_max * dt**2
        
        return pos * self.sign, vel * self.sign, acc * self.sign
    
    def get_total_time(self):
        return self.total_time
    
    def __repr__(self):
        return (f"TrapezoidalProfile(d={self.distance:.3f}, v_max={self.v_max:.3f}, "
                f"a_max={self.a_max:.3f}, T={self.total_time:.3f}s)")


class JointTrajectoryExecutor(Node):
    """
    Executes joint trajectories by publishing directly to the controller.

    """
    
    def __init__(self):
        super().__init__('trajectory_executor')
        
        self.joint_names = ["lift_joint", "j1", "j2", "j3", "j4", "j5", "j6"]
        
        # Action client for trajectory execution
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        
        # Subscribe to joint states for current position
        self.current_positions = None
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
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
    
    def _joint_state_callback(self, msg):
        """Store current joint positions and record if enabled."""
        positions = {}
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                positions[name] = msg.position[i]
        
        if len(positions) == len(self.joint_names):
            self.current_positions = np.array([positions[n] for n in self.joint_names])
        
        # Recording
        if self.recording and self.current_positions is not None:
            t = time.time() - self.record_start_time
            self.recorded_times.append(t)
            
            for i, name in enumerate(self.joint_names):
                self.recorded_positions[name].append(self.current_positions[i])
                # Compute velocity from position derivative
                if len(self.recorded_positions[name]) > 1:
                    dt = self.recorded_times[-1] - self.recorded_times[-2]
                    if dt > 0.001:
                        vel = (self.recorded_positions[name][-1] - 
                               self.recorded_positions[name][-2]) / dt
                    else:
                        vel = 0.0
                else:
                    vel = 0.0
                self.recorded_velocities[name].append(vel)
    
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
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout:
                raise RuntimeError("Timeout waiting for joint states")
        return self.current_positions.copy()
    
    def generate_trajectory_with_trapezoidal_profile(
        self, 
        start_joints, 
        end_joints, 
        v_max=0.5,  # rad/s or m/s
        a_max=1.0,  # rad/s² or m/s²
        dt=0.02     # 50 Hz sampling
    ):
        """
        Generate a JointTrajectory using our trapezoidal velocity profile.
        
        Args:
            start_joints: Starting joint positions
            end_joints: Ending joint positions
            v_max: Maximum velocity for the motion
            a_max: Maximum acceleration
            dt: Time step for trajectory sampling
            
        Returns:
            JointTrajectory message ready to execute
        """
        start = np.array(start_joints)
        end = np.array(end_joints)
        delta = end - start
        
        # Compute distance in joint space (Euclidean norm)
        distance = np.linalg.norm(delta)
        
        if distance < 1e-6:
            self.get_logger().warn("Start and end positions are the same")
            return None
        
        # Direction unit vector
        direction = delta / distance
        
        # Create trapezoidal profile for this distance
        profile = TrapezoidalVelocityProfile(distance, v_max, a_max)
        
        self.get_logger().info(f"Trajectory profile: {profile}")
        self.get_logger().info(f"  Distance: {distance:.4f}")
        self.get_logger().info(f"  Duration: {profile.total_time:.3f}s")
        self.get_logger().info(f"  Peak velocity: {profile.v_peak:.4f}")
        
        # Build trajectory message
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        # Sample the profile at regular intervals
        t = 0.0
        while t <= profile.total_time + dt:
            # Evaluate profile at this time
            pos_scalar, vel_scalar, acc_scalar = profile.evaluate(t)
            
            # Scale along direction vector to get joint positions
            joint_positions = start + direction * pos_scalar
            joint_velocities = direction * vel_scalar
            joint_accelerations = direction * acc_scalar
            
            # Create trajectory point
            point = JointTrajectoryPoint()
            point.positions = joint_positions.tolist()
            point.velocities = joint_velocities.tolist()
            point.accelerations = joint_accelerations.tolist()
            
            # Set timestamp
            sec = int(t)
            nanosec = int((t - sec) * 1e9)
            point.time_from_start = Duration(sec=sec, nanosec=nanosec)
            
            trajectory.points.append(point)
            t += dt
        
        # Ensure final point is exactly at goal
        final_point = JointTrajectoryPoint()
        final_point.positions = end.tolist()
        final_point.velocities = [0.0] * 7
        final_point.accelerations = [0.0] * 7
        sec = int(profile.total_time)
        nanosec = int((profile.total_time - sec) * 1e9)
        final_point.time_from_start = Duration(sec=sec, nanosec=nanosec)
        
        # Replace last point with exact final point
        if trajectory.points:
            trajectory.points[-1] = final_point
        
        self.get_logger().info(f"Generated trajectory with {len(trajectory.points)} points")
        
        return trajectory
    
    def execute_trajectory(self, trajectory, wait=True):
        """
        Execute a trajectory using the action interface.
        
        Args:
            trajectory: JointTrajectory message
            wait: Whether to wait for completion
            
        Returns:
            True if successful
        """
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
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("Trajectory goal rejected!")
            return False
        
        self.get_logger().info("Trajectory accepted, executing...")
        
        if wait:
            # Wait for result
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
            result = result_future.result()
            
            if result.result.error_code == 0:
                self.get_logger().info("Trajectory execution SUCCEEDED")
                return True
            else:
                self.get_logger().error(f"Trajectory execution failed: {result.result.error_code}")
                return False
        
        return True
    
    def move_with_trapezoidal_profile(self, target_joints, v_max=0.5, a_max=1.0):
        """
        High-level function to move to target using trapezoidal profile.
        
        Args:
            target_joints: Target joint positions [7]
            v_max: Maximum velocity
            a_max: Maximum acceleration
            
        Returns:
            True if successful
        """
        # Get current position
        current = self.wait_for_current_state()
        
        # Generate trajectory with our profile
        trajectory = self.generate_trajectory_with_trapezoidal_profile(
            current, target_joints, v_max, a_max
        )
        
        # Execute
        return self.execute_trajectory(trajectory)


def smooth_signal(signal, window_size=15):
    """
    Apply moving average smoothing to reduce noise.
    
    Args:
        signal: Input signal array
        window_size: Size of smoothing window
    
    Returns:
        Smoothed signal
    """
    if len(signal) < window_size:
        return signal
    
    # Pad signal to handle edges
    pad_size = window_size // 2
    padded = np.pad(signal, (pad_size, pad_size), mode='edge')
    
    # Moving average
    kernel = np.ones(window_size) / window_size
    smoothed = np.convolve(padded, kernel, mode='valid')
    
    return smoothed


def plot_velocities_corrected(data, output_path):
    """
    Plot joint velocities comparing COMMANDED (ideal trapezoidal) vs MEASURED (actual).
    Showing:
    1. The ideal trapezoidal profile we commanded
    2. The actual measured velocities (with smoothing applied)
    """
    fig, axes = plt.subplots(4, 1, figsize=(16, 14))
    
    # Get time data
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
    
    # =========================================================
    # Plot 1: LIFT JOINT - Commanded vs Measured
    # =========================================================
    ax1 = axes[0]
    
    # Commanded (ideal trapezoidal) - thick line
    if 'commanded_velocities' in data and 'lift_joint' in data['commanded_velocities']:
        cmd_vel = data['commanded_velocities']['lift_joint']
        if len(cmd_vel) > 0:
            ax1.plot(commanded_time[:len(cmd_vel)], cmd_vel, 
                    'b-', linewidth=3, label='COMMANDED (ideal)', alpha=0.9)
    
    # Measured (actual from robot) - thin line with smoothing
    if 'lift_joint' in data['velocities']:
        vel = data['velocities']['lift_joint']
        vel_smooth = smooth_signal(vel, window_size=21)
        ax1.plot(measured_time[:len(vel_smooth)], vel_smooth, 
                'r-', linewidth=1.5, label='MEASURED (smoothed)', alpha=0.7)
    
    ax1.set_ylabel('Velocity (m/s)')
    ax1.set_title('LIFT JOINT: Commanded Trapezoidal vs Measured', fontsize=12, fontweight='bold')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    ax1.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    
    # =========================================================
    # Plot 2: ARM JOINTS - Commanded (ideal trapezoidal)
    # =========================================================
    ax2 = axes[1]
    colors = ['red', 'green', 'blue', 'cyan', 'magenta', 'orange']
    
    for i, jname in enumerate(['j1', 'j2', 'j3', 'j4', 'j5', 'j6']):
        if 'commanded_velocities' in data and jname in data['commanded_velocities']:
            cmd_vel = data['commanded_velocities'][jname]
            if len(cmd_vel) > 0:
                ax2.plot(commanded_time[:len(cmd_vel)], cmd_vel,
                        color=colors[i], linewidth=2, label=jname, alpha=0.8)
    
    ax2.set_ylabel('Velocity (rad/s)')
    ax2.set_title('ARM JOINTS: COMMANDED Velocities (Ideal Trapezoidal Profile)', 
                  fontsize=12, fontweight='bold')
    ax2.legend(loc='upper right', ncol=3)
    ax2.grid(True, alpha=0.3)
    ax2.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    
    # =========================================================
    # Plot 3: ARM JOINTS - Measured (actual, smoothed)
    # =========================================================
    ax3 = axes[2]
    
    for i, jname in enumerate(['j1', 'j2', 'j3', 'j4', 'j5', 'j6']):
        if jname in data['velocities']:
            vel = data['velocities'][jname]
            vel_smooth = smooth_signal(vel, window_size=21)
            ax3.plot(measured_time[:len(vel_smooth)], vel_smooth,
                    color=colors[i], linewidth=1.5, label=jname, alpha=0.8)
    
    ax3.set_ylabel('Velocity (rad/s)')
    ax3.set_title('ARM JOINTS: MEASURED Velocities (Smoothed)', fontsize=12, fontweight='bold')
    ax3.legend(loc='upper right', ncol=3)
    ax3.grid(True, alpha=0.3)
    ax3.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    
    # =========================================================
    # Plot 4: Joint Positions (showing motion profile)
    # =========================================================
    ax4 = axes[3]
    
    # Commanded positions
    if 'commanded_positions' in data and 'lift_joint' in data['commanded_positions']:
        cmd_pos = data['commanded_positions']['lift_joint']
        if len(cmd_pos) > 0:
            ax4.plot(commanded_time[:len(cmd_pos)], cmd_pos,
                    'b-', linewidth=3, label='lift (cmd)', alpha=0.8)
    
    # Measured positions
    if 'lift_joint' in data['positions']:
        pos = data['positions']['lift_joint']
        ax4.plot(measured_time[:len(pos)], pos,
                'b--', linewidth=1, label='lift (meas)', alpha=0.5)
    
    for i, jname in enumerate(['j1', 'j2', 'j3', 'j4', 'j5', 'j6']):
        # Commanded positions (solid line)
        if 'commanded_positions' in data and jname in data['commanded_positions']:
            cmd_pos = data['commanded_positions'][jname]
            if len(cmd_pos) > 0:
                ax4.plot(commanded_time[:len(cmd_pos)], cmd_pos,
                        color=colors[i], linewidth=2, label=f'{jname} (cmd)', alpha=0.8)
        
        # Measured positions (dashed line)
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
    
    # Add overall title
    fig.suptitle('TRAPEZOIDAL VELOCITY PROFILE: Commanded vs Actual', 
                 fontsize=14, fontweight='bold', y=1.02)
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Plot saved to: {output_path}")
    plt.close()


def main():
    """
    Main function demonstrating trapezoidal velocity profile execution.
    """
    rclpy.init()
    
    logger = get_logger("cartesian_motion")
    
    logger.info("=" * 70)
    logger.info(" Trapezoidal Velocity Profile Motion Control")
    logger.info("=" * 70)
    
    # Create executor node
    executor_node = JointTrajectoryExecutor()
    
    # Spin in background thread
    spin_thread = threading.Thread(
        target=lambda: rclpy.spin(executor_node),
        daemon=True
    )
    spin_thread.start()
    
    # Wait for joint states
    logger.info("Waiting for joint states...")
    time.sleep(2.0)
    current = executor_node.wait_for_current_state()
    logger.info(f"Current joints: {current}")
    
    # Start recording
    executor_node.start_recording()
    
    # =========================================================
    # MOTION PARAMETERS - Tuned for VISIBLE trapezoidal profiles
    # =========================================================
    # Theory: For trapezoidal profile, need d > v_max²/a_max
    # For VISIBLE constant velocity phase: d > 2*v_max²/a_max
    #
    # With v_max=0.2, a_max=0.3: d_critical = 0.04/0.3 = 0.13 rad
    # With d > 0.26 rad, we get clear trapezoids
    #
    # Reference: Craig, "Introduction to Robotics", Ch. 7
    V_MAX = 0.2       # Reduced for clearer trapezoidal shape
    A_MAX = 0.3       # Reduced acceleration
    LIFT_V_MAX = 0.15 # Slower for lift (clearer profile)
    LIFT_A_MAX = 0.2
    
    # =========================================================
    # PHASE 1: Move to initial configuration
    # =========================================================
    logger.info("\n" + "=" * 60)
    logger.info("PHASE 1: Moving to initial configuration")
    logger.info("=" * 60)
    
    # [lift, j1, j2, j3, j4, j5, j6]
    # Larger initial motion to show clear trapezoidal profile
    initial_config = np.array([0.0, 0.5, 0.5, 0.0, 0.0, 0.0, 0.0])
    
    logger.info(f"Target: {initial_config}")
    logger.info(f"Using v_max={V_MAX}, a_max={A_MAX}")
    
    success = executor_node.move_with_trapezoidal_profile(
        initial_config, v_max=V_MAX, a_max=A_MAX
    )
    
    if success:
        logger.info("Phase 1 complete!")
    time.sleep(1.0)
    
    # =========================================================
    # PHASE 2: XY motion at Z=0.0m (lift at bottom)
    # =========================================================
    logger.info("\n" + "=" * 60)
    logger.info("PHASE 2: XY motion at Z=0.0m")
    logger.info("=" * 60)
    
    # Waypoints exercising ALL joints (lift, j1, j2, j3, j4, j5, j6)
    # Large spacing ensures clear trapezoidal profiles
    waypoints_z0 = [
        np.array([0.0, 0.3, 0.3, 0.2, 0.2, 0.2, 0.2]),   # Start with all joints
        np.array([0.0, 0.8, 0.3, 0.5, 0.2, 0.2, 0.2]),   # Move j1, j3
        np.array([0.0, 0.8, 0.8, 0.5, 0.5, 0.2, 0.2]),   # Move j2, j4
        np.array([0.0, 0.3, 0.8, 0.2, 0.5, 0.5, 0.5]),   # Move j1, j3, j5, j6
    ]
    
    for i, wp in enumerate(waypoints_z0):
        logger.info(f"  Waypoint {i+1}/{len(waypoints_z0)} at Z=0.0m")
        executor_node.move_with_trapezoidal_profile(wp, v_max=V_MAX, a_max=A_MAX)
        time.sleep(0.5)
    
    # =========================================================
    # PHASE 3: Lift to Z=0.5m with trapezoidal profile
    # =========================================================
    logger.info("\n" + "=" * 60)
    logger.info("PHASE 3: Lift to Z=0.5m (TRAPEZOIDAL LIFT MOTION)")
    logger.info("=" * 60)
    
    current = executor_node.wait_for_current_state()
    lift_target = current.copy()
    lift_target[0] = 0.5  # Lift to 0.5m
    
    logger.info(f"Lift from {current[0]:.3f}m to 0.5m")
    logger.info(f"Using lift v_max={LIFT_V_MAX}, a_max={LIFT_A_MAX}")
    
    executor_node.move_with_trapezoidal_profile(
        lift_target, v_max=LIFT_V_MAX, a_max=LIFT_A_MAX
    )
    time.sleep(1.0)
    
    # =========================================================
    # PHASE 4: XY motion at Z=0.5m
    # =========================================================
    logger.info("\n" + "=" * 60)
    logger.info("PHASE 4: XY motion at Z=0.5m")
    logger.info("=" * 60)
    
    # Waypoints at Z=0.5m exercising all joints
    waypoints_z05 = [
        np.array([0.5, 0.2, 0.2, 0.3, 0.3, 0.3, 0.3]),   # Start position
        np.array([0.5, 0.7, 0.2, 0.0, 0.3, 0.0, 0.3]),   # Move j1, j3, j5
        np.array([0.5, 0.7, 0.7, 0.0, 0.0, 0.0, 0.0]),   # Move j2, j4, j5, j6
        np.array([0.5, 0.2, 0.7, 0.3, 0.3, 0.3, 0.3]),   # Move j1, all wrist
    ]
    
    for i, wp in enumerate(waypoints_z05):
        logger.info(f"  Waypoint {i+1}/{len(waypoints_z05)} at Z=0.5m")
        executor_node.move_with_trapezoidal_profile(wp, v_max=V_MAX, a_max=A_MAX)
        time.sleep(0.5)
    
    # =========================================================
    # PHASE 5: Lift to Z=1.0m
    # =========================================================
    logger.info("\n" + "=" * 60)
    logger.info("PHASE 5: Lift to Z=1.0m (TRAPEZOIDAL LIFT MOTION)")
    logger.info("=" * 60)
    
    current = executor_node.wait_for_current_state()
    lift_target = current.copy()
    lift_target[0] = 1.0
    
    logger.info(f"Lift from {current[0]:.3f}m to 1.0m")
    executor_node.move_with_trapezoidal_profile(
        lift_target, v_max=LIFT_V_MAX, a_max=LIFT_A_MAX
    )
    time.sleep(1.0)
    
    # =========================================================
    # PHASE 6: XY motion at Z=1.0m
    # =========================================================
    logger.info("\n" + "=" * 60)
    logger.info("PHASE 6: XY motion at Z=1.0m (top)")
    logger.info("=" * 60)
    
    # Waypoints at Z=1.0m exercising all joints
    waypoints_z1 = [
        np.array([1.0, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]),   # Start
        np.array([1.0, 0.7, 0.2, -0.2, 0.2, -0.2, 0.2]), # Move j1, j3, j5
        np.array([1.0, 0.7, -0.3, -0.2, -0.2, -0.2, -0.2]), # Move j2, j4, j5, j6
        np.array([1.0, 0.2, -0.3, 0.2, -0.2, 0.2, -0.2]),  # Move j1, alternate wrist
    ]
    
    for i, wp in enumerate(waypoints_z1):
        logger.info(f"  Waypoint {i+1}/{len(waypoints_z1)} at Z=1.0m")
        executor_node.move_with_trapezoidal_profile(wp, v_max=V_MAX, a_max=A_MAX)
        time.sleep(0.5)
    
    # =========================================================
    # PHASE 7: Return to home
    # =========================================================
    logger.info("\n" + "=" * 60)
    logger.info("PHASE 7: Return to home")
    logger.info("=" * 60)
    
    home = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    executor_node.move_with_trapezoidal_profile(home, v_max=V_MAX, a_max=A_MAX)
    
    # =========================================================
    # SAVE RESULTS
    # =========================================================
    time.sleep(1.0)
    data = executor_node.stop_recording()
    
    logger.info("\n" + "=" * 60)
    logger.info("GENERATING VELOCITY PLOTS")
    logger.info("=" * 60)
    
    os.makedirs(RESULTS_DIR, exist_ok=True)
    # Fix permissions for mounted volume (container runs as root, host may be different user)
    try:
        import pwd
        ubuntu_uid = pwd.getpwnam('ubuntu').pw_uid
        ubuntu_gid = pwd.getpwnam('ubuntu').pw_gid
        os.chown(RESULTS_DIR, ubuntu_uid, ubuntu_gid)
    except (KeyError, PermissionError):
        pass  # User doesn't exist or no permission - skip
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    plot_path = os.path.join(RESULTS_DIR, f"trapezoidal_velocities_{timestamp}.png")
    
    plot_velocities_corrected(data, plot_path)
    
    # Fix file permissions for host access
    try:
        os.chown(plot_path, ubuntu_uid, ubuntu_gid)
    except (NameError, PermissionError):
        pass  # Variable not defined or no permission - skip
    
    logger.info(f"\nPlot saved to: {plot_path}")
    
    # Cleanup
    executor_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

