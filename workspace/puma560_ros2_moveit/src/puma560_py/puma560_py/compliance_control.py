#!/usr/bin/env python3
"""
Position-Based Admittance Control for Constant Force Regulation

Based on the direct position-compliance formulation:
    x_cmd = x_eq + Kf * (F_d - F_meas)

Where:
    x_eq   = equilibrium position (found during contact establishment)
    Kf     = compliance gain (m/N)
    F_d    = desired contact force
    F_meas = measured contact force

Approach:
    - Direct position adjustment based on force error
    - Very stable for position-controlled robots

The robot's position directly determines the contact force.
If force is too low, push forward. If too high, pull back.

Filter Design and Stability:
============================
The low-pass filter for force measurements is a first-order exponential filter:
    y[k] = alpha * x[k] + (1 - alpha) * y[k-1]

The time constant tau relates to alpha and sampling frequency f as:
    tau = 1 / (alpha * f)

The choice of alpha depends on:
1. Contact stiffness
2. Desired settling time
3. Sensor noise level

Author: Sahruday Patti
"""

import os
import time
import threading
import numpy as np
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped, Wrench
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration

from moveit_msgs.srv import GetPositionFK, GetPositionIK
from moveit_msgs.msg import MoveItErrorCodes

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ============================================================
# CONFIGURATION
# ============================================================

PRE_CONTACT_JOINTS = {
    'lift_joint': 0.3456,
    'j1': 0.0,
    'j2': 0.4298,
    'j3': 0.8268,
    'j4': 0.0,
    'j5': -0.3077,
    'j6': 0.0,
}

# Position-based admittance parameters
F_DESIRED = 100.0       # Target contact force (N)
KF_COMPLIANCE = 0.00003 # Compliance gain (m/N) - 0.03mm per Newton error
CONTROL_HZ = 50.0       # Control rate (Hz)
FILTER_ALPHA = 0.005    # VERY heavy filtering (tau ≈ 4s at 50Hz)

# Two-zone deadband for steady state
DEADBAND_INNER = 8.0    # +/-8N: True steady state (92-108N)
DEADBAND_OUTER = 20.0   # 8-20N below target: Slow correction zone
DEADBAND_HIGH = 25.0    # Only retract if F > 125N

# Two-level correction rates (gain scheduling)
CORRECTION_FAST = 0.0001   # Fast: 0.1mm/s for large errors (F < 80N)
CORRECTION_SLOW = 0.00005  # Slow: 0.05mm/s for small errors (80-92N)

# Contact establishment - wait until near target force
APPROACH_VEL = 0.002    # Approach velocity (m/s) = 2mm/s
CONTACT_THRESHOLD = 80.0 # Wait until force is near target

WALL_X = 0.87           # Wall surface position (m)
DURATION = 30.0         # Total duration (s)


# Results directory - mounted to host for persistence
# This path is bind-mounted to ${localWorkspaceFolder}/results on the host
DEFAULT_RESULTS_DIR = '/root/ros2_ws/src/puma560_ros2_moveit/results'


class PositionBasedAdmittance(Node):
    """
    Position-based admittance controller for constant force regulation.

    """
    
    def __init__(self):
        super().__init__('compliance_control')
        
        self.joint_names = ['lift_joint', 'j1', 'j2', 'j3', 'j4', 'j5', 'j6']
        
        # ============================================================
        # DECLARE ROS2 PARAMETERS
        # ============================================================
        self._declare_parameters()
        
        # MoveIt services
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        # Trajectory action
        self.traj_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory'
        )
        
        # State
        self.current_joints = None
        self.create_subscription(JointState, '/joint_states', self._joint_cb, 10)
        
        # Force sensor
        self.raw_force = 0.0
        self.filtered_force = 0.0
        self.force_offset = 0.0
        self.create_subscription(Wrench, '/ft_sensor', self._force_cb, 10)
        
        # Data recording
        self.data = {
            'time': [], 'raw_force': [], 'filtered_force': [], 'contact_force': [],
            'x_actual': [], 'x_cmd': [], 'x_eq': [], 'force_error': []
        }
        
        self._log_config()
    
    def _declare_parameters(self):
        """
        Declare all ROS2 parameters with descriptions and constraints.
        
        """
        # Force control parameters
        self.declare_parameter(
            'f_desired', 
            F_DESIRED,
            ParameterDescriptor(
                description='Target contact force in Newtons',
                floating_point_range=[FloatingPointRange(
                    from_value=0.0, to_value=500.0, step=0.0
                )]
            )
        )
        
        self.declare_parameter(
            'kf_compliance',
            KF_COMPLIANCE,
            ParameterDescriptor(
                description='Compliance gain in m/N (position change per Newton error)'
            )
        )
        
        self.declare_parameter(
            'control_hz',
            CONTROL_HZ,
            ParameterDescriptor(
                description='Control loop frequency in Hz'
            )
        )
        
        self.declare_parameter(
            'filter_alpha',
            FILTER_ALPHA,
            ParameterDescriptor(
                description='Force filter coefficient (0-1). Lower = more filtering. '
                           'Time constant tau = 1/(alpha*control_hz)'
            )
        )
        
        # Deadband parameters
        self.declare_parameter(
            'deadband_inner',
            DEADBAND_INNER,
            ParameterDescriptor(description='Inner deadband (steady-state tolerance) in N')
        )
        
        self.declare_parameter(
            'deadband_outer',
            DEADBAND_OUTER,
            ParameterDescriptor(description='Outer deadband (slow correction zone) in N')
        )
        
        self.declare_parameter(
            'deadband_high',
            DEADBAND_HIGH,
            ParameterDescriptor(description='High force threshold for retraction in N')
        )
        
        # Correction rate parameters
        self.declare_parameter(
            'correction_fast',
            CORRECTION_FAST,
            ParameterDescriptor(description='Fast correction rate in m/s')
        )
        
        self.declare_parameter(
            'correction_slow',
            CORRECTION_SLOW,
            ParameterDescriptor(description='Slow correction rate in m/s')
        )
        
        # Approach parameters
        self.declare_parameter(
            'approach_vel',
            APPROACH_VEL,
            ParameterDescriptor(description='Approach velocity in m/s')
        )
        
        self.declare_parameter(
            'contact_threshold',
            CONTACT_THRESHOLD,
            ParameterDescriptor(description='Force threshold to detect contact in N')
        )
        
        # Environment parameters
        self.declare_parameter(
            'wall_x',
            WALL_X,
            ParameterDescriptor(description='Wall surface X position in m')
        )
        
        self.declare_parameter(
            'duration',
            DURATION,
            ParameterDescriptor(description='Total control duration in seconds')
        )
        
        self.declare_parameter(
            'results_dir',
            DEFAULT_RESULTS_DIR,
            ParameterDescriptor(
                description='Directory to save result plots. Use absolute path for Docker persistence.'
            )
        )
    
    def _get_param(self, name):
        """Get a parameter value by name."""
        return self.get_parameter(name).value
    
    def _log_config(self):
        """Log the current parameter configuration."""
        f_desired = self._get_param('f_desired')
        kf = self._get_param('kf_compliance')
        alpha = self._get_param('filter_alpha')
        control_hz = self._get_param('control_hz')
        
        # Calculate filter time constant for reference
        tau = 1.0 / (alpha * control_hz) if alpha > 0 else float('inf')
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("POSITION-BASED ADMITTANCE CONTROL")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"  Control law: x_cmd = x_eq + Kf*(F_d - F)")
        self.get_logger().info(f"  F_desired = {f_desired} N")
        self.get_logger().info(f"  Kf = {kf} m/N = {kf*1000:.4f} mm/N")
        self.get_logger().info(f"  Filter α = {alpha} (tau ≈ {tau:.2f}s at {control_hz}Hz)")
        self.get_logger().info(f"  Deadbands: inner=±{self._get_param('deadband_inner')}N, "
                              f"outer={self._get_param('deadband_outer')}N")
        self.get_logger().info("=" * 60)
    
    def _joint_cb(self, msg):
        joints = {}
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                joints[name] = msg.position[i]
        if len(joints) == 7:
            self.current_joints = joints
    
    def _force_cb(self, msg):
        self.raw_force = msg.force.x
        alpha = self._get_param('filter_alpha')
        self.filtered_force = (alpha * msg.force.x + 
                               (1 - alpha) * self.filtered_force)
    
    def get_contact_force(self):
        """Get calibrated contact force."""
        return self.filtered_force - self.force_offset
    
    def calibrate_force(self):
        """Calibrate force sensor in free space."""
        self.get_logger().info("Calibrating force sensor...")
        time.sleep(0.5)
        
        readings = []
        for _ in range(50):
            readings.append(self.filtered_force)
            time.sleep(0.04)
        
        self.force_offset = np.mean(readings)
        self.get_logger().info(f"  Offset = {self.force_offset:.2f} N")
    
    def wait_ready(self, timeout=30.0):
        self.fk_client.wait_for_service(timeout_sec=timeout)
        self.ik_client.wait_for_service(timeout_sec=timeout)
        self.traj_client.wait_for_server(timeout_sec=timeout)
        
        t0 = time.time()
        while self.current_joints is None and (time.time() - t0) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        return self.current_joints is not None
    
    def compute_fk(self, joints):
        req = GetPositionFK.Request()
        req.header.frame_id = 'world'
        req.fk_link_names = ['link7']
        req.robot_state.joint_state.name = self.joint_names
        req.robot_state.joint_state.position = [joints[n] for n in self.joint_names]
        
        future = self.fk_client.call_async(req)
        
        # Wait for future with timeout (executor handles spinning)
        timeout = 2.0
        start_wait = time.time()
        while not future.done() and (time.time() - start_wait) < timeout:
            time.sleep(0.01)
        
        if future.done() and future.result() and future.result().error_code.val == MoveItErrorCodes.SUCCESS:
            return future.result().pose_stamped[0].pose
        return None
    
    def compute_ik(self, pose, seed):
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'arm'
        
        ps = PoseStamped()
        ps.header.frame_id = 'world'
        ps.pose = pose
        req.ik_request.pose_stamped = ps
        
        req.ik_request.robot_state.joint_state.name = self.joint_names
        req.ik_request.robot_state.joint_state.position = [seed[n] for n in self.joint_names]
        req.ik_request.timeout = Duration(sec=0, nanosec=100000000)
        
        future = self.ik_client.call_async(req)
        
        # Wait for future with timeout (executor handles spinning)
        timeout = 1.0
        start_wait = time.time()
        while not future.done() and (time.time() - start_wait) < timeout:
            time.sleep(0.01)
        
        if future.done() and future.result() and future.result().error_code.val == MoveItErrorCodes.SUCCESS:
            result = {}
            resp = future.result()
            for name in self.joint_names:
                idx = resp.solution.joint_state.name.index(name)
                result[name] = resp.solution.joint_state.position[idx]
            return result
        return None
    
    def send_joint_cmd(self, joints, duration=0.05):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        
        pt = JointTrajectoryPoint()
        pt.positions = [joints[n] for n in self.joint_names]
        pt.time_from_start = Duration(sec=0, nanosec=int(duration * 1e9))
        traj.points.append(pt)
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        self.traj_client.send_goal_async(goal)
    
    def move_to_start(self):
        """Move to pre-contact position."""
        self.get_logger().info("Moving to start position...")
        
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        
        pt = JointTrajectoryPoint()
        pt.positions = [PRE_CONTACT_JOINTS[n] for n in self.joint_names]
        pt.time_from_start = Duration(sec=4, nanosec=0)
        traj.points.append(pt)
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        
        future = self.traj_client.send_goal_async(goal)
        
        # Wait for goal acceptance with timeout
        timeout = 10.0
        start_wait = time.time()
        while not future.done() and (time.time() - start_wait) < timeout:
            time.sleep(0.01)
        
        if future.done() and future.result():
            result_future = future.result().get_result_async()
            
            # Wait for trajectory execution
            start_wait = time.time()
            while not result_future.done() and (time.time() - start_wait) < timeout:
                time.sleep(0.05)
        
        time.sleep(1.0)
        self.get_logger().info("At start position")
    
    def run_control(self):
        """
        Main control loop with two phases:
        1. Approach: Move toward wall until contact
        2. Regulate: Use position-based admittance to maintain force
        
        """
        # Get all parameters at start for efficiency
        f_desired = self._get_param('f_desired')
        kf_compliance = self._get_param('kf_compliance')
        control_hz = self._get_param('control_hz')
        deadband_inner = self._get_param('deadband_inner')
        deadband_outer = self._get_param('deadband_outer')
        deadband_high = self._get_param('deadband_high')
        correction_fast = self._get_param('correction_fast')
        correction_slow = self._get_param('correction_slow')
        approach_vel = self._get_param('approach_vel')
        contact_threshold = self._get_param('contact_threshold')
        wall_x = self._get_param('wall_x')
        duration = self._get_param('duration')
        
        # Get starting pose
        pose = self.compute_fk(self.current_joints)
        if not pose:
            self.get_logger().error("FK failed")
            return
        
        x_actual = pose.position.x
        y_fixed = pose.position.y
        z_fixed = pose.position.z
        orientation = pose.orientation
        seed = self.current_joints.copy()
        
        self.get_logger().info("")
        self.get_logger().info(f"Start X: {x_actual*1000:.1f}mm, Wall: {wall_x*1000:.0f}mm")
        self.get_logger().info(f"Gap: {(wall_x - x_actual)*1000:.1f}mm")
        self.get_logger().info("")
        
        dt = 1.0 / control_hz
        x_cmd = x_actual
        x_equilibrium = None  # Will be set when contact is established
        
        # Position limits
        x_min = x_actual - 0.01
        x_max = wall_x + 0.02
        
        # Clear data
        self.data = {k: [] for k in self.data}
        
        t_start = time.time()
        last_log = -1.0
        phase = "APPROACH"
        correction = 0.0  # Initialize for logging
        
        while (time.time() - t_start) < duration:
            t = time.time() - t_start
            loop_start = time.time()
            
            # Get actual position
            if self.current_joints:
                p = self.compute_fk(self.current_joints)
                if p:
                    x_actual = p.position.x
            
            # Get force
            F = self.get_contact_force()
            force_error = f_desired - F
            
            # ========================================
            # PHASE 1: APPROACH (until contact)
            # ========================================
            if phase == "APPROACH":
                if F > contact_threshold:
                    # Contact established! Record equilibrium position
                    x_equilibrium = x_actual
                    phase = "REGULATE"
                    self.get_logger().info("")
                    self.get_logger().info("=" * 40)
                    self.get_logger().info(f"CONTACT! x_eq = {x_equilibrium*1000:.2f}mm")
                    self.get_logger().info("Starting force regulation...")
                    self.get_logger().info("=" * 40)
                else:
                    # Slowly approach wall
                    x_cmd = x_actual + approach_vel * dt
            
            # ========================================
            # PHASE 2: REGULATE (two-zone control with gain scheduling)
            # ========================================
            elif phase == "REGULATE":
                if abs(force_error) < deadband_inner:
                    # INNER ZONE: True steady state - do nothing
                    correction = 0.0
                    x_cmd = x_actual
                elif force_error > deadband_outer:
                    # LARGE ERROR (F < target - outer): Fast correction
                    x_equilibrium += correction_fast * dt
                    correction = correction_fast * dt
                    x_cmd = x_equilibrium
                elif force_error > deadband_inner:
                    # OUTER ZONE: Slow correction
                    x_equilibrium += correction_slow * dt
                    correction = correction_slow * dt
                    x_cmd = x_equilibrium
                elif force_error < -deadband_high:
                    # ABOVE TARGET (F > target + high): Retract
                    correction = kf_compliance * force_error
                    x_cmd = x_equilibrium + correction
                else:
                    # Between inner and high above target: Hold (conservative)
                    correction = 0.0
                    x_cmd = x_actual
            
            # Clamp position
            x_cmd = np.clip(x_cmd, x_min, x_max)
            
            # Send command
            target_pose = Pose()
            target_pose.position.x = x_cmd
            target_pose.position.y = y_fixed
            target_pose.position.z = z_fixed
            target_pose.orientation = orientation
            
            new_joints = self.compute_ik(target_pose, seed)
            if new_joints:
                self.send_joint_cmd(new_joints, dt * 0.9)
                seed = new_joints
            
            # Record data
            self.data['time'].append(t)
            self.data['raw_force'].append(self.raw_force)
            self.data['filtered_force'].append(self.filtered_force)
            self.data['contact_force'].append(F)
            self.data['x_actual'].append(x_actual)
            self.data['x_cmd'].append(x_cmd)
            self.data['x_eq'].append(x_equilibrium if x_equilibrium else x_actual)
            self.data['force_error'].append(force_error)
            
            # Log
            if t - last_log >= 1.0:
                if phase == "APPROACH":
                    self.get_logger().info(f"t={t:5.1f}s [APPROACH] F={F:5.1f}N X={x_actual*1000:.2f}mm")
                else:
                    self.get_logger().info(
                        f"t={t:5.1f}s [REGULATE] F={F:6.1f}N (err={force_error:+5.1f}) "
                        f"X={x_actual*1000:.2f}mm corr={correction*1000:+.2f}mm"
                    )
                last_log = t
            
            # Timing
            elapsed = time.time() - loop_start
            if elapsed < dt:
                time.sleep(dt - elapsed)
        
        self.get_logger().info("")
        self.get_logger().info("Control complete")
    
    def plot_results(self):
        """Generate and save plots of the control results."""
        if len(self.data['time']) < 10:
            return
        
        # Get parameter values for plotting
        f_desired = self._get_param('f_desired')
        deadband_inner = self._get_param('deadband_inner')
        deadband_outer = self._get_param('deadband_outer')
        wall_x = self._get_param('wall_x')
        
        fig, axes = plt.subplots(4, 1, figsize=(14, 12))
        t = np.array(self.data['time'])
        
        # Plot 1: Raw vs filtered force
        ax = axes[0]
        ax.plot(t, self.data['raw_force'], 'b-', lw=0.5, alpha=0.3, label='Raw')
        ax.plot(t, self.data['filtered_force'], 'g-', lw=1.5, label='Filtered')
        ax.axhline(self.force_offset, color='r', ls='--', label=f'Offset={self.force_offset:.1f}')
        ax.set_ylabel('Force (N)')
        ax.set_title('FT SENSOR READINGS')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
        
        # Plot 2: Contact force tracking
        ax = axes[1]
        ax.plot(t, self.data['contact_force'], 'b-', lw=1.5, label='Contact Force')
        ax.axhline(f_desired, color='r', ls='--', lw=2, label=f'Target={f_desired}N')
        ax.axhline(0, color='k', ls='-', lw=0.5)
        ax.fill_between(t, f_desired-deadband_inner, f_desired+deadband_inner, 
                       alpha=0.3, color='green', label=f'Inner ±{deadband_inner}N')
        ax.fill_between(t, f_desired-deadband_outer, f_desired-deadband_inner, 
                       alpha=0.15, color='yellow', label='Outer zone')
        ax.set_ylabel('Force (N)')
        ax.set_title('CONTACT FORCE TRACKING')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
        
        # Plot 3: Position
        ax = axes[2]
        ax.plot(t, np.array(self.data['x_actual'])*1000, 'g-', lw=1.5, label='Actual X')
        ax.plot(t, np.array(self.data['x_cmd'])*1000, 'b--', lw=1, alpha=0.7, label='Commanded X')
        ax.plot(t, np.array(self.data['x_eq'])*1000, 'r:', lw=1, label='Equilibrium')
        ax.axhline(wall_x*1000, color='k', ls='--', lw=1, alpha=0.5, label=f'Wall={wall_x*1000:.0f}mm')
        ax.set_ylabel('Position (mm)')
        ax.set_title('END-EFFECTOR X POSITION')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
        
        # Plot 4: Force error
        ax = axes[3]
        ax.plot(t, self.data['force_error'], 'm-', lw=1.5)
        ax.axhline(0, color='k', ls='-', lw=1)
        ax.fill_between(t, -deadband_inner, deadband_inner, alpha=0.3, color='green', 
                       label=f'Inner ±{deadband_inner}N')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Force Error (N)')
        ax.set_title('FORCE ERROR (F_d - F_meas)')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
        
        fig.suptitle(f'TWO-ZONE FORCE CONTROL: F_d={f_desired}N, Inner=±{deadband_inner}N, Outer={deadband_outer}N', 
                     fontsize=12, fontweight='bold')
        plt.tight_layout()
        
        results_dir = self._get_param('results_dir')
        os.makedirs(results_dir, exist_ok=True)
        os.chmod(results_dir, 0o777)  # Make directory deletable by any user
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(results_dir, f"admittance_{timestamp}.png")
        plt.savefig(path, dpi=150, bbox_inches='tight')
        os.chmod(path, 0o666)  # Make file deletable by any user (Docker permission fix)
        self.get_logger().info(f"Plot saved: {path}")
        plt.close()


def main():
    """
    Main entry point for compliance control.
    
    Uses MultiThreadedExecutor for proper thread safety when making
    service calls while the node is spinning.

    """
    rclpy.init()
    
    logger = get_logger("compliance_control")
    
    controller = PositionBasedAdmittance()
    
    # Use MultiThreadedExecutor for thread safety
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(controller)
    
    # Spin executor in background thread
    def spin_executor():
        try:
            executor.spin()
        except Exception:
            pass
    
    spin_thread = threading.Thread(target=spin_executor, daemon=True)
    spin_thread.start()
    
    f_desired = controller._get_param('f_desired')
    logger.info("=" * 60)
    logger.info(" POSITION-BASED ADMITTANCE CONTROL")
    logger.info(f" Maintain constant {f_desired}N force against wall")
    logger.info("=" * 60)
    
    if not controller.wait_ready():
        logger.error("Services not ready")
        executor.shutdown()
        rclpy.shutdown()
        return
    
    controller.move_to_start()
    controller.calibrate_force()
    
    time.sleep(0.5)
    controller.run_control()
    controller.plot_results()
    
    logger.info("Done.")
    try:
        executor.shutdown()
        controller.destroy_node()
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()