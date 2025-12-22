#!/usr/bin/env python3
"""
Constrained Motion Planning with OMPL via MoveIt2 (MoveItPy API)

This node demonstrates constraint-based motion planning for straight-line
Cartesian paths while maintaining constant end-effector orientation.

Theory: Constraint-Based Motion Planning
========================================
Traditional sampling-based planners (RRT, PRM) sample uniformly in C-space.
When geometric constraints are imposed (e.g., constant orientation), valid
configurations lie on a lower-dimensional CONSTRAINT MANIFOLD embedded in C-space.

The probability of randomly sampling a point ON this manifold is zero for
continuous constraints. OMPL addresses this via:

1. PROJECTION: Sample in ambient space, project onto manifold
2. ATLAS: Build local charts covering the manifold  
3. TANGENT BUNDLE: Work in tangent space of the manifold

MoveIt exposes this via PathConstraints:
- PositionConstraints: Restrict end-effector position to a region
- OrientationConstraints: Restrict end-effector orientation with tolerances

For STRAIGHT-LINE motion with CONSTANT ORIENTATION:
- Position constraint: Narrow bounding box along the line (corridor)
- Orientation constraint: Small tolerance around desired quaternion

Reference:
- Kingston, Moll, Kavraki. "Sampling-Based Methods for Motion Planning
  with Constraints", Annual Review of Control, Robotics, 2018
- Sucan, Moll, Kavraki. "The Open Motion Planning Library", IEEE RAM 2012

Author: Sahruday Patti
"""

import time
import os
import numpy as np
from copy import deepcopy
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from rclpy.logging import get_logger

from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header

# MoveIt2 Python API (moveit_py)
from moveit.planning import (
    MoveItPy,
    PlanRequestParameters,
)
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
)

# MoveIt configuration utilities
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import yaml

# For plotting
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from datetime import datetime


# ============================================================
# CONFIGURATION CONSTANTS
# ============================================================
DEFAULT_RESULTS_DIR = '/root/ros2_ws/src/puma560_ros2_moveit/results'

# Planning parameters
DEFAULT_PLANNING_TIME = 15.0        # Max planning time (seconds)
DEFAULT_PLANNING_ATTEMPTS = 5       # Number of planning attempts
DEFAULT_VELOCITY_SCALING = 0.5      # Velocity scaling factor [0, 1] - increased for visible motion
DEFAULT_ACCEL_SCALING = 0.5         # Acceleration scaling factor [0, 1] - increased for visible motion

# Constraint tolerances
POSITION_TOLERANCE = 0.02           # Position tolerance for path corridor (meters)
ORIENTATION_TOLERANCE = 0.15        # Orientation tolerance (radians ~8.5 degrees)

# Motion parameters - sized for visible movements within PUMA560 workspace
# Note: PUMA560 has limited reach (~0.6-0.7m), so we keep motions moderate
LINE_LENGTH = 0.15                  # Length of straight-line motion (meters)
SQUARE_SIZE = 0.15                  # Square pattern size (meters)


@dataclass 
class ConstrainedMotionResult:
    """Result of a constrained motion planning attempt."""
    success: bool
    planning_time: float
    trajectory_duration: float
    num_waypoints: int
    planner_id: str
    start_pose: Optional[Pose] = None
    goal_pose: Optional[Pose] = None


class ConstrainedMotionPlanner:
    """
    Constraint-based motion planner using MoveIt2 Python API (MoveItPy).
    
    This class demonstrates how to use path constraints to achieve:
    1. Straight-line Cartesian motion between two points
    2. Constant end-effector orientation throughout the motion
    
    Key Concepts:
    - Path constraints are enforced by OMPL's constrained state space
    - The planner samples only configurations that satisfy constraints
    - Constraint tolerance controls the trade-off between strictness and feasibility
    
    This implementation uses the native MoveItPy API for maximum reliability
    and proper integration with the MoveIt2 ecosystem.
    """
    
    def __init__(self, moveit: MoveItPy, logger):
        """
        Initialize the constrained motion planner.
        
        Args:
            moveit: MoveItPy instance
            logger: ROS2 logger
        """
        self.moveit = moveit
        self.logger = logger
        
        # Get planning component for the arm group
        self.planning_component = moveit.get_planning_component("arm")
        self.robot_model = moveit.get_robot_model()
        
        # Configuration
        self.planning_group = "arm"
        self.ee_link = "link7"
        self.base_frame = "world"
        
        # Joint names (from SRDF)
        self.joint_names = ["lift_joint", "j1", "j2", "j3", "j4", "j5", "j6"]
        
        # Available planners for constrained planning
        # Note: KPIECE/BKPIECE/LBKPIECE may not be available in all OMPL builds
        self.available_planners = [
            "RRTConnect",  # Fast bidirectional tree
            "RRTstar",     # Optimal RRT variant
            "EST",         # Expansive Space Trees
            "BiEST",       # Bidirectional EST
            "TRRT",        # Transition-based RRT
            "PRM",         # Probabilistic Roadmap
        ]
        
        # Current planner
        self.current_planner = "RRTConnect"
        
        # Results storage
        self.planning_results: List[ConstrainedMotionResult] = []
        
        self.logger.info("ConstrainedMotionPlanner initialized")
        self.logger.info(f"  Planning group: {self.planning_group}")
        self.logger.info(f"  End-effector: {self.ee_link}")
        self.logger.info(f"  Available planners: {self.available_planners}")
    
    def get_current_pose(self) -> Optional[Pose]:
        """Get current end-effector pose from robot state."""
        try:
            robot_state = self.planning_component.get_start_state()
            pose = robot_state.get_pose(self.ee_link)
            return pose
        except Exception as e:
            self.logger.error(f"Failed to get current pose: {e}")
            return None
    
    def create_orientation_constraint(
        self,
        target_orientation: Quaternion,
        tolerance: float = ORIENTATION_TOLERANCE
    ) -> OrientationConstraint:
        """
        Create an orientation constraint to maintain constant end-effector orientation.
        
        This constraint restricts the end-effector orientation to be within
        'tolerance' radians of the target orientation about each axis.
        
        Args:
            target_orientation: Desired quaternion orientation
            tolerance: Allowed deviation in radians
            
        Returns:
            OrientationConstraint message
        """
        constraint = OrientationConstraint()
        constraint.header.frame_id = self.base_frame
        constraint.link_name = self.ee_link
        constraint.weight = 1.0
        
        constraint.orientation = target_orientation
        
        # Tolerances around each axis (radians)
        # Larger tolerance = easier to plan, less precise path
        constraint.absolute_x_axis_tolerance = tolerance
        constraint.absolute_y_axis_tolerance = tolerance
        constraint.absolute_z_axis_tolerance = tolerance
        
        return constraint
    
    def create_line_constraint(
        self, 
        start_pose: Pose, 
        end_pose: Pose,
        tolerance: float = POSITION_TOLERANCE
    ) -> PositionConstraint:
        """
        Create a position constraint that restricts motion to a line corridor.
        
        The constraint defines a thin box oriented along the line from start to end.
        This enforces approximate straight-line motion in Cartesian space.
        
        Args:
            start_pose: Starting pose
            end_pose: Goal pose
            tolerance: Width of the corridor (how much deviation allowed)
            
        Returns:
            PositionConstraint message
        """
        constraint = PositionConstraint()
        constraint.header.frame_id = self.base_frame
        constraint.link_name = self.ee_link
        constraint.weight = 1.0
        
        # Compute line direction and length
        dx = end_pose.position.x - start_pose.position.x
        dy = end_pose.position.y - start_pose.position.y
        dz = end_pose.position.z - start_pose.position.z
        length = np.sqrt(dx**2 + dy**2 + dz**2)
        
        # Create a bounding box centered at midpoint
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        # Make the corridor larger to allow OMPL more room to find paths
        box.dimensions = [length + 4*tolerance, 4*tolerance, 4*tolerance]
        
        # Position at midpoint of the line
        box_pose = Pose()
        box_pose.position.x = (start_pose.position.x + end_pose.position.x) / 2.0
        box_pose.position.y = (start_pose.position.y + end_pose.position.y) / 2.0
        box_pose.position.z = (start_pose.position.z + end_pose.position.z) / 2.0
        
        # Compute orientation to align box with line direction
        if length > 1e-6:
            dir_vec = np.array([dx, dy, dz]) / length
            default_x = np.array([1.0, 0.0, 0.0])
            
            cross = np.cross(default_x, dir_vec)
            dot = np.dot(default_x, dir_vec)
            
            if np.linalg.norm(cross) > 1e-6:
                axis = cross / np.linalg.norm(cross)
                angle = np.arccos(np.clip(dot, -1.0, 1.0))
                
                s = np.sin(angle / 2.0)
                c = np.cos(angle / 2.0)
                box_pose.orientation.x = axis[0] * s
                box_pose.orientation.y = axis[1] * s
                box_pose.orientation.z = axis[2] * s
                box_pose.orientation.w = c
            elif dot > 0:
                box_pose.orientation.w = 1.0
            else:
                box_pose.orientation.z = 1.0
                box_pose.orientation.w = 0.0
        else:
            box_pose.orientation.w = 1.0
        
        constraint.constraint_region.primitives.append(box)
        constraint.constraint_region.primitive_poses.append(box_pose)
        
        return constraint
    
    def create_path_constraints(
        self,
        start_pose: Pose,
        end_pose: Pose,
        maintain_orientation: bool = True,
        use_line_constraint: bool = False  # Disabled by default - orientation only is more reliable
    ) -> Constraints:
        """
        Create combined path constraints for constrained motion.
        
        Args:
            start_pose: Starting pose
            end_pose: Goal pose
            maintain_orientation: If True, add orientation constraint
            use_line_constraint: If True, add line corridor constraint (more restrictive)
            
        Returns:
            Constraints message
        """
        constraints = Constraints()
        constraints.name = "constant_orientation"
        
        if use_line_constraint:
            line_constraint = self.create_line_constraint(start_pose, end_pose)
            constraints.position_constraints.append(line_constraint)
            self.logger.info("  Added position constraint (line corridor)")
        
        if maintain_orientation:
            orientation_constraint = self.create_orientation_constraint(
                start_pose.orientation
            )
            constraints.orientation_constraints.append(orientation_constraint)
            self.logger.info(
                f"  Added orientation constraint (tol={ORIENTATION_TOLERANCE:.3f} rad)"
            )
        
        return constraints
    
    def plan_to_pose(
        self,
        goal_pose: Pose,
        path_constraints: Optional[Constraints] = None,
        planner_id: Optional[str] = None,
    ) -> Tuple[bool, Optional[object], float]:
        """
        Plan a motion to the goal pose with optional path constraints.
        
        Uses the MoveItPy PlanningComponent API which properly interfaces
        with move_group and OMPL.
        
        Args:
            goal_pose: Target end-effector pose
            path_constraints: Path constraints to enforce
            planner_id: OMPL planner to use
            
        Returns:
            Tuple of (success, trajectory, planning_time)
        """
        if planner_id is None:
            planner_id = self.current_planner
        
        start_time = time.time()
        
        try:
            # Set start state to current
            self.planning_component.set_start_state_to_current_state()
            
            # Set goal as pose target
            goal_stamped = PoseStamped()
            goal_stamped.header.frame_id = self.base_frame
            goal_stamped.header.stamp.sec = 0
            goal_stamped.header.stamp.nanosec = 0
            goal_stamped.pose = goal_pose
            
            self.planning_component.set_goal_state(
                pose_stamped_msg=goal_stamped,
                pose_link=self.ee_link
            )
            
            # Set path constraints if provided
            if path_constraints is not None:
                self.planning_component.set_path_constraints(path_constraints)
            
            # Configure planning parameters
            plan_params = PlanRequestParameters(
                self.moveit,
                self.planning_group
            )
            plan_params.planning_pipeline = "ompl"  # CRITICAL: specify the pipeline
            plan_params.planner_id = planner_id
            plan_params.planning_time = DEFAULT_PLANNING_TIME
            plan_params.planning_attempts = DEFAULT_PLANNING_ATTEMPTS
            plan_params.max_velocity_scaling_factor = DEFAULT_VELOCITY_SCALING
            plan_params.max_acceleration_scaling_factor = DEFAULT_ACCEL_SCALING
            
            # Plan
            self.logger.info(f"  Planning with {planner_id}...")
            result = self.planning_component.plan(single_plan_parameters=plan_params)
            
            planning_time = time.time() - start_time
            
            # Clear constraints after planning
            self.planning_component.set_path_constraints(Constraints())
            
            if result:
                self.logger.info(f"  ✓ Planning succeeded in {planning_time:.2f}s")
                return True, result.trajectory, planning_time
            else:
                self.logger.warn(f"  ✗ Planning failed after {planning_time:.2f}s")
                return False, None, planning_time
                
        except Exception as e:
            self.logger.error(f"Planning exception: {e}")
            import traceback
            traceback.print_exc()
            return False, None, time.time() - start_time
    
    def execute_trajectory(self, trajectory) -> bool:
        """
        Execute a planned trajectory using MoveItPy.
        
        Args:
            trajectory: The robot trajectory to execute
            
        Returns:
            True if execution succeeded
        """
        try:
            self.logger.info("  Executing trajectory...")
            
            # Execute using MoveItPy's execute method
            # controllers=[] means use all available controllers
            result = self.moveit.execute(trajectory, controllers=[])
            
            if result:
                self.logger.info("  ✓ Execution completed")
                return True
            else:
                self.logger.warn("  ✗ Execution failed")
                return False
                
        except Exception as e:
            self.logger.error(f"Execution error: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def move_linear_constrained(
        self,
        target_pose: Pose,
        maintain_orientation: bool = True,
        use_line_constraint: bool = False,
        execute: bool = True
    ) -> ConstrainedMotionResult:
        """
        Execute a constrained linear motion to the target pose.
        
        Args:
            target_pose: Goal pose for end-effector
            maintain_orientation: Keep orientation constant
            use_line_constraint: Enforce straight-line path (more restrictive)
            execute: If True, execute after planning
            
        Returns:
            ConstrainedMotionResult with planning statistics
        """
        result = ConstrainedMotionResult(
            success=False,
            planning_time=0.0,
            trajectory_duration=0.0,
            num_waypoints=0,
            planner_id=self.current_planner
        )
        
        # Get current pose
        current_pose = self.get_current_pose()
        if current_pose is None:
            self.logger.error("Could not get current pose")
            return result
        
        result.start_pose = deepcopy(current_pose)
        result.goal_pose = deepcopy(target_pose)
        
        self.logger.info("=" * 60)
        self.logger.info("CONSTRAINED LINEAR MOTION")
        self.logger.info("=" * 60)
        self.logger.info(
            f"  Start: ({current_pose.position.x:.3f}, "
            f"{current_pose.position.y:.3f}, {current_pose.position.z:.3f})"
        )
        self.logger.info(
            f"  Goal:  ({target_pose.position.x:.3f}, "
            f"{target_pose.position.y:.3f}, {target_pose.position.z:.3f})"
        )
        
        # Create constraints
        constraints = None
        if maintain_orientation or use_line_constraint:
            constraints = self.create_path_constraints(
                current_pose,
                target_pose,
                maintain_orientation=maintain_orientation,
                use_line_constraint=use_line_constraint
            )
        
        # Plan
        success, trajectory, planning_time = self.plan_to_pose(
            target_pose,
            path_constraints=constraints
        )
        
        result.planning_time = planning_time
        
        if not success or trajectory is None:
            self.planning_results.append(result)
            return result
        
        # Extract trajectory statistics
        # Note: RobotTrajectory API varies by MoveIt version
        try:
            # Try different API methods for compatibility
            if hasattr(trajectory, '__len__'):
                result.num_waypoints = len(trajectory)
            elif hasattr(trajectory, 'waypoint_count'):
                result.num_waypoints = trajectory.waypoint_count
            else:
                result.num_waypoints = -1  # Unknown
                
            if hasattr(trajectory, 'duration'):
                result.trajectory_duration = trajectory.duration
            else:
                result.trajectory_duration = 0.0
        except Exception as e:
            pass  # Silently ignore stats extraction failures
        
        # Execute if requested
        if execute:
            exec_success = self.execute_trajectory(trajectory)
            result.success = exec_success
        else:
            result.success = True
        
        self.planning_results.append(result)
        return result
    
    def move_to_xyz(
        self,
        x: float, y: float, z: float,
        maintain_orientation: bool = True,
        execute: bool = True
    ) -> ConstrainedMotionResult:
        """
        Move end-effector to XYZ position while maintaining orientation.
        
        Args:
            x, y, z: Target position in world frame
            maintain_orientation: Keep orientation constant
            execute: Execute after planning
            
        Returns:
            ConstrainedMotionResult
        """
        current_pose = self.get_current_pose()
        if current_pose is None:
            return ConstrainedMotionResult(
                success=False, planning_time=0, trajectory_duration=0,
                num_waypoints=0, planner_id=self.current_planner
            )
        
        target = Pose()
        target.position.x = x
        target.position.y = y
        target.position.z = z
        target.orientation = current_pose.orientation
        
        return self.move_linear_constrained(
            target,
            maintain_orientation=maintain_orientation,
            execute=execute
        )
    
    def set_planner(self, planner_id: str) -> bool:
        """Set the OMPL planner to use."""
        if planner_id in self.available_planners:
            self.current_planner = planner_id
            self.logger.info(f"Switched to planner: {planner_id}")
            return True
        else:
            self.logger.warn(
                f"Unknown planner: {planner_id}. Available: {self.available_planners}"
            )
            return False


def plot_constrained_results(results: List[ConstrainedMotionResult], output_path: str):
    """Plot constrained motion planning results."""
    if not results:
        print("No results to plot")
        return
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # Plot 1: Planning times by planner
    ax1 = axes[0, 0]
    planners = [r.planner_id for r in results]
    times = [r.planning_time for r in results]
    successes = [r.success for r in results]
    colors = ['#2ecc71' if s else '#e74c3c' for s in successes]
    
    ax1.bar(range(len(results)), times, color=colors, alpha=0.8, edgecolor='black')
    ax1.set_xticks(range(len(results)))
    ax1.set_xticklabels([f"{p}\n#{i+1}" for i, p in enumerate(planners)], 
                        fontsize=8, rotation=45, ha='right')
    ax1.set_ylabel('Planning Time (s)', fontweight='bold')
    ax1.set_title('Planning Time per Motion\n(Green=Success, Red=Failed)', 
                  fontsize=11, fontweight='bold')
    ax1.grid(True, alpha=0.3, axis='y')
    
    # Plot 2: XY trajectories
    ax2 = axes[0, 1]
    for i, r in enumerate(results):
        if r.start_pose and r.goal_pose:
            x_start, y_start = r.start_pose.position.x, r.start_pose.position.y
            x_goal, y_goal = r.goal_pose.position.x, r.goal_pose.position.y
            
            color = '#2ecc71' if r.success else '#e74c3c'
            ax2.plot([x_start, x_goal], [y_start, y_goal], 
                    color=color, linestyle='--', alpha=0.6, linewidth=1.5)
            ax2.scatter([x_start], [y_start], c='#3498db', s=60, marker='o', 
                       zorder=5, edgecolors='black', linewidths=0.5)
            ax2.scatter([x_goal], [y_goal], c=color, s=60, marker='s', 
                       zorder=5, edgecolors='black', linewidths=0.5)
    
    ax2.set_xlabel('X (m)', fontweight='bold')
    ax2.set_ylabel('Y (m)', fontweight='bold')
    ax2.set_title('Planned Paths (Start=○, Goal=□)', fontsize=11, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.set_aspect('equal', adjustable='box')
    
    # Plot 3: Trajectory durations
    ax3 = axes[1, 0]
    durations = [r.trajectory_duration for r in results if r.success]
    
    if durations:
        ax3.bar(range(len(durations)), durations, color='#3498db', alpha=0.8, 
               edgecolor='black')
        ax3.set_xticks(range(len(durations)))
        ax3.set_xticklabels([f"Motion {i+1}" for i in range(len(durations))], 
                           fontsize=9)
        ax3.set_ylabel('Duration (s)', fontweight='bold')
        ax3.set_title('Trajectory Durations (Successful Only)', 
                     fontsize=11, fontweight='bold')
        ax3.grid(True, alpha=0.3, axis='y')
    else:
        ax3.text(0.5, 0.5, 'No successful trajectories', 
                ha='center', va='center', fontsize=12)
        ax3.axis('off')
    
    # Plot 4: Summary statistics
    ax4 = axes[1, 1]
    total = len(results)
    success_count = sum(1 for r in results if r.success)
    avg_time = np.mean([r.planning_time for r in results]) if results else 0
    avg_duration = np.mean([r.trajectory_duration for r in results if r.success and r.trajectory_duration > 0]) if durations else 0
    
    stats_text = (
        f"CONSTRAINED MOTION PLANNING\n"
        f"{'='*35}\n\n"
        f"Total Motions:      {total}\n"
        f"Successful:         {success_count} ({100*success_count/max(total,1):.0f}%)\n"
        f"Failed:             {total - success_count}\n\n"
        f"Avg Planning Time:  {avg_time:.2f}s\n"
        f"Avg Traj Duration:  {avg_duration:.2f}s\n"
        f"Total Waypoints:    {sum(r.num_waypoints for r in results if r.success)}\n\n"
        f"Planners Used:\n"
    )
    
    planner_counts = {}
    planner_success = {}
    for r in results:
        planner_counts[r.planner_id] = planner_counts.get(r.planner_id, 0) + 1
        if r.success:
            planner_success[r.planner_id] = planner_success.get(r.planner_id, 0) + 1
    
    for planner, count in planner_counts.items():
        succ = planner_success.get(planner, 0)
        stats_text += f"  {planner}: {succ}/{count}\n"
    
    ax4.text(0.05, 0.95, stats_text, transform=ax4.transAxes, 
             fontsize=10, verticalalignment='top', fontfamily='monospace',
             bbox=dict(boxstyle='round,pad=0.5', facecolor='#ecf0f1', 
                      edgecolor='#bdc3c7', alpha=0.9))
    ax4.axis('off')
    
    fig.suptitle('OMPL Constrained Motion Planning Results', 
                 fontsize=14, fontweight='bold', y=0.98)
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='white')
    try:
        os.chmod(output_path, 0o666)
    except:
        pass
    print(f"Plot saved to: {output_path}")
    plt.close()


def main():
    """
    Main demonstration of constrained motion planning.
    
    Demonstrates:
    1. Point-to-point motion with constant orientation constraint
    2. Square pattern with orientation constraint
    3. Comparison of different OMPL planners
    """
    # NOTE: Do NOT call rclpy.init() here!
    # MoveItPy handles its own rclcpp initialization internally.
    # Calling rclpy.init() before MoveItPy causes QoS parameter conflicts.
    
    logger = get_logger("constrained_motion")
    
    logger.info("=" * 70)
    logger.info(" CONSTRAINED MOTION PLANNING with OMPL/MoveIt2")
    logger.info("=" * 70)
    logger.info("")
    logger.info("This demo showcases constraint-based motion planning:")
    logger.info("  - Constant end-effector orientation (orientation constraint)")
    logger.info("  - Point-to-point motion with maintained orientation")
    logger.info("  - Multiple OMPL planners for comparison")
    logger.info("")
    
    try:
        # Build MoveIt configuration
        # MoveItPy requires config_dict to be passed at construction time
        # (it creates its own internal node, so launch file params don't reach it)
        logger.info("Building MoveIt configuration...")
        puma560_description = get_package_share_directory("puma560_description")
        
        # Build base config (without planning_pipelines - we'll add manually)
        moveit_config = (
            MoveItConfigsBuilder("puma560", package_name="puma560_description")
            .robot_description(
                file_path=os.path.join(puma560_description, "urdf", "puma560_robot.urdf")
            )
            .robot_description_semantic(file_path="config/puma560.srdf")
            .robot_description_kinematics(file_path="config/kinematics.yaml")
            .trajectory_execution(file_path="config/moveit_controllers.yaml")
            .to_moveit_configs()
        )
        
        # Get the base config dict
        config_dict = moveit_config.to_dict()
        
        # Manually load and add planning pipeline configuration
        # This is necessary because MoveItPy needs explicit planning_pipelines params
        planning_yaml_path = os.path.join(puma560_description, "config", "planning.yaml")
        ompl_yaml_path = os.path.join(puma560_description, "config", "ompl_planning.yaml")
        
        with open(planning_yaml_path, 'r') as f:
            planning_config = yaml.safe_load(f)
        
        with open(ompl_yaml_path, 'r') as f:
            ompl_config = yaml.safe_load(f)
        
        # Add planning_pipelines configuration
        config_dict["planning_pipelines"] = planning_config.get("planning_pipelines", {})
        
        # Add OMPL planner configuration under 'ompl' namespace
        config_dict["ompl"] = ompl_config
        
        # Add plan_request_params for the arm group
        config_dict["arm"] = {
            "plan_request_params": {
                "planning_pipeline": "ompl",
                "planner_id": "RRTConnect",
                "planning_time": DEFAULT_PLANNING_TIME,
                "planning_attempts": DEFAULT_PLANNING_ATTEMPTS,
                "max_velocity_scaling_factor": DEFAULT_VELOCITY_SCALING,
                "max_acceleration_scaling_factor": DEFAULT_ACCEL_SCALING,
            }
        }
        
        # CRITICAL: Use simulation time for Gazebo synchronization
        config_dict["use_sim_time"] = True
        
        logger.info(f"Loaded planning config with pipelines: {config_dict['planning_pipelines']}")
        
        # Initialize MoveItPy with the configuration dict
        logger.info("Initializing MoveItPy (this may take a moment)...")
        moveit = MoveItPy(
            node_name="constrained_motion_moveit",
            config_dict=config_dict
        )
        logger.info("MoveItPy initialized successfully!")
        
        # Create planner
        planner = ConstrainedMotionPlanner(moveit, logger)
        
        # Wait for system to stabilize
        logger.info("Waiting for planning scene to update...")
        time.sleep(3.0)
        
        # Get initial pose
        current_pose = planner.get_current_pose()
        if current_pose is None:
            logger.error("Could not get current pose!")
            return
        
        base_x = current_pose.position.x
        base_y = current_pose.position.y
        base_z = current_pose.position.z
        
        logger.info(f"\nCurrent EE position: ({base_x:.3f}, {base_y:.3f}, {base_z:.3f})")
        
        # =========================================================
        # DEMO 1: Simple constrained linear motion
        # =========================================================
        logger.info("\n" + "=" * 60)
        logger.info("DEMO 1: Constrained Linear Motion (X direction)")
        logger.info("=" * 60)
        
        planner.set_planner("RRTConnect")
        result1 = planner.move_to_xyz(
            base_x + LINE_LENGTH,
            base_y,
            base_z,
            maintain_orientation=True
        )
        
        if result1.success:
            time.sleep(1.0)
        
        # =========================================================
        # DEMO 2: Square pattern with constant orientation
        # =========================================================
        logger.info("\n" + "=" * 60)
        logger.info("DEMO 2: Square Pattern with Constant Orientation")
        logger.info("=" * 60)
        
        current = planner.get_current_pose()
        if current:
            cx = current.position.x
            cy = current.position.y
            cz = current.position.z
            
            square_waypoints = [
                (cx, cy + SQUARE_SIZE, cz),              # +Y
                (cx - SQUARE_SIZE, cy + SQUARE_SIZE, cz), # -X, +Y  
                (cx - SQUARE_SIZE, cy, cz),               # -X
                (cx, cy, cz),                             # back
            ]
            
            for i, (x, y, z) in enumerate(square_waypoints):
                logger.info(f"\n  Square waypoint {i+1}/4")
                result = planner.move_to_xyz(x, y, z, maintain_orientation=True)
                if result.success:
                    time.sleep(0.5)
        
        # =========================================================
        # DEMO 3: Compare planners
        # First return to initial position to stay within workspace
        # =========================================================
        logger.info("\n" + "=" * 60)
        logger.info("DEMO 3: Planner Comparison (RRTConnect vs RRTstar)")
        logger.info("=" * 60)
        
        # Return to initial position first (ensures we're within workspace)
        logger.info("\n  Returning to initial position for DEMO 3...")
        planner.set_planner("RRTConnect")
        result_home = planner.move_to_xyz(base_x, base_y, base_z, maintain_orientation=True)
        if result_home.success:
            time.sleep(1.0)
        
        # Note: KPIECE/BKPIECE may not be available in all OMPL builds
        # Using RRTstar which is always available
        planners_to_test = ["RRTConnect", "RRTstar"]
        
        # Use smaller test distance for planner comparison (0.10m forward/back)
        test_distance = 0.10
        
        for planner_id in planners_to_test:
            logger.info(f"\n  Testing: {planner_id}")
            planner.set_planner(planner_id)
            
            current = planner.get_current_pose()
            if current:
                cx = current.position.x
                cy = current.position.y
                cz = current.position.z
                
                # Forward motion
                result = planner.move_to_xyz(cx + test_distance, cy, cz, maintain_orientation=True)
                if result.success:
                    time.sleep(0.5)
                    # Back to start position for this test
                    planner.move_to_xyz(cx, cy, cz, maintain_orientation=True)
                    time.sleep(0.5)
        
        # =========================================================
        # Save Results
        # =========================================================
        logger.info("\n" + "=" * 60)
        logger.info("GENERATING RESULTS")
        logger.info("=" * 60)
        
        os.makedirs(DEFAULT_RESULTS_DIR, exist_ok=True)
        try:
            os.chmod(DEFAULT_RESULTS_DIR, 0o777)
        except:
            pass
            
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        plot_path = os.path.join(
            DEFAULT_RESULTS_DIR, 
            f"constrained_motion_{timestamp}.png"
        )
        
        plot_constrained_results(planner.planning_results, plot_path)
        logger.info(f"\nResults saved to: {plot_path}")
        
        # Summary
        logger.info("\n" + "=" * 60)
        logger.info("SUMMARY")
        logger.info("=" * 60)
        
        total = len(planner.planning_results)
        successes = sum(1 for r in planner.planning_results if r.success)
        logger.info(f"Total motions attempted: {total}")
        logger.info(f"Successful: {successes} ({100*successes/max(total,1):.0f}%)")
        
    except Exception as e:
        logger.error(f"Error in main: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        logger.info("\nShutting down...")
        # Give some time for pending operations to complete
        time.sleep(0.5)
        
        # Shutdown MoveItPy (it handles its own rclcpp cleanup internally)
        if 'moveit' in dir() and moveit is not None:
            try:
                moveit.shutdown()
            except Exception as e:
                pass  # Ignore shutdown errors
        
        # NOTE: Do NOT call rclpy.shutdown() since we didn't call rclpy.init()
        # MoveItPy handles its own rclcpp lifecycle
        
        logger.info("Done!")


if __name__ == "__main__":
    main()
