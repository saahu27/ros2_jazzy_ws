"""
Constrained Motion Planning Launch File.

This launch file starts the complete system for demonstrating OMPL
constraint-based motion planning via MoveIt2:

1. Gazebo simulation with PUMA560 robot + lift
2. ros2_control controllers (joint_state_broadcaster, arm_controller)
3. MoveIt move_group (with OMPL constrained planners)
4. Constrained motion planning demonstration node

Key Concepts:
- Uses MoveItPy for Python-based motion planning
- Configures OMPL planners for constraint-aware planning
- Demonstrates path constraints (position + orientation)

Usage:
    ros2 launch puma560_py constrained_motion.launch.py
    ros2 launch puma560_py constrained_motion.launch.py use_rviz:=true

Author: Sahruday Patti
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Package directories
    puma560_description = get_package_share_directory("puma560_description")
    ros_gz_sim = get_package_share_directory("ros_gz_sim")
    
    # =========================================================
    # LAUNCH ARGUMENTS
    # =========================================================
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(puma560_description, "urdf", "puma560_robot.urdf"),
        description="Absolute path to robot urdf file"
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        name="use_rviz",
        default_value="true",  # Default to true for visualization
        description="Whether to start RViz"
    )
    
    planner_arg = DeclareLaunchArgument(
        name="planner",
        default_value="RRTConnect",
        description="OMPL planner to use (RRTConnect, KPIECE, EST, etc.)"
    )
    
    # =========================================================
    # ENVIRONMENT SETUP
    # =========================================================
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(puma560_description).parent.resolve())]
    )
    
    ros_distro = os.environ.get("ROS_DISTRO", "humble")
    is_ignition = "True" if ros_distro == "humble" else "False"
    physics_engine = "" if ros_distro == "humble" else "--physics-engine gz-physics-dartsim-plugin"
    
    # =========================================================
    # ROBOT DESCRIPTION
    # Use dedicated constrained motion controller config with:
    # - Velocity interface for feedforward control (same as cartesian_motion)
    # - High PID gains for stable, non-wobbling motion
    # - 1000Hz update rate for minimal tracking lag
    # =========================================================
    robot_description = ParameterValue(
        Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=",
            is_ignition,
            " position_gain:=1000.0",  # High gain for stable control
            " controller_config:=controller_constrained.yaml",
            " use_velocity_interface:=true"  # Enable velocity feedforward
        ]),
        value_type=str
    )
    
    # =========================================================
    # GAZEBO SIMULATION
    # =========================================================
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True
        }]
    )
    
    # Delay robot_state_publisher for clock initialization
    delayed_robot_state_publisher = TimerAction(
        period=2.0,
        actions=[robot_state_publisher_node]
    )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments=[
            ("gz_args", f" -v 4 -r empty.sdf {physics_engine}")
        ]
    )
    
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "puma560_robot"],
    )
    
    delayed_gz_spawn_entity = TimerAction(
        period=3.0,
        actions=[gz_spawn_entity]
    )
    
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]
    )
    
    # =========================================================
    # ROS2 CONTROLLERS
    # =========================================================
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager"
        ],
    )
    
    delayed_joint_state_broadcaster = TimerAction(
        period=4.0,
        actions=[joint_state_broadcaster_spawner]
    )
    
    delayed_arm_controller = TimerAction(
        period=5.0,
        actions=[arm_controller_spawner]
    )
    
    # =========================================================
    # MOVEIT CONFIGURATION
    # Enhanced for constrained planning
    # =========================================================
    moveit_config = (
        MoveItConfigsBuilder("puma560", package_name="puma560_description")
        .robot_description(file_path=os.path.join(puma560_description, "urdf", "puma560_robot.urdf"))
        .robot_description_semantic(file_path="config/puma560.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])  # Explicitly use OMPL pipeline
        .to_moveit_configs()
    )
    
    # Additional planning parameters for constrained motion
    planning_params = {
        "use_sim_time": True,
        "publish_robot_description_semantic": True,
        # Constrained planning parameters
        "planning_scene_monitor_options": {
            "publish_planning_scene": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
        },
    }
    
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            planning_params,
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )
    
    delayed_move_group = TimerAction(
        period=7.0,
        actions=[move_group_node]
    )
    
    # =========================================================
    # RVIZ (with MoveIt plugin for visualization)
    # =========================================================
    rviz_config = os.path.join(puma560_description, "config", "moveit.rviz")
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": True}
        ],
        condition=IfCondition(LaunchConfiguration("use_rviz"))
    )
    
    delayed_rviz = TimerAction(
        period=8.0,
        actions=[rviz_node]
    )
    
    # =========================================================
    # CONSTRAINED MOTION PLANNING NODE
    # MoveItPy loads ALL configs internally via MoveItConfigsBuilder
    # Do NOT pass any parameters - MoveItPy handles use_sim_time too
    # Passing parameters causes QoS conflicts with MoveItPy's internal node
    # =========================================================
    constrained_motion_node = Node(
        package="puma560_py",
        executable="constrained_motion",
        name="constrained_motion",
        output="screen",
        # NO parameters - MoveItPy handles everything internally
    )
    
    # Delay motion node to let MoveIt fully initialize
    # MoveItPy needs move_group to be fully ready before connecting
    delayed_constrained_motion = TimerAction(
        period=18.0,  # Extra time for MoveItPy initialization
        actions=[constrained_motion_node]
    )
    
    # =========================================================
    # LAUNCH DESCRIPTION
    # =========================================================
    # Launch sequence with proper timing:
    # 0s: Gazebo, gz_ros2_bridge (clock bridge)
    # 2s: robot_state_publisher
    # 3s: gz_spawn_entity
    # 4s: joint_state_broadcaster
    # 5s: arm_controller
    # 7s: move_group
    # 8s: rviz (if enabled)
    # 15s: constrained_motion (MoveItPy needs extra init time)
    return LaunchDescription([
        # Arguments
        model_arg,
        use_rviz_arg,
        planner_arg,
        
        # Environment
        gazebo_resource_path,
        
        # Gazebo simulation
        gazebo,
        gz_ros2_bridge,
        delayed_robot_state_publisher,
        delayed_gz_spawn_entity,
        
        # Controllers
        delayed_joint_state_broadcaster,
        delayed_arm_controller,
        
        # MoveIt
        delayed_move_group,
        
        # RViz
        delayed_rviz,
        
        # Constrained motion planning
        delayed_constrained_motion,
    ])

