"""
Combined launch file for Joint Space Motion demonstration.

This launch file starts everything needed:
1. Gazebo simulation with the PUMA560 robot + lift
2. ros2_control controllers (joint_state_broadcaster, arm_controller)
3. Joint space motion node with per-joint trapezoidal profiles

NOTE: This demo does NOT require MoveIt (no IK/FK needed) since we're
controlling directly in joint space.

IMPORTANT: Uses velocity command interfaces for proper trapezoidal
velocity profile tracking. The controller uses:
- interpolation_method: "none" (preserves our profile)
- command_interfaces: [position, velocity] (velocity feedforward)
- High PID gains for tight tracking
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
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory


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
        default_value="false",
        description="Whether to start RViz"
    )
    
    use_cpp_arg = DeclareLaunchArgument(
        name="use_cpp",
        default_value="false",
        description="Whether to use C++ implementation instead of Python"
    )
    
    # =========================================================
    # ENVIRONMENT SETUP
    # =========================================================
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(puma560_description).parent.resolve())]
    )
    
    # ROS distro detection for Gazebo compatibility
    ros_distro = os.environ.get("ROS_DISTRO", "humble")
    is_ignition = "True" if ros_distro == "humble" else "False"
    physics_engine = "" if ros_distro == "humble" else "--physics-engine gz-physics-dartsim-plugin"
    
    # =========================================================
    # ROBOT DESCRIPTION
    # =========================================================
    # For trapezoidal velocity tracking: use high position gain, velocity interfaces, and trapezoidal controller config
    # This is CRITICAL for proper per-joint trapezoidal profile execution
    robot_description = ParameterValue(
        Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=",
            is_ignition,
            " position_gain:=1000.0",
            " controller_config:=controller_trapezoidal.yaml",
            " use_velocity_interface:=true"
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
    
    # Delay controllers to let Gazebo start
    delayed_joint_state_broadcaster = TimerAction(
        period=3.0,
        actions=[joint_state_broadcaster_spawner]
    )
    
    delayed_arm_controller = TimerAction(
        period=4.0,
        actions=[arm_controller_spawner]
    )
    
    # =========================================================
    # RVIZ (optional - for visualization)
    # =========================================================
    rviz_config = os.path.join(puma560_description, "rviz", "display.rviz")
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("use_rviz"))
    )
    
    delayed_rviz = TimerAction(
        period=6.0,
        actions=[rviz_node]
    )
    
    # =========================================================
    # JOINT SPACE MOTION NODE (Python or C++ based on use_cpp)
    # =========================================================
    # Python implementation
    joint_space_motion_node_py = Node(
        package="puma560_py",
        executable="joint_space_motion",
        name="joint_space_motion",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=UnlessCondition(LaunchConfiguration("use_cpp"))
    )
    
    # C++ implementation
    joint_space_motion_node_cpp = Node(
        package="puma560_cpp",
        executable="joint_space_motion",
        name="joint_space_motion",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("use_cpp"))
    )
    
    # Delay motion node to let controllers fully initialize
    # Shorter delay than cartesian_motion since we don't need MoveIt
    delayed_joint_space_motion_py = TimerAction(
        period=8.0,
        actions=[joint_space_motion_node_py]
    )
    
    delayed_joint_space_motion_cpp = TimerAction(
        period=8.0,
        actions=[joint_space_motion_node_cpp]
    )
    
    # =========================================================
    # LAUNCH DESCRIPTION
    # =========================================================
    return LaunchDescription([
        # Arguments
        model_arg,
        use_rviz_arg,
        use_cpp_arg,
        
        # Environment
        gazebo_resource_path,
        
        # Gazebo simulation
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
        
        # Controllers
        delayed_joint_state_broadcaster,
        delayed_arm_controller,
        
        # RViz (optional)
        delayed_rviz,
        
        # Joint space motion (Python or C++ based on use_cpp argument)
        delayed_joint_space_motion_py,
        delayed_joint_space_motion_cpp,
    ])
