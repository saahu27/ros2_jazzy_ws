"""
C++ Cartesian Motion Launch File.

Launches the C++ implementation of Cartesian motion with:
1. Gazebo simulation with PUMA560 robot + lift
2. ros2_control controllers (joint_state_broadcaster, arm_controller)
3. MoveIt move_group (provides IK/FK services)
4. C++ Cartesian motion node
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
        default_value="false",
        description="Whether to start RViz"
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
    # =========================================================
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
    # NOTE: robot_state_publisher is DELAYED to allow Gazebo clock to initialize first.
    # This prevents TF2 time jump warnings caused by sim time not being available yet.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True
        }]
    )
    
    # Delay robot_state_publisher by 2s to allow Gazebo clock bridge to initialize
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
    
    # Spawn entity after robot_state_publisher is ready (needs robot_description topic)
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
    
    # Adjusted delays to account for robot_state_publisher delay
    delayed_joint_state_broadcaster = TimerAction(
        period=4.0,
        actions=[joint_state_broadcaster_spawner]
    )
    
    delayed_arm_controller = TimerAction(
        period=5.0,
        actions=[arm_controller_spawner]
    )
    
    # =========================================================
    # MOVEIT
    # =========================================================
    moveit_config = (
        MoveItConfigsBuilder("puma560", package_name="puma560_description")
        .robot_description(file_path=os.path.join(puma560_description, "urdf", "puma560_robot.urdf"))
        .robot_description_semantic(file_path="config/puma560.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )
    
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
            {"publish_robot_description_semantic": True}
        ],
        arguments=["--ros-args", "--log-level", "warn"],
    )
    
    delayed_move_group = TimerAction(
        period=7.0,
        actions=[move_group_node]
    )
    
    # =========================================================
    # RVIZ (optional)
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
    # C++ CARTESIAN MOTION NODE
    # =========================================================
    cartesian_motion_node = Node(
        package="puma560_cpp",
        executable="cartesian_motion",
        name="cartesian_motion",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )
    
    delayed_cartesian_motion = TimerAction(
        period=12.0,
        actions=[cartesian_motion_node]
    )
    
    # =========================================================
    # LAUNCH DESCRIPTION
    # =========================================================
    # Launch sequence with proper timing to avoid TF2 time jump warnings:
    # 0s: Gazebo, gz_ros2_bridge (clock bridge)
    # 2s: robot_state_publisher (after clock is available)
    # 3s: gz_spawn_entity (needs robot_description from RSP)
    # 4s: joint_state_broadcaster
    # 5s: arm_controller
    # 7s: move_group
    # 12s: cartesian_motion application
    return LaunchDescription([
        model_arg,
        use_rviz_arg,
        gazebo_resource_path,
        gazebo,
        gz_ros2_bridge,
        delayed_robot_state_publisher,
        delayed_gz_spawn_entity,
        delayed_joint_state_broadcaster,
        delayed_arm_controller,
        delayed_move_group,
        delayed_rviz,
        delayed_cartesian_motion,
    ])

