"""
Combined launch file for Cartesian Motion demonstration.

launch file starts everything needed:
1. Gazebo simulation with the PUMA560 robot + lift
2. ros2_control controllers (joint_state_broadcaster, arm_controller)
3. MoveIt move_group (provides IK/FK services)
4. True Cartesian motion node

"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
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
    
    # ROS distro detection for Gazebo compatibility
    ros_distro = os.environ.get("ROS_DISTRO", "humble")
    is_ignition = "True" if ros_distro == "humble" else "False"
    physics_engine = "" if ros_distro == "humble" else "--physics-engine gz-physics-dartsim-plugin"
    
    # =========================================================
    # ROBOT DESCRIPTION
    # =========================================================
    # For trapezoidal velocity tracking: use high position gain, velocity interfaces, and trapezoidal controller config
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
    
    # Delay MoveIt to let controllers start
    delayed_move_group = TimerAction(
        period=6.0,
        actions=[move_group_node]
    )
    
    # =========================================================
    # RVIZ
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
    # CARTESIAN MOTION NODE
    # =========================================================
    cartesian_motion_node = Node(
        package="puma560_py",
        executable="cartesian_motion",
        name="cartesian_motion",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )
    
    # Delay motion node to let MoveIt fully initialize
    delayed_cartesian_motion = TimerAction(
        period=12.0,
        actions=[cartesian_motion_node]
    )
    
    # =========================================================
    # LAUNCH DESCRIPTION
    # =========================================================
    return LaunchDescription([
        # Arguments
        model_arg,
        use_rviz_arg,
        
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
        
        # MoveIt
        delayed_move_group,
        
        # RViz 
        delayed_rviz,
        
        # Cartesian motion
        delayed_cartesian_motion,
    ])
