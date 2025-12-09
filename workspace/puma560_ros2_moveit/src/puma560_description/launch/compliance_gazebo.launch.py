"""
Launch file for compliance control testing.
Spawns robot and wall in Gazebo with MoveIt and RViz.

Wall position: X = 0.8m (in front of robot)
Robot should extend arm to touch the wall.

Usage:
  ros2 launch puma560_description compliance_gazebo.launch.py

In RViz:
  1. Use "MotionPlanning" panel to drag the interactive marker
  2. Click "Plan & Execute" to move the robot
  3. Position end-effector near the wall (X ≈ 0.75m)
"""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription, 
    SetEnvironmentVariable,
    TimerAction,
    LogInfo
)
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    puma560_description = get_package_share_directory("puma560_description")
    
    # World file with wall
    world_file = os.path.join(puma560_description, "worlds", "wall_world.sdf")

    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(puma560_description, "urdf", "puma560_robot.urdf"),
        description="Absolute path to robot urdf file"
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(puma560_description).parent.resolve())]
    )
    
    ros_distro = os.environ.get("ROS_DISTRO", "humble")
    is_ignition = "True" if ros_distro == "humble" else "False"

    robot_description = ParameterValue(
        Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=",
            is_ignition
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True
        }]
    )

    # Launch Gazebo with our custom world (includes DART physics)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch"),
            "/gz_sim.launch.py"
        ]),
        launch_arguments=[
            ("gz_args", [f" -v 4 -r {world_file}"])
        ]
    )

    # Spawn robot
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "puma560",
            "-x", "0.0",
            "-y", "0.0", 
            "-z", "0.0",
        ],
    )

    # Bridge clock
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ]
    )

    # Controller manager and controllers (delayed to allow Gazebo to start)
    controller_launch = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg="Starting controllers..."),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(puma560_description, 'launch', 'controller.launch.py')
                )
            )
        ]
    )

    # ========================================
    # MoveIt Configuration for motion planning
    # ========================================
    moveit_config = (
        MoveItConfigsBuilder("puma560", package_name="puma560_description")
        .robot_description(file_path=os.path.join(puma560_description, "urdf", "puma560_robot.urdf"))
        .robot_description_semantic(file_path="config/puma560.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # MoveIt move_group node (delayed to start after controllers)
    move_group_node = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg="Starting MoveIt move_group..."),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[
                    moveit_config.to_dict(), 
                    {"use_sim_time": True},
                    {"publish_robot_description_semantic": True}
                ],
            )
        ]
    )

    # RViz with MoveIt plugin (delayed to start after move_group)
    rviz_node = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg="Starting RViz with MoveIt..."),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                    {"use_sim_time": True}
                ],
            )
        ]
    )

    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        LogInfo(msg="=" * 60),
        LogInfo(msg="  COMPLIANCE CONTROL TEST ENVIRONMENT"),
        LogInfo(msg="=" * 60),
        LogInfo(msg="  Wall position: X = 0.8m"),
        LogInfo(msg="  Robot at origin (X = 0)"),
        LogInfo(msg=""),
        LogInfo(msg="  In RViz:"),
        LogInfo(msg="    1. Add 'MotionPlanning' display"),
        LogInfo(msg="    2. Drag interactive marker to position EE"),
        LogInfo(msg="    3. Click 'Plan & Execute'"),
        LogInfo(msg="=" * 60),
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
        controller_launch,
        move_group_node,
        rviz_node,
    ])

