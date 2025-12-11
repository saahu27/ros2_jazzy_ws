"""
C++ Compliance Control Launch File.

Launches the C++ implementation of compliance control with:
1. Gazebo simulation with wall_world (includes wall for pushing against)
2. ros2_control controllers (joint_state_broadcaster, arm_controller)
3. MoveIt move_group (provides IK/FK services)
4. Force/Torque sensor bridge (Ignition topic to ROS 2 topic)
5. C++ Compliance control node
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
    
    run_controller_arg = DeclareLaunchArgument(
        name="run_controller",
        default_value="true",
        description="Whether to run the compliance controller node"
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
    
    # World file with wall for compliance control
    wall_world_path = os.path.join(puma560_description, "worlds", "wall_world.sdf")
    
    # =========================================================
    # ROBOT DESCRIPTION
    # =========================================================
    # For compliance control: use low position gain (0.1), no velocity interfaces
    robot_description = ParameterValue(
        Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=",
            is_ignition,
            " position_gain:=0.1",
            " controller_config:=controller_compliance.yaml",
            " use_velocity_interface:=false"
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
            ("gz_args", f" -v 4 -r {wall_world_path}")
        ]
    )
    
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "puma560_robot"],
    )
    
    gz_ros2_bridge_clock = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge_clock",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]
    )
    
    # =========================================================
    # FORCE/TORQUE SENSOR BRIDGE
    # =========================================================
    gz_ros2_bridge_ft = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge_ft_sensor",
        arguments=[
            "/ft_sensor@geometry_msgs/msg/Wrench[ignition.msgs.Wrench"
        ],
        output="screen"
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
    
    delayed_move_group = TimerAction(
        period=6.0,
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
    # C++ COMPLIANCE CONTROL NODE
    # =========================================================
    compliance_control_node = Node(
        package="puma560_cpp",
        executable="compliance_control",
        name="compliance_control",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("run_controller"))
    )
    
    delayed_compliance_control = TimerAction(
        period=12.0,
        actions=[compliance_control_node]
    )
    
    # =========================================================
    # LAUNCH DESCRIPTION
    # =========================================================
    return LaunchDescription([
        model_arg,
        use_rviz_arg,
        run_controller_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge_clock,
        gz_ros2_bridge_ft,
        delayed_joint_state_broadcaster,
        delayed_arm_controller,
        delayed_move_group,
        delayed_rviz,
        delayed_compliance_control,
    ])

