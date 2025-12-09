import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Get package directories
    puma560_description_dir = get_package_share_directory('puma560_description')
    
    # Build MoveIt config for the execute node
    moveit_config = (
        MoveItConfigsBuilder("puma560", package_name="puma560_description")
        .robot_description(file_path="urdf/puma560_robot.urdf")
        .robot_description_semantic(file_path="config/puma560.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(file_path="config/planning.yaml")
        .to_moveit_configs()
    )
    
    # 1. Launch Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(puma560_description_dir, 'launch', 'gazebo.launch.py')
        )
    )
    
    # 2. Launch Controller (after 10 seconds for Gazebo to initialize)
    controller_launch = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg="Starting controller..."),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(puma560_description_dir, 'launch', 'controller.launch.py')
                )
            )
        ]
    )
    
    # 3. Launch joint_space Motion node (after 15 seconds total - 5 more for controller)
    joint_space_motion_node = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg="Starting joint_space motion execution..."),
            Node(
                package="puma560_py",
                executable="joint_space_motion",
                parameters=[
                    moveit_config.to_dict(),
                    {'use_sim_time': True}
                ],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        LogInfo(msg="Starting Gazebo simulation..."),
        gazebo_launch,
        controller_launch,
        joint_space_motion_node,
    ])

