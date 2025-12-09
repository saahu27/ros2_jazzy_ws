"""
Launch file for cartesian motion with trapezoidal velocity profiles.

"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for corrected cartesian motion."""
    
    # This node doesn't need MoveIt configs since it publishes directly
    # to the JointTrajectoryController
    
    cartesian_motion_node = Node(
        package="puma560_py",
        executable="cartesian_motion",
        name="cartesian_motion",
        output="screen",
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        cartesian_motion_node,
    ])

