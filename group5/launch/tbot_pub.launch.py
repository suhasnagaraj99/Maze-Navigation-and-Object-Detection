"""
Launch file for the tbot nodes
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Main function for the launch file
    """
    # find the parameter file
    parameter_file = os.path.join(
        get_package_share_directory('group5'),
        'config',
        'params.yaml'
    )
    
    aruco_pub = Node(
        package="group5",
        executable="tbot",
        parameters=[parameter_file]
    )
    
    ld = LaunchDescription()
    ld.add_action(aruco_pub)
    return ld