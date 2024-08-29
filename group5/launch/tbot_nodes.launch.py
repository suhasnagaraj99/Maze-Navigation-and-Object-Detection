"""
Launch file for the tbot nodes
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    aruco_broadcaster = Node(
        package="group5",
        executable="aruco_broadcaster",
        parameters=[{'use_sim_time': True}]
    )
    
    battery_broadcaster = Node(
        package="group5",
        executable="battery_broadcaster",
        parameters=[{'use_sim_time': True}]
    )   
    
    
    battery_listner = Node(
        package="group5",
        executable="battery_listner",
        parameters=[{'use_sim_time': True}]
    ) 
    
    ld = LaunchDescription()
    ld.add_action(aruco_broadcaster)
    ld.add_action(battery_broadcaster)
    ld.add_action(battery_listner)
    return ld