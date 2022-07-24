#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""ROS2 Twist Message to Robot Motion Launch Script.

This script moves a robot car.

Revision History:
        2021-08-26 (Animesh Bala Ani): Baseline Software.

Example:
        $ colcon build --symlink-install && source install/local_setup.bash && ros2 launch ros2_twist_message_to_robot_motion launch.py
        $ source install/local_setup.bash && ros2 launch ros2_twist_message_to_robot_motion launch.py
        $ ros2 launch ros2_twist_message_to_robot_motion launch.py

"""


#___Import Modules:
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


#___Function:
def generate_launch_description():
    
    # Create launch configuration variables
    robot_type = LaunchConfiguration('robot_type')
    
    
    # Declare the launch arguments
    declare_robot_type_cmd = DeclareLaunchArgument(
        'robot_type',
        default_value='jetbot',
        description='Type of Robot to drive.')
    
    
    # Specify the actions
    execute_cmd = Node(
        package = 'ros2_twist_message_to_robot_motion',
        node_executable = robot_type,
        name='twist_to_robot_motion')

        
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_robot_type_cmd)
    
    # Add all actions
    ld.add_action(execute_cmd)
        
    return ld


#                                                                              
# end of file
"""ANI717"""
