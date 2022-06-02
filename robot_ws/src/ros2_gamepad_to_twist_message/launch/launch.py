#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""ROS2 Gamepad to Twist Message Launch Script.

This script publishes gamepad controller data to "\cmd_vel" topic as Twist 
message.

Revision History:
        2021-10-01 (ANI717 - Animesh Bala Ani): Baseline Software.

Example:
        $ colcon build --symlink-install && source install/local_setup.bash && ros2 launch ros2_gamepad_to_twist_message launch.py
        $ source install/local_setup.bash && ros2 launch ros2_gamepad_to_twist_message launch.py
        $ ros2 launch ros2_gamepad_to_twist_message launch.py

"""


#___Import Modules:
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


#___Function:
def generate_launch_description():
    
    # Create launch configuration variables
    gamepad_type = LaunchConfiguration('gamepad_type')
    
    
    # Declare the launch arguments
    declare_gamepad_type_cmd = DeclareLaunchArgument(
        'gamepad_type',
        default_value='logitech',
        description='Type of Gamepad to use.')
    
    
    # Specify the actions
    execute_cmd = Node(
        package = 'ros2_gamepad_to_twist_message',
        node_executable = gamepad_type,
        name='gamepad_to_twist')

        
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_gamepad_type_cmd)
    
    # Add all actions
    ld.add_action(execute_cmd)
        
    return ld


#                                                                              
# end of file
"""ANI717"""
