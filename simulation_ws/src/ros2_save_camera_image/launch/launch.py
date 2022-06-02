#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""ROS2 Package launch File to Save Camera Images.

This script collects image and twist message data from ros2 topics and saves 
annotated images.

Revision History:
        2021-08-26 (Animesh Bala Ani): Baseline Software.

Example:
        $ colcon build --symlink-install && source install/local_setup.bash && ros2 launch ros2_save_camera_image launch.py
        $ source install/local_setup.bash && ros2 launch ros2_save_camera_image launch.py
        $ ros2 launch ros2_save_camera_image launch.py

"""


#___Import Modules:
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


#___Function:
def generate_launch_description():
    
    # Create launch configuration variables
    cam2image = LaunchConfiguration('cam2image')
    
    
    # Declare the launch arguments
    declare_cam2image_cmd = DeclareLaunchArgument(
        'cam2image',
        default_value='True',
        description='Type of Robot to drive.')
    
    
    # Specify the actions
    execute_cmd = Node(
        package = 'ros2_save_camera_image',
        node_executable = 'execute',
        name='save_camera_image')
    
    cam2image_cmd = Node(
        condition=IfCondition(cam2image),
        package = 'image_tools',
        node_executable = 'cam2image',
        name='cam2image')
    
      
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_cam2image_cmd)
    
    # Add all actions
    ld.add_action(execute_cmd)
    ld.add_action(cam2image_cmd)
        
    return ld


#                                                                              
# end of file
"""ANI717"""