#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""ROS2 Deep Learning to Twist Message Launch Script.

This script publishes twist message predicted from deep learning model to 
"\cmd_vel" topic.

Revision History:
        2021-10-01 (ANI717 - Animesh Bala Ani): Baseline Software.

Example:
        $ colcon build --symlink-install && source install/local_setup.bash && ros2 launch ros2_deep_learning_to_twist_message launch.py
        $ source install/local_setup.bash && ros2 launch ros2_deep_learning_to_twist_message launch.py
        $ ros2 launch ros2_deep_learning_to_twist_message launch.py

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
    model_type = LaunchConfiguration('model_type')
    cam2image = LaunchConfiguration('cam2image')
    
    
    # Declare the launch arguments
    declare_model_type_cmd = DeclareLaunchArgument(
        'model_type',
        default_value='onnx',
        description='Type of Model and Runtime to use.')
    
    declare_cam2image_cmd = DeclareLaunchArgument(
        'cam2image',
        default_value='True',
        description='Type of Robot to drive.')
    
    
    # Specify the actions
    execute_cmd = Node(
        package = 'ros2_deep_learning_to_twist_message',
        node_executable = model_type,
        name='deeplearning_to_twist')
    
    cam2image_cmd = Node(
        condition=IfCondition(cam2image),
        package = 'image_tools',
        node_executable = 'cam2image',
        name='cam2image')

        
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_model_type_cmd)
    ld.add_action(declare_cam2image_cmd)
    
    # Add all actions
    ld.add_action(execute_cmd)
    ld.add_action(cam2image_cmd)
        
    return ld


#                                                                              
# end of file
"""ANI717"""