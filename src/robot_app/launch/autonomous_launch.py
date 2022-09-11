#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""ROS2 ANI717 Robot App Launch Script.

This script moves Jetbot Autonomously.

Revision History:
        2021-10-01 (ANI717 - Animesh Bala Ani): Baseline Software.

Example:
        $ colcon build --symlink-install && source install/local_setup.bash && ros2 launch robot_app autonomous_launch.py
        $ source install/local_setup.bash && ros2 launch robot_app autonomous_launch.py
        $ ros2 launch robot_app autonomous_launch.py

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
    robot_type = LaunchConfiguration('robot_type')
    cam2image = LaunchConfiguration('cam2image')
    csijetson = LaunchConfiguration('csijetson')
    
    
    # Declare the launch arguments
    declare_model_type_cmd = DeclareLaunchArgument(
        'model_type',
        default_value='onnx',
        description='Type of Model and Runtime to use.')
    
    declare_robot_type_cmd = DeclareLaunchArgument(
        'robot_type',
        default_value='jetbot',
        description='Type of Robot to drive.')
    
    declare_cam2image_cmd = DeclareLaunchArgument(
        'cam2image',
        default_value='False',
        description='Execute cam2image or not.')
    
    declare_csijetson_cmd = DeclareLaunchArgument(
        'csijetson',
        default_value='True',
        description='Using CSI camera on Jetson Nano or not.')
    
    
    # Specify the actions
    deep_learning_to_twist_cmd = Node(
        package = 'ros2_deep_learning_to_twist_message',
        node_executable = model_type,
        name='deeplearning_to_twist')
    
    twist_to_motion_cmd = Node(
        package = 'ros2_twist_message_to_robot_motion',
        node_executable = robot_type,
        name='twist_to_robot_motion')
    
    cam2image_cmd = Node(
        condition=IfCondition(cam2image),
        package = 'image_tools',
        node_executable = 'cam2image',
        name='cam2image')
    
    csijetson_cmd = Node(
        condition=IfCondition(csijetson),
        package = 'ros2_csi_camera_publish',
        node_executable = 'jetson',
        name='csi_camera_publish')

        
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_model_type_cmd)
    ld.add_action(declare_robot_type_cmd)
    ld.add_action(declare_cam2image_cmd)
    ld.add_action(declare_csijetson_cmd)
    
    # Add all actions
    ld.add_action(deep_learning_to_twist_cmd)
    ld.add_action(twist_to_motion_cmd)
    ld.add_action(cam2image_cmd)
    ld.add_action(csijetson_cmd)
        
    return ld


#                                                                              
# end of file
"""ANI717"""
