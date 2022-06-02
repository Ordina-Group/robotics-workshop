#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""ROS2 Robot State Publish Launch File.

This script provides robot states. 

Revision History:

        2021-10-22 (ANI717 - Animesh Bala Ani): Baseline Software.

Example:
        $ colcon build --symlink-install && source install/local_setup.bash && ros2 launch ros2_robot_simulation states.py
        $ source install/local_setup.bash && ros2 launch ros2_robot_simulation states.py
        $ ros2 launch ros2_robot_simulation states.py


"""


#___Import Modules:
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


#___Function:
def generate_launch_description():
	
    # Get the package directory
    package_dir = get_package_share_directory('ros2_robot_simulation')
    
    
    # Create launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf = [package_dir, '/models/urdf/', LaunchConfiguration('urdf')]
    
    
    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_urdf_cmd = DeclareLaunchArgument(
        'urdf',
        default_value='jetbot.xml')
    
    
    # Specify the actions
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state',
        output='screen',
        arguments=[urdf, 'robot_description:=robot_state'])
    
    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='robot_joint_state',
        output='screen',
        arguments=[urdf, 'robot_description:=robot_state'])

        
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_urdf_cmd)
    
    # Add all actions
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_cmd)
        
    return ld


#                                                                              
# end of file
"""ANI717"""
