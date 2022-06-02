#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""ROS2 Robot Spawn Launch File.

This script spawns URDF robots. 

Revision History:

        2021-10-22 (ANI717 - Animesh Bala Ani): Baseline Software.

Example:
        $ colcon build --symlink-install && source install/local_setup.bash && ros2 launch ros2_robot_simulation spawn.py
        $ source install/local_setup.bash && ros2 launch ros2_robot_simulation spawn.py
        $ ros2 launch ros2_robot_simulation spawn.py

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
    x_pos = LaunchConfiguration('x_pos')
    y_pos = LaunchConfiguration('y_pos')
    z_pos = LaunchConfiguration('z_pos')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')
    urdf = [package_dir, '/models/urdf/', LaunchConfiguration('urdf')]
    
    
    # Declare the launch arguments    
    declare_x_pos_cmd = DeclareLaunchArgument(
        'x_pos',
        default_value='0.0')
    
    declare_y_pos_cmd = DeclareLaunchArgument(
        'y_pos',
        default_value='0.0')
    
    declare_z_pos_cmd = DeclareLaunchArgument(
        'z_pos',
        default_value='0.0')
    
    declare_roll_cmd = DeclareLaunchArgument(
        'roll',
        default_value='0.0')
    
    declare_pitch_cmd = DeclareLaunchArgument(
        'pitch',
        default_value='0.0')
    
    declare_yaw_cmd = DeclareLaunchArgument(
        'yaw',
        default_value='0.0')
    
    declare_urdf_cmd = DeclareLaunchArgument(
        'urdf',
        default_value='jetbot.xml')
    
    
    # Specify the actions
    spawn_robot_cmd = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=[
                '-entity',
                'robot',
                '-file',
                urdf,
                '-x',
                x_pos,
                '-y',
                y_pos,
                '-z',
                z_pos,
                '-R',
                roll,
                '-P',
                pitch,
                '-Y',
                yaw])

    
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_x_pos_cmd)
    ld.add_action(declare_y_pos_cmd)
    ld.add_action(declare_z_pos_cmd)
    ld.add_action(declare_roll_cmd)
    ld.add_action(declare_pitch_cmd)
    ld.add_action(declare_yaw_cmd)
    ld.add_action(declare_urdf_cmd)
    
    # Add all actions
    ld.add_action(spawn_robot_cmd)
        
    return ld


#                                                                              
# end of file
"""ANI717"""
