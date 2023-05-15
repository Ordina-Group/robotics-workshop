#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""ROS2 ANI717 Robot App Launch Script.

This script moves Jetbot with a Gamepad and saves Annotated Images.

Revision History:
        2021-10-01 (ANI717 - Animesh Bala Ani): Baseline Software.

Example:
        $ colcon build --symlink-install && source install/local_setup.bash && ros2 launch robot_app gamepad_launch.py
        $ source install/local_setup.bash && ros2 launch robot_app gamepad_launch.py
        $ ros2 launch robot_app gamepad_launch.py

"""


#___Import Modules:
from launch import LaunchDescription
import pathlib
# from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.conditions import LaunchConfigurationEquals
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


#___Function:
def generate_launch_description():
    
    # Create launch configuration variables
    gamepad_type = LaunchConfiguration('gamepad_type')
    robot_type = LaunchConfiguration('robot_type')
    
    
    # Declare the launch arguments
    declare_gamepad_type_cmd = DeclareLaunchArgument(
        'gamepad_type',
        default_value='playstation',
        description='Type of Gamepad to use.')
    
    declare_robot_mode_cmd = DeclareLaunchArgument(
        'robot_mode',
        default_value='livestream',
        description='Livestream or Snapshot mode'
    )
    
    declare_robot_type_cmd = DeclareLaunchArgument(
        'robot_type',
        default_value='jetbot',
        description='Type of Robot to drive.')

    # Include other launch files
    camera_capture = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(pathlib.Path(f"{get_package_share_directory('camera_capture')}/launch/camera_capture.launch.py"))
        ),
    )
    ros_to_api = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(pathlib.Path(f"{get_package_share_directory('ros_to_api')}/launch/livestream.launch.py"))
        ),
    )

    gamepad_to_twist_cmd = Node(
        package = 'ros2_gamepad_to_twist_message',
        executable = gamepad_type,
        name='gamepad_to_twist')
    
    twist_to_motion_cmd = Node(
        package = 'ros2_twist_message_to_robot_motion',
        executable = robot_type,
        name='twist_to_robot_motion')

        
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_gamepad_type_cmd)
    ld.add_action(declare_robot_mode_cmd)
    ld.add_action(declare_robot_type_cmd)
    
    # Add all actions
    ld.add_action(gamepad_to_twist_cmd)
    ld.add_action(camera_capture)
    ld.add_action(ros_to_api)
    ld.add_action(twist_to_motion_cmd)
        
    return ld


#                                                                              
# end of file
"""ANI717"""