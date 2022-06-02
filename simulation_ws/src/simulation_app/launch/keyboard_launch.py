#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""ROS2 Twist to Robot Motion Launch Script.

This script moves Jetbot.

Revision History:
        2021-08-26 (ANI717 - Animesh Bala Ani): Baseline Software.

Example:
        $ colcon build --symlink-install && . install/setup.bash && ros2 launch simulation_app keyboard_launch.py
        $ . install/setup.bash && ros2 launch simulation_app keyboard_launch.py
        $ ros2 launch simulation_app keyboard_launch.py

"""


#___Import Modules:
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


#___Function:
def generate_launch_description():
    
    # Get the package directory
    ros2_world_simulation_dir = get_package_share_directory('ros2_world_simulation')
    ros2_robot_simulation_dir = get_package_share_directory('ros2_robot_simulation')
    
    
    # Create launch configuration variables
    use_simulator = LaunchConfiguration('use_simulator')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    x_pos = LaunchConfiguration('x_pos')
    y_pos = LaunchConfiguration('y_pos')
    z_pos = LaunchConfiguration('z_pos')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')
    urdf_file = LaunchConfiguration('urdf_file')
    
    
    # Declare the launch arguments
    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator')
    
    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to execute gzclient)')
    
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(ros2_world_simulation_dir, 'worlds', 'racetrack_day.world'),
        description='Full path to world model file to load')
    
    declare_x_pos_cmd = DeclareLaunchArgument(
        'x_pos',
        default_value='2.75')
    
    declare_y_pos_cmd = DeclareLaunchArgument(
        'y_pos',
        default_value='-14.0')
    
    declare_z_pos_cmd = DeclareLaunchArgument(
        'z_pos',
        default_value='0.5')
    
    declare_roll_cmd = DeclareLaunchArgument(
        'roll',
        default_value='0.0')
    
    declare_pitch_cmd = DeclareLaunchArgument(
        'pitch',
        default_value='0.0')
    
    declare_yaw_cmd = DeclareLaunchArgument(
        'yaw',
        default_value='0.0')
    
    declare_urdf_file_cmd = DeclareLaunchArgument(
        'urdf_file',
        default_value='jetbot.xml')
    
    
    # Specify the actions
    world_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros2_world_simulation_dir, 'launch', 'launch.py')),
        launch_arguments={'use_simulator': use_simulator,
                          'headless': headless,
                          'world': world}.items())
    
    spawn_robot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros2_robot_simulation_dir, 'launch', 'spawn.py')),
        launch_arguments={'x_pos': x_pos,
                          'y_pos': y_pos,
                          'z_pos': z_pos,
                          'roll': roll,
                          'pitch': pitch,
                          'yaw': yaw,
                          'urdf': urdf_file}.items())
    
    robot_states_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros2_robot_simulation_dir, 'launch', 'states.py')),
        launch_arguments={'urdf': urdf_file,}.items())
    
    save_image_cmd = Node(
        package = 'ros2_save_camera_image',
        executable = 'execute')
    
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_x_pos_cmd)
    ld.add_action(declare_y_pos_cmd)
    ld.add_action(declare_z_pos_cmd)
    ld.add_action(declare_roll_cmd)
    ld.add_action(declare_pitch_cmd)
    ld.add_action(declare_yaw_cmd)
    ld.add_action(declare_urdf_file_cmd)
    
    # Add all actions
    ld.add_action(world_launch_cmd)
    ld.add_action(spawn_robot_cmd)
    ld.add_action(robot_states_cmd)
    ld.add_action(save_image_cmd)
        
    return ld


#                                                                              
# end of file
"""ANI717"""
