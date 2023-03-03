#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""ROS2 Twist to Jetbot Move.

This script subscribes to "/cmd_vel" topic, reads Twist message, and moves a 
robot car.

Revision History:
        2021-08-18 (ANI717 - Animesh Bala Ani): Baseline Software.

Example:
        $ colcon build --symlink-install && source install/local_setup.bash && ros2 run ros2_twist_message_to_robot_motion jetbot
        $ source install/local_setup.bash && ros2 run ros2_twist_message_to_robot_motion jetbot
        $ ros2 run ros2_twist_message_to_robot_motion jetbot

"""


#___Import Modules:
import os
import json
import atexit
import traitlets
from traitlets.config.configurable import Configurable, SingletonConfigurable

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ament_index_python.packages import get_package_share_directory

#__Classes
class Validate_Twist(Node):
    """TWIST to Validate Twist data.
    
    This class contains all methods to read TWIST message and move a robot car. 
    
    """

    def __init__(self, subscribe_topic='/topic_thalia', x_calibration=0.25, z_calibration=0.25):
        
        super().__init__('validate_twist')
        
        # initialize subscriber
        self.subscription = self.create_subscription(Twist, subscribe_topic, self.listener_callback, 1)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Twist, 'topic_motion', 1)
        
        # initialize variables
        self.x_calibration = x_calibration
        self.z_calibration = z_calibration


    def listener_callback(self, msg):
        """Listener Callback Function
        
        This method collects data from geometry twist message topic and runs robot.
        
        """
        
        # parses data from subscribed topic message
        x = self.x_calibration*float(msg.linear.x)
        z = self.z_calibration*float(msg.angular.z)

        #validations
        if x > 1.0 :
             x = 1.0
        if x < -1.0:
            x = -1.0

        if z > 1.0 :
            z = 1.0
        if z < -1.0:
            z = -1.0
        

        self.get_logger().info("x ={:.1f}".format(x))
        self.get_logger().info("z ={:.1f}".format(z))

        # initializes Twist message
        twist = Twist()
        # Creates Twist message
        twist.angular.z = round(z, 1)
        twist.linear.x = round(x, 1)

        # Publishes message
        self.publisher_.publish(twist)


#___Main Method:
def main(args=None):
    """This is the Main Method.
    
    """
    
    # initializes node and run robot car
    rclpy.init(args=args)
    twist_to_motion = Validate_Twist(subscribe_topic, x_calibration, z_calibration)
    rclpy.spin(twist_to_motion)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    twist_to_motion.destroy_node()
    rclpy.shutdown()


#___Driver Program:
if __name__ == '__main__':
    main()


#                                                                              
# end of file
"""ORDINA"""
