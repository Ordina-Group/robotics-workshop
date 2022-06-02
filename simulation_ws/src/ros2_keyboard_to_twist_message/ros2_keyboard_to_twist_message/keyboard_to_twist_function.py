#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""ROS2 Gamepad to Twist Message Publish Executable.

This script publishes keyboard data to "\cmd_vel" topic as Twist message.

Revision History:
        2021-10-01 (ANI717 - Animesh Bala Ani): Baseline Software.

Example:
        $ colcon build && source install/setup.bash && ros2 run ros2_keyboard_to_twist_message execute
        $ source install/setup.bash && ros2 run ros2_keyboard_to_twist_message execute
        $ ros2 run ros2_keyboard_to_twist_message execute

"""


#___Import Modules:
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import getch


#___Global Variables:
PUBLISH_TOPIC = '/cmd_vel'


#__Classes
class GamepadTwist(Node):
    """Gamepad Twist Class.
    
    This class contains all methods to publish gamepad controller data. 
    
    """

    def __init__(self):
        super().__init__('gamepad_publisher')
        self.publisher_ = self.create_publisher(Twist, PUBLISH_TOPIC, 10)
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # initialization
        self.max_z = 1.0
        self.max_x = 0.5
        self.z = 0.0
        self.x = 0.0

    def timer_callback(self):
        """Timer Callback Function
        
        This method publishes data as Twist message.
        
        """

        # initializes Twist message
        twist = Twist()
        
        if getch.getch() == 'a':
            self.z = max(0.0, self.z)
            self.z = min(self.max_z, self.z + 0.1)
                
        if getch.getch() == 'd':
            self.z = min(0.0, self.z)
            self.z = max(-self.max_z, self.z - 0.1)
        
        if getch.getch() == 'w':
            self.x = min(self.max_x, self.x + 0.1)
        
        if getch.getch() == 's':
            self.x = max(-self.max_x, self.x - 0.1)
        
        if getch.getch() == 'k':
            self.z = 0.0
        
        if getch.getch() == 'l':
            self.x = 0.0
        
        # creates Twist message
        twist.angular.z = self.z
        twist.linear.x = self.x

        # publishes message
        self.publisher_.publish(twist)
        self.get_logger().info("Angular Z: {:.2f}, Linear X: {:.2f}".format(self.z, self.x))

        return None


def main(args=None):
    """This is the Main Method.
    
    """
    
    # initializes node and start publishing
    rclpy.init(args=args)
    gamepad_publisher = GamepadTwist()
    rclpy.spin(gamepad_publisher)
    
    # shuts down and releases everything
    gamepad_publisher.destroy_node()
    rclpy.shutdown()
    
    return None


if __name__ == '__main__':
    main()


#                                                                              
# end of file
"""ANI717"""
