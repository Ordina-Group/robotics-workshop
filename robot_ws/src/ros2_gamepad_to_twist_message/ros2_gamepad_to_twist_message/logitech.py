#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""ROS2 Publisher for Gamepad Data as Twist Message.

This script publishes gamepad controller data to "\cmd_vel" topic as Twist 
message.

Revision History:
        2021-10-01 (ANI717 - Animesh Bala Ani): Baseline Software.

Example:
        $ colcon build --symlink-install && source install/local_setup.bash && ros2 run ros2_gamepad_to_twist_message logitech
        $ source install/local_setup.bash && ros2 run ros2_gamepad_to_twist_message logitech
        $ ros2 run ros2_gamepad_to_twist_message logitech

"""


#___Import Modules:
import os
import json
from inputs import get_gamepad

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ament_index_python.packages import get_package_share_directory


#___Global Variables:
SETTINGS = os.path.join(get_package_share_directory('ros2_gamepad_to_twist_message'), "settings.json")


#__Classes
class GamepadTwist(Node):
    """Gamepad Twist Class.
    
    This class contains all methods to publish gamepad controller data. 
    
    """

    def __init__(self, publish_topic='/cmd_vel', publish_frequency=100):
        super().__init__('gamepad_publisher')
        
        # publisher initialization
        self.publisher_ = self.create_publisher(Twist, publish_topic, 1)
        self.timer = self.create_timer(1/publish_frequency, self.timer_callback)
        
        # variable initialization
        self.max_z = 1
        self.max_x = 1
        self.z = 0.0
        self.x = 0.0

    def timer_callback(self):
        """Timer Callback Function
        
        This method publishes gamepad data as Twist message.
        
        """

        # initializes Twist message
        twist = Twist()
        
        # obtains gamepad data
        events = get_gamepad()
        for event in events:
            if event.code == 'ABS_RX':
                self.max_z = max(abs(event.state), self.max_z)
                self.z = event.state/self.max_z
            
            if event.code == 'ABS_Y':
                self.max_x = max(abs(event.state), self.max_x)
                self.x = - event.state/self.max_x
                
        # creates Twist message
        twist.angular.z = round(self.z, 1)
        twist.linear.x = round(self.x, 1)

        # publishes message
        self.publisher_.publish(twist)
        self.get_logger().info("Angular Z: {:.1f}, Linear X: {:.1f}".format(self.z, self.x))

        return None


def main(args=None):
    """This is the Main Method.
    
    """
    
    # parse settings from json file
    with open(SETTINGS) as fp:
        content = json.load(fp)
        publish_topic = content["publish_topic"]
        publish_frequency = content["publish_frequency"]
    
    # initializes node and start publishing
    rclpy.init(args=args)
    gamepad_publisher = GamepadTwist(publish_topic, publish_frequency)
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
