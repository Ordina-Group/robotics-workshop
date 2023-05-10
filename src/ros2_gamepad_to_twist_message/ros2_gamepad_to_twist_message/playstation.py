#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""ROS2 Publisher for Gamepad Data as Twist Message.

This script publishes gamepad controller data to "\cmd_vel" topic as Twist
message.

Revision History:
        14-08-2022 - Setup for playstation 4 controller

Example:
        $ colcon build --symlink-install && source install/local_setup.bash && ros2 run ros2_gamepad_to_twist_message playstation
        $ source install/local_setup.bash && ros2 run ros2_gamepad_to_twist_message playstation
        $ ros2 run ros2_gamepad_to_twist_message playstation

"""

import json
import os
import pygame as controller
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String, Bool

# ___Trick to solve pygame video init error
os.environ["SDL_VIDEODRIVER"] = "dummy"


# ___Global Variables:
SETTINGS = os.path.join(get_package_share_directory('ros2_gamepad_to_twist_message'), "settings.json")


class GamepadTwist(Node):
    """Gamepad Twist Class.

    This class contains all methods to publish gamepad controller data.

    """

    def __init__(self, publish_topic='/cmd_vel', publish_frequency=100):
        super().__init__('gamepad_publisher')

        # publisher initialization
        self.pub_joystick = self.create_publisher(Twist, publish_topic, 1)
        self.pub_camera_trigger = self.create_publisher(Bool, '/camera/cam_0/snapshot/trigger', 2)
        self.pub_camera_livestream = self.create_publisher(Bool, '/camera/cam_0/livestream/state', 2)

        self.timer = self.create_timer(1 / publish_frequency, self.timer_callback)

        # variable initialization
        self.max_z = 1
        self.max_x = 1
        self.z = 0.0
        self.x = 0.0

    def neg_n(self, x):
        """ Toggle value from - to +, or + to -. """
        neg = x * (-1)
        return neg

    def timer_callback(self):
        """Timer Callback Function

        This method publishes gamepad data as Twist message.

        """

        # initializes Twist message
        twist = Twist()

        # retrieve any events from the controller
        for event in controller.event.get():
            if event.type == controller.JOYAXISMOTION:
                self.get_logger().info(f"Event axis: {event.axis}")
                # Move Forward and Backward
                if event.axis == 1:
                    if event.value > 0.1 or event.value < -0.2:
                        rot_x = round(event.value, 1)
                        self.x = self.neg_n(rot_x)
                    else:
                        self.x = 0.0

                # Move Left And Right
                if event.axis == 2:
                    if event.value > 0.1 or event.value < -0.2:
                        rot_z = round(event.value, 1)
                        self.z = rot_z               
                    else:
                        self.z = 0.0
               
            if event.type == controller.JOYBUTTONDOWN:
                if event.button == 0:
                    #make sure no camera livestream is running
                    # msg = Bool()
                    # msg.data = False
                    # self.pub_camera_livestream.publish(msg)
                    # self.get_logger().info(" [2 - O] Button pressed")
                    #make picutre
                    msg = Bool()
                    msg.data = True
                    self.pub_camera_trigger.publish(msg)
                    self.get_logger().info(" [0 - Square] Button pressed")

                if event.button == 1:
                    msg = Bool()
                    msg.data = True
                    self.pub_camera_livestream.publish(msg)
                    self.get_logger().info(" [1 - X] Button pressed")
                if event.button == 2:
                    msg = Bool()
                    msg.data = False
                    self.pub_camera_livestream.publish(msg)
                    self.get_logger().info(" [2 - O] Button pressed")

        # Creates Twist message
        twist.angular.z = round(self.z, 1)
        twist.linear.x = round(self.x, 1)

        # Publishes message
        self.pub_joystick.publish(twist)

        return None

def main(args=None):
    """
        This is the Main Method.
    """
    controller.init()
    controller.joystick.init()
    joysticks = [controller.joystick.Joystick(x) for x in range(controller.joystick.get_count())]

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
