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

import os
import json
import pygame as controller

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String

import sys
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
        self.publisher_ = self.create_publisher(Twist, publish_topic, 1)
        self.publisher2 = self.create_publisher(String, '/trigger', 2)
        self.timer = self.create_timer(1 / publish_frequency, self.timer_callback)

        # variable initialization
        self.max_z = 1
        self.max_x = 1
        self.z = 0.0
        self.x = 0.0

        # # controller initialization
        # controller.init()
        # controller.joystick.init()

    def timer_callback(self):
        """Timer Callback Function
        
        This method publishes gamepad data as Twist message.
        
        """

        # initializes Twist message
        twist = Twist()

        axis = {}

        # these are the identifiers for the PS4's accelerometers
        AXIS_X = 2
        AXIS_Y = 1

        # variables we'll store the rotations in, initialised to zero
        rot_x = 0.0
        rot_y = 0.0

        rot_x_last = 0.0
        rot_y_last = 0.0

        # copy rot_x/rot_y into axis[] in case we don't read any
        axis[AXIS_X] = rot_x
        axis[AXIS_Y] = rot_y

        # retrieve any events from the controller
        for event in controller.event.get():
            if event.type == controller.JOYAXISMOTION:
                # Move Forward and Backward
                if (event.axis ==2):
                    if(event.value > 0.1 or event.value < -0.2):
                        rot_z = round(event.value,1)
                        self.z = rot_z
                        # self.get_logger().info(f" self.z = {self.z}")
                        # self.get_logger().info(f"- event value = {event.value}")
                    else:
                        self.z =0.0

                # Move Left And Right
                if (event.axis ==1):
                    if(event.value > 0.1 or event.value < -0.2):
                        rot_x = round(event.value,1)
                        self.x = rot_x
                        # self.get_logger().info(f"self.z = {self.x}")
                        # self.get_logger().info(f"event value = {event.value}")
                    else:
                        self.x =0.0


            # Test button pressed
            if event.type == controller.JOYBUTTONDOWN:
                if event.button == 0:
                    msg = String()
                    msg.data = 'pressed'
                    self.publisher2.publish(msg)
                    self.get_logger().info("baclhblack")

        rot_x = axis[AXIS_X]
        rot_y = axis[AXIS_Y]

         # creates Twist message
        twist.angular.z = round(self.z, 1)
        twist.linear.x = round(self.x, 1)
    

        # publishes message
        self.publisher_.publish(twist)
        # self.get_logger().info("Angular Z: {:.1f}, Linear X: {:.1f}".format(rot_y, rot_x))


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

