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

        # controller initialization
        controller.init()
        controller.joystick.init()

    def timer_callback(self):

        axis = {}

        # these are the identifiers for the PS4's accelerometers
        AXIS_X = 3
        AXIS_Y = 4

        # variables we'll store the rotations in, initialised to zero
        rot_x = 0.0
        rot_y = 0.0

        # copy rot_x/rot_y into axis[] in case we don't read any
        axis[AXIS_X] = rot_x
        axis[AXIS_Y] = rot_y

        # retrieve any events from the controller
        for event in controller.event.get():
            if event.type == controller.JOYAXISMOTION:
                axis[event.axis] = round(event.value, 2)
                # TODO
                # hier moeten die axis zo gemaakt worden zoals bij de waveshare
                # wat er nu staat is geen oplossing of iets

            if event.type == controller.JOYBUTTONDOWN:
                if event.button == 0:
                    msg = String()
                    msg.data = 'pressed'
                    self.publisher2.publish(msg)

        rot_x = axis[AXIS_X]
        rot_y = axis[AXIS_Y]

        return None


def main(args=None):
    """
        This is the Main Method.
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
