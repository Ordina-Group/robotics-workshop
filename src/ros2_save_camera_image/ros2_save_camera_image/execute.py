#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""ROS2 Image Saving Tool.

This script saves image with twist messages as annotation.

Revision History:
        2021-10-02 (ANI717 - Animesh Bala Ani): Baseline Software.

Example:
        $ colcon build --symlink-install && source install/local_setup.bash && ros2 run ros2_save_camera_image execute
        $ source install/local_setup.bash && ros2 run ros2_save_camera_image execute
        $ ros2 run ros2_save_camera_image execute

"""

# ___Import Modules:
import os
import cv2
import json
import datetime
import requests
import random
import string
import subprocess
import numpy as np
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

# Global Variables:
SETTINGS = os.path.join(get_package_share_directory('ros2_save_camera_image'), "settings.json")


# Classes
class ImageSubscriber(Node):

    def __init__(self, cloud_url, robot_name, image_topic='/image', twist_topic='/cmd_vel'):
        super().__init__('image_subscriber')

        self.subscription = self.create_subscription(
            String,
            '/trigger',
            self.manual_update,
            1)

        # variable initialization
        self.cloud_url = cloud_url
        self.robot_name = robot_name

        self.register_to_cloud()


    def register_to_cloud(self):
        url = "{0}/robot".format(self.cloud_url)

        jsonRobot = {'name': self.robot_name,
                     'liveStream': 'rtsp://{0}:8554/robotstream'.format(self.get_ip_address())}

        try:
            response = requests.post(url, json=jsonRobot, timeout=10)
            self.get_logger().info("Status code: " + str(response.status_code))
        except Exception as e:
            self.get_logger().info("Back-end couldn't be reached: ")


    def update_to_cloud(self):
        url = "{0}/robot".format(self.cloud_url)

        jsonRobot = {'name': self.robot_name,
                     'liveStream': 'rtsp://{0}:8554/robotstream'.format(self.get_ip_address())}

        try:
            response = requests.put(url, json=jsonRobot, timeout=10)
            self.get_logger().info("Status code: " + str(response.status_code))
        except Exception as e:
            self.get_logger().info("Back-end couldn't be reached: ")
        

    def get_ip_address(self):
        cmd = "ifconfig %s | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1'" % 'wlan0'
        return subprocess.check_output(cmd, shell=True).decode('ascii')[:-1]
    

    def manual_update(self, msg):
        if msg.data == 'register':
            self.update_to_cloud()


# Main Method:
def main(args=None):

    with open(SETTINGS) as fp:
        content = json.load(fp)
        image_topic = content["image_topic"]
        twist_topic = content["twist_topic"]
        cloud_url = content["cloud_url"]
        robot_name = content["robot_name"]

    # initializes node and save annotated images
    rclpy.init(args=args)
    cloud_subscriber = ImageSubscriber(cloud_url, robot_name, image_topic, twist_topic)
    rclpy.spin(cloud_subscriber)

    # shuts down and releases everything
    cloud_subscriber.destroy_node()
    rclpy.shutdown()


# Driver Program:
if __name__ == '__main__':
    main()
