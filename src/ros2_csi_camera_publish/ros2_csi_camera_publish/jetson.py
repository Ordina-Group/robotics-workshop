#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""ROS2 CSI Camera Image Publisher.

This script publishes csi camera image to a ROS2 topic in sensor_msgs.msg/Image 
format. And starts rtmp stream to rtmp://localhost;1935/live/stream

Example:
        $ colcon build --symlink-install && source install/local_setup.bash && ros2 run ros2_csi_camera_publish jetson
        $ source install/local_setup.bash && ros2 run ros2_csi_camera_publish jetson
        $ ros2 run ros2_csi_camera_publish jetson

"""

# ___Import Modules:
import os
import nanocamera as nano
import json
import numpy as np
import subprocess 
import socket
import requests

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory

# ___Global Variables:
SETTINGS = os.path.join(get_package_share_directory('ros2_csi_camera_publish'), "settings.json")


# __Classes:
class CameraPublisher(Node):

    def __init__(self):
        super().__init__('camera_publisher')
        server = 'server'
        name = 'name'

        if not self.has_parameter(server):
            self.declare_parameter(server)

        if not self.has_parameter(name):
            self.declare_parameter(name)

        self.server_url = self.get_parameter(server).value
        self.robot_name = self.get_parameter(name).value

        self.register_to_cloud()
        self.create_livestream()


    def create_livestream(self):
        try:
            subprocess.Popen(["./host_rtsp_server", "nvarguscamerasrc ! nvvidconv ! nvv4l2h264enc ! h264parse ! rtph264pay name=pay0 pt=96",str(self.get_ip_address())], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:
            self.get_logger().info("failed to set up rtsp server error :\n {e}")


    def get_ip_address(self):
        cmd = "ifconfig %s | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1'" % 'wlan0'
        return subprocess.check_output(cmd, shell=True).decode('ascii')[:-1]

    def register_to_cloud(self):
        try:
            requests.post('http://{0}:8080/robot'.format(self.server_url), json={"name": self.robot_name,
                                                                                "ipAddress": self.get_ip_address()})
        except Exception as e:
            self.get_logger().info("Could not register to cloud:\n {0}".format(e))



# ___Main Method:
def main(args=None):

    with open(SETTINGS) as fp:
        content = json.load(fp)
        publish_topic = content["publish_topic"]
        publish_frequency = content["publish_frequency"]
        capture_width = content["capture_width"]
        capture_height = content["capture_height"]
        framerate = content["framerate"]
        flip_method = content["flip_method"]
        display_width = content["display_width"]
        display_height = content["display_height"]

    rclpy.init(args=args)
    livestream = CameraPublisher()
    rclpy.spin(livestream)

    livestream.destroy_node()
    rclpy.shutdown()


# ___Driver Program:
if __name__ == '__main__':
    main()

#
# end of file
"""ORDINA"""
