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
        self.create_livestream()


    def create_livestream(self):

        hostname=socket.gethostname()
        IPAddr=socket.gethostbyname(hostname)

        try:
            subprocess.Popen(["./host_rtsp_server", "nvarguscamerasrc ! nvvidconv ! nvv4l2h264enc ! h264parse ! rtph264pay name=pay0 pt=96",str(IPAddr)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print("rtsp server is :\n rtsp://{IPAddr}:8554/stream/raytesnel")
        except Exception as e:
            print("failed to set up rtsp server error :\n {e}")


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
