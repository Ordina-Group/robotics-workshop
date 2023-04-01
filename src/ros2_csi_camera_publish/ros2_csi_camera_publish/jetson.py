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

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory

# ___Global Variables:
SETTINGS = os.path.join(get_package_share_directory('ros2_csi_camera_publish'), "settings.json")


# __Classes:
class CameraPublisher(Node):
    """Camera Publisher Class.

    This class contains all methods to publish csi camera data as 
    sensor_msgs.msg/Image format. 
    
    """

    def __init__(self, cap, publish_topic='/cmd_vel', publish_frequency=100):
        super().__init__('camera_publisher')

        # initialize publisher
        self.publisher_ = self.create_publisher(Image, publish_topic, 1)
        self.timer = self.create_timer(1/publish_frequency, self.timer_callback)

        # set image counter
        self.cap = cap
        self.i = 0

    def timer_callback(self):
        """Timer Callback Function
        
        This method captures images and publishes required data in ros 2 topic.
        
        """

        if self.cap.isReady():
            # reads image data
            frame = self.cap.read()

            # processes image data and converts to ros 2 message
            msg_image = Image()
            msg_image.header.stamp = Node.get_clock(self).now().to_msg()
            msg_image.header.frame_id = 'ORDINA'
            msg_image.height = np.shape(frame)[0]
            msg_image.width = np.shape(frame)[1]
            msg_image.encoding = "bgr8"
            msg_image.is_bigendian = False
            msg_image.step = np.shape(frame)[2] * np.shape(frame)[1]
            msg_image.data = np.array(frame).tobytes()

            self.publisher_.publish(msg_image)
            self.get_logger().info('%d Image published to image topic' % self.i)

        # image counter increment
        self.i += 1

        return None


# ___Main Method:
def main(args=None):
    """This is the Main Method.
    
    """

    # parse settings from json file
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

    cap = nano.Camera(flip=flip_method, width=capture_width, height=capture_height, fps=framerate)

    # initializes node and start publishing
    rclpy.init(args=args)
    camera_publisher = CameraPublisher(cap, publish_topic, publish_frequency)
    rclpy.spin(camera_publisher)

    # shuts down nose and releases everything
    camera_publisher.destroy_node()
    rclpy.shutdown()

    return None


# ___Driver Program:
if __name__ == '__main__':
    main()

#
# end of file
"""ORDINA"""
