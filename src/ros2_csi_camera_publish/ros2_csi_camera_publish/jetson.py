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
import cv2
import json
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String

# ___Global Variables:
SETTINGS = os.path.join(get_package_share_directory('ros2_csi_camera_publish'), "settings.json")


# __Functions:
def gstreamer_pipeline(capture_width=320, capture_height=240, display_width=320,
                       display_height=240, framerate=30, flip_method=0):
    """Copyright (c) 2019 JetsonHacks
    
    """

    return (
            "nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM), "
            "width=(int)%d, height=(int)%d, "
            "format=(string)NV12, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
            % (capture_width, capture_height, framerate, flip_method, display_width, display_height)
    )


# __Classes:
class CameraPublisher(Node):
    """Camera Publisher Class.

    This class contains all methods to publish csi camera data as 
    sensor_msgs.msg/Image format. 
    
    """

    def __init__(self, publish_topic='/cmd_vel', trigger_topic='/trigger', publish_frequency=100):
        super().__init__('camera_publisher')

        # initialize publisher
        self.publisher_ = self.create_publisher(Image, publish_topic, 1)
        self.subscription1 = self.create_subscription(String, trigger_topic, self.listener_callback1, 10)

        # set image counter
        self.i = 0

    def listener_callback1(self, msg):
        """Timer Callback Function
        
        This method captures images and publishes required data in ros 2 topic.
        
        """
        cap = cv2.VideoCapture(gstreamer_pipeline(1920, 1080,
                                                  1920, 1080,
                                                  30, 0, ), cv2.CAP_GSTREAMER)

        if cap.isOpened():
            # reads image data
            ret, frame = cap.read()

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

            if msg.data == 'pressed':
                # publishes message
                self.publisher_.publish(msg_image)
                self.get_logger().info('%d Images Published' % self.i)
                cap.release()

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

    trigger_topic = '/trigger'

    # initializes node and start publishing
    rclpy.init(args=args)
    camera_publisher = CameraPublisher(publish_topic, trigger_topic, publish_frequency)
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
