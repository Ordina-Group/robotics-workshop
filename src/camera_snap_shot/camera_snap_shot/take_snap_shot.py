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
SETTINGS = os.path.join(get_package_share_directory('camera_snap_shot'), "config/camera_snap_shot_settings.json")
with open(SETTINGS) as fp:
    json_settings = json.load(fp)

# __Functions:
def gstreamer_pipeline(capture_width=json_settings["capture_width"],
                       capture_height=json_settings["capture_height"],
                       display_width=json_settings["display_width"],
                       display_height=json_settings["display_height"],
                       framerate=json_settings["framerate"],
                       flip_method=json_settings["flip_method"]):
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

    def __init__(self, publish_topic='/image', trigger_topic='/trigger', publish_frequency=10):
        super().__init__('camera_publisher')

        # initialize publisher
        self.publisher_ = self.create_publisher(Image, publish_topic, publish_frequency)
        self.subscription1 = self.create_subscription(String, trigger_topic, self.listener_callback1, 1)

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
            msg_image.header.frame_id = str(self.i)
            msg_image.height = np.shape(frame)[0]
            msg_image.width = np.shape(frame)[1]
            msg_image.encoding = "bgr8"
            msg_image.is_bigendian = False
            msg_image.step = np.shape(frame)[2] * np.shape(frame)[1]
            msg_image.data = np.array(frame).tobytes()

            if msg.data == json_settings["camera_trigger_message"]:
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

    # initializes node and start publishing
    rclpy.init(args=args)
    camera_publisher = CameraPublisher(
        json_settings["publish_topic"],
        json_settings["trigger_topic"],
        json_settings["publish_frequency"]
    )
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