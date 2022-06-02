#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""ROS2 CSI Camera Image Publisher.

This script publishes csi camera image to a ROS2 topic in sensor_msgs.msg/Image 
format. 

Revision History:
        2021-11-14 (ANI717 - Animesh Bala Ani): Baseline Software.

Example:
        $ colcon build --symlink-install && source install/local_setup.bash && ros2 run ros2_csi_camera_publish jetson
        $ source install/local_setup.bash && ros2 run ros2_csi_camera_publish jetson
        $ ros2 run ros2_csi_camera_publish jetson

"""


#___Import Modules:
import os
import cv2
import json
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory


#___Global Variables:
SETTINGS = os.path.join(get_package_share_directory('ros2_csi_camera_publish'), "settings.json")


#__Functions:
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


#__Classes:
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
        
        # set image counter and videocapture object
        self.cap = cap
        self.i = 0

        
    def timer_callback(self):
        """Timer Callback Function
        
        This method captures images and publishes required data in ros 2 topic.
        
        """

        if self.cap.isOpened():
            
            # reads image data
            ret, frame = self.cap.read()

            # processes image data and converts to ros 2 message
            msg = Image()
            msg.header.stamp = Node.get_clock(self).now().to_msg()
            msg.header.frame_id = 'ANI717'
            msg.height = np.shape(frame)[0]
            msg.width = np.shape(frame)[1]
            msg.encoding = "bgr8"
            msg.is_bigendian = False
            msg.step = np.shape(frame)[2] * np.shape(frame)[1]
            msg.data = np.array(frame).tobytes()

            # publishes message
            self.publisher_.publish(msg)
            self.get_logger().info('%d Images Published' % self.i)
        
        # image counter increment
        self.i += 1
        
        return None


#___Main Method:
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
        
    # creates OpenCV Videocapture object
    cap = cv2.VideoCapture(gstreamer_pipeline(capture_width, capture_height, 
                                              display_width, display_height,
                                              framerate, flip_method,), cv2.CAP_GSTREAMER)
    
    # initializes node and start publishing
    rclpy.init(args=args)
    camera_publisher = CameraPublisher(cap, publish_topic, publish_frequency)
    rclpy.spin(camera_publisher)

    # shuts down nose and releases everything
    camera_publisher.destroy_node()
    rclpy.shutdown()
    cap.release()
    
    return None


#___Driver Program:
if __name__ == '__main__':
    main()


#                                                                              
# end of file
"""ANI717"""
