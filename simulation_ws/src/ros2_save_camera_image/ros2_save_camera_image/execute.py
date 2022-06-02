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


#___Import Modules:
import os
import cv2
import json
import datetime
import numpy as np
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from ament_index_python.packages import get_package_share_directory


#___Global Variables:
SETTINGS = os.path.join(get_package_share_directory('ros2_save_camera_image'), "settings.json")


#__Classes
class ImageSubscriber(Node):
    """Image Saving Class.
    
    This class contains all methods to save annotated images. 
    
    """


    def __init__(self, image_folder, image_topic='/image', twist_topic='/cmd_vel'):
        super().__init__('image_subscriber')
        
        # image topic subscriber initialization
        self.subscription1 = self.create_subscription(Image, image_topic, self.listener_callback1, 10)
        self.subscription1  # prevent unused variable warning
        self.image_count = 0
        
        # twist topic subscriber initialization
        self.subscription2 = self.create_subscription(Twist, twist_topic, self.listener_callback2, 10)
        self.subscription2
        
        # variable initialization
        self.image_folder = image_folder
        self.x = 0
        self.z = 0
    

    def listener_callback1(self, msg):
        """Listener Callback Function 1
        
        This method collects data from image topic and saves them.
        
        """
        
        # parse image data from subscribed topic
        height = msg.height
        width = msg.width
        channel = msg.step//msg.width
        frame = np.reshape(msg.data, (height, width, channel))
        self.get_logger().info("Image Received")
        
        # write image with annotation in name
        cv2.imwrite(os.path.join(self.image_folder, '{0:07d}_z{1:02d}_x{2:02d}.jpg'.format(self.image_count, self.z, self.x)), frame)
        self.image_count += 1
    
    
    def listener_callback2(self, msg):
        """Listener Callback Function 1
        
        This method collects data from geometry twist message topic.
        
        """
        
        # parse geometry twist data from subscribed topic
        self.x = int(msg.linear.x*5) + 5
        self.z = int(msg.angular.z*5) + 5


#___Main Method:
def main(args=None):
    """This is the Main Method.
    
    """
    
    # parse settings from json file
    with open(SETTINGS) as fp:
        content = json.load(fp)
        image_topic = content["image_topic"]
        twist_topic = content["twist_topic"]
        data_directory = content["data_directory"]
    
    # set directory for saving images
    image_folder = os.path.join(data_directory,
                                   '{0:04d}_{1:02d}_{2:02d}_{3:02d}_{4:02d}'.format(datetime.datetime.now().year,
                                                                                    datetime.datetime.now().month,
                                                                                    datetime.datetime.now().day,
                                                                                    datetime.datetime.now().hour,
                                                                                    datetime.datetime.now().minute))
    Path(image_folder).mkdir(parents=True, exist_ok=True)
    
    # initializes node and save annotated images
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber(image_folder, image_topic, twist_topic)
    rclpy.spin(image_subscriber)

    # shuts down and releases everything
    image_subscriber.destroy_node()
    rclpy.shutdown()


#___Driver Program:
if __name__ == '__main__':
    main()


#                                                                              
# end of file
"""ANI717"""
