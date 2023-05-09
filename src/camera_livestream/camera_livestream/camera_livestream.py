#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""ROS2 CSI Camera Image Publisher.
This script publishes csi camera image to a ROS2 topic in sensor_msgs.msg/Image format.
Example:
        $ colcon build --symlink-install
        $ ros2 launch camera_snap_shot camera_snap_shot.launch.py
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
from cv_bridge import CvBridge

# ___Global Variables:
SETTINGS = os.path.join(get_package_share_directory('camera_livestream'), "config/camera_livestream_settings.json")
with open(SETTINGS) as fp:
    json_settings = json.load(fp)

# __Functions:
def gstreamer_pipeline(capture_width=str(json_settings["capture_width"]),
                       capture_height=str(json_settings["capture_height"]),
                       framerate=str(json_settings["framerate"])):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        f"width=(int){capture_width}, height=(int){capture_height}, "
        f"format=(string)NV12, framerate=(fraction){framerate}/1 ! "
        "nvvidconv ! "
        "video/x-raw, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! "
        "appsink"
    )


# __Classes:
class CameraPublisher(Node):
    """Camera Publisher Class.
    This class contains all methods to publish csi camera data as 
    sensor_msgs.msg/Image format. 
    
    """

    def __init__(self, publish_topic=json_settings["publish_topic"], trigger_topic=json_settings["trigger_topic"]):
        super().__init__('camera_publisher')

        # initialize publisher
        self.publisher_ = self.create_publisher(Image, publish_topic, 1)
        self.subscription1 = self.create_subscription(String, trigger_topic, self.start_livestream, 1)

        # set image counter
        self.image_number = 0
        self.bridge = CvBridge()

    def start_livestream(self):
        """Timer Callback Function
        
        This method captures images and publishes required data in ros 2 topic.
        
        """
        cap = cv2.VideoCapture(gstreamer_pipeline())
        os.environ['DISPLAY']=':0'
        while cap.isOpened():
            # reads image data
            ret, frame = cap.read()

            
            # processes image data and converts to ros 2 message
            msg_image = self.bridge.cv2_to_imgmsg(np.array(frame), "bgr8")
            self.publisher_.publish(msg_image)
                      
            # shows local livestream

            cv2.imshow("Video Frame", frame)

            # for debugging stops when pressing q
            if cv2.waitKey(25) & 0xFF == ord('q'):
                    break

        cap.release()
        cv2.destroyAllWindows()


# ___Main Method:
def main(args=None):
    """
    This is the Main Method.

    """
    # initializes node and start publishing
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    camera_publisher.start_livestream()
    rclpy.spin(camera_publisher)

    # shuts down nose and releases everything
    camera_publisher.destroy_node()
    rclpy.shutdown()

    return None


# ___Driver Program:
if __name__ == '__main__':
    main()