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
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import ffmpeg_streaming
import subprocess


# ___Global Variables:
SETTINGS = os.path.join(get_package_share_directory('camera_capture'), "config/camera_capture_settings.json")
with open(SETTINGS) as fp:
    json_settings = json.load(fp)

# __Functions:
def gstreamer_pipeline(capture_width=str(json_settings["capture_width_livestream"]),
                       capture_height=str(json_settings["capture_width_livestream"]),
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

    def __init__(self, 
                 publish_livestream_topic:str,
                 publish_snapshot_topic:str, 
                 snapshot_trigger_topic:str, 
                 livestream_state_topic:str
                 ):
        super().__init__('camera_publisher')

        # initialize publisher & subscirbers
        self.pub_cam_snapshot = self.create_publisher(Image, publish_snapshot_topic, 1)
        self.pub_cam_livestream = self.create_publisher(Image, publish_livestream_topic, 1)
        self.sub_cam_livestream_state = self.create_subscription(Bool, livestream_state_topic, self.start_livestream_callback, 1)
        self.sub_cam_snapshot_trigger = self.create_subscription(Bool, snapshot_trigger_topic, self.capture_snapshot_callback, 1)

        self.cap = cv2.VideoCapture(gstreamer_pipeline())
        self.bridge = CvBridge()
        self.image_location = f"{json_settings['image_location']}"

        # set image counter
        self.image_number = 0

    def start_livestream_callback(self, topic_msg):
        """Timer Callback Function
        
        This method starts a livestream
        
        """
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, json_settings["capture_height_livestream"])
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, json_settings["capture_width_livestream"])
        self._logger.info(f'message received on topic livestream \nwith message: {topic_msg}\nwith data :{topic_msg.data}')
        timer_period = 0.03  # seconds TODO: make into settings 30hz
        if self.cap.isOpened():
            self.get_logger().info('camera is available')
            if topic_msg.data == True:
                self._logger.info('starting livestream')
                self.timer = self.create_timer(timer_period, self.timer_callback)
            elif topic_msg.data == False:
                self._logger.info('stopping livestream')
                self.timer.cancel()
                self.timer.destroy()
        else:
            self.get_logger().info('camera not available')

    def timer_callback(self):
        ret, frame = self.cap.read()
        # cv2.imshow("Video Frame", frame)
        msg_image = self.bridge.cv2_to_imgmsg(np.array(frame), "bgr8")
        self.pub_cam_livestream.publish(msg_image)

    def capture_snapshot_callback(self, topic_msg):
        """Timer Callback Function
        
        This method captures images, publishes required data in ros 2 topic and saves it to a folder.
        
        """
        self.get_logger().info(f'message received on topic snapshot \nwith message: {topic_msg}\nwith data :{topic_msg.data}')
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, json_settings["capture_height_snapshot"])
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, json_settings["capture_width_snapshot"])
        if self.cap.isOpened():

            # reads image data
            ret, frame = self.cap.read()
            print(type(frame))
            msg_image = self.bridge.cv2_to_imgmsg(np.array(frame), "bgr8")
            msg_image.header.frame_id = str(self.image_number)
            # saves image to folder
            cv2.imwrite(f"{self.image_location}/image{self.image_number}.jpg", frame)
            self.get_logger().info(f'image saved at image{self.image_number}.jpg')
            #post camera snapshot on topic
            self.pub_cam_snapshot.publish(msg_image)

            # self.cap.release()
            self.image_number += 1
            # keep max 5 files
            prevent_overflood_image(self.image_number)
        else:
            self.get_logger().info('camera not available')
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, json_settings["capture_height_livestream"])
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, json_settings["capture_width_livestream"])


def prevent_overflood_image(img_number):
    if img_number >4:
        item_to_delete = f"{json_settings['image_location']}/image{img_number-5}.jpg"
        if os.path.exists(item_to_delete):
            os.remove(item_to_delete)



# ___Main Method:
def main(args=None):
    """
    This is the Main Method.

    """
    # initializes node and start publishing
    cam_0_topic = json_settings['publish_topic_camera']+json_settings['camera_id']
    livestream_state_topic = cam_0_topic + json_settings['publish_topic_livestream']+json_settings['trigger_topic_livestream']
    snapshot_trigger_topic = cam_0_topic + json_settings['publish_topic_snapshot']+json_settings['trigger_topic_snapshot']
    publish_livestream_topic = cam_0_topic + json_settings['publish_topic_livestream']
    publish_snapshot_topic = cam_0_topic + json_settings['publish_topic_snapshot']
    rclpy.init(args=args)
    camera_publisher = CameraPublisher(
        livestream_state_topic=livestream_state_topic,
        snapshot_trigger_topic=snapshot_trigger_topic,
        publish_livestream_topic=publish_livestream_topic,
        publish_snapshot_topic=publish_snapshot_topic,        
        )
    rclpy.spin(camera_publisher)

    # shuts down nose and releases everything
    camera_publisher.destroy_node()
    rclpy.shutdown()

    return None


# ___Driver Program:
if __name__ == '__main__':
    main()
