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
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge


# ___Global Variables:
SETTINGS = os.path.join(get_package_share_directory('camera_snap_shot'), "config/camera_snap_shot_settings.json")
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

        # initialize publisher & subscirbers
        self.pub_cam_snapshot = self.create_publisher(Image, publish_topic, 1)
        self.pub_cam_livestream = self.create_publisher(Image, publish_topic, 1)
        self.sub_cam_snapshot_trigger = self.create_subscription(String, trigger_topic, self.capture_snapshot, 1)
        self.sub_cam_livestream_state = self.create_subscription(String, trigger_topic, self.start_livestream, 1)

        self.cap = cv2.VideoCapture(gstreamer_pipeline())
        self.bridge = CvBridge()
        self.image_location = f"{json_settings['image_location']}"

        # set image counter
        self.image_number = 0

    def start_livestream(self):
        """Timer Callback Function
        
        This method starts a livestream
        
        """
        os.environ['DISPLAY']=':0' # for developing, this is needed to open up a window to show the local livestream
        while self.cap.isOpened():
            # reads image data
            ret, frame = self.cap.read()
            msg_image = define_img_file(frame)
            # msg_image = self.bridge.cv2_to_imgmsg(np.array(frame), "bgr8")
            self.pub_camera_livestream.publish(msg_image)


            # for debugging shows local livestream
            cv2.imshow("Video Frame", frame)

            # for debugging stops when pressing q
            if cv2.waitKey(25) & 0xFF == ord('q'):
                    break

        self.cap.release()
        cv2.destroyAllWindows() #for debugging

    def capture_snapshot(self, topic_msg):
        """Timer Callback Function
        
        This method captures images, publishes required data in ros 2 topic and saves it to a folder.
        
        """
        if self.cap.isOpened() and topic_msg.data == True:
            # reads image data
            ret, frame = self.cap.read()
            print(type(frame))
            # processes image data and converts to ros 2 message

            msg_image = define_img_file(frame=frame,
                                        frame_id=str(self.image_number),
                                        time_stamp=Node.get_clock(self).now().to_msg()
                                        )
            # save image local (is this needed?)
            cv2.imwrite(f"{self.image_location}/image{self.image_number}.jpg", frame)
            self.get_logger().info(f'image saved at image{self.image_number}.jpg')
            #post camera snapshot on topic
            self.pub_camera_snapshot.publish(msg_image)

            self.cap.release()
            self.image_number += 1
            #remove keep max 5 files
            prevent_overflood_image(self.image_number)
        return None


def prevent_overflood_image(img_number):
    if img_number >4:
        item_to_delete = f"{json_settings['image_location']}/image{img_number-5}.jpg"
    if os.path.exists(item_to_delete):
        os.remove(item_to_delete)


def define_img_file(frame, frame_id=0 , time_stamp=0):
    """
    prepares the Image(), adds headers, heights etc.
    """
    msg_image = Image()
    msg_image.header.stamp = time_stamp
    msg_image.header.frame_id = frame_id
    msg_image.height = np.shape(frame)[0]
    msg_image.width = np.shape(frame)[1]
    msg_image.encoding = "bgr8"
    msg_image.is_bigendian = False
    msg_image.step = np.shape(frame)[2] * np.shape(frame)[1]
    msg_image.data = np.array(frame).tobytes()
    
    return(msg_image)

# ___Main Method:
def main(args=None):
    """
    This is the Main Method.

    """
    # initializes node and start publishing
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)

    # shuts down nose and releases everything
    camera_publisher.destroy_node()
    rclpy.shutdown()

    return None


# ___Driver Program:
if __name__ == '__main__':
    main()
