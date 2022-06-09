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
rtmp_url = "rtmp://127.0.0.1:1935/live/stream"

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

# def open_ffmpeg_stream_process(self):
#     args = (
#         "ffmpeg -re -stream_loop -1 -f rawvideo -pix_fmt "
#         "rgb24 -s 1920x1080 -i pipe:0 -pix_fmt yuv420p "
#         "-f rtsp rtsp://192.168.2.55:1935/live/stream"
#     ).split()
#     return subprocess.Popen(args, stdin=subprocess.PIPE)


# def capture_loop():
#     ffmpeg_process = open_ffmpeg_stream_process()
#     capture = cv2.VideoCapture(<video/stream>)    
#     while True:
#         grabbed, frame = capture.read()
#         if not grabbed:
#             break
#         ffmpeg_process.stdin.write(frame.astype(np.uint8).tobytes())
#     capture.release()
#     ffmpeg_process.stdin.close()
#     ffmpeg_process.wait()

def start_rtmp_stream():
    # In my mac webcamera is 0, also you can set a video file name instead, for example "/home/user/demo.mp4"
    path = 0
    cap = cv2.VideoCapture(path)

    # gather video info to ffmpeg
    fps = int(cap.get(cv2.CAP_PROP_FPS))
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # command and params for ffmpeg
    command = ['ffmpeg',
            '-y',
            '-f', 'rawvideo',
            '-vcodec', 'rawvideo',
            '-pix_fmt', 'bgr24',
            '-s', "{}x{}".format(width, height),
            '-r', str(fps),
            '-i', '-',
            '-c:v', 'libx264',
            '-pix_fmt', 'yuv420p',
            '-preset', 'ultrafast',
            '-f', 'flv',
            rtmp_url]

    # using subprocess and pipe to fetch frame data
    p = subprocess.Popen(command, stdin=subprocess.PIPE)


    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("frame read failed")
            break


#__Classes:
class CameraPublisher(Node):
    """Camera Publisher Class.

    This class contains all methods to publish csi camera data as 
    sensor_msgs.msg/Image format. 
    
    """

    
    def __init__(self, cap, publish_topic='/cmd_vel', trigger_topic='/trigger', publish_frequency=100):
        super().__init__('camera_publisher')
        
        # initialize publisher
        self.publisher_ = self.create_publisher(Image, publish_topic, 1)
        # self.timer = self.create_timer(1/publish_frequency, self.timer_callback)
        self.subscription1 = self.create_subscription(String, trigger_topic, self.listener_callback1, 10)
        
        # set image counter and videocapture object
        self.cap = cap
        self.i = 0

        
    def timer_callback(self):
        """Timer Callback Function
        
        This method captures images and publishes required data in ros 2 topic.
        
        """
        if msg.data == 'pressed':
            if self.cap.isOpened():
                
                # reads image data
                ret, frame = self.cap.read()

                # processes image data and converts to ros 2 message
                msg = Image()
                msg.header.stamp = Node.get_clock(self).now().to_msg()
                msg.header.frame_id = 'ORDINA'
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
    
    trigger_topic = '/trigger'

    # creates OpenCV Videocapture object
    cap = cv2.VideoCapture(gstreamer_pipeline(capture_width, capture_height, 
                                              display_width, display_height,
                                              framerate, flip_method,), cv2.CAP_GSTREAMER)
    
    # initializes node and start publishing
    rclpy.init(args=args)
    # start_rtmp_stream()
    # out = cv2.VideoWriter("appsrc ! video/x-raw,format=BGR,width=1920,height=1080,framerate=30/1 ! videoconvert ! video/x-raw,format=BGRx ! nvvidconv ! nvv4l2h264enc insert-vui=1 insert-sps-pps=1 ! h264parse ! rtph264pay ! udpsink host=127.0.0.1 port=5000", cv2.CAP_GSTREAMER, 0, 30, (1920,1080))
    camera_publisher = CameraPublisher(cap, publish_topic, trigger_topic, publish_frequency)
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
"""ORDINA"""
