"""Create an example subscriber.

Create a node named "example_subscriber" and initialise a subscriber. The topic and
queue_size of the subscriber are taken from their respective parameters. These
parameters are passed in the launch file.

A callback function is defined, which handles the incoming message.

"""
from __future__ import annotations
from typing import Optional
import subprocess
import cv2
import os
import sys


import rclpy
from sensor_msgs.msg import Image
from support.extended_node import ExtendedNode
from cv_bridge import CvBridge
import ffmpeg_streaming
from ffmpeg_streaming import Formats, Bitrate, Representation, Size

from . import PACKAGE_NAME


class ExampleSubscriber(ExtendedNode):
    def __init__(self):
        super().__init__(PACKAGE_NAME)
        base = self._get_parameter_value("topic_camera")
        cam_id = self._get_parameter_value("camera_id")
        livestream = self._get_parameter_value("topic_livestream")
        topic: str = base + cam_id + livestream
        self.framerate: int = self._get_parameter_value("framerate")
        self.image_width: int = self._get_parameter_value("image_width")
        self.image_height: int = self._get_parameter_value("image_height")

        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        
        self.first_time = True

        self.get_logger().info(f"topic: {topic}")
        queue_size: int = self._get_parameter_value("queue_size")
        self.subscription = self.create_subscription(Image, topic, self.listener_callback, queue_size)
        # self.create_ffmpeg_process(self.image_width,self.image_height)
        self.bridge = CvBridge()
        os.environ['DISPLAY']=':0' # for developing, this is needed to open up a window to show the local livestream
        # self.setup_HSL_stream()



    def listener_callback(self, msg: Image) -> None:
        """Handle incoming message from subscription."""
        # self.get_logger().info(f"I heard: {msg.data}")
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.out = cv2.VideoWriter('livestream/output.mp4', self.fourcc, self.framerate, (self.image_width, self.image_height))
        self.out.write(frame)
        self.out.release()
        cv2.imshow("frame", frame)
        cv2.waitKey(1)
        if self.first_time:
            self.setup_HSL_stream()
            self.first_time = False

        


        

    def setup_HSL_stream(self):
        video_file = "livestream/output.mp4"
        video= ffmpeg_streaming.input(video_file, capture = True, framerate=self.framerate, vcodec="h264", acodec="aac")
        hsl = video.hls(Formats.h264())
        _720p  = Representation(Size(720, 480), Bitrate(overall=497664))
        hsl.representations(_720p)
        hsl.output('livestream/hsl.m3u8')



def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ExampleSubscriber()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
