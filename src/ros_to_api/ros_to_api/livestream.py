"""Create an example subscriber.

Create a node named "example_subscriber" and initialise a subscriber. The topic and
queue_size of the subscriber are taken from their respective parameters. These
parameters are passed in the launch file.

A callback function is defined, which handles the incoming message.

"""
from __future__ import annotations
import threading
from typing import Optional

import cv2
from cv_bridge import CvBridge
import rclpy
from sensor_msgs.msg import Image
from support.extended_node import ExtendedNode
from fastapi import FastAPI
from fastapi.responses import StreamingResponse, Response
import uvicorn
import asyncio
import threading
from . import PACKAGE_NAME


app = FastAPI()


class ExampleSubscriber(ExtendedNode):
    def __init__(self):
        super().__init__(PACKAGE_NAME)
        self.get_logger().info("ExampleSubscriber initialised")
        livestream_topic: str = self._get_parameter_value("subscriber_topic_livestream")
        snapshot_topic: str = self._get_parameter_value("subscriber_topic_snapshot")
        queue_size: int = self._get_parameter_value("subscriber_queue_size")
        self.message = None
        self.subscription = self.create_subscription(Image, snapshot_topic, self.listener_callback_snapshot, queue_size)
        self.subscription = self.create_subscription(Image, livestream_topic, self.listener_callback_livestream, queue_size)
        self.image_list = [None,None,None,None,None]
        self.bridge = CvBridge()

        @app.get("/livestream")
        async def publish_livestream():
            return StreamingResponse(self.get_message(), media_type="multipart/x-mixed-replace;boundary=frame")

        @app.get("/snapshot/{id}")
        async def publish_snapshot(id: int):
            self.get_logger().info(f"Snapshot requested: {id}")
            return StreamingResponse(self.get_image(id), media_type="multipart/x-mixed-replace;boundary=frame")


    async def get_message(self):
        while True:
            await asyncio.sleep(0.03)
            frame = self.bridge.imgmsg_to_cv2(self.livestream_msg, "bgr8")
            (flag, encodedImage) = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            image = (b'--frame\r\n'b'Content-Type: image/jpg\r\n\r\n' +
                    bytearray(encodedImage) + b'\r\n')
            yield image
    
    async def get_image(self,number):
        frame = self.bridge.imgmsg_to_cv2(self.image_list[number], "bgr8")
        (flag, encodedImage) = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        image = (b'--frame\r\n'b'Content-Type: image/jpg\r\n\r\n' +
                bytearray(encodedImage) + b'\r\n')
        yield image
        
    def listener_callback_snapshot(self, msg: Image) -> None:
        """Handle incoming message from subscription."""
        self.get_logger().info(f"I will upload foto")
        snapshot_msg = msg
        self.image_list.pop(4)
        self.image_list.insert(0,snapshot_msg)
        self.get_logger().info(f"Foto uploaded:")


        

    def listener_callback_livestream(self, msg: Image) -> None:
        """Handle incoming message from subscription."""
        self.livestream_msg = msg


def main(args: Optional[list[str]] = None) -> None:
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()
    uvicorn.run(app, host="0.0.0.0", port=8080)
    node.destroy_node()
    rclpy.shutdown()


rclpy.init()
node = ExampleSubscriber()

if __name__ == "__main__":
    main()
