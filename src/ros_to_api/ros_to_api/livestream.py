"""Create an example subscriber.

Create a node named "example_subscriber" and initialise a subscriber. The topic and
queue_size of the subscriber are taken from their respective parameters. These
parameters are passed in the launch file.

A callback function is defined, which handles the incoming message.

"""
from __future__ import annotations
from typing import Optional

import rclpy
from sensor_msgs.msg import Image
from support.extended_node import ExtendedNode
from fastapi import FastAPI
from fastapi.responses import StreamingResponse
import cv2

from cv_bridge import CvBridge


import uvicorn
import threading
from . import PACKAGE_NAME


app = FastAPI()

@app.get( '/listen' ) 
async  def  publish ():
    # msg = Image()

    # response  =  {  
    #     'msg' :  msg.data 
    # } 
    return StreamingResponse(node.get_message(), media_type="multipart/x-mixed-replace;boundary=frame")
    # return  {'data' : next(node.get_message())} 

class ExampleSubscriber(ExtendedNode):
    def __init__(self):
        super().__init__(PACKAGE_NAME)
        self.get_logger().info("ExampleSubscriber initialised")
        topic: str = self._get_parameter_value("subscriber_topic")
        queue_size: int = self._get_parameter_value("subscriber_queue_size")
        self.message = None
        self.subscription = self.create_subscription(Image, topic, self.listener_callback, queue_size)
        self.bridge = CvBridge()

    def get_message(self):
        frame = self.bridge.imgmsg_to_cv2(self.message, "bgr8")
        (flag, encodedImage) = cv2.imencode(".jpg", frame)
        yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' +
               bytearray(encodedImage) + b'\r\n')
        # yield self.message
        
    def listener_callback(self, msg: Image) -> None:
        """Handle incoming message from subscription."""
        self.get_logger().info(f"I heard somthing")
        self.message = msg



def main(args: Optional[list[str]] = None) -> None:

    # rclpy.spin(node)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()
    uvicorn.run(app, host="0.0.0.0", port=8080)
    node.destroy_node()
    rclpy.shutdown()

rclpy.init()
node = ExampleSubscriber()

if __name__ == "__main__":
    main()