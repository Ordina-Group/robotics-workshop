#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""ROS2 Publisher for ONNX Model Prediction as Twist Message.

This script publishes twist message predicted from ONNX model to "\cmd_vel" 
topic.

Revision History:
        2021-10-01 (ANI717 - Animesh Bala Ani): Baseline Software.

Example:
        $ colcon build --symlink-install && source install/local_setup.bash && ros2 run ros2_deep_learning_to_twist_message onnx
        $ source install/local_setup.bash && ros2 run ros2_deep_learning_to_twist_message onnx
        $ ros2 run ros2_deep_learning_to_twist_message onnx

"""


#___Import Modules:
import os
import json
import cv2
import numpy as np
import onnxruntime as ort

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from ament_index_python.packages import get_package_share_directory


#___Global Variables:
SETTINGS = os.path.join(get_package_share_directory('ros2_deep_learning_to_twist_message'), "settings.json")
MODELS = os.path.join(get_package_share_directory('ros2_deep_learning_to_twist_message'), "models")


#___Functions
def calibration(z, x, previous_z, previous_x):
    return round(z/5 - 1, 1), 0.7


def normalize(img, mean, std, max_pixel_value=255.0):
    mean = np.array(mean, dtype=np.float32)
    mean *= max_pixel_value

    std = np.array(std, dtype=np.float32)
    std *= max_pixel_value

    denominator = np.reciprocal(std, dtype=np.float32)

    img = img.astype(np.float32)
    img -= mean
    img *= denominator
    return img


#__Classes
class ONNXTwist(Node):
    """ONNX Twist Class.
    
    This class contains all methods to predict twist message from ONNX model. 
    
    """

    def __init__(self, xsession, zsession, model_input_shape, image_topic='/image', twist_topic='/cmd_vel', publish_frequency=100):
        super().__init__('onnx_publisher')
        
        # initialize subscriber
        self.subscription = self.create_subscription(Image, image_topic, self.listener_callback, 1)
        self.subscription  # prevent unused variable warning
        
        # initialize publisher
        self.publisher_ = self.create_publisher(Twist, twist_topic, 1)
        self.timer = self.create_timer(1/publish_frequency, self.timer_callback)
        
        # variable initialization
        self.model_input_shape = model_input_shape
        self.xsession = xsession
        self.zsession = zsession
        self.z = 0.0
        self.x = 0.0
        
    
    def listener_callback(self, msg):
        """Listener Callback Function
        
        This method subscribes to image topic, reads image data and makes 
        prediction.
        
        """

        # parses data from subscribed topic message
        height = msg.height
        width = msg.width
        channel = msg.step//msg.width
        frame = np.reshape(msg.data, (height, width, channel))
        
        # transform image
        frame = cv2.resize(frame, dsize=self.model_input_shape[-2:])
        frame = normalize(frame, mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5], max_pixel_value=255.0)
        frame = cv2.dnn.blobFromImage(frame)
        
        # makes prediction for angular z and linear x value from onnx model
        self.z = self.zsession.run(None, {"input.1": frame.astype(np.float32)},)[0][0][0]
        self.x = self.xsession.run(None, {"input.1": frame.astype(np.float32)},)[0][0][0]
        
        self.get_logger().info("Angular Z: {:.1f}, Linear X: {:.1f}".format(self.z, self.x))
        
        return None
    

    def timer_callback(self):
        """Timer Callback Function
        
        This method publishes gamepad data as Twist message.
        
        """

        # initializes Twist message
        twist = Twist()
        
        # creates Twist message
        twist.angular.z, twist.linear.x = calibration(self.z, self.x, twist.angular.z, twist.linear.x)

        # publishes message
        self.publisher_.publish(twist)
        self.get_logger().info("Angular Z: {:.1f}, Linear X: {:.1f}".format(self.z, self.x))

        return None


def main(args=None):
    """This is the Main Method.
    
    """
    
    # parse settings from json file
    with open(SETTINGS) as fp:
        content = json.load(fp)
        xsession = ort.InferenceSession(os.path.join(MODELS, content["xmodel"]))
        zsession = ort.InferenceSession(os.path.join(MODELS, content["zmodel"]))
        model_input_shape = tuple(content["model_input_shape"])
        image_topic = content["image_topic"]
        twist_topic = content["twist_topic"]
        publish_frequency = content["publish_frequency"]
    
    # initializes node and start publishing
    rclpy.init(args=args)
    onnx_publisher = ONNXTwist(xsession, zsession, model_input_shape, image_topic, twist_topic, publish_frequency)
    rclpy.spin(onnx_publisher)
    
    # shuts down and releases everything
    onnx_publisher.destroy_node()
    rclpy.shutdown()
    
    return None


if __name__ == '__main__':
    main()


#                                                                              
# end of file
"""ANI717"""
