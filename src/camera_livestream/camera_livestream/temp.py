import os
import cv2
import json
import numpy as np


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String
from std_msgs.msg import String, UInt8MultiArray, MultiArrayDimension 
import random

fp ={
    "publish_topic": "/image",
    "publish_frequency": 10,
    "capture_width": 720,
    "capture_height": 480,
    "framerate": 30,
    "flip_method": 0,
    "display_width": 320,
    "display_height": 240,
    "trigger_topic": "/camera_trigger",
    "camera_trigger_message": "pressed",
    "image_location": "snapshot_files"
}
json_settings = fp
 

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
# temp_message = random.choices(range(256),k=10000)
cap = cv2.VideoCapture(gstreamer_pipeline())
os.environ['DISPLAY']=':0'
while cap.isOpened():
    # reads image data
    ret, frame = cap.read()
    msg_image = UInt8MultiArray()
    
    # real_message = temp_message
    # real_message = np.array(frame, dtype=np.uint8)
    # print(frame)
    # print(real_message)

    # msg_image.layout.dim.append(MultiArrayDimension())
    # msg_image.layout.dim.append(MultiArrayDimension())
    # msg_image.layout.dim.append(MultiArrayDimension())
    # msg_image.layout.dim[0].label = "height"
    # msg_image.layout.dim[0].size = 1080
    # msg_image.layout.dim[0].stride = 3*1920*1080 
    # msg_image.layout.dim[1].label = "width"
    # msg_image.layout.dim[1].size = 1920
    # msg_image.layout.dim[1].stride = 3*1920
    # msg_image.layout.dim[2].label = "channel"
    # msg_image.layout.dim[2].size = 3
    # msg_image.layout.dim[2].stride = 3
    # msg_image.layout.data_offset = 0
    msg_image.data = frame.flatten().tolist()
    # msg_image.data = real_message
    # processes image data and converts to ros 2 message
    cv2.imshow("Video Frame", frame)
    if cv2.waitKey(25) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()