
<p align="center">
  <h1 align="center">Ordina Robotics Workspace</h1>
</p>

<p align="justify">
Contains ROS2 packages to run robot car to collect annotated camera images while controlled by Gamepad.
</p>


## Quick Links
* [Robot Workspace](https://github.com/OrdinaNederland/robotics-workshop/tree/main/src)
* [Robot Workshop Documentation](https://github.com/OrdinaNederland/robotics-docs/wiki)


## Colaborators
* [Ricky van Rijn](https://www.linkedin.com/in/rickyvanrijn) (Software Development)<br/>
* [Remco van Gorsel](https://www.linkedin.com/in/remco-van-gorsel) (Software Development)<br/>
* [Louigimar Richardson](https://nl.linkedin.com/in/louigimar-richardson-04265a106) (Software Development)

## Design Diagram
<img src="https://github.com/OrdinaNederland/robotics-workshop/blob/main/Robot%20Workspace.png" alt="Robot Workspace Diagram" class="inline"/><br/>

## Directory Tree
```
Robotics Platform
    ├── Robot Workspace
    │   └── src
    │        ├── ROS2 Gamepad to Twist Message
    │        ├── ROS2 Deep Learning to Twist Message
    │        ├── ROS2 Twist Message to Robot Motion
    │        ├── ROS2 CSI Camera Publish
    │        ├── ROS2 Save Camera Image
    │        └── Robot App
    └──
```

## Developed ROS2 Packages
#### Common Packages:<br/>
* [ros2_deep_learning_model_to_twist_message](https://github.com/OrdinaNederland/robotics-workshop/tree/main/src/ros2_deep_learning_to_twist_message)<br/>
* [ros2_save_camera_image](https://github.com/OrdinaNederland/robotics-workshop/tree/main/src/ros2_save_camera_image)<br/>

#### Robot Packages:<br/>
* [ros2_csi_camera_publish](https://github.com/OrdinaNederland/robotics-workshop/tree/main/src/ros2_csi_camera_publish)<br/>
* [ros2_gamepad_to_twist_message](https://github.com/OrdinaNederland/robotics-workshop/tree/main/src/ros2_gamepad_to_twist_message)<br/>
* [ros2_twist_message_to_robot_motion](https://github.com/OrdinaNederland/robotics-workshop/tree/main/src/ros2_twist_message_to_robot_motion)<br/>

## [Robot Workspace](https://github.com/OrdinaNederland/robotics-workshop/tree/main/src) (JetBot)
#### Download Workspace
```
git clone https://github.com/OrdinaNederland/robotics-workshop
```

#### Build, Source & Launch Package (Gamepad)
```
cd ~/robotics-workshop
chmod +x host_rtsp_server
colcon build --symlink-install && source install/local_setup.bash
ROS_DOMAIN_ID=<INSERT ROBOT NUMBER> ros2 launch robot_app gamepad_launch.py gamepad_type:=playstation
```

#### first time building (needed for livestream gstreamer):
```
sudo apt-get install libgstrtspserver-1.0 libgstreamer1.0-dev
```

## Acknowledgement
[Jetbot](https://jetbot.org/master/)<br/>
[Jetbot URDF Mesh](https://github.com/aws-samples/aws-robomaker-jetbot-ros)<br/>
[AWS Robomaker Racetrack World](https://github.com/aws-robotics/aws-robomaker-racetrack-world)<br/>
[QEngineering Base Image Ubuntu 20.04](https://github.com/Qengineering/Jetson-Nano-Ubuntu-20-image)<br/>
