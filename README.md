
<p align="center">
  <h1 align="center">Ordina Robotics Workspace</h1>
</p>

<p align="justify">
Contains ROS2 packages to run robot car to collect annotated camera images while controlled by Gamepad. Also contains packages to run a robot car autonomously with a trained neural network.
</p>


## Quick Links
[Robot Workspace](https://github.com/OrdinaNederland/RoboticaWorkshopRobot/tree/main/robot_ws)<br/>
[Simulation Workspace](https://github.com/OrdinaNederland/RoboticaWorkshopRobot/tree/main/simulation_ws)<br/>


## Colaborators
* [Ricky van Rijn](https://www.linkedin.com/in/rickyvanrijn) (Software Development)<br/>
* [Remco van Gorsel](https://www.linkedin.com/in/remco-van-gorsel) (Software Development)<br/>


## Design Diagram
<img src="https://github.com/OrdinaNederland/RoboticaWorkshopRobot/blob/main/Robot%20Workspace.png" alt="Robot Workspace Diagram" class="inline"/><br/>
<img src="https://github.com/OrdinaNederland/RoboticaWorkshopRobot/blob/main/Simulation%20Workspace.png" alt="Simulation Workspace Diagram" class="inline"/><br/>

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
    └── Simulation Workspace
        └── src
             ├── ROS2 Keyboard to Twist Message
             ├── ROS2 Deep Learning to Twist Message
             ├── ROS2 Save Camera Image
             ├── ROS2 World Simulation
             ├── ROS2 Robot Simulation
             └── Simulation App
```

## Developed ROS2 Packages
#### Common Packages:<br/>
* [ros2_deep_learning_model_to_twist_message](https://github.com/OrdinaNederland/RoboticaWorkshopRobot/tree/main/robot_ws/src/ros2_deep_learning_to_twist_message)<br/>
* [ros2_save_camera_image](https://github.com/OrdinaNederland/RoboticaWorkshopRobot/tree/main/robot_ws/src/ros2_save_camera_image)<br/>

#### Robot Packages:<br/>
* [ros2_csi_camera_publish](https://github.com/OrdinaNederland/RoboticaWorkshopRobot/tree/main/robot_ws/src/ros2_csi_camera_publish)<br/>
* [ros2_gamepad_to_twist_message](https://github.com/OrdinaNederland/RoboticaWorkshopRobot/tree/main/robot_ws/src/ros2_gamepad_to_twist_message)<br/>
* [ros2_twist_message_to_robot_motion](https://github.com/OrdinaNederland/RoboticaWorkshopRobot/tree/main/robot_ws/src/ros2_twist_message_to_robot_motion)<br/>

#### Simulation Packages:<br/>
* [ros2_keyboard_to_twist_message](https://github.com/OrdinaNederland/RoboticaWorkshopRobot/tree/main/simulation_ws/src/ros2_keyboard_to_twist_message)<br/>
* [ros2_world_simulation](https://github.com/OrdinaNederland/RoboticaWorkshopRobot/tree/main/simulation_ws/src/ros2_world_simulation)<br/>
* [ros2_robot_simulation](https://github.com/OrdinaNederland/RoboticaWorkshopRobot/tree/main/simulation_ws/src/ros2_robot_simulation)<br/>


## [Robot Workspace](https://github.com/OrdinaNederland/RoboticaWorkshopRobot/tree/main/robot_ws) (JetBot)
#### Download Workspace
```
git clone https://github.com/OrdinaNederland/RoboticaWorkshopRobot
```

#### Build, Source & Launch Package (Gamepad)
```
cd ~/RoboticaWorkshopRobot/robot_ws/
colcon build --symlink-install && source install/local_setup.bash
ros2 launch robot_app gamepad_launch.py
```

#### Build, Source & Launch Package (Self Driving)
```
cd ~/RoboticaWorkshopRobot/robot_ws/
colcon build --symlink-install && source install/local_setup.bash
ros2 launch robot_app autonomous_launch.py
```


## [Simulation Workspace](https://github.com/OrdinaNederland/RoboticaWorkshopRobot/tree/main/simulation_ws) (AWS RoboMaker)
#### Download Workspace
```
git clone https://github.com/OrdinaNederland/RoboticaWorkshopRobot
```

#### Install ROS2 Dependency
```
cd ~/RoboticaWorkshopRobot/simulation_ws/
sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

#### Build, Source & Launch Package (Keyboard)
Terminal 1
```
cd ~/RoboticaWorkshopRobot/simulation_ws/
export DISPLAY=:0
colcon build --symlink-install && source install/local_setup.bash
ros2 launch simulation_app keyboard_launch.py
```

Terminal 2 (Run following commands and select `/image` as `Image View`
```
export DISPLAY=:0
rqt
```

Terminal 3
```
cd ~/RoboticaWorkshopRobot/simulation_ws/
source install/local_setup.bash && ros2 run ros2_keyboard_to_twist_message execute
```

#### Build, Source & Launch Package (Self Driving)
```
cd ~/RoboticaWorkshopRobot/simulation_ws/
export DISPLAY=:0
colcon build --symlink-install && source install/local_setup.bash
ros2 launch simulation_app autonomous_launch.py
```


## Acknowledgement
[Jetbot](https://jetbot.org/master/)<br/>
[Jetbot URDF Mesh](https://github.com/aws-samples/aws-robomaker-jetbot-ros)<br/>
[AWS Robomaker Racetrack World](https://github.com/aws-robotics/aws-robomaker-racetrack-world)<br/>
[QEngineering Base Image Ubuntu 20.04](https://github.com/Qengineering/Jetson-Nano-Ubuntu-20-image)<br/>
