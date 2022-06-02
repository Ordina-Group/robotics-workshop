<p align="center">
  <h1 align="center">ANI717 Robot Workspace</h1>
</p>

<img src="https://github.com/ANI717/ANI717_Robotics/blob/main/Robot%20Workspace.png" alt="Robot Workspace Diagram" class="inline"/><br/>

## Colaborators
[Animesh Bala Ani](https://www.linkedin.com/in/ani717/)


## Table of Contents
* [Install Required Packages (Jetson Nano)](#jetson) <br/>
* [Add User to Input Group](#user) <br/>
* [Install Dependency](#install) <br/>
* [Build, Source & Launch Package (Gamepad)](#gamepad) <br/>
* [Build, Source & Launch Package (Self Driving))](#self) <br/>
* [Launch Arguments](#arg) <br/>
* [Zip Images for Download](#zip) <br/>


## Install Required Packages (Jetson Nano) <a name="jetson"></a>
Install `ROS2 Dashing`, `ONNXRuntime-GPU`, `Inputs` and `Adafruit_MotorHat`.<br/>
```
git clone https://github.com/ANI717/Headless-Jetson-Nano-Setup
cd ~/Headless-Jetson-Nano-Setup
chmod +x ./dashing.sh && ./dashing.sh
chmod +x ./torch2trt_onnx.sh && ./torch2trt_onnx.sh
sudo -H python3 -m pip install inputs Adafruit_MotorHat
```


## Add User to Input Group <a name="user"></a>
Run following command and reboot.<br/>
```
sudo gpasswd -a $USER input
```


## Install Dependency <a name="install"></a>
Install ROS2 dependency.<br/>
```
cd ~/ANI717_Robotics/simulation_ws/
sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```


## Build, Source & Launch Package (Gamepad) <a name="gamepad"></a>
```
cd ~/ANI717_Robotics/robot_ws/
colcon build --symlink-install && source install/local_setup.bash && ros2 launch robot_app gamepad_launch.py
```


## Build, Source & Launch Package (Self Driving) <a name="self"></a>
```
cd ~/ANI717_Robotics/robot_ws/
colcon build --symlink-install && source install/local_setup.bash && ros2 launch robot_app autonomous_launch.py
```


## Launch Arguments <a name="arg"></a>
Select Gamepad type from `logitech` or `waveshare` as arguments for `gamepad_type`.<br/>
Default `gamepad_type`:`logitech`<br/>

Select Deep Learning Model type as arguments for `model_type`.<br/>
Default `model_type`:`onnx`<br/>

Select `True` as argument for `cam2image` and `False` as argument for `csijetson` to run `cam2image`.<br/>
Select `False` as argument for `cam2image` and `True` as argument for `csijetson` to run `ros2_csi_camera_publish` package.<br/>
Default `cam2image`:`False`<br/>
Default `csijetson`:`True`<br/> 

Select Robot type from `jetbot` or `adafruit` as arguments for `robot_type`.<br/>
Default `robot_type`:`jetbot`<br/> 


## Zip Images for Download <a name="zip"></a>
```
cd ~/ANI717_Robotics/
zip -r images.zip images
```
