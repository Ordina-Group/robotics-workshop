<p align="center">
  <h1 align="center">ROS 2 Package to Publish Gamepad Data as Twist Message</h1>
</p>

<p align="justify">
ROS 2 Package to Publish Gamepad Controller Data as Twist Message for Robot Movement.<br/>
</p>


## Colaborators
[Animesh Bala Ani](https://www.linkedin.com/in/ani717/)


## Table of Contents
* [Key Mapping](#key) <br/>
* [Install Dependency](#install) <br/>
* [Add User to Input Group](#user) <br/>
* [Build, Source & Launch Package](#launch) <br/>
* [Launch Arguments](#arg) <br/>
* [Settings](#set) <br/>


## Key Mapping <a name="key"></a>
`Left Joystick Up`:`+ linear.x`<br/>
`Left Joystick Down`:`- linear.x`<br/>
`Right Joystick Right`:`+ angular.z`<br/>
`Right Joystick Left`:`- angular.z`<br/>

`linear.x range`:`-1 to +1`<br/>
`angular.z range`:`-1 to +1`<br/>


## Install Dependency <a name="install"></a>
Install `Inputs`.<br/>
```
sudo -H python3 -m pip install inputs
```
Install ROS2 dependency.<br/>
```
sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```


## Add User to Input Group <a name="user"></a>
Run following command and reboot.<br/>
```
sudo gpasswd -a $USER input
```


## Build, Source & Launch Package <a name="launch"></a>
```
colcon build --symlink-install --packages-select ros2_gamepad_to_twist_message
source install/local_setup.bash
ros2 launch ros2_gamepad_to_twist_message launch.py
```
```
colcon build --symlink-install && source install/local_setup.bash && ros2 launch ros2_gamepad_to_twist_message launch.py
```


## Launch Arguments <a name="arg"></a>
Select Gamepad type from `logitech` or `waveshare` by editing `line 31` of `launch/launch.py` file.<br/>
Or use these names as arguments for `gamepad_type`.<br/>
Default `gamepad_type`:`logitech`<br/> 


## Settings <a name="set"></a>
Edit `settings.json` file to assign `publish topic` and `publish frequency`.<br/>
Default `publish topic`:`\cmd_vel`<br/> 
Default `publish frequency`:`100`<br/>
