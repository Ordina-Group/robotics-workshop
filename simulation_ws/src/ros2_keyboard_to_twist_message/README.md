<p align="center">
  <h1 align="center">ROS 2 Deep Learning Package for Robot Movement</h1>
</p>

<p align="justify">
ROS 2 Package to Publish Gamepad Controller Inputs to Twist Message for Robot Movement.
</p>

## Colaborators
[Animesh Bala Ani](https://www.linkedin.com/in/ani717/)


## Table of Contents
* [Key Mapping](#key) <br/>
* [Install Dependency](#install) <br/>
* [Build, Source & Run Package](#run) <br/>


## Key Mapping <a name="key"></a>
`W`:`+ linear.x`<br/>
`S`:`- linear.x`<br/>
`A`:`+ angular.z`<br/>
`D`:`- angular.z`<br/>

`L`:`linear.x = 0`<br/>
`K`:`angular.z = 0`<br/>

`linear.x range`:`-0.5 to +0.5`<br/>
`angular.z range`:`-1 to +1`<br/>


## Install Dependency <a name="install"></a>
Install `Getch`.<br/>
```
python3 -m pip install getch
```
Install ROS2 dependency.<br/>
```
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## Build, Source & Run Package <a name="run"></a>
```
colcon build --symlink-install --packages-select ros2_keyboard_to_twist_message
source install/local_setup.bash
ros2 run ros2_keyboard_to_twist_message execute
```
```
colcon build --symlink-install && source install/local_setup.bash && ros2 run ros2_keyboard_to_twist_message execute
```
