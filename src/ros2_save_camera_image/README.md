<p align="center">
  <h1 align="center">ROS 2 Package to Save Anotated Camera Image</h1>
</p>

<p align="justify">
ROS 2 Package to Save Camera Image Published in ROS2 Topic along with Geometry Twist Message as Annotation. Collected ROS 2 Geometry Twist <code>linear.x</code> and <code>angular.z</code> data are mapped from the range of <code>floating</code>:<code>-1 to +1</code> to <code>integer</code>:<code>0 to 10</code>. Then these values are embeded in the image name. This approach doesn't require additional files for saving annotations separately. Example:<code>0000001_z05_x05.jpg</code>.
</p>

## Colaborators
[Animesh Bala Ani](https://www.linkedin.com/in/ani717/)


## Table of Contents
* [Install Dependency](#install) <br/>
* [Build, Source & Launch Package](#launch) <br/>
* [Launch Arguments](#arg) <br/>
* [Settings](#set) <br/>


## Install Dependency <a name="install"></a>
Install ROS2 dependency.<br/>
```
sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```


## Build, Source & Launch Package <a name="launch"></a>
```
colcon build --symlink-install --packages-select ros2_save_camera_image
source install/local_setup.bash
ros2 launch ros2_save_camera_image launch.py
```
```
colcon build --symlink-install && source install/local_setup.bash && ros2 launch ros2_save_camera_image launch.py
```


## Launch Arguments <a name="arg"></a>
Select `True` by editing `line 38` of `launch/launch.py` file to launch `ros2 cam2image` for collecting image data with camera.<br/>
Or use `True` as argument for `cam2image`.<br/>
Default `cam2image`:`True`<br/> 


## Settings <a name="set"></a>
Edit `settings.json` file to assign `image topic`, `twist topic` and `data directory`.<br/>
Default `image topic`:`\images`<br/>
Default `twist topic`:`\cmd_vel`<br/> 
Default `data directory`:`..\images`<br/>
