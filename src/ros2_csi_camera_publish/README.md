<p align="center">
  <h1 align="center">ROS 2 CSI Camera Image Publish Package for Jetson Nano</h1>
</p>

<p align="justify">
ROS 2 Package to Publish CSI Camera Image as <code>sensor_msgs/Image</code> message on <code>Jetson Nano</code>.
</p>


## Colaborators
[Animesh Bala Ani](https://www.linkedin.com/in/ani717/)

## Table of Contents
* [Install Dependency](#install) <br/>
* [Build, Source & Run Package](#run) <br/>
* [Settings](#set) <br/>


## Install Dependency <a name="install"></a>
Install ROS2 dependency.<br/>
```
sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```


## Build, Source & Run Package <a name="run"></a>
```
colcon build --symlink-install --packages-select ros2_csi_camera_publish
source install/local_setup.bash
ros2 run ros2_csi_camera_publish jetson
```
```
colcon build --symlink-install && source install/local_setup.bash && ros2 run ros2_csi_camera_publish jetson
```


## Settings <a name="set"></a>
Edit `settings.json` file to assign `publish_topic`, `publish frequency`, `capture_width`, `capture_height`, `framerate`, `flip_method`, `display_width` and `display_height`.<br/>
Default `publish_topic`:`\image`<br/> 
Default `publish frequency`:`100`<br/>
Default `capture_width`:`320`<br/>
Default `capture_height`:`240`<br/>
Default `framerate`:`30`<br/>
Default `flip_method`:`0`<br/>
Default `display_width`:`320`<br/>
Default `display_height`:`240`<br/>
