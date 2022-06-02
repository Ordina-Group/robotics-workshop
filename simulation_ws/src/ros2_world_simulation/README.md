<p align="center">
  <h1 align="center">ROS 2 World Simulation</h1>
</p>

<p align="justify">
ROS 2 Package to Simulate a World File in Gazebo Simulation. One can put any <code>*.world</code> file in <code>worlds</code> folder and related <code>material</code>, <code>techture</code> and <code>mesh</code> files in <code>models</code> folder. Change <code>line 57</code> in <code>launch/launch.py</code> to select the world file. Default is <code>empty.world</code>.
</p>


## Colaborators
[Animesh Bala Ani](https://www.linkedin.com/in/ani717/)


## Table of Contents
* [Demonstration](#demo) <br/>
* [Install Dependency](#install) <br/>
* [Build, Source & Launch Package](#launch) <br/>
* [Launch Arguments](#arg) <br/>


## Demonstration <a name="demo"></a>
Demonstration of `worlds/racetrack_day.world` simulation.<br/>

<img src="https://github.com/ANI717/ANI717_Robotics/blob/main/world_simulation.jpg" alt="World Simulation" class="inline"/><br/>


## Install Dependency <a name="install"></a>
Install ROS2 dependency.<br/>
```
sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```


## Build, Source & Launch Package <a name="launch"></a>
```
export DISPLAY=:0
colcon build --symlink-install --packages-select ros2_world_simulation
source install/local_setup.bash
ros2 launch ros2_world_simulation launch.py
```
```
colcon build --symlink-install && source install/local_setup.bash && ros2 launch ros2_world_simulation launch.py
```


## Launch Arguments <a name="arg"></a>
Select `True` or `False` as arguments for `use_simulator` to decide whether to launch `gazebo server`.<br/>
Default `use_simulator`:`True`<br/>

Select `True` or `False` as arguments for `headless` to decide whether not to launch `gazebo client`.<br/>
Default `headless`:`False`<br/>

Select `complete path of world file` as arguments for `world` to simulate that world.<br/>
Default `world`:`os.path.join(package_dir, 'worlds', 'empty.world')`<br/>
