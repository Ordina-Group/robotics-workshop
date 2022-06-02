<p align="center">
  <h1 align="center">ANI717 Simulation Workspace</h1>
</p>

<img src="https://github.com/ANI717/ANI717_Robotics/blob/main/Simulation%20Workspace.png" alt="Simulation Workspace Diagram" class="inline"/><br/>

## Colaborators
[Animesh Bala Ani](https://www.linkedin.com/in/ani717/)

## Table of Contents
* [Install Dependency](#install) <br/>
* [Build, Source & Launch Package (Keyboard)](#keyboard) <br/>
* [Build, Source & Launch Package (Self Driving))](#self) <br/>
* [Launch Arguments](#arg) <br/>
* [Zip Images for Download](#zip) <br/>


## Install Dependency <a name="install"></a>
Install `OpenCV`, `ONNXRuntime-GPU` and `Getch`.<br/>
```
python3 -m pip install opencv-python onnxruntime-gpu getch
```
Install ROS2 dependency.<br/>
```
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```


## Build, Source & Launch Package (Keyboard) <a name="keyboard"></a>
Terminal 1
```
cd ANI717_Robotics/simulation_ws/
export DISPLAY=:0
colcon build --symlink-install && source install/local_setup.bash && ros2 launch simulation_app keyboard_launch.py
```

Terminal 2 (Run following commands and select `/image` as `Image View`
```
export DISPLAY=:0
rqt
```


Terminal 3
```
cd ANI717_Robotics/simulation_ws/
source install/local_setup.bash && ros2 run ros2_keyboard_to_twist_message execute
```


## Build, Source & Launch Package (Self Driving) <a name="self"></a>
```
cd ANI717_Robotics/simulation_ws/
export DISPLAY=:0
colcon build --symlink-install && source install/local_setup.bash && ros2 launch simulation_app autonomous_launch.py
```


## Launch Arguments <a name="arg"></a>
Select `True` or `False` as arguments for `use_simulator` to decide whether to launch `gazebo server`.<br/>
Default `use_simulator`:`True`<br/>

Select `True` or `False` as arguments for `headless` to decide whether not to launch `gazebo client`.<br/>
Default `headless`:`False`<br/>

Select `complete path of world file` as arguments for `world` to simulate that world.<br/>
Default `world`:`os.path.join(ros2_world_simulation_dir, 'worlds', 'racetrack_day.world')`<br/>

Select `name of robot urdf file` as arguments for `urdf_file` to simulate that robot.<br/>
Default `urdf_file`:`jetbot.xml`<br/>

Select Deep Learning Model type as arguments for `model_type`.<br/>
Default `model_type`:`onnx`<br/>

Select required values of following arguments for robot's initial position.<br/>
Default `x_pos`:`2.75`<br/>
Default `y_pos`:`-14.0`<br/>
Default `z_pos`:`0.5`<br/>
Default `roll`:`0.0`<br/>
Default `pitch`:`0.0`<br/>
Default `yaw`:`0.0`<br/>


## Zip Images for Download <a name="zip"></a>
```
cd ../
zip -r images.zip images
```
