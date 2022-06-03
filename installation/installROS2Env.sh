#!/bin/bash

sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

wget https://github.com/ros2/ros2/releases/download/release-foxy-20220208/ros2-foxy-20220208-linux-focal-arm64.tar.bz2 -P ~/Downloads

mkdir -p ~/ros2_foxy
cd ~/ros2_foxy
tar xf ~/Downloads/ros2-foxy-20220208-linux-focal-arm64.tar.bz2

sudo apt update -y
sudo apt upgrade -y
sudo apt install -y python3-rosdep
sudo rosdep init
rosdep update

export ROS_DISTRO=foxy
rosdep install --from-paths ~/ros2_foxy/ros2-linux/share --ignore-src -y --skip-keys "cyclonedds fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers"

echo "source ~/ros2_foxy/ros2-linux/setup.bash" >> ~/.bashrc
source .bashrc

sudo apt install python3-colcon-common-extensions -y

mkdir -p ~/dev_ws/src
