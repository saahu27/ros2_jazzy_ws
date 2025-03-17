#! /bin/bash

export DEBIAN_FRONTEND=noninteractive

sudo apt-get update -q \
    && sudo apt-get upgrade -y \
    && sudo apt-get dist-upgrade -y

sudo apt-get install curl lsb-release gnupg

sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update -q
sudo apt-get install -y gz-harmonic

sudo apt-get update -q && \
  apt-get install ros-${ROS_DISTRO}-ros-gz-sim -y \
                  ros-${ROS_DISTRO}-ros-gz-bridge -y \
                  ros-${ROS_DISTRO}-gz-ros2-control -y
