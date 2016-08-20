#!/bin/bash
set -e

if [ ! -f /etc/apt/sources.list.d/ros-latest.list ]
then
  echo "Configure apt sources."
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
fi
sudo apt-get update

echo "Installing dependencies."
sudo apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential


if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]
then
  echo "Initialising rosdep."
  sudo rosdep init
fi
echo "Updating rosdep."
rosdep update

if [ ! -d ~/work/ros/catkin_ws ]
then
  echo "Creating Catkin workspace."
  mkdir -p ~/work/ros/catkin_ws
fi
cd ~/work/ros/catkin_ws

if [ ! -f ~/work/ros/catkin_ws/kinetic-desktop-full-wet.rosinstall ]
then
  echo "Generating ROS workspace."
  rosinstall_generator desktop_full --rosdistro kinetic --deps --wet-only > kinetic-desktop-full-wet.rosinstall
  wstool init -j8 src kinetic-desktop-full-wet.rosinstall
else
  echo "Updating ROS workspace."
  wstool update -j8 -t src
fi

echo "Installing dependency packages."
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y --os $(lsb_release -si | awk '{print tolower($0)}'):$(lsb_release -sc)

echo "Disabling CUDA support."
catkin config --cmake-args -DWITH_CUDA=OFF -DBUILD_opencv_gpu=OFF
echo "Building ROS workspace."
catkin build
