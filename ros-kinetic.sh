#!/bin/bash

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
  rosdep update
fi

echo "Creating catkin workspace."
mkdir -p ~/work/ros/catkin_ws
cd ~/work/ros/catkin_ws

if [ ! -f ~/work/ros/catkin_ws/kinetic-desktop-full-wet.rosinstall ]
then
  echo "Generating ROS workspace."
  rosinstall_generator desktop_full --rosdistro kinetic --deps --wet-only > kinetic-desktop-full-wet.rosinstall
  wstool init -j4 src kinetic-desktop-full-wet.rosinstall
fi
echo "Installing ROS packages."
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y --os debian:jessie
