#!/bin/bash
set -e

CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
KINETIC_DIR="/home/yellow/work/ros-kinetic/catkin_ws"

echo "CURRENT_DIR: $CURRENT_DIR"
echo "KINETIC_DIR: $KINETIC_DIR"

if [ ! -f /etc/apt/sources.list.d/ros-latest.list ];
then
  echo "Configure apt sources."
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
fi
sudo apt-get update

echo "Installing dependencies."
sudo apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential
# FIXME: `python-numpy` not included in OpenCV3 dependencies.
sudo apt-get install -y python-catkin-tools python-numpy

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ];
then
  echo "Initialising rosdep."
  sudo rosdep init
fi
echo "Updating rosdep."
rosdep update

if [ ! -d $KINETIC_DIR ];
then
  echo "Creating Catkin workspace."
  mkdir -p $KINETIC_DIR
  echo "$CURRENT_DIR/kinetic-desktop-custom.rosinstall $KINETIC_DIR/kinetic-desktop-custom.rosinstall"
  cp $CURRENT_DIR/kinetic-desktop-custom.rosinstall $KINETIC_DIR/kinetic-desktop-custom.rosinstall
fi
echo "Changing directory to $KINETIC_DIR"
cd $KINETIC_DIR

#if [ ! -f $KINETIC_DIR/kinetic-desktop-full-wet.rosinstall ];
if [ ! -f $KINETIC_DIR/src/.rosinstall ];
then
  echo "Generating ROS workspace."
  # FIXME: Using own `.rosinstall` file to avoid broken packages. Could merge my changes.
  #rosinstall_generator desktop_full --rosdistro kinetic --deps --wet-only > kinetic-desktop-full-wet.rosinstall
  #wstool init -j8 src kinetic-desktop-full-wet.rosinstall
  wstool init -j8 src kinetic-desktop-custom.rosinstall
else
  echo "Updating ROS workspace."
  wstool update -j8 -t src
fi

echo "Installing dependency packages."
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y --os $(lsb_release -si | awk '{print tolower($0)}'):$(lsb_release -sc)

echo "Disabling CUDA support."
catkin config --cmake-args -DWITH_CUDA=OFF -DBUILD_opencv_gpu=OFF

echo "Build workspace? [Y/n]"
read input_variable
if echo "$input_variable" | grep -iq "^n";
then
  echo "Skipping workspace build."
else
  echo "Building workspace with Catkin."
  catkin build
fi
