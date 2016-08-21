#!/bin/bash
set -e

CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
KINETIC_DIR="/home/yellow/work/ros-kinetic/catkin_ws"

RED='\033[0;31m'
YEL='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YEL}CURRENT_DIR: $CURRENT_DIR${NC}"
echo -e "${YEL}KINETIC_DIR: $KINETIC_DIR${NC}"

if [ ! -f /etc/apt/sources.list.d/ros-latest.list ];
then
  echo -e "${YEL}Configure apt sources.${NC}"
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
fi
sudo apt-get update

echo -e "${YEL}Installing dependencies.${NC}"
sudo apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential
# FIXME: `python-numpy` not included in OpenCV3 dependencies.
sudo apt-get install -y python-catkin-tools python-numpy

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ];
then
  echo -e "${YEL}Initialising rosdep.${NC}"
  sudo rosdep init
fi
echo -e "${YEL}Updating rosdep.${NC}"
rosdep update

if [ ! -d $KINETIC_DIR ];
then
  echo -e "${YEL}Creating Catkin workspace.${NC}"
  mkdir -p $KINETIC_DIR
  echo -e "${YEL}$CURRENT_DIR/kinetic-desktop-custom.rosinstall $KINETIC_DIR/kinetic-desktop-custom.rosinstall${NC}"
  cp $CURRENT_DIR/kinetic-desktop-custom.rosinstall $KINETIC_DIR/kinetic-desktop-custom.rosinstall
fi
echo -e "${YEL}Changing directory to $KINETIC_DIR${NC}"
cd $KINETIC_DIR

#if [ ! -f $KINETIC_DIR/kinetic-desktop-full-wet.rosinstall ];
if [ ! -f $KINETIC_DIR/src/.rosinstall ];
then
  echo -e "${YEL}Generating ROS workspace.${NC}"
  # FIXME: Using own `.rosinstall` file to avoid broken packages. Could merge my changes.
  #rosinstall_generator desktop_full --rosdistro kinetic --deps --wet-only > kinetic-desktop-full-wet.rosinstall
  #wstool init -j8 src kinetic-desktop-full-wet.rosinstall
  wstool init -j8 src kinetic-desktop-custom.rosinstall
else
  echo -e "${YEL}Updating ROS workspace.${NC}"
  # Merge in any updates to original rosinstall.
  wstool merge -t src kinetic-desktop-custom.rosinstall
  wstool update -j8 -t src
fi

echo -e "${YEL}Installing dependency packages.${NC}"
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y --os $(lsb_release -si | awk '{print tolower($0)}'):$(lsb_release -sc)

echo -e "${YEL}Disabling CUDA support.${NC}"
catkin config --cmake-args -DWITH_CUDA=OFF -DBUILD_opencv_gpu=OFF

echo -e "${YEL}Build workspace? [Y/n]${NC}"
read input_variable
if echo "$input_variable" | grep -iq "^n";
then
  echo -e "${YEL}Skipping workspace build.${NC}"
else
  echo -e "${YEL}Building workspace with Catkin.${NC}"
  catkin build
fi
