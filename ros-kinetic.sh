#!/bin/bash
set -e

CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
KINETIC_DIR="/home/yellow/work/ros-kinetic"

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

# Create workspaces given path and rosinstall file.
function workspace {
  DIR="$KINETIC_DIR/$1" # Workspace path
  ROS=$2 # rosinstall file

  echo -e "${YEL}Workspace: $DIR - $ROS${NC}"

  if [ ! -d $DIR ];
  then
    echo -e "${YEL}Creating Catkin workspace.${NC}"
    mkdir -p $DIR
    echo -e "${YEL}$CURRENT_DIR/$ROS $DIR/$ROS${NC}"
    cp $CURRENT_DIR/$ROS $DIR/$ROS
  fi
  echo -e "${YEL}Changing directory to $DIR${NC}"
  cd $DIR

  #if [ ! -f $KINETIC_DIR/kinetic-desktop-full-wet.rosinstall ];
  if [ ! -f $DIR/src/.rosinstall ];
  then
    echo -e "${YEL}Generating ROS workspace.${NC}"
    # FIXME: Using own `.rosinstall` file to avoid broken packages. Could merge my changes.
    #rosinstall_generator desktop_full --rosdistro kinetic --deps --wet-only > kinetic-desktop-full-wet.rosinstall
    #wstool init -j8 src kinetic-desktop-full-wet.rosinstall
    wstool init -j8 src $ROS
  else
    echo -e "${YEL}Updating ROS workspace.${NC}"
    # Merge in any updates to original rosinstall.
    wstool merge -t src $ROS
    wstool update -j 8 -t src
  fi

  echo -e "${YEL}Installing dependency packages.${NC}"
  rosdep install --from-paths src --ignore-src --rosdistro kinetic -y --os $(lsb_release -si | awk '{print tolower($0)}'):$(lsb_release -sc)

  echo -e "${YEL}Disabling CUDA support.${NC}"
  catkin config --cmake-args -DWITH_CUDA=OFF -DBUILD_opencv_gpu=OFF

  echo -e "${YEL}Build workspace $DIR? [Y/n]${NC}"
  read input_variable
  if echo "$input_variable" | grep -iq "^n";
  then
    echo -e "${YEL}Skipping workspace build.${NC}"
  else
    echo -e "${YEL}Building workspace with Catkin.${NC}"
    catkin build
  fi
}

workspace catkin_ws kinetic-desktop-custom.rosinstall
source $KINETIC_DIR/catkin_ws/devel/setup.bash
workspace ws kinetic-kulbu.rosinstall
