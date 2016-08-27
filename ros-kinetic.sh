#!/bin/bash
set -o errexit
set -o pipefail

CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
KINETIC_DIR="/home/$USER/work/ros-kinetic"

RED='\033[0;31m'
YEL='\033[1;33m'
NC='\033[0m' # No Color

SKIP_UPDATES=0
NO_GPU=1

PROC_CNT=`getconf _NPROCESSORS_ONLN 2>/dev/null || getconf NPROCESSORS_ONLN 2>/dev/null || echo 1`

echo -e "${YEL}CURRENT_DIR: $CURRENT_DIR${NC}"
echo -e "${YEL}KINETIC_DIR: $KINETIC_DIR${NC}"
echo -e "${YEL}PROC_CNT: $PROC_CNT${NC}"

# Ask for SSH passphrase
# FIXME: Already set after reboot, although passphrase not saved.
#if [ -z "$SSH_AUTH_SOCK" ];
#then
ssh-add
#fi

if [ ! -f /etc/apt/sources.list.d/ros-latest.list ];
then
  echo -e "${YEL}Configure apt sources.${NC}"
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
fi
if [ ! $SKIP_UPDATES -eq 1 ];
then
  echo -e "${YEL}Updating apt.${NC}"
  sudo apt-get update

  # TODO: Prompt to `sudo apt-get remove --purge ros-kinetic*`

  echo -e "${YEL}Installing dependencies.${NC}"
  sudo apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential
  # FIXME: `python-numpy` not included in OpenCV3 dependencies.
  sudo apt-get install -y python-catkin-tools python-empy python-numpy
  sudo apt-get install -y cmake-qt-gui
else
  echo -e "${RED}Updating apt - skipped.${NC}"
fi

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ];
then
  echo -e "${YEL}Initialising rosdep.${NC}"
  sudo rosdep init
fi

# Create local rosdep repo
if [ ! -f /etc/ros/rosdep/local.yaml ];
then
  echo -e "${YEL}Creating rosdep local.yaml.${NC}"
  sudo cp -u $CURRENT_DIR/assets/rosdep/local.yaml /etc/ros/rosdep/local.yaml
fi
if [ ! -f /etc/ros/rosdep/sources.list.d/10-local.list ];
then
  echo -e "${YEL}Adding rosdep local.yaml.${NC}"
  sudo cp -u $CURRENT_DIR/assets/rosdep/10-local.list /etc/ros/rosdep/sources.list.d/10-local.list
fi

if [ ! $SKIP_UPDATES -eq 1 ];
then
  echo -e "${YEL}Updating rosdep.${NC}"
  rosdep update
else
  echo -e "${RED}Updating rosdep - skipped.${NC}"
fi

# Create workspaces given path and rosinstall file.
function workspace {
  DIR="$KINETIC_DIR/$1" # Workspace path
  ROS=$2 # rosinstall file

  if [ -z "$3" ];
  then
    EXT=""
  else
    EXT="$KINETIC_DIR/$3/devel" # Extend workspace
  fi

  echo -e "${YEL}Workspace: $DIR - $ROS${NC}"

  if [ ! -d $DIR ];
  then
    echo -e "${YEL}Creating Catkin workspace.${NC}"
    mkdir -p $DIR
  fi

  echo -e "${YEL}Copying $ROS to workspace.${NC}"
  cp -u $CURRENT_DIR/assets/rosinstall/$ROS $DIR/$ROS

  if [[ (! -d $DIR/.catkin_tools) || (! -f $DIR/src/.rosinstall) ]];
  then
    echo -e "${YEL}Initialising catkin workspace.${NC}"
    catkin init -w$DIR # --reset #> /dev/null #-s$DIR/src -l$DIR/log -b$DIR/build -d$DIR/devel -i$DIR/install

    if [ $NO_GPU -eq 1 ];
    then
      echo -e "${YEL}Disabling CUDA support.${NC}"
      catkin config -w$DIR --cmake-args -DWITH_CUDA=OFF -DBUILD_opencv_gpu=OFF #> /dev/null
    fi

    if [ ! -z $EXT ];
    then
      echo -e "${YEL}Extending workspace $EXT.${NC}"
      catkin config -w$DIR --extend $EXT #> /dev/null
    #else
    #  echo -e "${YEL}Root workspace.${NC}"
    #  catkin config -w$DIR --no-extend
    fi

    if [ -z $EXT ];
    then
      echo -e "${YEL}Initialising rosinstall for root workspace.${NC}"
      cp -u $CURRENT_DIR/assets/rosinstall/kinetic-init.rosinstall $DIR/kinetic-init.rosinstall
      wstool init -j $PROC_CNT $DIR/src $DIR/kinetic-init.rosinstall
    else
      echo -e "${YEL}Initialising rosinstall for extended workspace.${NC}"
      wstool init -j $PROC_CNT $DIR/src
    fi

    # Generate `devel/setup.bash`
    catkin build -j $PROC_CNT -w $DIR
  fi

  #rosinstall_generator desktop_full --rosdistro kinetic --deps --wet-only > kinetic-desktop-full-wet.rosinstall

  # Merge in any updates to original rosinstall.
  wstool merge -ky -t $DIR/src $DIR/$ROS
  if [ ! $SKIP_UPDATES -eq 1 ];
  then
    echo -e "${YEL}Updating ROS workspace.${NC}"
    wstool update -j $PROC_CNT -t $DIR/src
  else
    echo -e "${RED}Updating ROS workspace - skipped.${NC}"
  fi

  # Show resulting config
  #catkin config -w$DIR

  echo -e "${YEL}Installing dependency packages.${NC}"
  (source $DIR/devel/setup.bash && rosdep install --from-paths $DIR/src --ignore-src --rosdistro kinetic --os $(lsb_release -si | awk '{print tolower($0)}'):$(lsb_release -sc))

  #echo -e "${YEL}Build workspace $DIR? [Y/n]${NC}"
  #read input_variable
  #if echo "$input_variable" | grep -iq "^n";
  #then
  #  echo -e "${YEL}Skipping workspace build.${NC}"
  #else
    echo -e "${YEL}Building workspace with Catkin.${NC}"
    (source $DIR/devel/setup.bash && catkin build -j $PROC_CNT -w $DIR)
  #fi
}

workspace root_ws kinetic-desktop-custom.rosinstall
workspace kulbu_ws kinetic-kulbu.rosinstall root_ws
