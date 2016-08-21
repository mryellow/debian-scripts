#!/bin/bash
set -e

CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
KINETIC_DIR="/home/$USER/work/ros-kinetic"

RED='\033[0;31m'
YEL='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YEL}CURRENT_DIR: $CURRENT_DIR${NC}"
echo -e "${YEL}KINETIC_DIR: $KINETIC_DIR${NC}"

# Ask for SSH passphrase
# FIXME: Check need for auth before prompting.
ssh-add

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

# TODO: Add `/etc/ros/rosdep/local.yaml` with `python-pyassimp`
if [ ! -f /etc/ros/rosdep/local.yaml ];
then
  echo -e "${YEL}Creating rosdep local.yaml.${NC}"
  sudo tee /etc/ros/rosdep/local.yaml << EOF
python-pyassimp:
    debian:
      jessie:
        apt: [python-pyassimp]
EOF
fi
if [ ! -f /etc/ros/rosdep/sources.list.d/10-local.list ];
then
  echo -e "${YEL}Adding rosdep local.yaml.${NC}"
  sudo tee /etc/ros/rosdep/sources.list.d/10-local.list << EOF
yaml file:///etc/ros/rosdep/local.yaml
EOF
fi

echo -e "${YEL}Updating rosdep.${NC}"
rosdep update

# Create workspaces given path and rosinstall file.
function workspace {
  DIR="$KINETIC_DIR/$1" # Workspace path
  ROS=$2 # rosinstall file

  if [ -z "$3" ];
  then
    EXT=""
  else
    EXT="$KINETIC_DIR/$3" # Extend workspace
  fi

  echo -e "${YEL}Workspace: $DIR - $ROS${NC}"

  if [ ! -d $DIR ];
  then
    echo -e "${YEL}Creating Catkin workspace.${NC}"
    mkdir -p $DIR
  fi

  echo -e "${YEL}Copying $ROS to workspace.${NC}"
  cp -u $CURRENT_DIR/$ROS $DIR/$ROS

  if [ ! -f $DIR/src/.rosinstall ];
  then
    echo -e "${YEL}Generating ROS workspace.${NC}"
    # FIXME: Using own `.rosinstall` file to avoid broken packages. Could merge my changes.
    #rosinstall_generator desktop_full --rosdistro kinetic --deps --wet-only > kinetic-desktop-full-wet.rosinstall
    #wstool init -j8 src kinetic-desktop-full-wet.rosinstall
    wstool init -j8 $DIR/src $DIR/$ROS
  else
    echo -e "${YEL}Updating ROS workspace.${NC}"
    # Merge in any updates to original rosinstall.
    wstool merge -ky -t $DIR/src $DIR/$ROS
    wstool update -j 8 -t $DIR/src
  fi

  echo -e "${YEL}Installing dependency packages.${NC}"
  rosdep install --from-paths $DIR/src --ignore-src --rosdistro kinetic -y --os $(lsb_release -si | awk '{print tolower($0)}'):$(lsb_release -sc)

  echo -e "${YEL}Configuring workspace location.${NC}"
  catkin config --init -w$DIR -s$DIR/src -l$DIR/log -b$DIR/build -d$DIR/devel -i$DIR/install

  echo -e "${YEL}Disabling CUDA support.${NC}"
  catkin config --cmake-args -DWITH_CUDA=OFF -DBUILD_opencv_gpu=OFF

  if [ ! -z $EXT ];
  then
    echo -e "${YEL}Extending workspace $EXT.${NC}"
    catkin config --extend $EXT
  fi

  #echo -e "${YEL}Build workspace $DIR? [Y/n]${NC}"
  #read input_variable
  #if echo "$input_variable" | grep -iq "^n";
  #then
  #  echo -e "${YEL}Skipping workspace build.${NC}"
  #else
    echo -e "${YEL}Building workspace with Catkin.${NC}"
    catkin build -w $DIR
  #fi
}

workspace catkin_ws kinetic-desktop-custom.rosinstall
#source $KINETIC_DIR/catkin_ws/devel/setup.bash
workspace ws kinetic-kulbu.rosinstall catkin_ws
#cd $KINETIC_DIR/ws
#source $KINETIC_DIR/ws/devel/setup.bash
