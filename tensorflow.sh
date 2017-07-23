#!/bin/bash
set -o errexit
set -o pipefail

CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

RED='\033[0;31m'
YEL='\033[1;33m'
NC='\033[0m' # No Color

#echo -e "${YEL}Installing Python env.${NC}"
#sudo apt-get install python-pip python-dev python-virtualenv

#virtualenv --system-site-packages ~/work/tf
#cd ~/work/tf
#source bin/activate
##pip install --upgrade tensorflow # For Python 2.7 CPU
#pip install --upgrade https://storage.googleapis.com/tensorflow/linux/cpu/tensorflow-1.2.0-cp27-none-linux_x86_64.whl


# apt-get install nvidia-cuda-toolkit openjdk-8-jdk
echo -e "${YEL}Downloading Tensorflow source.${NC}"
git clone https://github.com/tensorflow/tensorflow ~/src/tensorflow
cd ~/src/tensorflow
git checkout v1.2.1

echo -e "${YEL}Installing bazel.${NC}"
echo "deb [arch=amd64] http://storage.googleapis.com/bazel-apt stable jdk1.8" | sudo tee /etc/apt/sources.list.d/bazel.list
curl https://bazel.build/bazel-release.pub.gpg | sudo apt-key add -
sudo apt-get update && sudo apt-get install bazel

echo -e "${YEL}Installing python deps.${NC}"
sudo apt-get install python-numpy python-dev python-pip python-wheel
sudo apt-get install libcupti-dev

echo -e "${YEL}Configuring Tensorflow build.${NC}"
./configure
## CPU #
echo -e "${YEL}Building tensorflow.${NC}"
bazel build --config=opt //tensorflow/tools/pip_package:build_pip_package
## GPU ##bazel build --config=opt --config=cuda //tensorflow/tools/pip_package:build_pip_package
echo -e "${YEL}Building package.${NC}"
bazel-bin/tensorflow/tools/pip_package/build_pip_package /tmp/tensorflow_pkg
sudo pip install /tmp/tensorflow_pkg/tensorflow-1.2.1-cp27-none-linux_x86_64.whl
