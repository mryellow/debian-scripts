#!/bin/bash

#https://mapproxy.org/docs/nightly/install.html
CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
INSTALL_DIR="/home/$USER/mapproxy"
#SOURCES_DIR="/home/$USER/mapproxy/cache"

RED='\033[0;31m'
YEL='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YEL}CURRENT_DIR: $CURRENT_DIR${NC}"
echo -e "${YEL}INSTALL_DIR: $INSTALL_DIR${NC}"
#echo -e "${YEL}SOURCES_DIR: $SOURCES_DIR${NC}"

echo -e "${YEL}Installing dependencies.${NC}"
sudo apt-get install -y python-virtualenv

if [ ! -d $INSTALL_DIR ];
then
  echo -e "${YEL}Configuring virtualenv.${NC}"
  virtualenv --system-site-packages mapproxy
  source $INSTALL_DIR/bin/activate
  pip install MapProxy
  mapproxy-util --version
  echo -e "${YEL}Configuring MapProxy.${NC}"
  mapproxy-util create -t base-config $INSTALL_DIR
  cp -u $CURRENT_DIR/assets/mapproxy/* $INSTALL_DIR/
fi
