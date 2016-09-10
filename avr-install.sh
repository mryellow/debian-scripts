#!/bin/bash
set -o errexit
set -o pipefail

CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

RED='\033[0;31m'
YEL='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YEL}Installing dependencies.${NC}"
sudo apt-get install -y gcc-avr binutils-avr gdb-avr avr-libc avrdude simulavr srecord
sudo apt-get install -y usbutils # lsusb

echo -e "${YEL}Updating UDEV rules.${NC}"
sudo cp -u $CURRENT_DIR/assets/avr/*.rules /etc/udev/rules.d/

echo -e "${YEL}Restarting UDEV service.${NC}"
sudo service udev restart
