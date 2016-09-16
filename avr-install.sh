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
sudo apt-get install -y minicom # UART

echo -e "${YEL}Updating UDEV rules.${NC}"
sudo cp -u $CURRENT_DIR/assets/avr/*.rules /etc/udev/rules.d/

if [ ! -f ~/src/saleae_logic.zip ];
then
  echo -e "${YEL}Downloading Saleae Logic.${NC}"
  wget -O ~/src/saleae_logic.zip "http://downloads.saleae.com/logic/1.2.10/Logic%201.2.10%20(64-bit).zip"
fi

if [ ! -d /home/$USER/saleae ];
then
  echo -e "${YEL}Installing Saleae Logic.${NC}"
  unzip -o ~/src/saleae_logic.zip -d ~/src/saleae_logic
  cp -ru "/home/$USER/src/saleae_logic/Logic 1.2.10 (64-bit)/" /home/$USER/saleae/
  cd /home/$USER/saleae/Drivers/ && ./install_driver.sh
fi

echo -e "${YEL}Restarting UDEV service.${NC}"
sudo service udev restart
