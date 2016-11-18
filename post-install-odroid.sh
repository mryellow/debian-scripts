#!/bin/bash
set -o errexit
set -o pipefail

RED='\033[0;31m'
YEL='\033[1;33m'
NC='\033[0m' # No Color

if ! grep swap /etc/fstab;
then
  echo -e "${YEL}Configure USB swap.${NC}"
  sudo sh -c 'echo "UUID=68e068ff-f919-4620-9a91-953dc1137fcb none swap sw,pri=5 0 0" >> /etc/fstab'
  sudo swapon -a
fi

sudo chown -R $USER:$USER /media/$USER/ros

