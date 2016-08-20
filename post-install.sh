#!/bin/bash

# As root
# adduser yellow sudo

# As user
sudo apt-get install -y git
git config --global user.name "Mr-Yellow"
git config --global user.email "mr-yellow@mr-yellow.com"

mkdir -p ~/src/
wget -O ~/src/atom-amd64.deb https://github.com/atom/atom/releases/download/v1.9.9/atom-amd64.deb
wget -O ~/src/google-chrome-stable_current_amd64.deb https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb
wget -O ~/src/steam_latest.deb https://steamcdn-a.akamaihd.net/client/installer/steam.deb

dpkg -i ~/src/atom-amd64.deb
dpkg -i ~/src/google-chrome-stable_current_amd64.deb
dpkg -i ~/src/steam_latest.deb

# Include contrib and non-free for nvidia drivers.
sudo sed --in-place=.old -e 's/main$/main contrib non-free/g' /etc/apt/sources.list
sudo apt-get update

# Bumblebee Hybrid graphics
sudo apt-get install -y bumblebee-nvidia primus mesa-utils
sudo adduser $USER bumblebee
#optirun glxgears -info

# Redshift blue-light management
sudo apt-get install -y redshift-plasmoid

sudo apt-get install -y apg xclip
