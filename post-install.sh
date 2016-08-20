#!/bin/bash
set -e

# As root
# adduser yellow sudo

# As user
#sudo apt-get install -y git
#git config --global user.name "Mr-Yellow"
#git config --global user.email "mr-yellow@mr-yellow.com"
#git config --global push.default simple
#ssh-keygen -t rsa -b 4096 -C "mr-yellow@mr-yellow.com"
#git clone git@github.com:mryellow/debian-scripts.git ~/scripts
#./scripts/post-install.sh

if [ ! -f ~/.gnupg/gpg.conf ]
then
  echo "Creating GPG key."
  mkdir -p ~/.gnupg/
  cat >> ~/.gnupg/gpg.conf <<EOF
personal-digest-preferences SHA256
cert-digest-algo SHA256
default-preference-list SHA512 SHA384 SHA256 SHA224 AES256 AES192 AES CAST5 ZLIB BZIP2 ZIP Uncompressed
EOF
  gpg --gen-key
fi

echo "Adding contrib and non-free sources for nvidia drivers."
sudo sed --in-place=.old -e 's/main$/main contrib non-free/g' /etc/apt/sources.list
sudo apt-get update
sudo apt-get -y upgrade

echo "Reconfigure fonts."
sudo apt-get install -y ttf-dejavu ttf-liberation ttf-mscorefonts-installer xfonts-terminus
sudo dpkg-reconfigure fontconfig-config
sudo dpkg-reconfigure fontconfig

echo "Installing Bumblebee Hybrid graphics."
sudo apt-get install -y bumblebee-nvidia primus mesa-utils
sudo adduser $USER bumblebee
#optirun glxgears -info

echo "Installing RedShift blue-light management."
sudo apt-get install -y redshift-plasmoid

echo "Installing utils."
sudo apt-get install -y apg xclip

if [ ! -f ~/src/installed ]
then
  echo "Downloading stand-alone packages."
  mkdir -p ~/src/
  wget -O ~/src/atom-amd64.deb https://github.com/atom/atom/releases/download/v1.9.9/atom-amd64.deb
  wget -O ~/src/google-chrome-stable_current_amd64.deb https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb
  wget -O ~/src/steam_latest.deb https://steamcdn-a.akamaihd.net/client/installer/steam.deb

  echo "Installing stand-alone packages."
  dpkg -i ~/src/atom-amd64.deb
  dpkg -i ~/src/google-chrome-stable_current_amd64.deb
  dpkg -i ~/src/steam_latest.deb
  sudo apt-get -fy install
  touch ~/src/installed
fi
