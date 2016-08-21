#!/bin/bash
set -e

RED='\033[0;31m'
YEL='\033[1;33m'
NC='\033[0m' # No Color

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
  echo -e "${YEL}Creating GPG key.${NC}"
  mkdir -p ~/.gnupg/
  cat >> ~/.gnupg/gpg.conf <<EOF
personal-digest-preferences SHA256
cert-digest-algo SHA256
default-preference-list SHA512 SHA384 SHA256 SHA224 AES256 AES192 AES CAST5 ZLIB BZIP2 ZIP Uncompressed
EOF
  gpg --gen-key
fi

if ! grep "contrib non-free" /etc/apt/sources.list;
then
  echo -e "${YEL}Adding contrib and non-free sources for nvidia drivers.${NC}"
  sudo sed --in-place=.old -e 's/main$/main contrib non-free/g' /etc/apt/sources.list
fi
sudo apt-get update
sudo apt-get upgrade -y

echo -e "${YEL}Reconfigure fonts.${NC}"
sudo apt-get install -y ttf-dejavu ttf-liberation ttf-mscorefonts-installer xfonts-terminus
sudo dpkg-reconfigure fontconfig-config
sudo dpkg-reconfigure fontconfig
cp -u fonts.conf ~/.fonts.conf

# FIXME: Host invalid.
#if [ ! -d ~/src/infinality/installed ]
#then
  #echo "Install font Infinality."
  #git clone https://github.com/chenxiaolong/Debian-Packages.git ~/src/infinality
  #sudo apt-get install -y debhelper devscripts docbook-to-man quilt
  #cd ~/src/infinality/freetype-infinality && ./build.sh
  #cd ~/src/infinality/fontconfig-infinality && ./build.sh
  #sudo dpkg -i ~/src/infinality/freetype-infinality/*.deb
  #sudo dpkg -i ~/src/infinality/fontconfig-infinality/*.deb
  #touch ~/src/infinality/installed
#fi

echo -e "${YEL}Installing Bumblebee Hybrid graphics.${NC}"
sudo apt-get install -y bumblebee-nvidia primus mesa-utils
sudo adduser $USER bumblebee
#optirun glxgears -info

echo -e "${YEL}Installing RedShift blue-light management.${NC}"
sudo apt-get install -y redshift-plasmoid

echo -e "${YEL}Installing utils.${NC}"
sudo apt-get install -y apg xclip

echo -e "${YEL}Installing common software.${NC}"
sudo apt-get install -y plasma-widget-ktorrent

if [ ! -f ~/src/installed ];
then
  mkdir -p ~/src/
  if [ ! -f ~/src/atom-amd64.deb ];
  then
    echo -e "${YEL}Downloading Atom.${NC}"
    wget -O ~/src/atom-amd64.deb https://github.com/atom/atom/releases/download/v1.9.9/atom-amd64.deb
  fi
  if [ ! -f ~/src/google-chrome-stable_current_amd64.deb ];
  then
    echo -e "${YEL}Downloading Chrome.${NC}"
    wget -O ~/src/google-chrome-stable_current_amd64.deb https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb
  fi
  if [ ! -f ~/src/steam_latest.deb ];
  then
    echo -e "${YEL}Downloading Steam.${NC}"
    wget -O ~/src/steam_latest.deb https://steamcdn-a.akamaihd.net/client/installer/steam.deb
  fi

  echo -e "${YEL}Installing stand-alone packages.${NC}"
  sudo dpkg -i ~/src/atom-amd64.deb
  sudo dpkg -i ~/src/google-chrome-stable_current_amd64.deb
  sudo dpkg -i ~/src/steam_latest.deb
  sudo apt-get -fy install
  touch ~/src/installed
fi

if ! grep "GIT_PROMPT_ONLY_IN_REPO" ~/.bashrc;
then
  echo -e "${YEL}Install GIT prompt.${NC}"
  echo "" >> ~/.bashrc
  echo "GIT_PROMPT_ONLY_IN_REPO=1" >> ~/.bashrc
  echo "source ~/scripts/git-prompt.sh" >> ~/.bashrc
  echo "PS1='\[\e]0;\u@\h: \w\a\]\${debian_chroot:+(\$debian_chroot)}\u@\h:\w\$(__git_ps1 \" (%s)\")$ '" >> ~/.bashrc
fi
