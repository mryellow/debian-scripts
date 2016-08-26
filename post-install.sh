#!/bin/bash
set -o errexit
set -o pipefail

CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

RED='\033[0;31m'
YEL='\033[1;33m'
NC='\033[0m' # No Color

if [ ! -f ~/.ssh/id_rsa ]
then
  echo -e "${YEL}Creating SSH key.${NC}"
  # Prompt for email address
  echo "Email: "
  read email
  ssh-keygen -t rsa -b 4096 -C "$email"
fi

if [ ! -f ~/.gnupg/gpg.conf ]
then
  echo -e "${YEL}Creating GPG key.${NC}"
  mkdir -p ~/.gnupg/
  cp -u $CURRENT_DIR/assets/gnupg/gpg.conf ~/.gnupg/gpg.conf
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
cp -u $CURRENT_DIR/assets/fonts.conf ~/.fonts.conf

echo -e "${YEL}Generating locales.${NC}"
sudo dpkg-reconfigure locales

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
sudo apt-get install -y python-virtualenv python-imaging python-yaml libproj0
sudo apt-get install -y python-yaml libgeos-dev libgdal-dev python-lxml #python-shapely #python-fiona

echo -e "${YEL}Installing common software.${NC}"
sudo apt-get install -y plasma-widget-ktorrent rawtherapee qgis python-qgis

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
  if [ ! -f ~/src/mendeleydesktop-latest.deb ];
  then
    echo -e "${YEL}Downloading Mendeley.${NC}"
    if [ `getconf LONG_BIT` = "64" ];
    then
        wget -O ~/src/mendeleydesktop-latest.deb https://www.mendeley.com/repositories/ubuntu/stable/amd64/mendeleydesktop-latest
    else
        wget -O ~/src/mendeleydesktop-latest.deb https://www.mendeley.com/repositories/ubuntu/stable/i386/mendeleydesktop-latest
    fi
  fi

  echo -e "${YEL}Installing stand-alone packages.${NC}"
  sudo apt-get install -y gvfs-bin
  sudo dpkg -i ~/src/atom-amd64.deb
  sudo apt-get install -y libpango1.0-0 libappindicator1 libcurl3
  sudo apt-get -fy install
  sudo dpkg -i ~/src/google-chrome-stable_current_amd64.deb
  sudo apt-get install -y curl zenity
  sudo apt-get -fy install
  sudo dpkg -i ~/src/steam_latest.deb
  sudo dpkg -i ~/src/mendeleydesktop-latest.deb
  
  if [ ! -d ~/.atom ];
  then
    atom # Spawn Atom to create directory
  fi
  
  if ! grep "showInvisibles" ~/.atom/config.cson;
  then
    echo -e "${YEL}Configuring Atom.${NC}"
    cat >> ~/.atom/config.cson <<EOF
  editor:
    showInvisibles: true
EOF
  fi
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

#GIMP_PLUGINS=`dpkg -L gimp | grep plug-ins | sed -n '1p'`
GIMP_SCRIPTS=~/.gimp-2.8/scripts
if [ ! -f $GIMP_SCRIPTS/sg-luminosity-masks.scm ];
then
  echo -e "${YEL}Installing GIMP script-fu.${NC}"
  mkdir -p $GIMP_SCRIPTS
  # Luminosity Masks: http://registry.gimp.org/node/28644
  cp -u $CURRENT_DIR/assets/gimp/sg-luminosity-masks.scm $GIMP_SCRIPTS/sg-luminosity-masks.scm
  chmod 0755 $GIMP_SCRIPTS/sg-luminosity-masks.scm
fi
