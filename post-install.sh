#!/bin/bash
set -o errexit
set -o pipefail

# TODO: Update to stretch.

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

if [ ! -f /etc/apt/sources.list.d/cisofy-lynis.list ];
then
  echo -e "${YEL}Add sources for lynis.${NC}"
  sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys C80E383C3DE9F082E01391A0366C67DE91CA5D5F
  sudo apt install apt-transport-https
  sudo sh -c 'echo "deb https://packages.cisofy.com/community/lynis/deb/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/cisofy-lynis.list'
fi

if [ ! -f /etc/apt/sources.list.d/influxdb.list ];
then
  echo -e "${YEL}Add sources for InfluxDB.${NC}"

  curl -sL https://repos.influxdata.com/influxdb.key | sudo apt-key add -
  sudo sh -c 'echo "deb https://repos.influxdata.com/debian $(lsb_release -sc) stable" > /etc/apt/sources.list.d/influxdb.list'
fi

sudo apt-get update
sudo apt-get upgrade -y

echo -e "${YEL}Installing lynis.${NC}"
sudo apt-get install lynis

echo -e "${YEL}Installing InfluxDB.${NC}"
sudo apt-get install influxdb

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

echo -e "${YEL}Installing software.${NC}"
sudo apt-get install -y apg xclip
sudo apt-get install -y python-virtualenv python-imaging python-yaml libproj0
sudo apt-get install -y python-yaml libgeos-dev libgdal-dev python-lxml #python-shapely #python-fiona
sudo apt-get install -y plasma-widget-ktorrent rawtherapee qgis python-qgis
sudo apt-get install -y gparted
sudo apt-get install -y nodejs npm nodejs-legacy
sudo apt-get install -y php5-cli
sudo apt-get install -y ntp

# TODO: Check if already installed
# TODO: Use absolute paths `--install-dir`
# curl -sS https://getcomposer.org/installer | sudo php -- --install-dir=/usr/local/bin --filename=composer

#echo -e "${YEL}Installing composer.${NC}"
#php -r "copy('https://getcomposer.org/installer', 'composer-setup.php');"
#php -r "if (hash_file('SHA384', 'composer-setup.php') === 'e115a8dc7871f15d853148a7fbac7da27d6c0030b848d9b3dc09e2a0388afed865e6a3d6b3c0fad45c48e2b5fc1196ae') { echo 'Installer verified'; } else { echo 'Installer corrupt'; unlink('composer-setup.php'); } echo PHP_EOL;"
#php composer-setup.php
#php -r "unlink('composer-setup.php');"

if [ ! -f ~/src/installed ];
then
  mkdir -p ~/src/
  if [ ! -f ~/src/atom-amd64.deb ];
  then
    echo -e "TODO: Could use APT for Atom"
    echo -e "${YEL}Downloading Atom.${NC}"
    wget -O ~/src/atom-amd64.deb https://github.com/atom/atom/releases/download/v1.28.2/atom-amd64.deb
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
  sudo apt-get install -y gvfs-bin tidy
  sudo dpkg -i ~/src/atom-amd64.deb
  sudo apt-get install -y cppcheck tidy
  sudo gem install scss_lint
  apm install editorconfig
  apm install language-lua linter
  apm install linter-lua linter-jscs linter-scss-lint linter-csslint
  apm install linter-cppcheck linter-solidity
  #linter-tidy
  apm install pigments
  apm upgrade --confirm false
  echo -e "${RED}Torch install required for linter-lua.${NC}"
  echo -e "${RED}Set linter-lua path to /home/$USER/torch/install/bin/luajit.${NC}"

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

#if [ ! -f /etc/init.d/icc-profile.sh ];
#then
#  echo -e "${YEL}Installing ICC profile.${NC}"
#  sudo cp -u "${CURRENT_DIR}/assets/icc/N156BGE-L41 #1 2016-11-24 04-15 2.2 M-S XYZLUT+MTX.icc" "/usr/share/color/icc/N156BGE-L41 #1 2016-11-24 04-15 2.2 M-S XYZLUT+MTX.icc"
#  sudo cp -u $CURRENT_DIR/assets/icc/icc-profile.sh /etc/init.d/icc-profile.sh
#  sudo chmod 0755 /etc/init.d/icc-profile.sh
#  sudo systemctl enable icc-profile.sh
#  sudo systemctl start icc-profile.service
#fi

echo -e "${RED}TODO:${NC}"
echo -e "${RED}Disable hardware rendering in Chrome.${NC}"
echo -e "${RED}Manual installation of Torch.${NC}"
