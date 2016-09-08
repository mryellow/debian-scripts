#!/bin/bash

if ! grep "e3b67f0e-608b-4c8e-8cd7-f962e64cd6bf" /etc/apt/sources.list;
then
  DIR="/home/$USER/caddy"
  mkdir -p $DIR
  echo "UUID=e3b67f0e-608b-4c8e-8cd7-f962e64cd6bf $DIR ext4 errors=remount-ro 0 1" | sudo tee -a /etc/fstab
  sudo mount $DIR

  # TODO: Should comment out /media/cdrom0
fi
