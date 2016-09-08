#!/bin/bash

if ! grep "e3b67f0e-608b-4c8e-8cd7-f962e64cd6bf" /etc/apt/sources.list;
then
  echo "UUID=e3b67f0e-608b-4c8e-8cd7-f962e64cd6bf /home/$USER/caddy ext4 errors=remount-ro 0 1" >> /etc/fstab
fi
