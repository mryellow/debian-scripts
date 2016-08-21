#!/bin/bash

for package in `dpkg --get-selections | grep -w 'install$' | cut -f 1`; do
    apt-get install --reinstall $package;
done
