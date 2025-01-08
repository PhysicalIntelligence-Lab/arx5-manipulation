#!/bin/sh
cp ./config/99-arxcan.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
