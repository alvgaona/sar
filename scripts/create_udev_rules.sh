#!/bin/bash

echo "remap the device serial port(ttyUSBX) to arduClone"
echo "arduClone usb connection as /dev/arduClone, check it using the command : ls -l /dev | grep ttyUSB"
echo "start copy arduClone.rules to /etc/udev/rules.d/"
#source /usr/share/colcon_cd/function/colcon_cd.sh
#colcon_cd sar_ws
sudo cp arduClone.rules  /etc/udev/rules.d
echo -e "\nRestarting udev\n"
sudo service udev reload
sudo service udev restart
sudo udevadm control --reload && sudo udevadm trigger
echo "finish"
