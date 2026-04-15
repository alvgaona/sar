#!/bin/bash

echo "delete remap the device serial port(ttyUSBX) to arduClone"
echo "sudo rm   /etc/udev/rules.d/arduClone.rules"
sudo rm   /etc/udev/rules.d/arduClone.rules
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish  delete"
