#!/bin/bash

################ CONFIGURING USB PERMISSIONS ###############
# From <https://github.com/bitcraze/crazyflie-lib-python#setting-udev-permissions>
sudo groupadd plugdev
sudo usermod -a -G plugdev $USER
sudo touch /etc/udev/rules.d/99-crazyradio.rules
echo '# Crazyradio (normal operation)' | sudo tee -a /etc/udev/rules.d/99-crazyradio.rules
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="7777", MODE="0664", GROUP="plugdev"' | sudo tee -a /etc/udev/rules.d/99-crazyradio.rules
echo '# Bootloader' | sudo tee -a /etc/udev/rules.d/99-crazyradio.rules
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="0101", MODE="0664", GROUP="plugdev"' |sudo tee -a /etc/udev/rules.d/99-crazyradio.rules
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0664", GROUP="plugdev"' | sudo tee -a /etc/udev/rules.d/99-crazyradio.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
