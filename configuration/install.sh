#!/bin/bash

############ CRAZYFLIE ################
# From <https://github.com/wuwushrek/sim_cf>
sudo apt-get install git zip qtcreator cmake build-essential genromfs ninja-build -y
sudo apt-get install protobuf-compiler libgoogle-glog-dev libeigen3-dev libxml2-utils
sudo apt-get install ros-kinetic-mav-comm ros-kinetic-joy
sudo apt-get install ros-kinetic-rqt-multiplot

cd ~/
git clone https://github.com/bitcraze/crazyflie-clients-python.git
cd crazyflie-clients-python
pip3 install --user -e .

cd ~/catkin_ws/src
git clone https://github.com/whoenig/crazyflie_ros.git
cd crazyflie_ros
git submodule update --init --recursive

cd ~/catkin_ws/src
git clone https://github.com/wuwushrek/sim_cf.git
cd sim_cf/
git submodule update --init --recursive
cd ~/catkin_ws
catkin_make
source devel/setup.bash

cd src/sim_cf/crazyflie-firmware/sitl_make/
mkdir build
cd build
cmake ..
make
cd ~/catkin_ws

################ EDITING PYTHONPATH ##################
echo 'export PYTHONPATH=$PYTHONPATH:~/catkin_ws/src/psc/src' >> ~/.bashrc

################ CONFIGURING USB PERMISSIONS ###############
# From <https://github.com/bitcraze/crazyflie-lib-python#setting-udev-permissions>
sudo groupadd plugdev
sudo usermod -a -G plugdev $USER
sudo touch /etc/udev/rules.d/99-crazyradio.rules
sudo echo '# Crazyradio (normal operation)' >> /etc/udev/rules.d/99-crazyradio.rules
sudo echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="7777", MODE="0664", GROUP="plugdev"' >> /etc/udev/rules.d/99-crazyradio.rules
sudo echo '# Bootloader' >> /etc/udev/rules.d/99-crazyradio.rules
sudo echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="0101", MODE="0664", GROUP="plugdev"' >> /etc/udev/rules.d/99-crazyradio.rules
sudo echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0664", GROUP="plugdev"' >> /etc/udev/rules.d/99-crazyradio.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
