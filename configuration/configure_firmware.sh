#!/bin/bash

sudo add-apt-repository ppa:team-gcc-arm-embedded/ppa
sudo apt-get update
sudo apt install gcc-arm-embedded
cd ~/crazyflie-clients-python
pip install --user -e .
sudo apt-get install python3-pyqt4 python3-pyqtgraph
cd ~/catkin_ws/src/sim_cf/crazyflie-firmware
make
# To transfer the firmware, turn on the drone on bootloader mode by holding the on button, then do a "make cload"