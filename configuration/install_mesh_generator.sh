#!/bin/bash

#Cloning repo
git clone https://github.com/aloysiogl/generate_mesh.git ~/catkin_ws/src/generate_mesh

#Installing cgal
cd ~/catkin_ws/src/generate_mesh/configure
./install_cgal.sh

#Building C++ shared library
cd ~/catkin_ws/src/generate_mesh/configure
./build.sh
