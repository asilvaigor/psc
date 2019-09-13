# PSC Drones

## Installation

Make sure you have Ubuntu 16 installed. First install ROS:
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Then initialize a ROS workspace:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ..
catkin_make
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

Next clone this repository:
```
cd ~/catkin_ws/src/
git clone https://github.com/asilvaigor/psc.git
```

Then run:
```
cd ~/catkin_ws/src/psc/configuration
./install.sh
```

## Configure PyCharm
Open the project on psc folder. Go on `File>Settings>Project>Project Interpreter`. Make sure you select python2.7 (`/usr/bin/python2.7`). Click on the gear button, then "Show All". Select the last button (tree), than add a path to `catkin_ws/src/sim_cf/crazyflie_gazebo/tools` and one to `catkin_ws/src/psc/src/`.