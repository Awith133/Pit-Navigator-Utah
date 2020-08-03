Home computer attempt - DEPENDS ubuntu 16/ ros kinetic
from a fresh computer with nothing installed
needs 17 GB of storage for everything

open a terminal and copy and paste this line by line:

####if you are reading this before cloning do:
#git clone --recursive https://github.com/jpmorris5/pit-navigator.git
####if after:
#git submodule update --init --recursive

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install ros-kinetic-webots-ros ros-kinetic-rviz ros-kinetic-navigation ros-kinetic-image-pipeline ros-kinetic-teb-local-planner
wget -q0- https://cyberbotics.com/Cyberbotics.asc | sudo apt-key add -
sudo apt-add-repository 'deb https://cyberbotics.com/debian/ binary-amd64/'
sudo apt-get update
sudo apt-get install webots
sudo apt install python-pip python3-pip
pip install catkin_pkg
pip3 install catkin_pkg
pip3 install scipy
pip install scipy==0.16
sudo apt update
sudo apt upgrade
cd ~/pit-navigator/Simulation/catkin_ws/src
git clone -b kinetic https://github.com/cyberbotics/webots_ros.git
rosdep update
cd webots_ros
rosdep install --from-paths src --ignore-src --rosdistro kinetic
cd ..
cd ..
echo 'export WEBOTS_HOME=/snap/webots/current/usr/share/webots' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$WEBOTS_HOME/lib' >> ~/.bashrc
chmod +x /home/alex/pit-navigator/Simulation/catkin_ws/src/smach_pit_exp/src/smach_node.py
chmod +x /home/alex/pit-navigator/Simulation/catkin_ws/src/webots_control/src/pioneer3at.py
chmod +x /home/alex/pit-navigator/Simulation/catkin_ws/src/visualization/src/visualize_test2.py
catkin_make -DCATKIN_BLACKLIST_PACKAGES="waypoint_pit_planner"
source devel/setup.bash
roslaunch teb_local_planner_tutorials demo3.launch
