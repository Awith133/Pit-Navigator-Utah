# Pit-Navigator-Utah
# INSTALL
##Home computer attempt - DEPENDS ubuntu 16/ ros kinetic
from a fresh computer with nothing installed
needs 17 GB of storage for everything

##open a terminal and copy and paste this line by line:

mkdir pit-navigator-utah

cd pit-navigator-utah

git clone https://github.com/Awith133/Pit-Navigator-Utah.git

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

cd ~/pit-navigator-utah/Simulation/catkin_ws/src

rosdep update

cd webots_ros

rosdep install --from-paths src --ignore-src --rosdistro kinetic

echo 'export WEBOTS_HOME=/snap/webots/current/usr/share/webots' >> ~/.bashrc

echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$WEBOTS_HOME/lib' >> ~/.bashrc

~/pit-navigator-utah/Simulation/catkin_ws/src/global_planner/sbpl_lattice_planner/sbpl/

rm build/

mkdir build

cd build

cmake ..

make

sudo make install

~/pit-navigator-utah/

chmod +x Simulation/catkin_ws/src/smach_pit_exp/src/smach_node.py

chmod +x Simulation/catkin_ws/src/webots_control/src/pioneer3at.py

chmod +x Simulation/catkin_ws/src/visualization/src/visualize_test2.py

chmod +x Simulation/catkin_ws/src/mprim_generator_node/src/MPrimGeneratorNode.py

~/pit-navigator-utah/Simulation/catkin_ws

rm src/CMakeLists.txt

rm build

rm devel

catkin_make 

source devel/setup.bash

##not a command but in demo3.launch file:
lines 14&15 need editing to match new file location 

roslaunch teb_local_planner_tutorials demo3.launch


# RUN
~/pit-navigator-utah/Simulation/catkin_ws/

source devel/setup.bash

roslaunch teb_local_planner_tutorials demo3.launch


# Important Files
catkin_ws/src/teb_local_planner_tutorials/launch/demo3.launch = main launch file for everything - needs to be edited to run on machine

catkin_ws/src/smach_pit_exp/src/smach_node.py = state machine - decides what the robot does when

catkin_ws/src/global_planner/sbpl_lattice_planner/src/sbpl_lattice_planner.cpp = ros node for global planner

catkin_ws/src/mprim_generator_node/src/MPrimGeneratorNode.py = generates the motions for the global planner

catkin_ws/src/relaxed_astar/src/RAstar_ros.cpp = old global planner with energy saving tacking

catkin_ws/src/teb_local_planner_tutorials/cfg =  bunch of configuration files for the local planner

catkin_ws/src/webots_control/src/pioneer3at.py = builds the simulation robot

catkin_ws/src/webots_ros/src/webots_launcher.py = builds launches webots based on lunar-env

/Documents/lunar-env = the 3D sim world

/server_parameters/parameters.yaml = bunch of Ros Parameters that needed to carry over from legacy code

catkin_ws/src/executive_smach = smach is pulled from here, needs this to run

catkin_ws/src/visualization = used to be some extra visuals that is now not supported


