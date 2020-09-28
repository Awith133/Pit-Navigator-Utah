# Pit-Navigator-Utah
# INSTALL
##Home computer attempt - DEPENDS ubuntu 16/ ros kinetic
from a fresh computer with nothing installed
needs 4 GB of storage for everything

##open a terminal and copy and paste this line by line:

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

wget -qO- https://cyberbotics.com/Cyberbotics.asc | sudo apt-key add -

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

cd ~/pit-navigator-utah/catkin_ws/src

rosdep update

cd Simulation_Control/webots_ros

rosdep install --from-paths src --ignore-src --rosdistro kinetic

echo 'export WEBOTS_HOME=/snap/webots/current/usr/share/webots' >> ~/.bashrc

echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$WEBOTS_HOME/lib' >> ~/.bashrc

cd ~/pit-navigator-utah/catkin_ws/src/global_planner/sbpl_lattice_planner/sbpl/

rm -r build/

mkdir build

cd build

cmake ..

make

sudo make install

cd ~/pit-navigator-utah/

chmod +x catkin_ws/src/smach_pit_exp/src/smach_node.py

chmod +x catkin_ws/src/webots_control/src/pioneer3at.py

chmod +x catkin_ws/src/visualization/src/visualize_test2.py

chmod +x catkin_ws/src/global_planner/mprim_generator_node/src/MPrimGeneratorNode.py

cd ~/pit-navigator-utah/catkin_ws

catkin_make 

source devel/setup.bash

##not a command but in demo3.launch file:
lines 5&6 need editing to match new file location 

roslaunch src/demo3.launch


# RUN
cd ~/pit-navigator-utah/catkin_ws/

source devel/setup.bash

roslaunch src/demo3.launch


# Important Files
catkin_ws/src/demo3.launch = main launch file for everything - needs to be edited to run on machine

catkin_ws/src/smach_pit_exp/src/smach_node.py = state machine - decides what the robot does when

catkin_ws/src/global_planner/sbpl_lattice_planner/src/sbpl_lattice_planner.cpp = ros node for global planner

catkin_ws/src/global_planner/mprim_generator_node/src/MPrimGeneratorNode.py = generates the motions for the global planner

catkin_ws/src/global_planner/relaxed_astar/src/RAstar_ros.cpp = old global planner with energy saving tacking

catkin_ws/src/cfg =  bunch of configuration files for the local planner and amcl and others

catkin_ws/src/Simulation_Control/webots_control/src/pioneer3at.py = builds the simulation robot

catkin_ws/src/Simulation_Control/webots_ros/src/webots_launcher.py = builds launches webots based on lunar-env

catkin_ws/src/Simulation_Control/<current_enviornment>/lunar-env = the 3D sim world

/server_parameters/parameters.yaml = bunch of Ros Parameters that needed to carry over from legacy code - deprecated

catkin_ws/src/executive_smach = smach is pulled from here, needs this to run

catkin_ws/src/visualization = used to be some extra visuals that is now not supported


