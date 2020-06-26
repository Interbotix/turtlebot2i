#!/usr/bin/env bash

echo "Installing just enough dependencies to access the Turtlebot2i remotely from your personal computer or to simulate in Gazebo..."

# Step 1: Install Turtlebot2i packages
echo "Installing Turtlebot2i dependencies..."
sudo apt -y install ros-kinetic-rtabmap-ros
sudo apt -y install ros-kinetic-kobuki-desktop
TURTLEBOT2I_WS=~/turtlebot2i_ws
mkdir -p $TURTLEBOT2I_WS/src
cd $TURTLEBOT2I_WS/src
git clone https://github.com/Interbotix/arbotix_ros.git -b turtlebot2i
git clone https://github.com/Interbotix/phantomx_pincher_arm.git
cd $TURTLEBOT2I_WS && catkin_make
cd src
git clone https://github.com/Interbotix/turtlebot2i.git

# Step 2: Setup Environment variables
echo "Setting up Environment variables..."
echo "export ROS_IP=$(hostname -I)" >> ~/.bashrc
echo "export TURTLEBOT_BASE=kobuki" >> ~/.bashrc
echo "export TURTLEBOT_3D_SENSOR=astra" >> ~/.bashrc
echo "export TURTLEBOT_STACKS=interbotix" >> ~/.bashrc
