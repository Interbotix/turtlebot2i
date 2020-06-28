#!/usr/bin/env bash

ubuntu_version="$(lsb_release -r -s)"

if [ $ubuntu_version != "16.04" ]; then
	echo -e "Unsupported Ubuntu verison: $ubuntu_version"
	echo -e "Turtlebot2i only works with 16.04"
	exit 1
fi

echo "Installing just enough dependencies to access the Turtlebot2i remotely from your personal computer or to simulate in Gazebo..."
echo "Note that ROS Kinetic must already be installed on your system (including Rviz and Gazebo) for the script to work."

# Step 1: Install Turtlebot2i packages
if [ ! -d "$TURTLEBOT2I_WS/src" ]; then
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
	echo "source $TURTLEBOT2I_WS/devel/setup.bash" >> ~/.bashrc
else
	echo "Turtlebot2i ROS packages already installed!"
fi
source $TURTLEBOT2I_WS/devel/setup.bash

# Step 2: Setup Environment variables
if [ -z ${TURTLEBOT_3D_SENSOR} ]; then
	echo "Setting up Environment variables..."
	echo '
	export TURTLEBOT_3D_SENSOR=astra
	export TURTLEBOT_3D_SENSOR2=sr300
	export TURTLEBOT_BATTERY=None
	export TURTLEBOT_STACKS=interbotix
	export TURTLEBOT_BASE=kobuki
	export TURTLEBOT_ARM=pincher
	export ROS_MASTER_URI=http://turtlebot.local:11311
	export ROS_IP=$(echo `hostname -I`)
	if [ -z ${ROS_IP} ]; then
	  export ROS_IP=127.0.0.1
	fi
	' >> ~/.bashrc
else
	echo "Environment variables already set!"
fi

echo "Installation Complete! Close this terminal and open a new one to finish."
echo "NOTE: Remember to comment out the 'source $TURTLEBOT2I_WS/devel/setup.bash' and\n'export ROS_MASTER_URI=http://turtlebot.local:11311' lines from the ~/.bashrc file when done using the Turtlebot!\nThen close out of your terminal and open a new one."
