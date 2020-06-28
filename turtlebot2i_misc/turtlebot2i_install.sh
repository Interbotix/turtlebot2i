#!/usr/bin/env bash

ubuntu_version="$(lsb_release -r -s)"

if [ $ubuntu_version != "16.04" ]; then
	echo -e "Unsupported Ubuntu verison: $ubuntu_version"
	echo -e "Turtlebot2i only works with 16.04"
	exit 1
fi

echo -e "\e[1;33m ***************************************** \e[0m"
echo -e "\e[1;33m The installation takes around 20 Minutes! \e[0m"
echo -e "\e[1;33m ***************************************** \e[0m"
sleep 4
start_time="$(date -u +%s)"

# Install SSH server to allow remote access
sudo apt -y install openssh-server

# Step 1: Install ROS Kinetic
if [ $(dpkg-query -W -f='${Status}' ros-kinetic-desktop-full 2>/dev/null | grep -c "ok installed") -eq 0 ]; then
  echo "Installing ROS..."
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
  sudo apt update
  sudo apt -y install ros-kinetic-desktop-full
	if [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
		sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
	fi
  echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
  sudo apt -y install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
  sudo rosdep init
  rosdep update
else
  echo "ros-kinetic-desktop-full is already installed!"
fi

source /opt/ros/kinetic/setup.bash

# Step 2: Install Orbbec packages
ORBBEC_WS=~/orbbec_ws
if [ ! -d "$ORBBEC_WS/src" ]; then
  echo "Installing Orbbec ROS packages..."
  sudo apt -y install ros-kinetic-rgbd-launch
  sudo apt -y install ros-kinetic-libuvc
  sudo apt -y install ros-kinetic-libuvc-camera
  sudo apt -y install ros-kinetic-libuvc-ros
  mkdir -p $ORBBEC_WS/src
  cd $ORBBEC_WS && catkin_make
  echo "source $ORBBEC_WS/devel/setup.bash" >> ~/.bashrc
  source $ORBBEC_WS/devel/setup.bash
  cd $ORBBEC_WS/src
  git clone https://github.com/orbbec/ros_astra_camera
  git clone https://github.com/orbbec/ros_astra_launch.git
  roscd astra_camera
  ./scripts/create_udev_rules
  cd $ORBBEC_WS && catkin_make
else
  echo "Orbbec ROS packages already installed!"
fi

# Step 3: Install Realsense packages

# Step 3A: Install librealsense2
if [ $(dpkg-query -W -f='${Status}' librealsense2 2>/dev/null | grep -c "ok installed") -eq 0 ]; then
  echo "Installing librealsense2..."
  sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
  sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
  version="2.33.1-0~realsense0.2140"
  sudo apt -y install librealsense2-udev-rules=${version}
  sudo apt -y install librealsense2-dkms=1.3.11-0ubuntu1
  sudo apt -y install librealsense2=${version}
  sudo apt -y install librealsense2-gl=${version}
  sudo apt -y install librealsense2-utils=${version}
  sudo apt -y install librealsense2-dev=${version}
  sudo apt -y install librealsense2-dbg=${version}
  sudo apt-mark hold librealsense2*
  sudo apt -y install ros-kinetic-ddynamic-reconfigure
else
  echo "librealsense2 already installed!"
fi

# Step 3B: Install realsense2 ROS Wrapper
REALSENSE_WS=~/realsense_ws
if [ ! -d "$REALSENSE_WS/src" ]; then
  echo "Installing RealSense ROS packages..."
  mkdir -p $REALSENSE_WS/src
  cd $REALSENSE_WS/src
  git clone https://github.com/Interbotix/realsense-ros.git -b turtlebot2i
  cd $REALSENSE_WS
  catkin_make clean
  catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
  catkin_make install
  echo "source $REALSENSE_WS/devel/setup.bash" >> ~/.bashrc
  source $REALSENSE_WS/devel/setup.bash
else
  echo "RealSense ROS packages already installed!"
fi

# Step 4: Install Turtlebot2i packages
TURTLEBOT2I_WS=~/turtlebot2i_ws
if [ ! -d "$TURTLEBOT2I_WS/src" ]; then
  echo "Installing Turtlebot2i ROS packages..."
  mkdir -p $TURTLEBOT2I_WS/src
  cd $TURTLEBOT2I_WS/src
  git clone https://github.com/Interbotix/turtlebot2i.git
  git clone https://github.com/Interbotix/arbotix_ros.git -b turtlebot2i
  git clone https://github.com/Interbotix/phantomx_pincher_arm.git
  cd turtlebot2i/turtlebot2i_misc
  sudo cp 99-turtlebot2i.rules /etc/udev/rules.d/
  sudo dpkg -i ros-kinetic-realsense-camera_99_all.deb
  sudo apt -y install ros-kinetic-turtlebot-*
  sudo apt -y install ros-kinetic-find-object-2d
  sudo apt -y install ros-kinetic-rtabmap-ros
  sudo apt -y install ros-kinetic-moveit
  sudo apt -y install ros-kinetic-octomap-ros
  sudo apt -y install ros-kinetic-manipulation-msgs
  sudo apt -y install ros-kinetic-controller-manager
  cd $TURTLEBOT_WS && catkin_make
  echo "source $TURTLEBOT2I_WS/devel/setup.bash" >> ~/.bashrc
  source $TURTLEBOT2I_WS/devel/setup.bash
else
  echo "Turtlebot2i ROS packages already installed!"
fi

# Step 5: Setup Environment variables
echo "Setting up Environment variables..."
echo "export ROS_IP=$(hostname -I)" >> ~/.bashrc
echo "export TURTLEBOT_3D_SENSOR=astra" >> ~/.bashrc
echo "export TURTLEBOT_3D_SENSOR2=sr300" >> ~/.bashrc
echo "export TURTLEBOT_BATTERY=None" >> ~/.bashrc
echo "export TURTLEBOT_STACKS=interbotix" >> ~/.bashrc
echo "export TURTLEBOT_BASE=kobuki" >> ~/.bashrc
echo "export TURTLEBOT_ARM=pincher" >> ~/.bashrc
sudo usermod -a -G dialout turtlebot

end_time="$(date -u +%s)"
elapsed="$(($end_time-$start_time))"

echo "Installation complete, took $elapsed seconds in total"
echo "NOTE: Remember to reboot the computer before using the robot!"
