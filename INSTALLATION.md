# Interbotix Turtlebot2i Installation Instructions


This guide documents how to install ROS Kinetic and all necessary packages to get up and running with the Interbotix Turtlebot2i.

#### Update Linux

The Intel NUC7i3BNH computer for the Turtlebot2i robot comes pre-installed with the [latest Ubuntu Desktop image (16.04.6)](https://releases.ubuntu.com/16.04/) already on it. After unpacking it, connect a mouse, HDMI monitor, and keyboard to it. Then, plug in the power supply and press the **Power** button.
A login screen should appear on your monitor with **turtlebot** as the user name. Conveniently, the password is the same as the user name so type *turtlebot* and hit **Enter**. Next, update the computer by performing the following steps.

1. Connect the Turtlebot computer to your local network. This is as simple as clicking the Wifi icon on the top right of the Desktop and selecting your desired network.
2. Press **Cntrl-Alt-T** to open a terminal screen, and type `sudo apt update`. If prompted for a password, type *turtlebot*.
3. After the process completes, type `sudo apt -y upgrade`. It might take a few minutes for the computer to upgrade. If prompted to update to *Ubuntu 18.04*, just decline.
4. Finally, type `sudo apt -y autoremove` to get rid of unnecessary software packages. Then close out of the terminal and reboot the computer.

#### Install Packages on the Turtlebot Computer
Once rebooted, login and open up a terminal as before. Instead of manually installing all the software needed for the robot, you will download and run an installation script. Follow the commands below to get started! Note that nothing (cameras, Kobuki, Arbotix) needs to be connected to the computer for the installation to work.

    $ sudo apt install curl
    $ curl 'https://raw.githubusercontent.com/Interbotix/turtlebot2i/master/turtlebot2i_misc/turtlebot2i_install.sh' > turtlebot2i_install.sh
    $ chmod +x turtlebot2i_install.sh
    $ ./turtlebot2i_install.sh

#### Install Packages on a Local (Remote) Computer
To interface with the Turtlebot from a local (remote) computer, connect your local computer to the same network to which the Turtlebot is connected. Then run the following commands on your *local* computer. Note that it will only install the packages necessary to remotely monitor the Turtlebot using Rviz. It will also allow you to simulate the robot in Gazebo.

    $ sudo apt install curl
    $ curl 'https://raw.githubusercontent.com/Interbotix/turtlebot2i/master/turtlebot2i_misc/turtlebot2i_remote_install.sh' > turtlebot2i_remote_install.sh
    $ chmod +x turtlebot2i_remote_install.sh
    $ ./turtlebot2i_remote_install.sh
