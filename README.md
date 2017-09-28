# Introduction

![Alt text](https://www.trossenrobotics.com/Shared/images/PImages/turtlebot2i/step4-3.jpg "Turtlebot2i")

## Definition

Turtlebot2i
=========

The Interbotix Turtlebot 2i stack provides Navigation &amp; Object Manipulation demos along with a [wiki](https://github.com/Interbotix/turtlebot2i/wiki) for configuring and using a [TurtleBot 2i](https://www.interbotix.com/interbotix-turtlebot-2i-mobile-ros-platform.aspx) with [ROS](http://www.ros.org).

### Benefits

The TurtleBot 2i improves upon previous iterations of the TurtleBot with a completely redesigned modular chassis and for the first time, native support of robotic arms. The TurtleBot 2i offers the Pincher MK3 4 DOF Robotic Arm as a fully supported standard option allowing the robot to interact with small objects in the real world, effectively transforming the TurtleBot into an extremely capable mobile manipulator. The Arbotix-M Robocontroller provides an interface for the Pincher Mk3 arm, which is implemented using MoveIt, an open source inverse kinematics solution, allowing users to control the arm using only high-level commands.

The TurtleBot 2i is powered by an Intel NUC BOXNUC6CAYH and features dual 3D camera configurations, using a dedicated long range Orbbec Astra for Navigation & Mapping, and the short range Intel RealSense camera SR300-Series as a dedicated Manipulation work space sensor. This new version is also accompanied by hands on tutorials and demos that showcase the robot's capabilities - enabling students, educators, researchers and developers to hit the ground running with a versatile open source ROS development platform. The demos and tutorials created for the 2i provide a hands-on tools for demonstrating how robots perceive and interact in the real world, focusing on real-world autonomous mobile robotics, automation technologies and object/environment manipulation.

### Challenges

Hardware is a fickle thing. This robot is the end result of several iterations due to the ever shifting hardware market. After working with the Intel Joule for a large portion of the development cycle, Intel ended support for many of their IoT devices. This left us to find a reasonable alternative that worked as well. With a bit of research, we landed on the Intel NUC BOXNUC6CAYH, a full computer system with a small footprint, low power consumption, and a processor with enough muscle to handle the Turtlebot2i.

Major considerations in development were:

* The form factor of the robotic manipulator, resulting in an updated version of the Pincher Arm
* The expandability of the system, using a centimeter grid hole pattern for easy sensor mounting
* The core computer used for ROS operation. Heavy consideration was given to laptops, SBCs, and small form factor PCs
* Custom wire harnesses and adapters, making a versatile set of power solutions that could be easily altered by hobbyists

### Contributing

The TurtleBot is an open community project.
If you have improvements or suggestions for the TurtleBot2i please open pull requests against [this document](https://github.com/Interbotix/turtlebot2i).

If you would like to suggest anything that doesn't fit into this document directly please start a thread on the [TurtleBot Forums](http://discourse.ros.org/c/turtlebot)

ROS Wiki : (http://www.ros.org/wiki/Robots/TurtleBot)


![TurtleBot Logo](http://www.turtlebot.com/assets/images/turtlebot_logo.png)
