cmake_minimum_required(VERSION 2.8.3)
project(phantomx_pincher_arm_bringup)

find_package(catkin REQUIRED COMPONENTS rospy arbotix_python)

catkin_python_setup()
catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS src/fake_joint_pub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})