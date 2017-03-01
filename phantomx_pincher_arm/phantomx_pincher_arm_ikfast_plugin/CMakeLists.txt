cmake_minimum_required(VERSION 2.8.12)
project(phantomx_pincher_arm_ikfast_plugin)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  moveit_core
  pluginlib
  roscpp
  tf_conversions
  )
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  LIBRARIES
  DEPENDS
  moveit_core
  pluginlib
  roscpp
  tf_conversions
  )

add_compile_options("-std=c++11")

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(include)

link_directories(${catkin_LIBRARY_DIRS})

set(MOVEIT_LIB_NAME phantomx_pincher_arm_moveit_ikfast_kinematics_plugin)

add_library(${MOVEIT_LIB_NAME} src/phantomx_pincher_arm_ikfast_moveit_plugin.cpp)
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION include)

install(FILES phantomx_pincher_arm_moveit_ikfast_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)















