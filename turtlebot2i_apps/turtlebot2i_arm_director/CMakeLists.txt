cmake_minimum_required(VERSION 2.8.3)
project(turtlebot2i_arm_director)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  moveit_core
  moveit_ros_planning_interface
  roscpp
  std_msgs
)

find_package(Boost REQUIRED system filesystem)
link_directories(${catkin_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})

include_directories(include
                    SYSTEM
                    ${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                   )

add_action_files(FILES NamedPoseDirector.action)
generate_messages(DEPENDENCIES actionlib_msgs)


catkin_package(DEPENDS roscpp moveit_core moveit_ros_planning_interface)
add_compile_options("-std=c++11")

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES turtlebot2i_arm_director
  CATKIN_DEPENDS moveit_core moveit_ros_planning_interface roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR})

add_executable(named_pose_director src/named_pose_director.cpp)
target_link_libraries(named_pose_director ${catkin_LIBRARIES})

add_dependencies(named_pose_director ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(named_pose_director turtlebot2i_arm_director_generate_messages_cpp)

add_compile_options(${MOVEIT_CXX_FLAGS})

