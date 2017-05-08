cmake_minimum_required(VERSION 2.8.3)
project(turtlebot2i_block_manipulation)

# setup
find_package(catkin REQUIRED actionlib actionlib_msgs interactive_markers pcl_ros roscpp visualization_msgs arbotix_msgs moveit_core moveit_ros_planning_interface)
find_package(Boost REQUIRED system filesystem)
find_package(PCL REQUIRED)
link_directories(${catkin_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS})
include_directories(include
                    SYSTEM
                    ${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
                   )

add_message_files(FILES BlockPose.msg BlockPoseArray.msg)

add_action_files(FILES
                 BlockDetection.action
                 InteractiveBlockManipulation.action
                 PickAndPlace.action
                 )

generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs)
catkin_package(DEPENDS actionlib actionlib_msgs interactive_markers pcl_ros roscpp visualization_msgs arbotix_msgs moveit_core moveit_ros_planning_interface)

add_compile_options("-std=c++11")

# nodes
add_executable(block_detection_action_server src/block_detection_action_server.cpp)
target_link_libraries(block_detection_action_server ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(block_detection_action_server turtlebot2i_block_manipulation_generate_messages_cpp)

add_executable(interactive_manipulation_action_server src/interactive_manipulation_action_server.cpp)
target_link_libraries(interactive_manipulation_action_server ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(interactive_manipulation_action_server turtlebot2i_block_manipulation_generate_messages_cpp)

add_executable(pick_and_place_action_server src/pick_and_place_action_server.cpp)
target_link_libraries(pick_and_place_action_server ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(pick_and_place_action_server turtlebot2i_block_manipulation_generate_messages_cpp)


# demo
add_executable(block_manipulation_demo demo/block_manipulation_demo.cpp)
target_link_libraries(block_manipulation_demo ${catkin_LIBRARIES})

add_dependencies(block_manipulation_demo turtlebot2i_block_manipulation_generate_messages_cpp arbotix_msgs_generate_messages_cpp)


# install
install(TARGETS block_detection_action_server interactive_manipulation_action_server pick_and_place_action_server
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS block_manipulation_demo
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY demo
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
