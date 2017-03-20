#ifndef NAVIGATION_UTILS_H_
#define NAVIGATION_UTILS_H_

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool goToDest(tf::Vector3 go_posi, tf::Quaternion q);

#endif
