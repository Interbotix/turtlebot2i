/*
 * Copyright (c) 2011, Vanadium Labs LLC
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Vanadium Labs LLC nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Michael Ferguson, Helen Oleynikova
 */

#include <ros/ros.h>
#include <tf/tf.h>

#include <actionlib/server/simple_action_server.h>
#include <turtlebot2i_marker_manipulation/PickAndDrawAction.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/solid_primitive_dims.h>

#include <geometry_msgs/PoseArray.h>

namespace turtlebot2i_marker_manipulation
{

class PickAndDrawServer
{
private:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<turtlebot2i_marker_manipulation::PickAndDrawAction> as_;
  std::string action_name_;

  turtlebot2i_marker_manipulation::PickAndDrawFeedback     feedback_;
  turtlebot2i_marker_manipulation::PickAndDrawResult       result_;
  turtlebot2i_marker_manipulation::PickAndDrawGoalConstPtr goal_;

  ros::Publisher target_pose_pub_;
  ros::Subscriber pick_and_draw_sub_;

  // Move groups to control arm and gripper with MoveIt!
  moveit::planning_interface::MoveGroupInterface arm_;
  moveit::planning_interface::MoveGroupInterface gripper_;

  // Parameters from goal
  std::string arm_link_;
  double gripper_open;
  double gripper_closed;
  double drawing_surface_height_; //height offset from ground for drawing surface

  // We use the planning_scene_interface::PlanningSceneInterface to manipulate the world
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  // Collision objects for the planning scene
  moveit_msgs::CollisionObject tool_collision_object_;
  moveit_msgs::CollisionObject tool_holder_collision_object_;

  // Arm (eef) heights relative to arm_base_link
  double arm_height_hover_tool_;   //hovering over tool in holder
  double arm_height_grasp_tool_;   //prepared to grasp tool
  double arm_height_detach_tool_;  //detatch grasped tool from holder
  double arm_height_prepare_draw_; //holding tool and preparing to lay ink
  double arm_height_draw_tool_;    //height when drawing with tool grasped      

public:
  PickAndDrawServer(const std::string name) :
    nh_("~"), as_(name, false), action_name_(name), arm_("pincher_arm"), gripper_("pincher_gripper")
  {
    // Register the goal and feedback callbacks
    as_.registerGoalCallback(boost::bind(&PickAndDrawServer::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&PickAndDrawServer::preemptCB, this));

    as_.start();

    target_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/target_pose", 1, true);

    nh_.param<double>("arm_height_hover_tool_", arm_height_hover_tool_, 0.01);
    nh_.param<double>("arm_height_grasp_tool_", arm_height_grasp_tool_, -0.03);
    nh_.param<double>("arm_height_detach_tool_", arm_height_detach_tool_, 0.0);
    nh_.param<double>("arm_height_prepare_draw_", arm_height_prepare_draw_, -0.005);
    nh_.param<double>("arm_height_draw_tool_", arm_height_draw_tool_, -0.030);
    nh_.param<double>("drawing_surface_height", drawing_surface_height_, 0.0058);
  }

  void goalCB()
  {
    ROS_INFO("[pick and draw] Received goal!");
    goal_ = as_.acceptNewGoal();
    arm_link_ = goal_->frame;
    gripper_open = goal_->gripper_open;
    gripper_closed = goal_->gripper_closed;

    arm_.setPoseReferenceFrame( arm_link_ );

    // Allow some leeway in position (meters) and orientation (radians)
    arm_.setGoalPositionTolerance(0.001);
    arm_.setGoalOrientationTolerance(0.1);

    // Allow replanning to increase the odds of a solution
    arm_.allowReplanning(true);

    if (goal_->topic.length() < 1)
    {
      pickAndDraw(goal_->pickup_pose, goal_->draw_pose);
    }
    else
    {
      pick_and_draw_sub_ = nh_.subscribe(goal_->topic, 1, &PickAndDrawServer::sendGoalFromTopic, this);
    }
  }

  void sendGoalFromTopic(const geometry_msgs::PoseArrayConstPtr& msg)
  {
    ROS_INFO("[pick and draw] Got goal from topic! %s", goal_->topic.c_str());
    pickAndDraw(msg->poses[0], msg->poses[1]);
    pick_and_draw_sub_.shutdown();
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }

  bool graspTool(  )
  {
    ROS_DEBUG( "[pick_and_draw] Grasping Tool" );
    /* close gripper */
    if (setGripper( gripper_closed ) == false)
    {
      ROS_ERROR( "[pick_and_draw] Failed to set gripper opening. Reset and try again." );
      return false;
    }
    ros::Duration(0.8).sleep();
    return true;
  }

  geometry_msgs::Pose drawArc( geometry_msgs::Pose target, double radius, double arc_start_rad, int arc_degrees )
  {
    ROS_DEBUG( "[pick_and_draw] Executing Arc" );

    double angle_resolution = 10.0; //Compute waypoint every 10 degrees
    
    double x_center = target.position.x; //center point of arc
    double y_center = target.position.y; 

    //Create cartesian path from waypoints
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose ee_point_goal; //end_effector_trajectory

    //Specify eef orientation
    tf::Quaternion q = tf::createQuaternionFromRPY(0.0, M_PI_2, 0.0); //90 degrees (wrist down)
    tf::quaternionTFToMsg(q, ee_point_goal.orientation);

    //Trajectory planning parameters (circle)
    double d_angle = angle_resolution*3.14/180;
    double angle = arc_start_rad;

    //Start trajectory plan
    ee_point_goal.position.x = x_center + radius*cos(angle);
    ee_point_goal.position.y = y_center + radius*sin(angle);
    ee_point_goal.position.z = target.position.z;
    waypoints.push_back(ee_point_goal);

    //Plan for the trajectory
    for (int i= 0; i< (arc_degrees/angle_resolution); i++)
    {
      //discretize the trajectory
      angle += d_angle;
      ee_point_goal.position.x = x_center + radius*cos(angle);
      ee_point_goal.position.y = y_center + radius*sin(angle);
      waypoints.push_back(ee_point_goal);
    }

    ROS_DEBUG( "[pick_and_draw] There are %d number of waypoints", waypoints.size() );

    //Move over first point in trajectory
    ee_point_goal = waypoints[0];
    ee_point_goal.position.z = arm_height_prepare_draw_ + drawing_surface_height_;
    if (moveArmTo(ee_point_goal) == false)
    {
      ROS_ERROR( "[pick_and_draw] Uable to move arm over start pose for trajectory" );
      return target;
    }

    //Move to first point in trajectory
    ee_point_goal = waypoints[0];
    if (moveArmTo(ee_point_goal) == false)
    {
      ROS_ERROR( "[pick_and_draw] Uable to move arm into start pose for trajectory" );
      ee_point_goal.position.z = arm_height_prepare_draw_ + drawing_surface_height_;
      return ee_point_goal;
    }

    // We want cartesian path to be interpolated at a eef_resolution
    double eef_resolution = std::min(0.1,radius*angle_resolution);
    double jump_threshold = 0.0; //disable the jump threshold
    moveit_msgs::RobotTrajectory trajectory;

    double fraction = arm_.computeCartesianPath(waypoints, eef_resolution, jump_threshold, trajectory);
    ROS_DEBUG("[pick_and_draw] Visualizing plan (cartesian path) (%.2f%% acheived)",  fraction * 100.0);
    if ( fraction != 1.0 )
    {
      ROS_ERROR( "[pick_and_draw] Unable to compute full path. Trajectory cannot be executed." );
      return ee_point_goal;
    }

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    my_plan.trajectory_ = trajectory;
    moveit::planning_interface::MoveItErrorCode result = arm_.execute(my_plan);
    if (bool(result) == false)
    {
      ROS_ERROR("[pick_and_draw] Planning interface execution failed (error %d)", result.val);
      return waypoints[0];
    }

    return waypoints[ waypoints.size()-1 ];
  }

  void pickAndDraw(const geometry_msgs::Pose& start_pose, const geometry_msgs::Pose& draw_pose)
  {
    //TODO: Not using draw_pose for drawing location

    ROS_INFO("[pick and draw] Picking. Drawing. Also placing.");

    geometry_msgs::Pose target_pose;

    /* open gripper */
    if (setGripper(gripper_open) == false)
      return;
    
    addTool( start_pose ); //Adding tool as collision object

    ROS_DEBUG( "[pick_and_draw] Hovering over tool" );

    /* hover over detected cube (tool holder) */
    target_pose = start_pose;
    tf::Quaternion q = tf::createQuaternionFromRPY(0.0, M_PI_2, 0.0); //Wrist pointed straight down
    tf::quaternionTFToMsg(q, target_pose.orientation);
    target_pose.position.z = arm_height_hover_tool_ + drawing_surface_height_;
    if (moveArmTo(target_pose) == false) //Move arm with orientation specified (not modified)
      return;

    ROS_DEBUG( "[pick_and_draw] Dropping down to pickup tool" );

    /* drop down */
    target_pose.position.z = arm_height_grasp_tool_ + drawing_surface_height_;
    if (moveArmTo(target_pose) == false) //Move arm with orientation specified (not modified)
      return;

    removeTool(); //Remove tool before grasping to prevent collision

    /* grasp tool */
    if ( graspTool() == false )
      return;

    /* add and attach tool */
    addTool( start_pose );
    attachTool();

    ROS_DEBUG( "[pick_and_draw] Detaching tool from tool holder (cube)" );
    /* raise up */
    target_pose.position.z = arm_height_detach_tool_ + drawing_surface_height_;
    if (moveArmTo(target_pose) == false) 
      return;

    /* add toolholder to prevent collisions */
    addToolHolder( start_pose );

    ROS_DEBUG( "[pick_and_draw] Moving to first target with tool" );

    /* hover over first target with grasped tool*/
    target_pose = start_pose;
    //tf::Quaternion q = tf::createQuaternionFromRPY(0.0, M_PI_2, 0.0); //Wrist pointed straight down
    tf::quaternionTFToMsg(q, target_pose.orientation);
    target_pose.position.x = 0.152; //Eyes
    target_pose.position.y = 0.025; //Left eye
    target_pose.position.z = arm_height_prepare_draw_ + drawing_surface_height_;
    if (moveArmTo(target_pose) == false) 
      return;

    /* draw first arc */
    target_pose.position.z = arm_height_draw_tool_ + drawing_surface_height_;
    geometry_msgs::Pose last_draw_pose = drawArc( target_pose, 0.02, 0.0, 360 ); //draw a circle around the pose point

    /* move up */
    target_pose = last_draw_pose;
    target_pose.position.z = arm_height_prepare_draw_ + drawing_surface_height_;
    if (moveArmTo(target_pose) == false)
      return;

    /*hover over other eye with same z position */
    target_pose.position.x = 0.152; //Eyes
    target_pose.position.y = -0.025; //Right eye
    if (moveArmTo(target_pose) == false)
      return;

    /* draw second arc */
    target_pose.position.z = arm_height_draw_tool_ + drawing_surface_height_;
    last_draw_pose = drawArc( target_pose, 0.02, 0.0, 360 ); //draw a circle around the pose point

    /* move up */
    target_pose = last_draw_pose;
    target_pose.position.z = arm_height_prepare_draw_ + drawing_surface_height_;
    if (moveArmTo(target_pose) == false)
      return;

    /*hover over other mouth with same z position */
    target_pose.position.x = 0.115; //Mouth
    target_pose.position.y = 0.0;
    if (moveArmTo(target_pose) == false)
      return;

    target_pose.position.z = arm_height_draw_tool_ + drawing_surface_height_;
    last_draw_pose = drawArc( target_pose, 0.02, 1.57, 180 ); //draw a half circle around the pose point (ccw from 90)

    /* raise up */
    target_pose = last_draw_pose;
    target_pose.position.z = arm_height_prepare_draw_ + drawing_surface_height_;
    if (moveArmTo(target_pose) == false)
      return;

    /* move back to tool holder */
    target_pose.position.x = start_pose.position.x;
    target_pose.position.y = start_pose.position.y;
    target_pose.position.z = arm_height_detach_tool_ + drawing_surface_height_;
    if (moveArmTo(target_pose) == false)
      return;

    /* remove toolholder collision object */
    removeToolHolder();

    /*drop down*/
    target_pose.position.z = arm_height_grasp_tool_ + drawing_surface_height_;
    if (moveArmTo(target_pose) == false)
      return;

    detachTool();
    removeTool();

    /* open gripper */
    if (setGripper(gripper_open) == false)
      return;
    ros::Duration(0.6).sleep(); // ensure that gripper properly release the tool before lifting the arm

    /*raise up away from tool sitting in holder*/
    target_pose.position.z = arm_height_hover_tool_ + drawing_surface_height_;
    if (moveArmTo(target_pose) == false) //Move arm with orientation specified (not modified)
      return;

    addTool( start_pose );

    /* move out of camera's view */
    if (moveArmTo("pose_parked") == false)
      return;

    removeTool();
    
    as_.setSucceeded(result_);

  }

private:
  /**
   * Move arm to a named configuration, normally described in the robot semantic description SRDF file.
   * @param target Named target to achieve
   * @return True of success, false otherwise
   */
  bool moveArmTo(const std::string& target)
  {
    ROS_DEBUG("[pick and draw] Move arm to '%s' position", target.c_str());
    if (arm_.setNamedTarget(target) == false)
    {
      ROS_ERROR("[pick and draw] Set named target '%s' failed", target.c_str());
      return false;
    }

    moveit::planning_interface::MoveItErrorCode result = arm_.move();
    if (bool(result) == true)
    {
      return true;
    }
    else
    {
      ROS_ERROR("[pick and draw] Move to target failed (error %d)", result.val);
      as_.setAborted(result_);
      return false;
    }
  }

  /**
   * Move arm to a target pose.
   * @param target Pose target to achieve
   * @return True of success, false otherwise
   */
  bool moveArmTo(const geometry_msgs::Pose& target)
  {
    int attempts = 0;
    ROS_DEBUG("[pick_and_draw] Moving arm to [%.2f, %.2f, %.2f, %.2f]",
             target.position.x, target.position.y, target.position.z, tf::getYaw(target.orientation));
    while (attempts < 5)
    {
      geometry_msgs::PoseStamped modify_target;
      modify_target.header.frame_id = arm_link_;
      modify_target.pose = target;

      double x = modify_target.pose.position.x;
      double y = modify_target.pose.position.y;
      double z = modify_target.pose.position.z;
      double d = sqrt(x*x + y*y);
      if (d > 0.3)
      {
        // Maximum reachable distance by the arm is 30 cm
        ROS_ERROR("Target pose out of reach [%f > %f]", d, 0.3);
        as_.setAborted(result_);
        return false;
      }

      target_pose_pub_.publish(modify_target);

      if (arm_.setPoseTarget(modify_target) == false)
      {
        ROS_ERROR("[pick_and_draw] Set pose target [%.2f, %.2f, %.2f, %.2f] failed",
                  modify_target.pose.position.x, modify_target.pose.position.y, modify_target.pose.position.z,
                  tf::getYaw(modify_target.pose.orientation));
        as_.setAborted(result_);
        return false;
      }

      moveit::planning_interface::MoveItErrorCode result = arm_.move();
      if (bool(result) == true)
      {
        return true;
      }
      else
      {
        ROS_ERROR("[pick_and_draw] Move to target failed (error %d) at attempt %d",
                  result.val, attempts + 1);
      }
      attempts++;
    }

    ROS_ERROR("[pick_and_draw] Move to target failed after %d attempts", attempts);
    as_.setAborted(result_);
    return false;
  }

  void addTool( const geometry_msgs::Pose & tool_pose )
  {
    // Add the dry erase marker "tool" as a collision object into the world
    double tool_size_x = 0.018;
    double tool_size_y = 0.018;
    double tool_size_z = 0.115;

    tool_collision_object_.header.stamp = ros::Time::now();
    tool_collision_object_.header.frame_id = arm_link_;

    tool_collision_object_.id = "tool";
    planning_scene_interface_.removeCollisionObjects(std::vector<std::string>(1, tool_collision_object_.id));

    tool_collision_object_.operation = moveit_msgs::CollisionObject::ADD;
    tool_collision_object_.primitives.resize(1);
    tool_collision_object_.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    tool_collision_object_.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    tool_collision_object_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = tool_size_x;
    tool_collision_object_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = tool_size_y;
    tool_collision_object_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = tool_size_z;
    tool_collision_object_.primitive_poses.resize(1);
    tool_collision_object_.primitive_poses[0].position.x = tool_pose.position.x;
    tool_collision_object_.primitive_poses[0].position.y = tool_pose.position.y;
    tool_collision_object_.primitive_poses[0].position.z = tool_pose.position.z + tool_size_z/2.0;

    ROS_DEBUG("[pick_and_draw] Adding tool as a collision object into the world");
    std::vector<moveit_msgs::CollisionObject> collision_objects(1, tool_collision_object_);
    planning_scene_interface_.addCollisionObjects(collision_objects);

    ros::Duration(3.0).sleep();
  }
  void removeTool()
  {
    ROS_DEBUG("[pick_and_draw] Removing the tool collision object");
    std::vector<std::string> object_ids;
    object_ids.push_back(tool_collision_object_.id);
    planning_scene_interface_.removeCollisionObjects(object_ids);

    ros::Duration(3.0).sleep();
  }
  void attachTool()
  {
    ROS_WARN("Attaching collision object to arm." );

    if ( arm_.attachObject(tool_collision_object_.id, "gripper_active_link") )
    {
      ROS_WARN( "The tool should be attached to gripper_link now" );
    }
    else ROS_ERROR( "Failed to attachTool to arm_" );

    ros::Duration(3.0).sleep();
  }
  void detachTool()
  {
    ROS_WARN( "Detaching collision object from arm." );
    arm_.detachObject(tool_collision_object_.id);
    ros::Duration(3.0).sleep();
  }

  void addToolHolder( const geometry_msgs::Pose & tool_pose )
  {
    // Add the toolholder collision object into the world
    double tool_size_x = 0.02;
    double tool_size_y = 0.02;
    double tool_size_z = 0.02;

    tool_holder_collision_object_.header.stamp = ros::Time::now();
    tool_holder_collision_object_.header.frame_id = arm_link_;

    tool_holder_collision_object_.id = "toolholder";
    planning_scene_interface_.removeCollisionObjects(std::vector<std::string>(1, tool_holder_collision_object_.id));

    tool_holder_collision_object_.operation = moveit_msgs::CollisionObject::ADD;
    tool_holder_collision_object_.primitives.resize(1);
    tool_holder_collision_object_.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    tool_holder_collision_object_.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    tool_holder_collision_object_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = tool_size_x;
    tool_holder_collision_object_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = tool_size_y;
    tool_holder_collision_object_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = tool_size_z;
    tool_holder_collision_object_.primitive_poses.resize(1);
    tool_holder_collision_object_.primitive_poses[0].position.x = tool_pose.position.x;
    tool_holder_collision_object_.primitive_poses[0].position.y = tool_pose.position.y;
    tool_holder_collision_object_.primitive_poses[0].position.z = tool_pose.position.z;;

    ROS_DEBUG("[pick_and_draw] Adding toolholder as a collision object into the world");
    std::vector<moveit_msgs::CollisionObject> collision_objects(1, tool_holder_collision_object_);
    planning_scene_interface_.addCollisionObjects(collision_objects);

    ros::Duration(3.0).sleep();
  }
  void removeToolHolder()
  {
    ROS_DEBUG("[pick_and_draw] Removing the toolholder collision object");
    std::vector<std::string> object_ids;
    object_ids.push_back(tool_holder_collision_object_.id);
    planning_scene_interface_.removeCollisionObjects(object_ids);

    ros::Duration(3.0).sleep();
  }

  /**
   * Set gripper opening.
   * @param opening Physical opening of the gripper, in meters
   * @return True of success, false otherwise
   */
  bool setGripper(float opening)
  {
    ROS_DEBUG("[pick and draw] Set gripper opening to %f", opening);
    if (gripper_.setJointValueTarget("gripper_joint", opening) == false)
    {
      ROS_ERROR("[pick and draw] Set gripper opening to %f failed", opening);
      return false;
    }

    moveit::planning_interface::MoveItErrorCode result = gripper_.move();
    if (bool(result) == true)
    {
      return true;
    }
    else
    {
      ROS_ERROR("[pick and draw] Set gripper opening failed (error %d)", result.val);
      as_.setAborted(result_);
      return false;
    }
  }
};

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_and_draw_action_server");

  turtlebot2i_marker_manipulation::PickAndDrawServer server("pick_and_draw");

  //TODO: Does not work with only: ros::spin();
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::Rate rate(1.0);
  while( ros::ok() )
  {
    ros::spinOnce();
    rate.sleep();
  }
  spinner.stop();

  return 0;
}
