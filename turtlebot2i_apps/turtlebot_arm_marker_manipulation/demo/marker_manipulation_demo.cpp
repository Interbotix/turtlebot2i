/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Distribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Distributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Distributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <turtlebot2i_marker_manipulation/ToolDetectionAction.h>
#include <turtlebot2i_marker_manipulation/PickAndDrawAction.h>
#include <arbotix_msgs/Relax.h>

#include <geometry_msgs/PoseArray.h>

#include <string>
#include <sstream>

const std::string gripper_param = "/gripper_controller";
const std::string pick_and_draw_topic = "/pick_and_draw";

namespace turtlebot2i_marker_manipulation
{

class MarkerManipulationAction
{
public:
  bool auto_draw_;

private:
  ros::NodeHandle nh_;
  
  geometry_msgs::PoseArray poseMsg;
  geometry_msgs::Pose start_pose_bumped, draw_pose;
  
  // Actions and services
  actionlib::SimpleActionClient<ToolDetectionAction> tool_detection_action_;
  actionlib::SimpleActionClient<PickAndDrawAction> pick_and_draw_action_;
  
  ToolDetectionGoal tool_detection_goal_;
  PickAndDrawGoal pick_and_draw_goal_;

  // Parameters
  std::string arm_link;
  double gripper_open, gripper_tighten, gripper_closed, table_height, toolholder_size, tool_size;

  turtlebot2i_marker_manipulation::BlockPoseArray block_list_; //list of the positions and colors of blocks we've found
  
  int blockCount;  //number of blocks found
  
public:
  
  MarkerManipulationAction() : nh_("~"),
    tool_detection_action_("block_detection", true),
    pick_and_draw_action_("pick_and_draw", true)
  {
    // Load parameters
    nh_.param<std::string>("arm_link", arm_link, "/arm_link");
    nh_.param<double>(gripper_param + "/max_opening", gripper_open, 0.031);
    nh_.param<double>("gripper_tighten", gripper_tighten, 0.006); //How far do we tighten the gripper beyond the object size
    nh_.param<double>("table_height", table_height, 0.0); //Position of the table/workarea in the world
    nh_.param<double>("toolholder_size", toolholder_size, 0.02);  //Size of toolholder to detect
    nh_.param<double>("tool_size", tool_size, 0.0185);  //Size of tool to grasp
    nh_.param<bool>("auto_draw", auto_draw_, false); //If we should detect toolholder and automatically execute drawing

    // Initialize goals
    tool_detection_goal_.frame = arm_link;
    tool_detection_goal_.table_height = table_height;
    tool_detection_goal_.toolholder_size = toolholder_size;
    
    pick_and_draw_goal_.frame = arm_link;
    pick_and_draw_goal_.gripper_open = gripper_open;
    pick_and_draw_goal_.gripper_closed = tool_size - gripper_tighten;
    pick_and_draw_goal_.topic = pick_and_draw_topic;
        
    ROS_INFO("Gripper settings: closed=%.4f block size=%.4f tighten=%.4f", (float)pick_and_draw_goal_.gripper_closed, (float)toolholder_size, (float)gripper_tighten );
    
    ROS_INFO("Finished initializing, waiting for servers...");
    
    tool_detection_action_.waitForServer();
    ROS_INFO(" 1. Found block_detection server.");

    pick_and_draw_action_.waitForServer();
    ROS_INFO(" 2. Found pick_and_draw server.");   
  }
  
  void detectTool()
  {
    // Detect tool location and callback "addTool" when done
    tool_detection_action_.sendGoal(tool_detection_goal_, boost::bind( &MarkerManipulationAction::addTool, this, _1, _2));
  }
  
  void addTool(const actionlib::SimpleClientGoalState& state, const ToolDetectionResultConstPtr& result)
  {
    turtlebot2i_marker_manipulation::BlockPose block;
    
    if (state != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_ERROR("addTool() Failed! %s",  state.toString().c_str());
      ros::shutdown();
    }
    
    blockCount = result->colored_blocks.poses.size();

    // Save blocks for later use during sorting
    block_list_.poses.clear();
    for (unsigned int i=0; i < blockCount; i++)
    {    
      block_list_.poses.push_back( result->colored_blocks.poses[i] );
    }
  }
  
  void finish(const actionlib::SimpleClientGoalState& state, const PickAndDrawResultConstPtr& result)
  {
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO(" Pick and draw - Succeeded!");
    }
    else
    {
      ROS_INFO(" Pick and draw - Failed! %s",  state.toString().c_str());
    }
            
    detectTool();
  }
  
  void executeDrawing() 
  {
    geometry_msgs::Pose endPose;

    // Verify we only have one detected tool/holder
    if (blockCount == 1)
    {
      sendDrawGoal(block_list_.poses[0], endPose);
    }
    else
    {
      ROS_ERROR( "executeDrawing() failed due to too many detected tool locations" );
    }
  }
    
  void sendDrawGoal(const turtlebot2i_marker_manipulation::BlockPose& start_block_pose, const geometry_msgs::Pose& drawing_pose)
  {
    geometry_msgs::Pose start_pose;
    start_pose.position = start_block_pose.position;
    start_pose.orientation = start_block_pose.orientation;

    double bump_size = 0.005;
  
    // Return pickup and place poses as the result of the action
    
    start_pose_bumped = start_pose;
    start_pose_bumped.position.z -= toolholder_size/2.0 - bump_size;
    
    draw_pose = drawing_pose;
    draw_pose.position.z -= toolholder_size/2.0 - bump_size;
    
    // Publish pickup and place poses for visualizing on RViz
    poseMsg.header.frame_id = arm_link;
    poseMsg.header.stamp = ros::Time::now();
    poseMsg.poses.push_back(start_pose_bumped);
    poseMsg.poses.push_back(draw_pose);
    
    pick_and_draw_goal_.pickup_pose = start_pose_bumped;
    pick_and_draw_goal_.draw_pose = draw_pose;
    pick_and_draw_goal_.topic = "";
    
    pick_and_draw_action_.sendGoal(pick_and_draw_goal_, boost::bind( &MarkerManipulationAction::finish, this, _1, _2));
    
    pick_and_draw_goal_.topic = pick_and_draw_topic;  // restore topic
   }
 };
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "marker_manipulation");
  turtlebot2i_marker_manipulation::MarkerManipulationAction manip;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  if ( manip.auto_draw_ )
  {
    ros::Duration(2.0).sleep();
    manip.detectTool();
    ros::Duration(5.0).sleep();
    manip.executeDrawing();
  }

  while (ros::ok())
  {
    //Allow user restarting, in case tool detection fails or scene changes
    std::cout << "d - Detect, m - Move" << std::endl;
    
    std::string instr;
    getline( std::cin, instr );
     
    if (instr == "d") { manip.detectTool(); }
    if (instr == "m") { manip.executeDrawing(); }
  }

  spinner.stop();
}
