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
#include <turtlebot2i_block_manipulation/BlockDetectionAction.h>
#include <turtlebot2i_block_manipulation/PickAndPlaceAction.h>
#include <turtlebot2i_block_manipulation/InteractiveBlockManipulationAction.h>
#include <arbotix_msgs/Relax.h>

#include <geometry_msgs/PoseArray.h>

#include <string>
#include <sstream>

const std::string gripper_param = "/gripper_controller";
const std::string pick_and_place_topic = "/pick_and_place";

namespace turtlebot2i_block_manipulation
{

class BlockManipulationAction
{
private:
  ros::NodeHandle nh_;
  
  geometry_msgs::PoseArray poseMsg;
  geometry_msgs::Pose start_pose_bumped, end_pose_bumped;
  
  // Actions and services
  actionlib::SimpleActionClient<BlockDetectionAction> block_detection_action_;
  actionlib::SimpleActionClient<InteractiveBlockManipulationAction> interactive_manipulation_action_;
  actionlib::SimpleActionClient<PickAndPlaceAction> pick_and_place_action_;
  
  BlockDetectionGoal block_detection_goal_;
  InteractiveBlockManipulationGoal interactive_manipulation_goal_;
  PickAndPlaceGoal pick_and_place_goal_;

  // Parameters
  std::string arm_link;
  double gripper_open, gripper_tighten, gripper_closed, z_up, z_down, block_size;
  
  double target_match_x, target_match_y, target_discard_x, target_discard_y, target_bin_height;

  turtlebot2i_block_manipulation::BlockPoseArray block_list_; //list of the positions and colors of blocks we've found
  
  int blockIndex;  // block we are working on
  int blockCount;  // number of blocks found
  
public:
    std::string armMode;
    bool auto_sort_;

  BlockManipulationAction() : nh_("~"),
    block_detection_action_("block_detection", true),
    interactive_manipulation_action_("interactive_manipulation", true),
    pick_and_place_action_("pick_and_place", true)
  {
    // Load parameters
    nh_.param<std::string>("arm_link", arm_link, "/arm_link");
    nh_.param<double>(gripper_param + "/max_opening", gripper_open, 0.042);
    nh_.param<double>("grip_tighten", gripper_tighten, 0.004);
    nh_.param<double>("z_up", z_up, 0.06);   // amount to lift during move
    nh_.param<double>("table_height", z_down, 0.0);
    nh_.param<double>("block_size", block_size, 0.02);  // block size to detect
    
    nh_.param<double>("target_match_x", target_match_x, 0.22);    // X target for matching block
    nh_.param<double>("target_match_y", target_match_y, -0.20);   // Y target for matching block

    nh_.param<double>("target_discard_x", target_discard_x, 0.22);    // X target for non-matching block
    nh_.param<double>("target_discard_y", target_discard_y, 0.20);   // Y target for non-matching block
    
    nh_.param<double>("target_bin_height", target_bin_height, 0.06);   // Where to lower the arm when dropping into bin

    nh_.param<bool>("auto_sort", auto_sort_, false); //If we should detect blocks and automatically sort them

    // Initialize goals
    block_detection_goal_.frame = arm_link;
    block_detection_goal_.table_height = z_down;
    block_detection_goal_.block_size = block_size;
    
    pick_and_place_goal_.frame = arm_link;
    pick_and_place_goal_.z_up = z_up;
    pick_and_place_goal_.gripper_open = gripper_open;
    pick_and_place_goal_.gripper_closed = block_size - gripper_tighten;
    pick_and_place_goal_.topic = pick_and_place_topic;
        
    ROS_INFO("Gripper settings: closed=%.4f block size=%.4f tighten=%.4f", (float)pick_and_place_goal_.gripper_closed, (float)block_size, (float)gripper_tighten );
    
    interactive_manipulation_goal_.block_size = block_size;
    interactive_manipulation_goal_.frame = arm_link;
    
    ROS_INFO("Finished initializing, waiting for servers...");
    
    block_detection_action_.waitForServer();
    ROS_INFO(" 1. Found block_detection server.");
    
    interactive_manipulation_action_.waitForServer();
    ROS_INFO(" 2. Found interactive_manipulation server.");
    
    pick_and_place_action_.waitForServer();
    ROS_INFO(" 3. Found pick_and_place server.");
    
  }
  
  void detectBlocks()
  {
  // Have Block Detection Server detect blocks and callback "addBlocks" when done
    block_detection_action_.sendGoal(block_detection_goal_, boost::bind( &BlockManipulationAction::addBlocks, this, _1, _2));
  }
  
  void addBlocks(const actionlib::SimpleClientGoalState& state, const BlockDetectionResultConstPtr& result)
  {
    //ROS_INFO(" Got block detection callback. Adding blocks.");
    turtlebot2i_block_manipulation::BlockPose block;
    
    if (state != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_ERROR("  Failed! %s",  state.toString().c_str());
      ros::shutdown();
    }
    
    blockCount = result->colored_blocks.poses.size();
    blockIndex = 0;

    // Save blocks for later use during sorting
    block_list_.poses.clear();
    for (unsigned int i=0; i < blockCount; i++)
    {    
      block_list_.poses.push_back( result->colored_blocks.poses[i] );
      //ROS_INFO("Added block %d x=%f", i, blockList[i].position.x);
    }

    // Add blocks to Interactive Manipulation Server for RViz visualization
    interactive_manipulation_action_.sendGoal(interactive_manipulation_goal_, boost::bind( &BlockManipulationAction::pickAndPlace, this, _1, _2));
  }
  
  void pickAndPlace(const actionlib::SimpleClientGoalState& state, const InteractiveBlockManipulationResultConstPtr& result)
  {
    if (state != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_ERROR("  Select Marker Failed! %s",  state.toString().c_str());
      ros::shutdown();
    }
    
    //ROS_INFO(" Got interactive marker callback.");
    ROS_INFO("Picking and placing...");
    
    pick_and_place_action_.sendGoal(pick_and_place_goal_, boost::bind( &BlockManipulationAction::finish, this, _1, _2));
  }
  
  void finish(const actionlib::SimpleClientGoalState& state, const PickAndPlaceResultConstPtr& result)
  {
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO(" Pick and place - Succeeded!");
    }
    else
    {
      ROS_INFO(" Pick and place - Failed! %s",  state.toString().c_str());
    }
            
    if (blockIndex < blockCount && armMode == "m")
    {
      organizeBlocks();
    }
    else
    {
      detectBlocks();
    }
  }
  
  //Organize the blocks by color
  void organizeBlocks() 
  {
    ROS_WARN_STREAM( "Organizing Blocks: Block # "<< blockIndex + 1 << ", RGB: " << block_list_.poses[blockIndex].color.r << ", " << block_list_.poses[blockIndex].color.g << ", " << block_list_.poses[blockIndex].color.b );
    int targetCount;
    
    geometry_msgs::Pose endPose;
    
    // This routine calculates the target position for block.
    // Blocks which match the target color go to one area, others go to second area
    //if (block_list_.poses[blockIndex].color.g > 128.0f )  // Does this block match target color?
    if ( block_list_.poses[blockIndex].color.g > block_list_.poses[blockIndex].color.r && block_list_.poses[blockIndex].color.g > block_list_.poses[blockIndex].color.b )
    {
      ROS_DEBUG("Block matches target Color");
      endPose.position.x = target_match_x;
      endPose.position.y = target_match_y;
    }
    else  // Not target color, use alternate destination
    {
      ROS_DEBUG("Block is not target Color");
      endPose.position.x = target_discard_x;
      endPose.position.y = target_discard_y;
    }
    endPose.position.z = target_bin_height;
    
    ROS_DEBUG("Block End Pose x=%.4f y=%.4f z=%.4f", endPose.position.x, endPose.position.y, endPose.position.z);
    
    // Go through list of blocks and move them
    if (blockIndex < blockCount)
    {
      moveBlock(block_list_.poses[blockIndex], endPose);
      blockIndex++;
    }
  }
    
  void moveBlock(const turtlebot2i_block_manipulation::BlockPose& start_block_pose, const geometry_msgs::Pose& end_pose)
  {
    geometry_msgs::Pose start_pose;
    start_pose.position = start_block_pose.position;
    start_pose.orientation = start_block_pose.orientation;

    double bump_size = 0.005;
  
    // Return pickup and place poses as the result of the action
    
    start_pose_bumped = start_pose;
    start_pose_bumped.position.z -= block_size/2.0 - bump_size;
    
    end_pose_bumped = end_pose;
    end_pose_bumped.position.z -= block_size/2.0 - bump_size;
    
    // Publish pickup and place poses for visualizing on RViz
    poseMsg.header.frame_id = arm_link;
    poseMsg.header.stamp = ros::Time::now();
    poseMsg.poses.push_back(start_pose_bumped);
    poseMsg.poses.push_back(end_pose_bumped);
    //ROS_INFO("Demo publishing to PickNPlace.  PoseX=%.4f", (float) poseMsg.poses[0].position.x);
    
    pick_and_place_goal_.pickup_pose = start_pose_bumped;
    pick_and_place_goal_.place_pose = end_pose_bumped;
    pick_and_place_goal_.topic = "";
    
    pick_and_place_action_.sendGoal(pick_and_place_goal_, boost::bind( &BlockManipulationAction::finish, this, _1, _2));
    
    pick_and_place_goal_.topic = pick_and_place_topic;  // restore topic
   }
 };
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "block_manipulation");
  turtlebot2i_block_manipulation::BlockManipulationAction manip;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  if ( manip.auto_sort_ )
  {
    ros::Duration(2.0).sleep();
    manip.detectBlocks();
    manip.armMode="d";
    ros::Duration(5.0).sleep();
    manip.organizeBlocks();
    manip.armMode="m";
  }

  while (ros::ok())
  {
    //Allow user restarting, in case block detection fails or scene changes
    std::cout << "d - Detect, m - Move" << std::endl;
    
    std::string instr;
    getline (std::cin, instr );
     
    if (instr == "d") {manip.detectBlocks(); manip.armMode="d";}
    if (instr == "m") {manip.organizeBlocks();manip.armMode="m";}
  }

  spinner.stop();
}
