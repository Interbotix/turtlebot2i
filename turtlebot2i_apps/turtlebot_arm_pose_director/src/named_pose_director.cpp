
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/solid_primitive_dims.h>

#include <actionlib/server/simple_action_server.h>
#include <turtlebot2i_arm_director/NamedPoseDirectorAction.h>

namespace turtlebot2i_arm_director
{

class NamedPoseDirector
{
private:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<turtlebot2i_arm_director::NamedPoseDirectorAction> as_;
  turtlebot2i_arm_director::NamedPoseDirectorGoalConstPtr goal_;
  turtlebot2i_arm_director::NamedPoseDirectorResult       result_;

  ros::Subscriber named_pose_sub_;

  // We use the planning_interface::PlanningSceneInterface to manipulate the world
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  // Move group to control arm with MoveIt!
  moveit::planning_interface::MoveGroupInterface arm_;

  std::string action_server_name_;

  // Parameters from node
  std::vector<double> floor_pose_;

  // Constants
  const double FLOOR_SIZE_X = 0.42;
  const double FLOOR_SIZE_Y = 0.6;
  const double FLOOR_SIZE_Z = 0.05;

public:
  NamedPoseDirector(const std::string name) :
    nh_("~"),
    as_(name, false),
    arm_("pincher_arm"),
    action_server_name_(name),
    floor_pose_({0.078, 0.0, -0.012})
  {
    // Register the goal and feeback callbacks.
    as_.registerGoalCallback(boost::bind(&NamedPoseDirector::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&NamedPoseDirector::preemptCB, this));
    
    as_.start();

    //Add a floor collision object to prevent trajectories that would put the arm below the base
    if ((nh_.getParam("floor_pose", floor_pose_) == true) && (floor_pose_.size() != 3))
    {
      ROS_ERROR("Invalid floor_pose vector size; must contain 3 values (x, y, z); ignoring");
      floor_pose_.clear();
    }
    addFloor();

    std::string startup_pose;
    if ( (nh_.getParam("startup_pose", startup_pose) == true) )
    {
      moveArmTo( startup_pose );
    }
  }

  void goalCB()
  {
    ROS_INFO("[NamedPoseDirector] Received goal!");

    // accept the new goal 
    goal_ = as_.acceptNewGoal();
    
    if ( moveArmTo( goal_->named_pose ) == false)
    {
      as_.setAborted(result_);
    }
    else
    {
      as_.setSucceeded(result_);
    }
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_server_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }

private:
  /**
   * Move arm to a named configuration, normally described in the robot semantic description SRDF file.
   * @param target Named target to achieve
   * @return True of success, false otherwise
   */
  bool moveArmTo(const std::string& target)
  {
    ROS_INFO("[NamedPoseDirector] Move arm to '%s' position", target.c_str());
    
    if (arm_.setNamedTarget(target) == false)
    {
      ROS_ERROR("[NamedPoseDirector] Set named target '%s' failed", target.c_str());
      return false;
    }

    moveit::planning_interface::MoveItErrorCode result = arm_.move();
    if (bool(result) == true)
    {
      return true;
    }
    else
    {
      ROS_ERROR("[NamedPoseDirector] Move to target failed (error %d)", result.val);
      return false;
    }
  }

  void addFloor()
  {
    // Add a floor as a collision object into the world
    moveit_msgs::CollisionObject co;
    co.header.stamp = ros::Time::now();
    co.header.frame_id = "base_link";

    co.id = "floor";
    planning_scene_interface_.removeCollisionObjects(std::vector<std::string>(1, co.id));

    co.operation = moveit_msgs::CollisionObject::ADD;
    co.primitives.resize(1);
    co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    co.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = FLOOR_SIZE_X;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = FLOOR_SIZE_Y;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = FLOOR_SIZE_Z;
    co.primitive_poses.resize(1);
    co.primitive_poses[0].position.x = floor_pose_[0] + FLOOR_SIZE_X/2.0;
    co.primitive_poses[0].position.y = floor_pose_[1];
    co.primitive_poses[0].position.z = floor_pose_[2] - FLOOR_SIZE_Z/2.0;

    ROS_INFO("Add the floor as a collision object into the world");
    std::vector<moveit_msgs::CollisionObject> collision_objects(1, co);
    planning_scene_interface_.addCollisionObjects(collision_objects);

    ros::Duration( 3.0 ).sleep();
  }
};

};

int main(int argc, char** argv)
{

  ros::init(argc, argv, "named_pose_director");

  // Setup an asynchronous spinner as the move group operations need continuous spinning
  ros::AsyncSpinner spinner(2);
  spinner.start();

  turtlebot2i_arm_director::NamedPoseDirector director( "named_pose_director" );

  ros::Rate rate(1.0);
  while( ros::ok() )
  {
    ros::spinOnce();
    rate.sleep();
  }

  spinner.stop();
  
  return 0;
}
