#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>
#include "navigation_utils.h"

namespace zone_destination_node
{

class DestinationProcessor
{
private:
  ros::NodeHandle node_;

  tf::TransformListener listener_;

  tf::Quaternion destination_rotation_[3];
  tf::Vector3 destination_position_[3];
  
  visualization_msgs::MarkerArray destination_rviz_markers_;

  ros::Publisher vis_pub_;
  ros::Subscriber destination_sub_;

  bool zone_identified_[3];

public:

  DestinationProcessor()
  {
    addMarker( &destination_rviz_markers_, 0, 1.0, 0.0, 0.0 );
    addMarker( &destination_rviz_markers_, 1, 0.0, 1.0, 0.0 );
    addMarker( &destination_rviz_markers_, 2, 0.0, 0.0, 1.0 );

    vis_pub_ = node_.advertise<visualization_msgs::MarkerArray>( "zone_destination_marker", 0 );
    destination_sub_ = node_.subscribe<std_msgs::String>("zone_destination_request", 1, &DestinationProcessor::destinationCallback, this );
  }

  void updateDestinations()
  {
    tf::StampedTransform transform;
    
    try
    {
      listener_.lookupTransform("/map", "/charging_zone", ros::Time(0), transform);
      
      destination_position_[0] = transform.getOrigin();
      destination_rviz_markers_.markers[0].pose.position.x = destination_position_[0].x();
      destination_rviz_markers_.markers[0].pose.position.y = destination_position_[0].y();
      destination_rviz_markers_.markers[0].pose.position.z = destination_position_[0].z();
      destination_rviz_markers_.markers[0].color.a = 1.0;

      destination_rotation_[0] = transform.getRotation();
      destination_rviz_markers_.markers[0].pose.orientation.x = destination_rotation_[0].getX();
      destination_rviz_markers_.markers[0].pose.orientation.y = destination_rotation_[0].getY();
      destination_rviz_markers_.markers[0].pose.orientation.z = destination_rotation_[0].getZ();
      destination_rviz_markers_.markers[0].pose.orientation.w = destination_rotation_[0].getW();

      zone_identified_[0] = true;
    }
    catch (tf::TransformException &ex)
    {
      //ROS_ERROR_THROTTLE( 0.5, "%s",ex.what());
    }
    try
    {
      listener_.lookupTransform("/map", "/pickup_zone", ros::Time(0), transform);

      destination_position_[1] = transform.getOrigin();
      destination_rviz_markers_.markers[1].pose.position.x = destination_position_[1].x();
      destination_rviz_markers_.markers[1].pose.position.y = destination_position_[1].y();
      destination_rviz_markers_.markers[1].pose.position.z = destination_position_[1].z();
      destination_rviz_markers_.markers[1].color.a = 1.0;

      destination_rotation_[1] = transform.getRotation();
      destination_rviz_markers_.markers[1].pose.orientation.x = destination_rotation_[1].getX();
      destination_rviz_markers_.markers[1].pose.orientation.y = destination_rotation_[1].getY();
      destination_rviz_markers_.markers[1].pose.orientation.z = destination_rotation_[1].getZ();
      destination_rviz_markers_.markers[1].pose.orientation.w = destination_rotation_[1].getW();

      zone_identified_[1] = true;
    }
    catch (tf::TransformException &ex)
    {
      //ROS_ERROR_THROTTLE( 0.5, "%s",ex.what());
    }
    try
    {
      listener_.lookupTransform("/map", "/sorting_zone", ros::Time(0), transform);
      destination_position_[2] = transform.getOrigin();
      destination_rviz_markers_.markers[2].pose.position.x = destination_position_[2].x();
      destination_rviz_markers_.markers[2].pose.position.y = destination_position_[2].y();
      destination_rviz_markers_.markers[2].pose.position.z = destination_position_[2].z();
      destination_rviz_markers_.markers[2].color.a = 1.0;

      destination_rotation_[2] = transform.getRotation();
      destination_rviz_markers_.markers[2].pose.orientation.x = destination_rotation_[2].getX();
      destination_rviz_markers_.markers[2].pose.orientation.y = destination_rotation_[2].getY();
      destination_rviz_markers_.markers[2].pose.orientation.z = destination_rotation_[2].getZ();
      destination_rviz_markers_.markers[2].pose.orientation.w = destination_rotation_[2].getW();

      zone_identified_[2] = true;
    }
    catch (tf::TransformException &ex)
    {
      //ROS_ERROR_THROTTLE( 0.5, "%s",ex.what());;
    }

    vis_pub_.publish( destination_rviz_markers_ );
  }

  void destinationCallback( const std_msgs::StringConstPtr& msg )
  {
    ROS_WARN_STREAM( "Received destination request for " << msg->data.c_str() );
    if ( msg->data.compare( "charging_zone" ) == 0 )
    {
      if ( !zone_identified_[0] )
      {
        ROS_ERROR( "Unable to navigate to charging_zone. Zone not identified." );
        return;
      }

      hideRvizMakers();
      if ( goToDest( destination_position_[0], destination_rotation_[0]) )
      {
        ROS_INFO( "Arrived at charging zone" );
      }
      else
      {
        ROS_ERROR( "Unable to reach charging zone" );
      }
      restoreRvizMarkers();
    }

    if ( msg->data.compare( "pickup_zone" ) == 0 )
    {
      if ( !zone_identified_[1] )
      {
        ROS_ERROR( "Unable to navigate to pickup_zone. Zone not identified." );
        return;
      }

      hideRvizMakers();
      if ( goToDest( destination_position_[1], destination_rotation_[1]) )
      {
        ROS_INFO( "Arrived at pickup zone" );

        //TODO as a demo could add autonomy 
        //Perform auto docking
        //Perform block sorting
        //Perform undocking
        //Request charging zone destination
      }
      else
      {
        ROS_ERROR( "Unable to reach pickup zone" );
      }
      restoreRvizMarkers();
    }

    if ( msg->data.compare( "sorting_zone" ) == 0 )
    {
      if ( !zone_identified_[2] )
      {
        ROS_ERROR( "Unable to navigate to sorting_zone. Zone not identified." );
        return;
      }

      hideRvizMakers();
      if ( goToDest( destination_position_[2], destination_rotation_[2]) )
      {
        ROS_INFO( "Arrived at sorting zone" );
      }
      else
      {
        ROS_ERROR( "Unable to reach sorting zone" );
      }
      restoreRvizMarkers();
    }
  }

private:
  
  void addMarker( visualization_msgs::MarkerArray * p_array, int id, double red, double green, double blue )
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "turtlebot_destinations";
    marker.id = id;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.25;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 0.0; // Invisible until identified
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    p_array->markers.push_back( marker );
  }

  void hideRvizMakers()
  {
    destination_rviz_markers_.markers[0].color.a = 0.0;
    destination_rviz_markers_.markers[1].color.a = 0.0;
    destination_rviz_markers_.markers[2].color.a = 0.0;
    vis_pub_.publish( destination_rviz_markers_ );
  }

  void restoreRvizMarkers()
  {
    if ( zone_identified_[0] ) destination_rviz_markers_.markers[0].color.a = 1.0;
    if ( zone_identified_[1] ) destination_rviz_markers_.markers[1].color.a = 1.0;
    if ( zone_identified_[2] ) destination_rviz_markers_.markers[2].color.a = 1.0;
    vis_pub_.publish( destination_rviz_markers_ );
  }
};

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "zone_destination_node");

  zone_destination_node::DestinationProcessor destination_processor;
  
  ros::Rate rate(5.0);
  while (ros::ok())
  {
    ros::spinOnce();
    destination_processor.updateDestinations();
    rate.sleep();
  }
  return 0;
};
