#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>

namespace zone_detection_node
{

class ZoneDetector
{
private:
  ros::NodeHandle node_;

  tf::TransformBroadcaster tf_broadcaster_;
  tf::StampedTransform tf_to_publish_;

  tf::TransformListener listener_;

  tf::Quaternion destination_rotation_[3];
  tf::Vector3 destination_position_[3];

  visualization_msgs::MarkerArray zone_rviz_markers_;

  ros::Publisher vis_pub_;

  tf::Transform zone_transforms_[3];

  double zone_offset_x_;

public:
  ZoneDetector()
  {
    addMarker( &zone_rviz_markers_, 0, 1.0, 0.0, 0.0 );
    addMarker( &zone_rviz_markers_, 1, 0.0, 1.0, 0.0 );
    addMarker( &zone_rviz_markers_, 2, 0.0, 0.0, 1.0 );

    destination_position_[0].setZ( 0.0 );
    destination_position_[1].setZ( 0.0 );
    destination_position_[2].setZ( 0.0 );

    vis_pub_ = node_.advertise<visualization_msgs::MarkerArray>( "zone_visualization_marker", 0 );

    //Load parameters
    node_.param<double>("zone_offset_x", zone_offset_x_, 0.5);
  }

  void lookupZones()
  {
    tf::StampedTransform transform;
    tf::StampedTransform displace_transform;

    tf::Vector3 offset_vector;
    offset_vector.setX( zone_offset_x_ ); //Offset destination in front of detected object

    displace_transform.setOrigin( offset_vector );
    
    try
    {
      listener_.lookupTransform("/map", "/object_1", ros::Time(0), transform);
      double angle = transform.getRotation().getAngle(); 
      transform *= displace_transform; //Displace charging destination and marker

      destination_position_[0] = transform.getOrigin();
      zone_rviz_markers_.markers[0].pose.position.x = destination_position_[0].x();
      zone_rviz_markers_.markers[0].pose.position.y = destination_position_[0].y();
      zone_rviz_markers_.markers[0].pose.position.z = destination_position_[0].z();
      zone_rviz_markers_.markers[0].color.a = 1.0;

      zone_transforms_[0].setOrigin( tf::Vector3(destination_position_[0].x(), destination_position_[0].y(), 0.0) );
      zone_transforms_[0].setRotation(tf::createQuaternionFromYaw(angle) * tf::createQuaternionFromYaw(3.14159));
      
      destination_rotation_[0] = zone_transforms_[0].getRotation();
      zone_rviz_markers_.markers[0].pose.orientation.x = destination_rotation_[0].getX();
      zone_rviz_markers_.markers[0].pose.orientation.y = destination_rotation_[0].getY();
      zone_rviz_markers_.markers[0].pose.orientation.z = destination_rotation_[0].getZ();
      zone_rviz_markers_.markers[0].pose.orientation.w = destination_rotation_[0].getW();

      ROS_INFO_STREAM_THROTTLE( 5, "Charging Area Visible" );
    }
    catch (tf::TransformException &ex)
    {
      //ROS_ERROR_THROTTLE( 0.5, "%s",ex.what());
    }
    try
    {
      listener_.lookupTransform("/map", "/object_2", ros::Time(0), transform);
      double angle = transform.getRotation().getAngle();
      transform *= displace_transform;

      destination_position_[1] = transform.getOrigin();
      zone_rviz_markers_.markers[1].pose.position.x = destination_position_[1].x();
      zone_rviz_markers_.markers[1].pose.position.y = destination_position_[1].y();
      zone_rviz_markers_.markers[1].pose.position.z = destination_position_[1].z();
      zone_rviz_markers_.markers[1].color.a = 1.0;

      zone_transforms_[1].setOrigin( tf::Vector3(destination_position_[1].x(), destination_position_[1].y(), 0.0) );
      zone_transforms_[1].setRotation(tf::createQuaternionFromYaw(angle) * tf::createQuaternionFromYaw(3.14159));

      destination_rotation_[1] = zone_transforms_[1].getRotation();
      zone_rviz_markers_.markers[1].pose.orientation.x = destination_rotation_[1].getX();
      zone_rviz_markers_.markers[1].pose.orientation.y = destination_rotation_[1].getY();
      zone_rviz_markers_.markers[1].pose.orientation.z = destination_rotation_[1].getZ();
      zone_rviz_markers_.markers[1].pose.orientation.w = destination_rotation_[1].getW();

      ROS_INFO_STREAM_THROTTLE( 5, "Pickup Area Visible" );
    }
    catch (tf::TransformException &ex)
    {
      //ROS_ERROR("%s",ex.what());
    }
    try
    {
      listener_.lookupTransform("/map", "/object_3", ros::Time(0), transform);
      double angle = transform.getRotation().getAngle();
      transform *= displace_transform;

      destination_position_[2] = transform.getOrigin();
      zone_rviz_markers_.markers[2].pose.position.x = destination_position_[2].x();
      zone_rviz_markers_.markers[2].pose.position.y = destination_position_[2].y();
      zone_rviz_markers_.markers[2].pose.position.z = destination_position_[2].z();
      zone_rviz_markers_.markers[2].color.a = 1.0;

      zone_transforms_[2].setOrigin( tf::Vector3(destination_position_[2].x(), destination_position_[2].y(), 0.0) );
      zone_transforms_[2].setRotation(tf::createQuaternionFromYaw(angle) * tf::createQuaternionFromYaw(3.14159));

      destination_rotation_[2] = zone_transforms_[2].getRotation();
      zone_rviz_markers_.markers[2].pose.orientation.x = destination_rotation_[2].getX();
      zone_rviz_markers_.markers[2].pose.orientation.y = destination_rotation_[2].getY();
      zone_rviz_markers_.markers[2].pose.orientation.z = destination_rotation_[2].getZ();
      zone_rviz_markers_.markers[2].pose.orientation.w = destination_rotation_[2].getW();

      ROS_INFO_STREAM_THROTTLE( 5, "Sorting Area Visible" );
    }
    catch (tf::TransformException &ex)
    {
      //ROS_ERROR("%s",ex.what());
    }

    if ( destination_position_[0].z() == 0.0 )
    {
      ROS_WARN_STREAM_THROTTLE( 5, "Zone 1: Charging area not yet identified." );
    }
    else
    {
      tf_broadcaster_.sendTransform(tf::StampedTransform(zone_transforms_[0], ros::Time::now(), "map", "charging_zone"));
      ROS_INFO_STREAM_THROTTLE( 5, "Charging Area X " << destination_position_[0].x() << " Y " << destination_position_[0].y() << " Z " << destination_position_[0].z() );
    } 

    if ( destination_position_[1].z() == 0.0 )
    {
      ROS_WARN_STREAM_THROTTLE( 5, "Zone 2: Pickup area not identified." );
    }
    else 
    {
      tf_broadcaster_.sendTransform(tf::StampedTransform(zone_transforms_[1], ros::Time::now(), "map", "pickup_zone"));
      ROS_INFO_STREAM_THROTTLE( 5, "Pickup Area X " << destination_position_[1].x() << " Y " << destination_position_[1].y() << " Z " << destination_position_[1].z() );
    }
    if ( destination_position_[2].z() == 0.0 )
    {
      ROS_WARN_STREAM_THROTTLE( 5, "Zone 3: Sorting area not identified." );
    }
    else
    {
      tf_broadcaster_.sendTransform(tf::StampedTransform(zone_transforms_[2], ros::Time::now(), "map", "sorting_zone"));
      ROS_INFO_STREAM_THROTTLE( 5, "Sorting Area X " << destination_position_[2].x() << " Y " << destination_position_[2].y() << " Z " << destination_position_[2].z() );
    }

    vis_pub_.publish( zone_rviz_markers_ );
  }

private:

  void addMarker( visualization_msgs::MarkerArray * p_array, int id, double red, double green, double blue )
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "turtlebot_zones";
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

};

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "zone_detection_node");

  zone_detection_node::ZoneDetector zone_detector;

  ros::Rate rate(5.0);
  while (ros::ok())
  {
    ros::spinOnce();
    zone_detector.lookupZones();
    rate.sleep();
  }
  return 0;
};
