#include <ros/ros.h>
#include <std_msgs/String.h>
//#include "type_converter.h"
#include <sstream>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

void local_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);

int main(int argc, char **argv)
{
  
  ros::init(argc, argv,"type_converter");
  ros::NodeHandle n;

  //read marker tf and convert it to pose
  ros::Publisher marker_pose = n.advertise<geometry_msgs::PoseStamped>("/mavros/vision_position/pose",1);
  tf::TransformListener listener;

  //read local_position pose and convert it to tf
  ros::Subscriber local_pos_pose = n.subscribe("/mavros/local_position/pose",1,local_pose_callback);

  while (n.ok())
  {
    tf::StampedTransform transform;
    try
    {
      listener.waitForTransform("camera_position", "world", ros::Time(0), ros::Duration(10.0) );
      listener.lookupTransform("camera_position", "world", ros::Time(0), transform);
    }
    catch(tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    
    geometry_msgs::PoseStamped vision_pos_pose;
    vision_pos_pose.pose.orientation.x = transform.getRotation().getX();
    vision_pos_pose.pose.orientation.y = transform.getRotation().getY();
    vision_pos_pose.pose.orientation.z = transform.getRotation().getZ();
    vision_pos_pose.pose.orientation.w = transform.getRotation().getW();

    vision_pos_pose.pose.position.x = transform.getOrigin().getX();
    vision_pos_pose.pose.position.y = transform.getOrigin().getY();
    vision_pos_pose.pose.position.z = transform.getOrigin().getZ();

    marker_pose.publish(vision_pos_pose);
  }

  ros::spin();
  return 0;
}

void local_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
  static tf::TransformBroadcaster br;
  tf::Transform local_pose_tf;
  local_pose_tf.setOrigin( tf::Vector3(pose_msg->pose.position.x, pose_msg->pose.position.y, pose_msg->pose.position.z) );
  local_pose_tf.setRotation( tf::Quaternion(pose_msg->pose.orientation.x, pose_msg->pose.orientation.y, pose_msg->pose.orientation.z, pose_msg->pose.orientation.w) );
  br.sendTransform(tf::StampedTransform(local_pose_tf, ros::Time::now(),"world", "local_position"));
}
