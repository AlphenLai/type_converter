#include <ros/ros.h>
#include <std_msgs/String.h>
//#include "type_converter.h"
#include <sstream>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

static double current_x = 0.0;
static double current_y = 0.0;
static double current_z = 0.0;
static geometry_msgs::Quaternion current_q;

void local_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);

int main(int argc, char **argv)
{
  ros::init(argc, argv,"type_converter");
  ros::NodeHandle n;

  //read marker tf and convert it to pose
  ros::Publisher marker_pose = n.advertise<geometry_msgs::PoseStamped>("/mavros/vision_position/pose",1);
  tf::TransformListener listener;

  //read local_position pose and convert it to tf
  static tf::TransformBroadcaster br;
  ros::Subscriber local_pos_pose = n.subscribe("/mavros/local_position/pose",1,local_pose_callback);

  ros::Time current_time = ros::Time::now();
  while (n.ok())
  {
    tf::StampedTransform transform;
    try
    {
      listener.waitForTransform("camera_position", "world", ros::Time(0), ros::Duration(0.5) );
      listener.lookupTransform("camera_position", "world", ros::Time(0), transform);
    }
    catch(tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    //convert poseStamped from stampedTF
    geometry_msgs::PoseStamped vision_pos_pose;
    //convert orientaiton
    geometry_msgs::Quaternion quat;
    tf::Quaternion tfQuat = transform.getRotation();
    quat.x = tfQuat.x();
    quat.y = tfQuat.y();
    quat.z = tfQuat.z();
    quat.w = tfQuat.w();
    vision_pos_pose.pose.orientation = quat;

    //convert position
    tf::Vector3 tfVec = transform.getOrigin();
    geometry_msgs::Point pt;
    pt.x = tfVec.getX();
    pt.y = tfVec.getY();
    pt.z = tfVec.getZ();
    vision_pos_pose.pose.position = pt;

    //convert header
    vision_pos_pose.header.frame_id = transform.frame_id_;
    vision_pos_pose.header.stamp = transform.stamp_;

    marker_pose.publish(vision_pos_pose);
  }

  geometry_msgs::TransformStamped local_pose_tf;
  local_pose_tf.transform.translation.x = current_x;
  local_pose_tf.transform.translation.y = current_y;
  local_pose_tf.transform.translation.z = current_z;
  local_pose_tf.transform.rotation = current_q;

  //header
  local_pose_tf.header.stamp = current_time;
  local_pose_tf.header.frame_id = "world";
  local_pose_tf.child_frame_id = "robot_link";

  br.sendTransform(local_pose_tf);

  ros::spin();
  return 0;
}

void local_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
  current_x = pose_msg->pose.position.x;
  current_y = pose_msg->pose.position.y;
  current_z = pose_msg->pose.position.z;
  current_q = pose_msg->pose.orientation;
}
