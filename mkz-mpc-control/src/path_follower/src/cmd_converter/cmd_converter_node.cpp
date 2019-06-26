/********************************************************
  Autonomous Driving Software
  Copyright (c) 2017 MSC Lab, UC Berkeley 
  All rights reserved.

 ********************************************************/

// ROS Includes
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <dbw_mkz_msgs/SteeringCmd.h>

geometry_msgs::TwistStamped ts;
dbw_mkz_msgs::SteeringCmd steering_cmd_;

void CmdCallback(const geometry_msgs::TwistStampedConstPtr &msg)
{
  ts = *msg;
}

void SteeringCallback(const dbw_mkz_msgs::SteeringCmd msg)
{
  steering_cmd_ = msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cmd_converter");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("twist_cmd",1,CmdCallback); 
  ros::Subscriber sub_steering_cmd = n.subscribe("path_follower/steering_cmd",1,SteeringCallback);

  ros::Publisher pub = n.advertise<geometry_msgs::TwistStamped>("/vehicle/cmd_vel_stamped",1); 
  ros::Publisher steering_pub = n.advertise<dbw_mkz_msgs::SteeringCmd>("vehicle/steering_cmd",1);
  ros::Rate loop_rate(50);

  ROS_INFO_STREAM("cmd_converter node starts");

  while(ros::ok())
  {
  	ros::spinOnce();
    ts.header.stamp = ros::Time::now();
  	pub.publish(ts);
    steering_pub.publish(steering_cmd_);
  	loop_rate.sleep();
  }

  return 0;
}
