/********************************************************
  Autonomous Driving Software
  Copyright (c) 2017 MSC Lab, UC Berkeley 
  All rights reserved.

 ********************************************************/

// ROS Includes
#include <ros/ros.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>

// User defined includes
#include "dbw_mkz_msgs/SteeringReport.h"

// cpp includes
#include <cmath>

//To do: define global variable 
double PI = 3.1415926;
int initial_read = 0;
int initialize = 0;
geometry_msgs::TwistStamped actual_vel;
geometry_msgs::PoseStamped actual_pose;
geometry_msgs::Point initial_position;
geometry_msgs::Point current_position;
geometry_msgs::Pose2D current_pose_2D;
tf::Quaternion measured_orientation;
tf::Quaternion initial_orientation;
tf::Quaternion actual_orientation;

void UTMCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  current_position = msg->pose.position;
  actual_pose.header.stamp = msg->header.stamp;
  actual_pose.pose.position = current_position;
  if (initial_read == 0)
  {
    initial_position.x = current_position.x;
    initial_position.y = current_position.y;
    initial_position.z = 0;
    initial_read = 1;
  }
}

void SteeringReportCallback(const dbw_mkz_msgs::SteeringReport::ConstPtr& msg)
{
  actual_vel.twist.linear.x = msg->speed;
  actual_vel.header.stamp = msg->header.stamp;
}

void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  measured_orientation = tf::Quaternion(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_estimation");
  ros::NodeHandle n;
  ros::Subscriber sub1 = n.subscribe("/vehicle/steering_report",1,SteeringReportCallback); 
  ros::Subscriber sub2 = n.subscribe("gnss_pose",1,UTMCallback);
  ros::Subscriber sub3 = n.subscribe("/xsens/imu/data",1,ImuCallback);
  ros::Publisher pub1 = n.advertise<geometry_msgs::TwistStamped>("current_velocity",1); 
  ros::Publisher pub2 = n.advertise<geometry_msgs::PoseStamped>("current_pose",1);
  ros::Publisher pub3 = n.advertise<geometry_msgs::Pose2D>("current_pose_2D",1);
  ros::Publisher pub4 = n.advertise<geometry_msgs::Quaternion>("relative_quaternion",1);
  ros::Rate loop_rate(10);
  double initial_yaw = 0; 
  double distance = 0;
  tf::Matrix3x3 initial_rot;
  tf::Matrix3x3 initial_imu_mea;
  tf::Matrix3x3 imu_mea;
  tf::Matrix3x3 imu_relative_rot;
  tf::Matrix3x3 actual_rot;
  geometry_msgs::Quaternion initial_relative_quaternion;

  ROS_INFO_STREAM("pose_estimation node starts");
  ROS_INFO_STREAM("drive the vehicle in straight lane to initialize yaw angle");

  while(ros::ok())
  {
  	ros::spinOnce();
    //publish velocity
    pub1.publish(actual_vel);
    //initialize yaw angle and publish pose
    distance = sqrt(pow(current_position.x-initial_position.x,2)+pow(current_position.y-initial_position.y,2));
    if (initialize == 0 && distance >= 2)
    {
      initial_yaw = atan2(current_position.y-initial_position.y,current_position.x-initial_position.x);
      initial_rot.setEulerYPR(initial_yaw,0,0);
      initial_rot.getRotation(initial_orientation);
      initial_imu_mea.setRotation(measured_orientation);
      actual_pose.pose.orientation.x = initial_orientation.x();
      actual_pose.pose.orientation.y = initial_orientation.y();
      actual_pose.pose.orientation.z = initial_orientation.z();
      actual_pose.pose.orientation.w = initial_orientation.w();
      current_pose_2D.x = current_position.x;
      current_pose_2D.y = current_position.y;
      current_pose_2D.theta = initial_yaw;
      ROS_INFO_STREAM("yaw angle has been initialized");
      pub2.publish(actual_pose);
      pub3.publish(current_pose_2D);
      initialize = 1;

      tf::Matrix3x3 initial_relative_rot;
      tf::Quaternion initial_relative_tf_quaternion;
      initial_relative_rot = initial_imu_mea.inverse()*initial_rot;
      initial_relative_rot.getRotation(initial_relative_tf_quaternion);
      initial_relative_quaternion.x = initial_relative_tf_quaternion.x();
      initial_relative_quaternion.y = initial_relative_tf_quaternion.y();
      initial_relative_quaternion.z = initial_relative_tf_quaternion.z();
      initial_relative_quaternion.w = initial_relative_tf_quaternion.w();
      pub4.publish(initial_relative_quaternion);            
      ROS_INFO_STREAM("publish relative quaternion");

    }
    //compute orientation and publish pose after initialization
    if (initialize == 1)
    {
      imu_mea.setRotation(measured_orientation);
      imu_relative_rot = imu_mea*initial_imu_mea.inverse();
      actual_rot = imu_relative_rot*initial_rot;
      actual_rot.getRotation(actual_orientation);
      actual_pose.pose.orientation.x = actual_orientation.x();
      actual_pose.pose.orientation.y = actual_orientation.y();
      actual_pose.pose.orientation.z = actual_orientation.z();
      actual_pose.pose.orientation.w = actual_orientation.w();
      current_pose_2D.x = current_position.x;
      current_pose_2D.y = current_position.y;
      double theta = 0;
      double pitch = 0;
      double roll = 0;
      actual_rot.getEulerYPR(theta,pitch,roll);
      if (theta<-PI)
      {
      	theta = theta + 2*PI;
      }
      if (theta>PI)
      {
      	theta = theta - 2*PI;
      }
      current_pose_2D.theta = theta;
      pub2.publish(actual_pose);
      pub3.publish(current_pose_2D);
      pub4.publish(initial_relative_quaternion);
    }
  	loop_rate.sleep();
  }

  return 0;
}
