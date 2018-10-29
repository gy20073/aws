/********************************************************
  Autonomous Driving Software
  Copyright (c) 2017 MSC Lab, UC Berkeley 
  All rights reserved.

 ********************************************************/

#ifndef NMEA2TFPOSE_CORE_H
#define NMEA2TFPOSE_CORE_H

// C++ includes
#include <string>
#include <memory>
#include <stdlib.h>

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nmea_msgs/Sentence.h>
#include <tf/transform_broadcaster.h>

#include "latlongtoUTM.h"

namespace nmea2pose_ws
{
class Nmea2PoseNode
{
public:
struct geo_pos_conv{
double x;
double y;
double z;
};
  Nmea2PoseNode();
  ~Nmea2PoseNode();

void run();

private:

  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // publisher
  ros::Publisher pub1_;

  // subscriber
  ros::Subscriber sub1_;

  // constants
  const std::string MAP_FRAME_;
  const std::string GPS_FRAME_;

  // variables

  geo_pos_conv geo_;
  geo_pos_conv last_geo_;
  double roll_, pitch_, yaw_;
  double orientation_time_, position_time_;
  ros::Time current_time_, orientation_stamp_;
  // callbacks
  void callbackFromNmeaSentence(const nmea_msgs::Sentence::ConstPtr &msg);

  // initializer
  void initForROS();

  // functions
  void publishPoseStamped();
  void publish();
  void createOrientation();
  void convert(std::vector<std::string> nmea);
};
  std::vector<std::string> split(const std::string &string);

}  // gnss_localizer
#endif
