/********************************************************
  Autonomous Driving Software
  Copyright (c) 2017 MSC Lab, UC Berkeley 
  All rights reserved.

 ********************************************************/

// ROS includes
#include <ros/ros.h>

#include "path_follower/nmea2pose_core.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nmea2pose");
  nmea2pose_ws::Nmea2PoseNode ntpn;
  ntpn.run();

  return 0;
}
