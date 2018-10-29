/********************************************************
  Autonomous Driving Software
  Copyright (c) 2017 MSC Lab, UC Berkeley 
  All rights reserved.

 ********************************************************/

#include "path_follower/nmea2pose_core.h"

namespace nmea2pose_ws
{
// Constructor 
Nmea2PoseNode::Nmea2PoseNode()
  : private_nh_("~")
  , MAP_FRAME_("map")
  , GPS_FRAME_("gps")
  , roll_(0)
  , pitch_(0)
  , yaw_(0)
  , orientation_time_(0)
  , position_time_(0)
  , current_time_(0)
  , orientation_stamp_(0)
{
  initForROS();
}

// Destructor
Nmea2PoseNode::~Nmea2PoseNode()
{
}

void Nmea2PoseNode::initForROS()
{
  // setup subscriber
  this->sub1_ = this->nh_.subscribe("nmea_sentence", 100, &Nmea2PoseNode::callbackFromNmeaSentence, this);
  // setup publisher
  this->pub1_ = this->nh_.advertise<geometry_msgs::PoseStamped>("gnss_pose", 10);
}

void Nmea2PoseNode::run()
{
  ros::spin();
}

void Nmea2PoseNode::publishPoseStamped()
{
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = this->MAP_FRAME_;
  pose.header.stamp = this->current_time_;
  pose.pose.position.x = this->geo_.x;
//ROS_INFO("%s",pose.pose.position);
  pose.pose.position.y = this->geo_.y;
//ROS_INFO("%s",pose.pose.position.y);
  pose.pose.position.z = this->geo_.z;
  pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(this->roll_, this->pitch_, this->yaw_);
  pub1_.publish(pose);
}

void Nmea2PoseNode::publish()
{
 // tf::Transform transform;
 // transform.setOrigin(tf::Vector3(geo_.y, geo_.x, geo_.z));
  tf::Quaternion quaternion;
  quaternion.setRPY(this->roll_, this->pitch_, this->yaw_);
 // transform.setRotation(quaternion);
 // br_.sendTransform(tf::StampedTransform(transform, current_time_, MAP_FRAME_, GPS_FRAME_));
}

void Nmea2PoseNode::createOrientation()
{
  this->yaw_ = atan2(this->geo_.x - this->last_geo_.x, this->geo_.y - this->last_geo_.y);
  this->roll_ = 0;
  this->pitch_ = 0;
}


void Nmea2PoseNode::convert(std::vector<std::string> nmea)
{
  try
  {
    if (nmea.at(0).compare(1, 3, "BES") == 0)
    {
      ROS_INFO("BESPOS is subscribed.");
    }
    else if (nmea.at(5) == "SOL_COMPUTED")
    {
      double lat = std::strtod(nmea.at(7).c_str(),NULL);
      double lon = std::strtod(nmea.at(8).c_str(),NULL);
      UTM coordinates = LatLon2Utm(lat,lon);
      this->geo_.x = coordinates.x;
      this->geo_.y = coordinates.y;
      this->geo_.z = 0;
      ROS_INFO("lat & long is subscribed.");
    }
    else if (nmea.at(0).compare(0,6,"[USB1]")==0)
    {
      ROS_INFO("GGA is subscribed.");
    }
  }catch (const std::exception &e)
  {
    ROS_WARN_STREAM("Message is invalid : " << e.what());
  }
}

std::vector<std::string> split(const std::string &string)
{
  std::vector<std::string> str_vec_ptr;
  std::string token;
  std::stringstream ss(string);

  while (getline(ss, token, ' '))
    str_vec_ptr.push_back(token);

  return str_vec_ptr;
}


void Nmea2PoseNode::callbackFromNmeaSentence(const nmea_msgs::Sentence::ConstPtr &msg)
{
  this->current_time_ = msg->header.stamp;
  convert(split(msg->sentence));
  publishPoseStamped();
  return;
}

}
