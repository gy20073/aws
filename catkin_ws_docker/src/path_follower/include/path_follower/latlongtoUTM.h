/********************************************************
  Autonomous Driving Software
  Copyright (c) 2017 MSC Lab, UC Berkeley 
  All rights reserved.

 ********************************************************/

#ifndef LATLONGTOUTM_H
#define LATLONGTOUTM_H

#include <cmath>
#include <ros/ros.h>
using namespace std;

struct UTM
{
	bool hemi; // 1: southern hemisphere; 0: northern hemisphere
	int zone;
	double x;
	double y;
};

UTM LatLon2Utm(double Lat, double Lon);

#endif  // LATLONGTOUTM_H
