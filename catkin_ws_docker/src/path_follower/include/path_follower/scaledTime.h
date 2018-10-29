/********************************************************
  Autonomous Driving Software
  Copyright (c) 2017 MSC Lab, UC Berkeley 
  All rights reserved.

 ********************************************************/


#ifndef DRIVING_COMMON_GLOBAL_SCALED_TIME_H_
#define DRIVING_COMMON_GLOBAL_SCALED_TIME_H_

#include <sys/time.h>

namespace vlr {

class Time {
public:
  static double current();
  static inline double scale() {return time_scale_;}
  static inline void scale(double new_scale) {time_scale_ = new_scale;}

private:
  static double time_scale_;
  static struct timeval tv_;
  static double t_;
};

} 

#endif
