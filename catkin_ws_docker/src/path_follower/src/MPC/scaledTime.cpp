/********************************************************
  Autonomous Driving Software
  Copyright (c) 2017 MSC Lab, UC Berkeley 
  All rights reserved.

 ********************************************************/

#include <errno.h>
#include <string.h>
#include "path_follower/vlrException.h"
#include "path_follower/scaledTime.h"

namespace vlr {

    double Time::time_scale_ = 1.0;
    double Time::t_=0;
    struct timeval Time::tv_;

    double Time::current() {

        if (gettimeofday(&tv_, NULL) < 0) {
            throw VLRException("Error in gettimeofday : " + std::string(strerror(errno)));
        }

        t_ = tv_.tv_sec + tv_.tv_usec / 1000000.0;
        return t_ * time_scale_;
    }

} 
