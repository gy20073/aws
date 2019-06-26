/********************************************************
  Autonomous Driving Software
  Copyright (c) 2017 MSC Lab, UC Berkeley 
  All rights reserved.

 ********************************************************/

#ifndef VLR_VEHICLE_H_
#define VLR_VEHICLE_H_

#include "MKZ_constants.h"
#include "Global.h"
namespace vlr {

    typedef struct {
        int torque_mode;
        double max_steering, max_throttle, max_brake, max_torque;
        double max_wheel_angle, max_wheel_rate;

        double steering_ratio, wheel_base, imu_to_cg_dist;
        double a, b, ftire_stiffness, rtire_stiffness;
        double mass, iz, tau;
        double max_steering_rate;
        double steer_inertia;

        double brake_decel_coef, throttle_accel_coef;
        int bicycle_model;   
    } vehicle_params;


    class vehicle_state {
        public:
            vehicle_state();
            void set_mkz_params(void);


            vehicle_params param;

             /* offset to real world coordinates */
            double origin_x, origin_y;
            char utmzone[5];

            int paused;

            double x, y, yaw;
            double v_x, v_y, yaw_rate;
            double wheel_angle, wheel_angle_rate;
            double commanded_wheel_angle, commanded_forward_accel;
            double actual_forward_accel;
            double lateral_accel;

            double added_error_x, added_error_y;

            double torque, throttle;

            int shifting;
            double shift_timer;
            int commanded_direction, direction;
     };

} 
#endif