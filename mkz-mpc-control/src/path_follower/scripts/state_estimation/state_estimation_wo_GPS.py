#!/usr/bin/env python

import rospy
from numpy import zeros, array, append, eye, diag
from math import sqrt
from tf import transformations
from ekf import ekf
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose2D, Quaternion
from path_follower.msg import state_Dynamic
from dbw_mkz_msgs.msg import SteeringReport
from system_models import f_BicycleModel, h_BicycleModel_withGPS, h_BicycleModel_withoutGPS

#setting parameters
a = 1.20;                          #m
b = 1.65;                          #m
ftire_stiffness = 140000.0;        #N/rad
rtire_stiffness = 120000.0;        #N/rad
mass = 1800.0;                     #kg
iz = 3270.0;                       #kg*m^2

vhMdl = (a,b,mass,iz)
trMdl = (ftire_stiffness,rtire_stiffness)

# measurement
Vx_meas       = 0
Yaw_meas      = 0
IMU_time      = 0
Yaw_meas_prev = 0
IMU_time_prev = 0 
Yawrate_meas  = 0
ax_meas       = 0
ay_meas       = 0
delta_meas    = 0

quaternion          = (0,0,0,0)
IMU_start           = 0

def SteeringReportCallback(data):
    global Vx_meas
    global delta_meas
    Vx_meas = data.speed
    delta_meas = data.steering_wheel_angle / 14.8

def IMUCallback(data):
    global Yaw_meas, Yaw_meas_prev, Yawrate_meas
    global IMU_time, IMU_time_prev
    global Yaw_initialize, IMU_start
    global ax_meas, ay_meas
    global quaternion

    ori        = data.orientation
    quaternion = (ori.x, ori.y, ori.z, ori.w)
    (roll, pitch, Yaw_meas) = transformations.euler_from_quaternion(quaternion)

    IMU_time = float(str(data.header.stamp))

    if IMU_start == 0:
        IMU_start    = 1
    else:
        Yawrate_meas = (Yaw_meas-Yaw_meas_prev)/(IMU_time-IMU_time_prev)

    Yaw_meas_prev    = Yaw_meas
    IMU_time_prev    = IMU_time

    ax_meas          = data.linear_acceleration.x
    ay_meas          = data.linear_acceleration.y

def state_estimation():
    global Vx_meas, Yaw_meas, delta_meas
    global start_X, start_Y, pub_flag

    # initialize node
    rospy.init_node('state_estimation', anonymous=True)

    # topic subscriptions / publications
    rospy.Subscriber('/xsens/imu_data', Imu, IMUCallback)
    rospy.Subscriber('vehicle/steering_report', SteeringReport, SteeringReportCallback)
    state_pub = rospy.Publisher('state_estimate', state_Dynamic, queue_size = 10)

    # set node rate
    loop_rate = 100
    dt        = 1.0 / loop_rate
    rate      = rospy.Rate(loop_rate)

    # estimation variables for Luemberger observer
    z_EKF            = array([0.,0.,0.,0.,0.,0.])
    z_EKF_prev       = array([0.,0.,0.,0.,0.,0.])
    states_est       = array([0.,0.,0.,0.,0.,0.])
    state_initialize = 0
    state_est_obj    = state_Dynamic()

    # estimation variables for EKF
    var_v     = 1.0e-04
    var_psi   = 1.0e-06

    var_ax    = 1.0e-04
    var_delta = 1.0e-04
    var_noise = 1.0e-04

    P         = eye(6)    # initial dynamics coveriance matrix
    Q         = diag(array([var_ax, var_delta, var_noise, var_noise, var_noise, var_noise]))     # process noise coveriance matrix
    R1         = diag(array([var_v, var_psi]))

    while (rospy.is_shutdown() != 1):

        if state_initialize == 0:
            z_EKF            = array([Vx_meas, 0, 0, 0, Yaw_meas, Yawrate_meas])
            state_est        = z_EKF
            state_initialize = 1

        else:
            u_ekf      = array([ax_meas, delta_meas])
            w_ekf      = array([0., 0., 0., 0., 0., 0.])
            args       = (u_ekf, vhMdl, trMdl, dt)
            z_EKF_prev = z_EKF

            y_ekf      = array([Vx_meas, Yaw_meas])
            v_ekf      = array([0.,0.])
            (z_EKF, P) = ekf(f_BicycleModel, z_EKF, w_ekf, v_ekf, P, h_BicycleModel_withoutGPS, y_ekf, Q, R1, args)
            state_est  = z_EKF

        state_est_obj.vx = state_est[0]
        state_est_obj.vy = state_est[1]
        state_est_obj.X  = state_est[2]
        state_est_obj.Y  = state_est[3]
        state_est_obj.psi= state_est[4]
        state_est_obj.wz = state_est[5]
        state_pub.publish(state_est_obj)    
        rate.sleep()

if __name__ == '__main__':
    try:
        state_estimation()
    except rospy.ROSInterruptException:
        pass
