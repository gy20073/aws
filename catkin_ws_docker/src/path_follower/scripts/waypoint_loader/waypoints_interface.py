#!/usr/bin/env python

import rospy
from path_follower.msg import state_Dynamic, Trajectory2D, TrajectoryPoint2D, Waypoints, Point2D
from tracks import Tra
from std_msgs.msg import String, Header
from scipy.interpolate import UnivariateSpline
import numpy as np
from math import floor, cos, sin 

# define the initial states and timestep
X = 0
Y = 0
psi = 0
X_ref = 0
Y_ref = 0
psi_ref = 0
stateEstimate_mark = False
Waypoints_mark = False
Waypoints_received = Waypoints()

def stateEstimateCallback(data):
    global X, Y, psi, stateEstimate_mark
    X = data.X
    Y = data.Y
    psi = data.psi
    stateEstimate_mark = True

def WaypointsCallback(data):
    global X_ref, Y_ref, psi_ref, Waypoints_received, Waypoints_mark
    Waypoints_received = data
    if stateEstimate_mark:
        Waypoints_mark = True
        X_ref = X
        Y_ref = Y
        psi_ref = psi

def main(dt):
    global X_ref, Y_ref, psi_ref, X, Y, psi, Waypoints_received, stateEstimate_mark
    rospy.init_node('waypoints_interface', anonymous=True)
    rospy.Subscriber('waypoints', Waypoints, WaypointsCallback)
    rospy.Subscriber('state_estimate', state_Dynamic, stateEstimateCallback)
    pub = rospy.Publisher('final_trajectory', Trajectory2D, queue_size=1)
    rate = rospy.Rate(1/dt)

    while (rospy.is_shutdown() != 1):
        if stateEstimate_mark and Waypoints_mark:
            num_steps_received = len(Waypoints_received.points)
            dt_received = Waypoints_received.dt
            horizon = dt_received * num_steps_received
            points = np.zeros((2, num_steps_received+1))
            #points[0, 0] = X_ref
            #points[1, 0] = Y_ref
            for i in range(num_steps_received):
                points[0, i+1] = Waypoints_received.points[i].x
                points[1, i+1] = Waypoints_received.points[i].y
            t_received = np.linspace(0, horizon, num_steps_received+1)
            num_points = int(floor(horizon / dt + 1))
            t = np.linspace(0, (num_points - 1) * dt, num_points)
            spl_x = UnivariateSpline(t_received, points[0, :], k=3)
            spl_y = UnivariateSpline(t_received, points[1, :], k=3)
            spl_x_dot = spl_x.derivative()
            spl_y_dot = spl_y.derivative()
            spl_x_val = spl_x(t)
            spl_y_val = spl_y(t)
            spl_x_dot_val = spl_x_dot(t)
            spl_y_dot_val = spl_y_dot(t)
            spl_v_val = np.sqrt(spl_x_dot_val**2 + spl_y_dot_val**2)
            spl_theta_val = np.arctan2(spl_y_dot_val, spl_x_dot_val)
            spl_yr_fn = UnivariateSpline(t, spl_theta_val, k=3).derivative()
            spl_yr_val = spl_yr_fn(t)

            traj = Trajectory2D()
            for i in range(num_points):
                pt = TrajectoryPoint2D()
                pt.t = t[i]
                pt.x = spl_x_val[i] * cos(psi_ref) - spl_y_val[i] * sin(psi_ref) + X_ref
                pt.y = spl_x_val[i] * sin(psi_ref) + spl_y_val[i] * cos(psi_ref) + Y_ref
                pt.theta = spl_theta_val[i] + psi_ref
                pt.v = spl_v_val[i]
                pt.kappa = spl_yr_val[i]
                traj.point.append(pt)
            traj.header = Header()
            traj.header.stamp = rospy.get_rostime()
            pub.publish(traj)
        rate.sleep()

if __name__ == '__main__':
    main(0.02)
