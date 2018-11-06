#!/usr/bin/env python

import rospy
from path_follower.msg import state_Dynamic, Trajectory2D, TrajectoryPoint2D, Waypoints, Waypoints_state, Point2D
from tracks import Tra
from std_msgs.msg import String, Header
from scipy.interpolate import UnivariateSpline
import numpy as np
from math import floor, cos, sin 

# define the initial states and timestep
ra_to_cg = 1.65
X = 0
Y = 0
vx = 0
vy = 0
psi = 0
X_ref = 0
Y_ref = 0
vx_ref = 0
vy_ref = 0
psi_ref = 0
xy_prev = np.zeros((2, 3))
t_prev = np.zeros(3)
t_now = 0
stateEstimate_mark = False
Waypoints_mark = 0
Waypoints_received = Waypoints()
state_received = state_Dynamic()

def stateEstimateCallback(data):
    global X, Y, psi, vx, vy, stateEstimate_mark, state_received
    X = data.X
    Y = data.Y
    vx = data.vx
    vy = data.vy
    psi = data.psi
    stateEstimate_mark = True
    state_received = data

def WaypointsCallback(data):
    global t_prev, t_now, xy_prev, X_ref, Y_ref, psi_ref, vx_ref, vy_ref, X, Y, vx, vy, psi, Waypoints_received, Waypoints_mark
    Waypoints_received = data
    if stateEstimate_mark:
        Waypoints_mark += 1
        X_ref = X + cos(psi) * ra_to_cg
        Y_ref = Y + sin(psi) * ra_to_cg
        vx_ref = vx
        vy_ref = vy
        psi_ref = psi
        xy_prev[:, :-1] = xy_prev[:, 1:]
        xy_prev[0, -1] = X_ref 
        xy_prev[1, -1] = Y_ref
        seconds = rospy.get_time()
        t_prev[:-1] = t_prev[1:]
        t_prev[-1] = seconds
        t_now = seconds


def main(dt):
    global t_prev, t_now, xy_prev, X_ref, Y_ref, psi_ref, vx_ref, vy_ref, Waypoints_received, stateEstimate_mark, state_received
    rospy.init_node('waypoints_interface', anonymous=True)
    rospy.Subscriber('waypoints', Waypoints, WaypointsCallback)
    rospy.Subscriber('state_estimate', state_Dynamic, stateEstimateCallback)
    pub = rospy.Publisher('final_trajectory', Trajectory2D, queue_size=1)
    pub2 = rospy.Publisher('waypoints_state_received', Waypoints_state, queue_size=1)
    rate = rospy.Rate(1/dt)

    while (rospy.is_shutdown() != 1):
        if stateEstimate_mark and Waypoints_mark >= 3:
            num_steps_received = len(Waypoints_received.points) - 1
            dt_received = Waypoints_received.dt
            horizon = dt_received * num_steps_received
            points = np.zeros((2, num_steps_received+3))
            points[:, :3] = xy_prev
            for i in range(num_steps_received):
                points[0, i+3] = Waypoints_received.points[i].x * cos(psi_ref) - Waypoints_received.points[i].y * sin(psi_ref) + X_ref
                points[1, i+3] = Waypoints_received.points[i].x * sin(psi_ref) + Waypoints_received.points[i].y * cos(psi_ref) + Y_ref
            t_received = np.linspace(dt_received, horizon, num_steps_received)
            t_received = np.append(t_prev-t_now, t_received)
            num_points = int(floor(horizon / dt + 1)) + 5
            t = np.linspace(0, (num_points - 1) * dt, num_points)
            w = np.ones(num_steps_received+3) * 0.1
            w[0:2] *= 20
            w[-1] *= 1.2
            w[int(num_steps_received/2)+2] *= 1.2
            spl_x = UnivariateSpline(t_received, points[0, :], k=3, w=w)
            spl_y = UnivariateSpline(t_received, points[1, :], k=3, w=w)
            spl_x_dot = spl_x.derivative()
            spl_y_dot = spl_y.derivative()
            spl_x_val = spl_x(t)
            spl_y_val = spl_y(t)
            spl_x_dot_val = spl_x_dot(t)
            spl_y_dot_val = spl_y_dot(t)
            spl_v_val = np.sqrt(spl_x_dot_val**2 + spl_y_dot_val**2)
            spl_theta_val = np.arctan2(spl_y_dot_val, spl_x_dot_val)
            spl_theta_val[0] = psi_ref
            w_theta = np.ones(spl_theta_val.shape[0]) 
            w_theta[0] = w_theta[0] * 10
            spl_theta_fn = UnivariateSpline(t, spl_theta_val, k=3, w=w_theta)
            spl_theta_val = spl_theta_fn(t)
            spl_yr_fn = spl_theta_fn.derivative()
            spl_yr_val = spl_yr_fn(t)

            traj = Trajectory2D()
            for i in range(num_points):
                pt = TrajectoryPoint2D()
                pt.t = t[i]
                pt.x = spl_x_val[i] 
                pt.y = spl_y_val[i]
                pt.theta = spl_theta_val[i]
                pt.v = spl_v_val[i]
                pt.kappa = spl_yr_val[i]
                traj.point.append(pt)
            traj.header = Header()
            traj.header.stamp = rospy.get_rostime()
            pub.publish(traj)
            waypoints_state_received = Waypoints_state()
            waypoints_state_received.waypoints = Waypoints_received
            waypoints_state_received.state = state_received
            waypoints_state_received.traj = traj
            pub2.publish(waypoints_state_received)
        rate.sleep()

if __name__ == '__main__':
    main(0.05)
