#!/usr/bin/env python

import rospy
from path_follower.msg import state_Dynamic, Waypoints, Point2D
from tracks import Tra
from std_msgs.msg import String, Header
from scipy.interpolate import UnivariateSpline
import numpy as np
from math import floor, cos, sin

# define the initial states and timestep
X = 0
Y = 0
stateEstimate_mark = False
def stateEstimateCallback(data):
    global vx, vy, X, Y, psi, stateEstimate_mark
    X = data.X
    Y = data.Y
    psi = data.psi
    stateEstimate_mark = True

def main(dt, dt_ros, horizon):
    global X, Y, psi, stateEstimate_mark

    # import track file
    track = Tra('Tra_3', horizon)
    rospy.init_node('toy_planner_waypoints', anonymous=True)
    rospy.Subscriber('state_estimate', state_Dynamic, stateEstimateCallback)
    pub = rospy.Publisher('waypoints', Waypoints, queue_size=1)

    rate = rospy.Rate(1/dt_ros)

    # first set the horizon to be very large to get where the vehicle is
    track.horizon = horizon
    track.currentIndex = 0
    num_points = 400

    while (rospy.is_shutdown() != 1):
        if stateEstimate_mark == True:
            track.currentIndex, _ = track.searchClosestPt(X, Y, track.currentIndex)
            if track.currentIndex + num_points < track.size:
                index_list = np.arange(track.currentIndex+int(dt/0.01), track.currentIndex+num_points, 1)
                num_steps = int(floor((track.t[track.currentIndex+num_points] - track.t[track.currentIndex+1]) / dt) + 1)
                t = np.linspace(track.t[track.currentIndex], track.t[track.currentIndex] + num_steps * dt, num_steps)
                t_sub = track.t[index_list]
                x_sub = track.x[index_list]
                y_sub = track.y[index_list]
                spl_x = UnivariateSpline(t_sub, x_sub, k=3)
                spl_y = UnivariateSpline(t_sub, y_sub, k=3)
                spl_x_val = spl_x(t)
                spl_y_val = spl_y(t)

                waypoints = Waypoints()
                for i in range(1, num_steps):
                    pt = Point2D()
                    pt.x = (spl_x_val[i] - X) * cos(psi) + (spl_y_val[i] - Y) * sin(psi)
                    pt.y = - (spl_x_val[i] - X) * sin(psi) + (spl_y_val[i] - Y) * cos(psi)
                    waypoints.points.append(pt)
                waypoints.dt = dt
                pub.publish(waypoints)
        rate.sleep()

if __name__ == '__main__':
    main(0.2, 0.2, 500)
