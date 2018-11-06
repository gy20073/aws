#!/usr/bin/env python

import rospy
from path_follower.msg import state_Dynamic, Trajectory2D, TrajectoryPoint2D
from tracks import Tra
from std_msgs.msg import String, Header
from scipy.interpolate import UnivariateSpline
import numpy as np

# define the initial states and timestep
X = 0
Y = 0
stateEstimate_mark = False
def stateEstimateCallback(data):
    global vx, vy, X, Y, psi, wz,stateEstimate_mark
    X = data.X
    Y = data.Y
    stateEstimate_mark = True

def main(dt, horizon):
    global vx, vy, X, Y, psi, wz, d_f,stateEstimate_mark

    # import track file
    track = Tra('Tra_1', horizon)
    rospy.init_node('toy_planner', anonymous=True)
    rospy.Subscriber('state_estimate', state_Dynamic, stateEstimateCallback)
    pub = rospy.Publisher('final_trajectory', Trajectory2D, queue_size=1)

    rate = rospy.Rate(1/dt)

    # first set the horizon to be very large to get where the vehicle is
    track.horizon = horizon
    track.currentIndex = 0

    num_points = 400
    w = np.ones(num_points) * 0.1
    w[0] = w[0] * 30
    w[-1] = w[-1] * 30

    while (rospy.is_shutdown() != 1):
        if stateEstimate_mark == True:
            track.currentIndex, _ = track.searchClosestPt(X, Y, track.currentIndex)
            if track.currentIndex + num_points < track.size:
                index_list = np.arange(track.currentIndex, track.currentIndex+num_points, 1)
                t = track.t[track.currentIndex:track.currentIndex+num_points]
                t_sub = track.t[index_list]
                x_sub = track.x[index_list] #- 0.16
                y_sub = track.y[index_list] #+ 0.44
                spl_x = UnivariateSpline(t_sub, x_sub, k=2)
                spl_y = UnivariateSpline(t_sub, y_sub, k=2)
                spl_x_dot = spl_x.derivative()
                spl_y_dot = spl_y.derivative()
                spl_x_val = spl_x(t)
                spl_y_val = spl_y(t)
                spl_x_dot_val = spl_x_dot(t)
                spl_y_dot_val = spl_y_dot(t)
                spl_v_val = np.sqrt(spl_x_dot_val**2 + spl_y_dot_val**2)
                spl_theta_val = np.arctan2(spl_y_dot_val, spl_x_dot_val)
                spl_yr_fn = UnivariateSpline(t, spl_theta_val, k=2).derivative()
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
        rate.sleep()

if __name__ == '__main__':
    main(0.02, 50)
