#!/usr/bin/env python

import rospy, math
from path_follower.msg import state_Dynamic, Trajectory2D, TrajectoryPoint2D, Waypoints, Waypoints_state, Point2D
from dbw_mkz_msgs.msg import SteeringCmd
from geometry_msgs.msg import TwistStamped
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
stateEstimate_mark = True
Waypoints_mark = False
Waypoints_received = Waypoints()
state_received = state_Dynamic()
steering_ratio = 14.8

def stateEstimateCallback(data):
    global X, Y, psi, vx, vy, stateEstimate_mark, state_received
    X = data.X + cos(data.psi) * ra_to_cg
    Y = data.Y + sin(psi) * ra_to_cg
    vx = data.vx
    vy = data.vy
    psi = data.psi
    stateEstimate_mark = True
    state_received = data

def WaypointsCallback(data):
    global X_ref, Y_ref, psi_ref, vx_ref, vy_ref, X, Y, vx, vy, psi, Waypoints_received, Waypoints_mark
    Waypoints_received = data
    if stateEstimate_mark:
        Waypoints_mark = True
        X_ref = X 
        Y_ref = Y 
        vx_ref = vx
        vy_ref = vy
        psi_ref = psi

def main(dt, t_vel, ref_index, p_k, p_d, p_i):
    global X_ref, Y_ref, psi_ref, vx_ref, vy_ref, Waypoints_received, stateEstimate_mark, state_received
    global error_d, error_i, error_p
    rospy.init_node('pid', anonymous=True)
    rospy.Subscriber('/waypoints', Waypoints, WaypointsCallback)
    rospy.Subscriber('state_estimate', state_Dynamic, stateEstimateCallback)
    pub = rospy.Publisher('/vehicle/cmd_vel_stamped', TwistStamped, queue_size=1)
    pub2 = rospy.Publisher('vehicle/steering_cmd', SteeringCmd, queue_size=1)
    rate = rospy.Rate(1/dt)

    error_p = 0
    error_i = 0
    error_d = 0

    while (rospy.is_shutdown() != 1):
        if stateEstimate_mark and Waypoints_mark:
            #print(len(Waypoints_received.points), "numwaypints")
            num_steps_received = len(Waypoints_received.points) - 1
            dt_received = Waypoints_received.dt
            horizon = dt_received * num_steps_received
            points = np.zeros((2, num_steps_received))

            # ToDO
            '''            wp = waypoints[6]  # 0.8 seconds in the future
            theta = math.atan2(max(wp[0], 0.01), wp[1]) - math.pi / 2  # range from -pi/2 to pi/2
            if waypoints[-2][0] < 0.5:
                theta = 0.0
            '''
            if Waypoints_received.points[-2].x < 0.5:
                error_theta = 0.0
            else:
                theta_tot = 0.0
                weighting = np.array([2, 4, 6, 6, 3, 2, 1]) # 7 values
                weighting = weighting * 1.0 / np.sum(weighting) * weighting.size

                factor = 2
                for i in range(num_steps_received//factor):
                    wp = Waypoints_received.points[i]  # 0.8 seconds in the future
                    wp = [wp.x, wp.y]
                    theta = math.atan2(max(wp[0], 0.01), wp[1]) - math.pi / 2  # range from -pi/2 to pi/2
                    theta_tot += theta * weighting[i]
                error_theta = theta_tot / (num_steps_received//factor)

            for i in range(num_steps_received):
                points[0, i] = Waypoints_received.points[i].x * cos(psi_ref) - Waypoints_received.points[i].y * sin(psi_ref) + X_ref
                points[1, i] = Waypoints_received.points[i].x * sin(psi_ref) + Waypoints_received.points[i].y * cos(psi_ref) + Y_ref
            t_received = np.linspace(dt_received, horizon, num_steps_received)
            spl_x = UnivariateSpline(t_received, points[0, :], k=3)
            spl_y = UnivariateSpline(t_received, points[1, :], k=3)
            spl_x_dot = spl_x.derivative()
            spl_y_dot = spl_y.derivative()
            spl_x_dot_val = spl_x_dot(t_vel)
            spl_y_dot_val = spl_y_dot(t_vel)
            #spl_v_val = 1.0 * np.sqrt(spl_x_dot_val**2 + spl_y_dot_val**2)

            #spl_v_val = 3.0 #this is for left turn
            spl_v_val = 2.0 # for the right turns

            twist_cmd = TwistStamped()
            twist_cmd.twist.linear.x = spl_v_val
            pub.publish(twist_cmd)
            ref_index_act = min(ref_index, num_steps_received) - 1
            ref_point = points[:, ref_index_act]

            lateral_error = (ref_point[0] - X) * sin(psi) - (ref_point[1] - Y) * cos(psi)


            lateral_error = error_theta
            print("error theta is ", error_theta)

            error_i += dt * (lateral_error + error_p) / 2.
            error_d = (lateral_error - error_p) / dt
            error_p = lateral_error 
            steering_angle = - (p_k * error_p + error_i * p_i + error_d * p_d)
            steering_cmd = SteeringCmd()
            steering_cmd.enable = True
            steering_cmd.steering_wheel_angle_cmd = steering_angle * steering_ratio
            pub2.publish(steering_cmd)
            rospy.loginfo('Lateral error: %f m, pk %f, id %d', lateral_error, p_k, ref_index_act)

        rate.sleep()

if __name__ == '__main__':
    #main(0.01, 0.2, 3, p_k=0.6, p_d=0.0, p_i=0.0)
    #main(0.01, 0.2, 3, p_k=0.3, p_d=0.0, p_i=0.0)
    #main(0.01, 0.2, 3, p_k=0.9, p_d=0.0, p_i=0.0)



    #main(0.01, 0.2, 3, p_k=0.5, p_d=0.08, p_i=0.1)

    # working parameters
    # for left turns
    #main(0.01, 0.2, 3, p_k=0.9, p_d=0.0, p_i=0.0)
    # for right turns
    #main(0.01, 0.2, 3, p_k=0.9, p_d=0.0, p_i=0.0)
    main(0.01, 0.2, 3, p_k=1.35, p_d=0.0, p_i=0.0)
    #main(0.01, 0.2, 3, p_k=0.9, p_d=0.0, p_i=0.0)