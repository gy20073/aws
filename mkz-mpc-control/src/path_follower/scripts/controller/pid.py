#!/usr/bin/env python

import rospy, math
from path_follower.msg import state_Dynamic, Trajectory2D, TrajectoryPoint2D, Waypoints, Waypoints_state, Point2D
from dbw_mkz_msgs.msg import SteeringCmd
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String

# define the initial states and timestep
ra_to_cg = 1.65
Waypoints_mark = False
Waypoints_received = Waypoints()
steering_ratio = 14.8

def WaypointsCallback(data):
    global Waypoints_received, Waypoints_mark
    Waypoints_received = data
    Waypoints_mark = True

global condition
condition = "s"

def on_key_received(data):
    key = data.data
    global condition
    condition = key
    print("received key in pid ", key)

def main(dt):
    global Waypoints_received, condition
    rospy.init_node('pid', anonymous=True)
    rospy.Subscriber('/waypoints', Waypoints, WaypointsCallback)
    pub = rospy.Publisher('/vehicle/cmd_vel_stamped', TwistStamped, queue_size=1)
    pub2 = rospy.Publisher('vehicle/steering_cmd', SteeringCmd, queue_size=1)
    rospy.Subscriber("/vehicle/mkz_key_command", String, on_key_received, queue_size=10)
    rate = rospy.Rate(1/dt)

    error_p = 0
    error_i = 0
    error_d = 0

    while (rospy.is_shutdown() != 1):
        if Waypoints_mark:
            # all parameter's default values
            spl_v_val = 3.0
            p_d = 0.0
            p_i = 0.0
            p_k = 0.9
            use_those_waypoints = range(len(Waypoints_received.points) - 1)
            if condition == "w":
                pass
            elif condition == "a":
                # left
                use_those_waypoints = range(len(Waypoints_received.points) - 1)
                p_k = 0.9
                spl_v_val = 3.0
            elif condition == "d":
                # right
                #p_k = 1.35
                #p_k = 0.6
                p_k = 0.9
                spl_v_val = 2.0
                use_those_waypoints = range(len(Waypoints_received.points)//2 - 1)
            elif condition == "s":
                pass
            else:
                print("unknown condition ", condition)

            if Waypoints_received.points[-2].x < 0.5:
                error_theta = 0.0
            else:
                theta_tot = 0.0
                #weighting = np.array([2, 4, 6, 6, 3, 2, 1]) # 7 values
                #weighting = weighting * 1.0 / np.sum(weighting) * weighting.size

                for i in use_those_waypoints:
                    wp = Waypoints_received.points[i]  # 0.8 seconds in the future
                    wp = [wp.x, wp.y]
                    theta = math.atan2(max(wp[0], 0.01), wp[1]) - math.pi / 2  # range from -pi/2 to pi/2
                    theta_tot += theta #* weighting[i]
                error_theta = theta_tot / len(use_those_waypoints)

            lateral_error = error_theta

            twist_cmd = TwistStamped()
            twist_cmd.twist.linear.x = spl_v_val
            pub.publish(twist_cmd)

            error_i += dt * (lateral_error + error_p) / 2.
            error_d = (lateral_error - error_p) / dt
            error_p = lateral_error 
            steering_angle = - (p_k * error_p + error_i * p_i + error_d * p_d)
            steering_cmd = SteeringCmd()
            steering_cmd.enable = True
            steering_cmd.steering_wheel_angle_cmd = steering_angle * steering_ratio
            pub2.publish(steering_cmd)
            rospy.loginfo('error: %f m, pk %f, speed %f, mode %s', lateral_error, p_k, spl_v_val, condition)
            print("using those waypoints ", use_those_waypoints)

            # collecting useful info and publish to a message topic
            message = "P={:.2f} \nSteer={:.2f} \nTarget Speed={:.2f} m/s \nActiveWP=%s \nController Condition=%s\n".format(
                                                                                        p_k,
                                                                                        steering_cmd.steering_wheel_angle_cmd,
                                                                                        spl_v_val,
                                                                                        str(use_those_waypoints),
                                                                                        condition)
            global controller_message_pub
            controller_message_pub.publish(message)
        rate.sleep()

if __name__ == '__main__':
    global controller_message_pub
    controller_message_pub = rospy.Publisher("/controller_hyper_param", String, queue_size=10)

    main(0.01)