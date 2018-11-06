#!/usr/bin/env python

import rospy
from path_follower.msg import state_Dynamic, Waypoints, Point2D
from tracks import Tra
from std_msgs.msg import String, Header
from scipy.interpolate import UnivariateSpline
import numpy as np
from math import floor, cos, sin
import pickle
import os, rospkg

def main(dt, dt_ros):
    # import pkl file
    rospack = rospkg.RosPack()
    file_name = os.path.join(rospack.get_path("path_follower"), "scripts", "waypoint_loader", 'output_0_768.avi.pkl')

    with open(file_name, 'rb') as f:
        data = pickle.load(f)

    rospy.init_node('toy_planner_waypoints', anonymous=True)
    pub = rospy.Publisher('waypoints', Waypoints, queue_size=1)
    rate = rospy.Rate(1/dt_ros)

    Index = 0
    while (rospy.is_shutdown() != 1):
        waypoints = Waypoints()
        for i in range(8):
            pt = Point2D()
            pt.x = data[Index][i, 0]
            pt.y = data[Index][i, 1]
            waypoints.points.append(pt)
        waypoints.dt = dt
        pub.publish(waypoints)
        Index += 1
        rate.sleep()

if __name__ == '__main__':
    main(0.2, 0.2)
