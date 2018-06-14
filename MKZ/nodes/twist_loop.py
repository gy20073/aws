#!/usr/bin/env python

# roslib related
import rospy, time, threading, sys
import numpy as np

# messages related
from dbw_mkz_msgs.msg import TwistCmd
from geometry_msgs.msg import Vector3


TWIST_COMM_TOPIC = "twist_predicted_results"
CONTROLLER_LATENCY = 0.0 # this is estimated without any evidence, change it to something smaller
global PREDICTION_HZ
PREDICTION_HZ = 3
TWIST_HZ = 50 # the doc said > 5Hz

# global variables
predicted_cmd = []
twist_pub = rospy.Publisher('cmd_vel_with_limits', TwistCmd, queue_size=10)
lock = threading.Lock()

def prediction_smoother(predicted_cmd):
    if len(predicted_cmd) == 0:
        output = [0, 0]
    else:
        # do the linear interpolation with the time
        t=np.array(predicted_cmd)
        output = [np.interp([time.time()+CONTROLLER_LATENCY], t[:, 2], t[:, 0])[0],
                  np.interp([time.time()+CONTROLLER_LATENCY], t[:, 2], t[:, 1])[0] ]

        if time.time()+CONTROLLER_LATENCY >= t[-1, 2]:
            print("warning: future prediction is lagged, last time-current", t[-1,2]- time.time())

    return output, predicted_cmd[-PREDICTION_HZ*2:]


def twist_loop():
    # timer for the loop
    global predicted_cmd, twist_pub
    count = 0

    while not rospy.is_shutdown():
        # smooth the prediction
        with lock:
            output, predicted_cmd = prediction_smoother(predicted_cmd)

        # construct the twist command
        twist_cmd = TwistCmd()
        twist_cmd.twist.linear.x = output[0]
        twist_cmd.twist.angular.z = output[1]
        twist_cmd.accel_limit = 1.0
        twist_cmd.decel_limit = 1.0

        # publish it
        twist_pub.publish(twist_cmd)

        time.sleep(1.0/TWIST_HZ)

        count += 1
        if count % TWIST_HZ == 0:
            print "publishing twist command at %d Hz" % TWIST_HZ
    print "twist publishing exit"


def on_prediction_received(m):
    with lock:
        predicted_cmd.append([m.x, m.y, m.z])

if __name__ == "__main__":
    if len(sys.argv) > 1:
        global PREDICTION_HZ
        PREDICTION_HZ = int(sys.argv[1])

    rospy.init_node('twist_control_publisher', anonymous=True)
    rospy.Subscriber(TWIST_COMM_TOPIC, Vector3, on_prediction_received, queue_size=1)
    twist_loop()
