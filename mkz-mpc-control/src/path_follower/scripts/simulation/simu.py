#!/usr/bin/env python
import rospy
import scipy.io 
from path_follower.msg import state_Dynamic, Trajectory2D, TrajectoryPoint2D
import os, rospkg
from geometry_msgs.msg import TwistStamped
from dbw_mkz_msgs.msg import SteeringReport,SteeringCmd
from vehicle_opt import vehicle
from numpy import sign
from math import cos, sin 

action = [0, 0]
dt = 0.02
car = vehicle(dt, 10, False, 0, 0, 0, False)
state = [558640.9252, 4196656.6405+1, 1.20719921+0.2, 3.9, 0, 0, 0]  
#state = [558641.0886, 4196656.1972, 1.2337, 4.6, 0, 0, 0]  
car.setState(state)

def cmd_vel_stampedCallback(data):
    global action, car
    action[0] = (data.twist.linear.x - car.state[3])/ dt / car.maxDvxdt


def steering_cmdCallback(data):
    global action, car
    steering_cmd = data.steering_wheel_angle_cmd
    steering_rate = data.steering_wheel_angle_velocity
    if steering_rate == 0:
        steering_rate = car.maxSteeringRate
    action[1] = min(abs(steering_rate / car.maxSteeringRate), abs(car.state[6] * car.steeringRatio - steering_cmd) / dt / car.maxSteeringRate)
    action[1] = action[1] * sign(steering_cmd - car.state[6] * car.steeringRatio); 

def simu():
    global action, car
    rospy.init_node('simu', anonymous=True)

    rate = rospy.Rate(1/dt)
    rospy.loginfo("simulator node starts")

    rospy.Subscriber('/vehicle/cmd_vel_stamped', TwistStamped, cmd_vel_stampedCallback,queue_size=1)
    rospy.Subscriber('/vehicle/steering_cmd', SteeringCmd, steering_cmdCallback,queue_size=1)
    pub1 = rospy.Publisher('/vehicle/steering_report', SteeringReport, queue_size=1)
    pub2 = rospy.Publisher('state_estimate', state_Dynamic, queue_size=1)
    steering_report = SteeringReport()
    state_report = state_Dynamic()
    while (rospy.is_shutdown() != 1): 
        car.simulate(action)
        steering_report.steering_wheel_angle = car.state[6] * car.steeringRatio  
        state_report.vx = car.state[3];
        state_report.vy = car.state[4] - car.state[5] * car.vhMdl[1];
        state_report.X = car.state[0] - cos(car.state[2]) * car.vhMdl[1];
        state_report.Y = car.state[1] - sin(car.state[2]) * car.vhMdl[1];
        state_report.psi = car.state[2];
        state_report.wz = car.state[5];
        pub1.publish(steering_report)
        pub2.publish(state_report)  
        rate.sleep()

if __name__ == '__main__':
    simu()
