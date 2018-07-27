#!/usr/bin/env python

# roslib related
import rospy, time
from threading import Thread

# messages related
from dbw_mkz_msgs.msg import ThrottleCmd, BrakeCmd, SteeringCmd

class ControlInterface(object):
    def __init__(self):
        self._throttle = 0.0
        self._throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=10)
        self._brake = 0.0
        self._brake_pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=10)
        self._steer = 0.0
        self._steer_pub = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=10)
        self.start_loop()

    def set_throttle(self, new_throttle):
        self._throttle = new_throttle

    def set_break(self, new_brake):
        self._brake = new_brake

    def set_steer(self, new_steer):
        self._steer = new_steer

    def pub_once(self):
        throttle_cmd = ThrottleCmd()
        throttle_cmd.enable = True
        throttle_cmd.pedal_cmd_type = 2 # percentage
        throttle_cmd.pedal_cmd = self._throttle
        self._throttle_pub.publish(throttle_cmd)

        brake_cmd = BrakeCmd()
        brake_cmd.enable = True
        brake_cmd.pedal_cmd_type = 2 # percentage
        brake_cmd.pedal_cmd = self._brake
        self._brake_pub.publish(brake_cmd)

        steer_cmd = SteeringCmd()
        steer_cmd.enable = True
        # rad, range -8.2 to 8.2
        steer_cmd.steering_wheel_angle_cmd = self._steer
        self._steer_pub.publish(steer_cmd)

    def pub_loop(self):
        counter = 0
        while True:
            counter += 1
            self.pub_once()
            time.sleep(1.0 / 50)
            if counter % 100 == 0:
                print("publishing at 50hz")

    def start_loop(self):
        thread = Thread(target=self.pub_loop())
        thread.start()

if __name__ == "__main__":
    rospy.init_node('raw_control_publisher', anonymous=True)
    controller = ControlInterface()
    controller.set_throttle(0.5)
