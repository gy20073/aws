#!/usr/bin/env python

import rospy, time, threading
# messages related
from dbw_mkz_msgs.msg import ThrottleCmd, BrakeCmd, SteeringCmd, TwistCmd
from std_msgs.msg import Float64
import dynamic_reconfigure.client
#from geometry_msgs.msg import TwistStamped

class ControlInterface(object):
    def __init__(self, no_speed_control = False):
        self._throttle = 0.0
        self._throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=10)
        self._brake = 0.0
        self._brake_pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=10)
        self._steer = 0.0
        self._steer_pub = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=10)
        self.no_speed_control = no_speed_control
        if no_speed_control:
            self._twist_speed = 0.0
            #self._twist_pub = rospy.Publisher('/vehicle/cmd_vel_stamped', TwistStamped, queue_size=1)
            self._twist_pub = rospy.Publisher('/vehicle/cmd_vel_with_limits', TwistCmd, queue_size=1)
            self._speed_state = "GO" # or "STOP"
            self._speed_counter = 0
            # TODO: the naming might be bad
            #self._dclient = dynamic_reconfigure.client.Client("dbw_mkz_twist_controller", timeout=15)
            
        self.counter = 0
        self.last_time = time.time()
        self.start_loop()

    def set_throttle(self, new_throttle):
        # percentage from 0 to 1
        self._throttle = new_throttle

    def set_break(self, new_brake):
        # percentage, from 0 to 1
        self._brake = new_brake

    def set_steer(self, new_steer):
        # range -8.2 to 8.2
        self._steer = new_steer

    def set_twist_speed(self, new_speed):
        self._twist_speed = new_speed

    def set_speed_features(self, **dict):
        should_stop = self._should_stop_score(dict)

        if self._speed_state == "GO":
            if should_stop > 0.6: # correspond to throttle 0, brake 0.2
                self._speed_counter += 1
            else:
                self._speed_counter = 0
            if self._speed_counter >= 2:
                # decide to stop
                print("change to stop mode")
                self._speed_state = "STOP"
                self._speed_counter = 0
        else: # STOP mode
            if should_stop < 0.1: # correspond to throttle 0.8, brake 0.0
                self._speed_counter += 1
            else:
                self._speed_counter = 0
            if self._speed_counter >= 4:
                print("change to go mode")
                self._speed_state = "GO"
                self._speed_counter = 0


    def pub_once(self):
        if not self.no_speed_control:
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
        else:
            twist_cmd = TwistCmd()
            #print('===== Twist_sped = {}'.format(self._twist_speed))
            if self._speed_state == "GO":
                twist_cmd.twist.linear.x = self._twist_speed
            else: # STOP
                twist_cmd.twist.linear.x = 0.0
            twist_cmd.accel_limit = 1.0 # m/s^2
            twist_cmd.decel_limit = 2.0 # m/s^2
            self._twist_pub.publish(twist_cmd)
            #self._dclient.update_configuration({"accel_ki":0.1, "accel_kp":0.4})
            
        steer_cmd = SteeringCmd()
        steer_cmd.enable = True
        # rad, range -8.2 to 8.2
        steer_cmd.steering_wheel_angle_cmd = self._steer
        # TODO: safty, from 0 to 8.2 rad/s
        steer_cmd.steering_wheel_angle_velocity = 10.0
        self._steer_pub.publish(steer_cmd)

    def start_loop(self):
        self.counter += 1
        if self.counter % 350 == 0:
            #print("time eplapsed is ", time.time() - self.last_time)
            #self.last_time = time.time()
            print("brake", self._brake, "throttle", self._throttle, "steer", self._steer)
        self.pub_once()
        if not rospy.is_shutdown():
            threading.Timer(1.0 / 350, self.start_loop).start()


if __name__ == "__main__":
    rospy.init_node('raw_control_publisher', anonymous=True)
    controller = ControlInterface()
    #controller.set_throttle(0.2)
    controller.set_break(1.0)