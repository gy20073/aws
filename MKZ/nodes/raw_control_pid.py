#!/usr/bin/env python
from control_interface import ControlInterface

# roslib related
import roslib, rospy
roslib.load_manifest('dbw_mkz_msgs')
# messages related
from dbw_mkz_msgs.msg import SteeringReport
from geometry_msgs.msg import Vector3
from std_msgs.msg import String

global condition
condition = "s"

def on_key_received(data):
    key = data.data
    global condition
    condition = key
    print("received key in pid ", key)
    global CONTROL_MODE
    initialize_control_constants(CONTROL_MODE)


def initialize_control_constants(control_mode):
    global SAFETY_SPEED, THROTTLE_CONSTANT, STEERING_CONSTANT, condition

    # TODO: condition has values in [s, a, d, w], change the params below

    if control_mode == 'GTAV':
        SAFETY_SPEED = 17.0  #km/h
        THROTTLE_CONSTANT = 0.8
        STEERING_CONSTANT = -12.1
    elif control_mode == 'CARLA0.9.X':
        SAFETY_SPEED = 8.0  # km/h
        THROTTLE_CONSTANT = 0.4
        STEERING_CONSTANT = -3.5
    elif control_mode == 'WAYPOINTS_REAL_CAR_LEFT':
        SAFETY_SPEED = 9.0  # km/h
        THROTTLE_CONSTANT = 0.2
        STEERING_CONSTANT =  -9.5
    elif control_mode == 'WAYPOINTS_REAL_CAR_RIGHT':
        SAFETY_SPEED = 9.0  # km/h
        THROTTLE_CONSTANT = 0.3
        STEERING_CONSTANT = -8.0
    else: # default is carla 0.8 autopilot
        SAFETY_SPEED = 17.0  #km/h
        THROTTLE_CONSTANT = 0.8
        STEERING_CONSTANT = -20.1
    print("p constants set to safety speed: ", SAFETY_SPEED,
          " Throttle ", THROTTLE_CONSTANT,
          " Steer ", STEERING_CONSTANT)

vehicle_real_speed_kmh = 0.0
debug_speed = 0
controller = None

def on_stb_received(data):
    if controller is None:
        return

    # TODO parse steer, throttle, brake
    steer = data.x
    throttle = data.y
    brake = data.z

    if vehicle_real_speed_kmh > SAFETY_SPEED:
        print "speed still larger than safty speed, cropped. (It will affect performance)"
        throttle = 0.0

    # convert the output to the format of vehicle format
    # the meaning of the predicted value
    # the meaning of the required value
    # TODO: right now no smoother

    controller.set_throttle(throttle * THROTTLE_CONSTANT)
    controller.set_break(brake * 0.0)
    controller.set_steer(steer * STEERING_CONSTANT)  # 8.2 in range
    print('>>> Steering value = {} | real speed = {}'.format(steer * STEERING_CONSTANT, vehicle_real_speed_kmh))

def on_speed_received(data):
    global vehicle_real_speed_kmh
    speed = data.speed
    vehicle_real_speed_kmh = speed * 3.6

    global debug_speed
    debug_speed += 1
    if debug_speed % 100 == 0:
        print("current speed is: ", vehicle_real_speed_kmh, " km/h")


if __name__ == "__main__":
    rospy.init_node('raw_control_pid')

    global CONTROL_MODE
    CONTROL_MODE= 'WAYPOINTS_REAL_CAR_RIGHT' #'WAYPOINTS_REAL_CAR_RIGHT' #"WAYPOINTS_REAL_CAR"
    initialize_control_constants(CONTROL_MODE)

    global controller
    controller = ControlInterface()

    rospy.Subscriber("/vehicle/steering_report", SteeringReport, on_speed_received, queue_size=1)
    rospy.Subscriber('/raw_controls', Vector3, on_stb_received, queue_size=1)
    rospy.Subscriber("/vehicle/mkz_key_command", String, on_key_received, queue_size=10)
    rospy.spin()