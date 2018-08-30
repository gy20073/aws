#!/usr/bin/env python

# This file put (1)driving model, (2)keyboard input (3)publish twist command (4)receive image together

# roslib related
import roslib
roslib.load_manifest('dbw_mkz_msgs')
import rospy, importlib, inspect

# messages related
from sensor_msgs.msg import Image as sImage
from cv_bridge import CvBridge
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from dbw_mkz_msgs.msg import SteeringReport

# standard system packages
import sys, os, time

# some constants
SAFETY_SPEED = 8.0 # in km/h, cap the speed if larger than it

KEYBOARD_TOPIC = "mkz_key_command"
INPUT_IMAGE_TOPIC = "/image_sender_0"

global bridge, driving_model, vis_pub_full, vehicle_real_speed_kmh, direction, controller
debug_speed = 0
vehicle_real_speed_kmh = 0.0
driving_model = None
direction = 2.0

from control_interface import ControlInterface

def on_image_received(data):
    # this would directly receive the raw image from the driver
    if driving_model is None or vehicle_real_speed_kmh is None:
        return

    time0 = time.time()
    global bridge, driving_model, vis_pub_full, controller

    img = bridge.imgmsg_to_cv2(data, "bgr8")
    # flip because the camera is flipped
    img = img[::-1, ::-1, :]

    control, vis = driving_model.compute_action(img, vehicle_real_speed_kmh, direction,
                                           save_image_to_disk=False, return_vis=True)

    # safty guards to guard against dangerous situation
    if vehicle_real_speed_kmh > SAFETY_SPEED:
        print "speed still larger than safty speed, cropped. (It will affect performance)"
        control.throttle = 0.0

    # convert the output to the format of vehicle format
    # the meaning of the predicted value
    # the meaning of the required value
    # TODO: right now no smoother
    controller.set_throttle(control.throttle * 0.1)
    controller.set_break(control.brake * 0.0)
    controller.set_steer(control.steer * -3.1)  # 8.2 in range

    vis_pub_full.publish(bridge.cv2_to_imgmsg(vis, "rgb8"))

    print ("total time for on image receive is ", time.time()-time0)
    print "on image received"


def on_key_received(data):
    key = data.data
    global direction
    if key == "s":
        # set to no prior function
        print("Prior: setting to no prior")
        direction = 2.0
    elif key == "w":
        print("Prior: setting to straight prior")
        direction = 5.0
    elif key == "a":
        print("Prior: setting to left prior")
        direction = 3.0
    elif key == "d":
        print("Prior: setting to right prior")
        direction = 4.0
    else:
        print("invalid command", key)

def get_file_real_path():
    abspath = os.path.abspath(inspect.getfile(inspect.currentframe()))
    return os.path.realpath(abspath)

def on_speed_received(data):
    global vehicle_real_speed_kmh
    speed = data.speed
    vehicle_real_speed_kmh = speed * 3.6

    global debug_speed
    debug_speed += 1
    if debug_speed % 100 == 0:
        print("current speed is: ", vehicle_real_speed_kmh, " km/h")

def get_driver_config():
    driver_conf = lambda: None  # an object that could add attributes dynamically
    driver_conf.image_cut = [0, 100000]
    driver_conf.host = None
    driver_conf.port = None
    driver_conf.use_planner = False  # fixed
    driver_conf.carla_config = None  # This is not used by CarlaMachine but it's required
    return driver_conf

if __name__ == "__main__":
    rospy.init_node('BDD_Driving_Model')
    exp_id = sys.argv[1]

    # a global shared data structure, list of [forward speed in m/s, yaw rate in rad/s]
    global bridge
    bridge = CvBridge()

    global vis_pub_full
    vis_pub_full = rospy.Publisher('/vis_continuous_full', sImage, queue_size=10)

    global controller
    controller = ControlInterface()

    global driving_model
    # BDD Driving model related
    driving_model_code_path = os.path.join(os.path.dirname(get_file_real_path()), "../../CIL_modular/")
    os.chdir(driving_model_code_path)
    sys.path.append("drive_interfaces/carla/comercial_cars")
    from carla_machine import *
    driving_model = CarlaMachine("0", exp_id, get_driver_config(), 0.1)

    # subscribe to many topics
    rospy.Subscriber(INPUT_IMAGE_TOPIC, sImage, on_image_received, queue_size=1)
    rospy.Subscriber(KEYBOARD_TOPIC, String, on_key_received, queue_size=10)
    rospy.Subscriber("dbw_mkz_msgs/SteeringReport", SteeringReport, on_speed_received, queue_size=1)
    rospy.spin()
