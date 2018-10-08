#!/usr/bin/env python

# This file put (1)driving model, (2)keyboard input (3)publish twist command (4)receive image together

# roslib related
import roslib, math, cv2
roslib.load_manifest('dbw_mkz_msgs')
import rospy, importlib, inspect, threading

# messages related
from sensor_msgs.msg import Image as sImage
from cv_bridge import CvBridge
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from dbw_mkz_msgs.msg import SteeringReport
from sensor_msgs.msg import NavSatFix

# standard system packages
import sys, os, time

# some constants
CONTROL_MODE = 'CARLA0.9.X' #''GTAV' #'CARLA0.9.X'

KEYBOARD_TOPIC = "mkz_key_command"
INPUT_IMAGE_TOPIC = "/image_sender_0"
INPUT_IMAGE_TOPIC_LEFT = "/camera_array/cam1/image_raw"
INPUT_IMAGE_TOPIC_RIGHT = "/camera_array/cam2/image_raw"

global bridge, driving_model, vis_pub_full, vehicle_real_speed_kmh, direction, controller
debug_speed = 0
vehicle_real_speed_kmh = 0.0
driving_model = None
direction = 2.0
use_left_right = False
IMAGE_TEMPORAL_DOWNSAMPLE_FACTOR = 3
count_left = 0
count_right = 0
count_middle = 0
IM_WIDTH = 768
IM_HEIGHT = 576
last_computation = time.time()

from control_interface import ControlInterface

left_cache = None
left_lock = threading.Lock()

def initialize_control_constants(control_mode):
    global SAFETY_SPEED, THROTTLE_CONSTANT, STEERING_CONSTANT

    if control_mode == 'GTAV':
        SAFETY_SPEED = 17.0  #km/h
        THROTTLE_CONSTANT = 0.8
        STEERING_CONSTANT = -12.1
    elif control_mode == 'CARLA0.9.X':
        SAFETY_SPEED = 8.0  # km/h
        THROTTLE_CONSTANT = 0.4
        STEERING_CONSTANT = -3.5
    else: # default is carla 0.8 autopilot
        SAFETY_SPEED = 17.0  #km/h
        THROTTLE_CONSTANT = 0.8
        STEERING_CONSTANT = -20.1


def on_image_received_left(data):
    global count_left
    count_left += 1
    if count_left % IMAGE_TEMPORAL_DOWNSAMPLE_FACTOR != 0:
        return

    global left_cache
    img = bridge.imgmsg_to_cv2(data, "bgr8")
    #img = cv2.resize(img, (IM_WIDTH, IM_HEIGHT))
    # flip because the camera is flipped
    img = img[::-1, ::-1, :]
    left_lock.acquire()
    left_cache = img
    left_lock.release()

right_cache = None
right_lock = threading.Lock()
def on_image_received_right(data):
    global count_right
    count_right += 1
    if count_right % IMAGE_TEMPORAL_DOWNSAMPLE_FACTOR != 0:
        return

    global right_cache
    img = bridge.imgmsg_to_cv2(data, "bgr8")
    #img = cv2.resize(img, (IM_WIDTH, IM_HEIGHT))
    # flip because the camera is flipped
    img = img[::-1, ::-1, :]
    right_lock.acquire()
    right_cache = img
    right_lock.release()

# this is the center image
def on_image_received(data):
    # this would directly receive the raw image from the driver
    if driving_model is None or vehicle_real_speed_kmh is None:
        return

    time0 = time.time()
    global bridge, driving_model, vis_pub_full, controller

    img = bridge.imgmsg_to_cv2(data, "bgr8")
    #img = cv2.resize(img, (IM_WIDTH, IM_HEIGHT))
    # flip because the camera is flipped
    img = img[::-1, ::-1, :]

    if use_left_right:
        if left_cache is None or right_cache is None:
            return
        left_lock.acquire()
        right_lock.acquire()
        sensors = [left_cache, img, right_cache]
        left_lock.release()
        right_lock.release()

    else:
        sensors = [img]
    control, vis = driving_model.compute_action(sensors, vehicle_real_speed_kmh, direction,
                                                save_image_to_disk=False, return_vis=True)
    #control, vis = driving_model.compute_action(sensors, 0.0, direction, save_image_to_disk=False, return_vis=True)

    # safty guards to guard against dangerous situation
    if vehicle_real_speed_kmh > SAFETY_SPEED:
        print "speed still larger than safty speed, cropped. (It will affect performance)"
        control.throttle = 0.0

    print('>>>>>> Real speed = {}'.format(vehicle_real_speed_kmh))
    # convert the output to the format of vehicle format
    # the meaning of the predicted value
    # the meaning of the required value
    # TODO: right now no smoother

    controller.set_throttle(control.throttle * THROTTLE_CONSTANT)
    controller.set_break(control.brake * 0.0)
    controller.set_steer(control.steer * STEERING_CONSTANT)  # 8.2 in range
    print('>>> Steering value = {} | Steering constant = {}'.format(control.steer * STEERING_CONSTANT, STEERING_CONSTANT))
    vis_pub_full.publish(bridge.cv2_to_imgmsg(vis, "rgb8"))

    print ("total time for on image receive is ", time.time()-time0)
    time_now = time.time()
    global last_computation
    print("model running HZ is ", 1.0 / (time_now - last_computation))
    last_computation = time.time()
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

global gps_arr
def on_gps_received(data):
    lat = data.latitude
    lng = data.longitude

    min_direction = 2.0
    min_dist = 99999999.0
    for entry in gps_arr:
        dist = math.sqrt((lat-entry[0])**2 + (lng-entry[1])**2)
        if dist < min_dist:
            min_direction = entry[2]
            min_dist = dist

    print("Prior: setting to prior", min_direction)

    global direction
    direction = min_direction

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

    use_auto_traj = sys.argv[2]
    if use_auto_traj.lower() == "true":
        use_auto_traj = True
        gps_traj_file = sys.argv[3]
    else:
        use_auto_traj = False

    global use_left_right
    if sys.argv[4].lower() == "true":
        use_left_right = True
    else:
        use_left_right = False

    # a global shared data structure, list of [forward speed in m/s, yaw rate in rad/s]
    global bridge
    bridge = CvBridge()

    initialize_control_constants(CONTROL_MODE)

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
    driving_model = CarlaMachine("0", exp_id, get_driver_config(), 0.1,
                                 gpu_perception=[0, 1],
                                 perception_paths="path_docker_newseg",
                                 batch_size=3 if use_left_right else 1)

    # subscribe to many topics
    rospy.Subscriber(INPUT_IMAGE_TOPIC, sImage, on_image_received, queue_size=1)
    if use_left_right:
        rospy.Subscriber(INPUT_IMAGE_TOPIC_LEFT, sImage, on_image_received_left, queue_size=1)
        rospy.Subscriber(INPUT_IMAGE_TOPIC_RIGHT, sImage, on_image_received_right, queue_size=1)


    if use_auto_traj:
        rospy.Subscriber("/fix", NavSatFix, on_gps_received, queue_size=1)
        with open(gps_traj_file, "r") as f:
            global gps_arr
            lines = f.readlines()
            gps_arr = []
            for line in lines:
                sp = line.split(", ")
                gps_arr.append([float(sp[0]), float(sp[1]), int(float(sp[2]))])
            print(gps_arr)
    else:
        rospy.Subscriber(KEYBOARD_TOPIC, String, on_key_received, queue_size=10)
    rospy.Subscriber("/vehicle/steering_report", SteeringReport, on_speed_received, queue_size=1)
    rospy.spin()
