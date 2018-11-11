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
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
from dbw_mkz_msgs.msg import SteeringReport


from sensor_msgs.msg import NavSatFix

# standard system packages
import sys, os, time
import numpy as np
from multiprocessing import Process, Pipe


from path_follower.msg import Waypoints, Point2D
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
IMAGE_TEMPORAL_DOWNSAMPLE_FACTOR = 1
IMAGE_DOWNSAMPLE_MAIN = 1
count_left = 0
count_right = 0
count_middle = 0
IM_WIDTH = 768
IM_HEIGHT = 576
last_computation = time.time()
raw_control_pub = None

left_cache = None
left_lock = threading.Lock()

def on_image_received_left(data):
    global count_left
    count_left += 1
    if count_left % IMAGE_TEMPORAL_DOWNSAMPLE_FACTOR != 0:
        return

    global left_cache
    img = bridge.imgmsg_to_cv2(data, "bgr8")
    img = cv2.resize(img, (IM_WIDTH, IM_HEIGHT))
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
    img = cv2.resize(img, (IM_WIDTH, IM_HEIGHT))
    # flip because the camera is flipped
    img = img[::-1, ::-1, :]
    right_lock.acquire()
    right_cache = img
    right_lock.release()


def worker(exp_id, use_left_right, conn):
    print("begin initialization")
    from carla_machine import *
    driving_model = CarlaMachine("0", exp_id, get_driver_config(), 0.1,
                                 gpu_perception=[0, 1],
                                 perception_paths="path_docker_newseg",
                                 batch_size=3 if use_left_right else 1)
    print("initialization finished")
    while True:
        # there should be a communication protocol
        print("waiting to recive")
        param = conn.recv()
        print("I received somthing")
        control, vis = driving_model.compute_action(*param, save_image_to_disk=False, return_vis=True, return_extra=False)
        print("compute finishe")
        conn.send([{"throttle": control.throttle, "brake": control.brake, "steer": control.steer}, vis])
        print("aftet sendoing out the message")

# this is the center image
def on_image_received(data):
    # this would directly receive the raw image from the driver
    if vehicle_real_speed_kmh is None:
        return

    global count_middle
    count_middle += 1
    if count_middle % IMAGE_DOWNSAMPLE_MAIN != 0:
        return

    time0 = time.time()
    global bridge, driving_model, vis_pub_full, controller, waypoint_pub, parent_conn, dbw_enable, vis_pub_full_enabled

    img = bridge.imgmsg_to_cv2(data, "bgr8")
    # deliberately not resizing the image, since it might be zoomed later
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
    t00 = time.time()

    '''
    print("before sending out the images")
    parent_conn.send([sensors, vehicle_real_speed_kmh, direction])
    print("before receving")
    control_dict, vis = parent_conn.recv()
    control = lambda : None
    control.steer = control_dict["steer"]
    control.throttle = control_dict["throttle"]
    control.brake = control_dict["brake"]
    print("after receiving")
    '''
    control, vis = driving_model.compute_action(sensors, vehicle_real_speed_kmh, direction,
                                                save_image_to_disk=False, return_vis=True, return_extra=False)
    #print("time for compute action is ", time.time() - t00)
    #control, vis = driving_model.compute_action(sensors, 0.0, direction, save_image_to_disk=False, return_vis=True)
    global use_waypoint

    if use_waypoint:
        # shift the waypoint forward by the computation time
        time_passed = time.time() - time0
        time_list = np.array([0.2*(i+1) for i in range(control.shape[0])])
        time_list -= time_passed # this typically less than 200ms
        desired_time = np.array([0.2*(i+1) for i in range(control.shape[0]-1)])

        wp0 = np.interp(desired_time, time_list, control[:, 0])
        wp1 = np.interp(desired_time, time_list, control[:, 1])
        control = np.stack([wp0, wp1], axis=1)

        # TODO: have a manner to control the safety speed
        waypoints = Waypoints()
        for i in range(1, control.shape[0]):
            pt = Point2D()
            pt.x = control[i, 0]
            pt.y = -control[i, 1]
            waypoints.points.append(pt)
        waypoints.dt = 0.2
        waypoint_pub.publish(waypoints)

    else:
        v = Vector3()
        v.x = control.steer
        v.y = control.throttle
        v.z = control.brake
        global raw_control_pub
        if raw_control_pub != None:
            raw_control_pub.publish(v)

    #cv2.imshow("Cameras", vis)
    #cv2.waitKey(5)

    msg = bridge.cv2_to_imgmsg(vis, "rgb8")
    vis_pub_full.publish(msg)
    if dbw_enable:
        print("enabling, republishing the message to a second topic")
        vis_pub_full_enabled.publish(msg)

    time_now = time.time()
    global last_computation
    print ("total time for on image receive is ", time.time()-time0, ". Runs at ", 1.0 / (time_now - last_computation), "Hz")
    last_computation = time.time()



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

def on_dbw_enable_changed(data):
    global dbw_enable
    dbw_enable = data.data

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

    global use_waypoint
    if sys.argv[5].lower() == "true":
        use_waypoint = True
    else:
        use_waypoint = False

    # a global shared data structure, list of [forward speed in m/s, yaw rate in rad/s]
    global bridge
    bridge = CvBridge()

    global vis_pub_full, vis_pub_full_enabled
    vis_pub_full = rospy.Publisher('/vis_continuous_full', sImage, queue_size=1)
    rospy.Subscriber("/vehicle/dbw_enabled", Bool, on_dbw_enable_changed, queue_size=1)
    vis_pub_full_enabled = rospy.Publisher('/vis_continuous_full_enabled', sImage, queue_size=1)

    if not use_waypoint:
        pass
    else:
        global waypoint_pub
        waypoint_pub = rospy.Publisher('/waypoints', Waypoints, queue_size=1)

    global driving_model, parent_conn
    # BDD Driving model related
    driving_model_code_path = os.path.join(os.path.dirname(get_file_real_path()), "../../CIL_modular/")
    os.chdir(driving_model_code_path)
    sys.path.append("drive_interfaces/carla/comercial_cars")
    from carla_machine import *

    driving_model = CarlaMachine("0", exp_id, get_driver_config(), 0.1,
                                 gpu_perception=[0, 1],
                                 perception_paths="path_docker_newseg",
                                 batch_size=3 if use_left_right else 1)

    '''
    # TODO: start the worker
    parent_conn, child_conn = Pipe()
    p = Process(target=worker, args=(exp_id, use_left_right, child_conn))
    p.start()
    time.sleep(30)
    print("by now, the perception stack should finished starting")
    '''

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

    global raw_control_pub
    raw_control_pub = rospy.Publisher('/raw_controls', Vector3, queue_size=1)

    rospy.spin()
