#!/usr/bin/env python

# This file put (1)driving model, (2)keyboard input (3)publish twist command (4)receive image together

# roslib related
import roslib, cv2, pickle, sys, math, pynmea2
import rospy, threading

# messages related
from sensor_msgs.msg import Image as sImage
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import String
from dbw_mkz_msgs.msg import SteeringReport
from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from nmea_msgs.msg import Sentence

INPUT_IMAGE_TOPIC = "/image_sender_0"
INPUT_IMAGE_TOPIC_LEFT = "/camera_array/cam1/image_raw"
INPUT_IMAGE_TOPIC_RIGHT = "/camera_array/cam2/image_raw"

global bridge, driving_model, vis_pub_full, vehicle_real_speed_kmh, direction, controller

IM_WIDTH = 768*2
IM_HEIGHT = 576*2

left_cache = None
left_lock = threading.Lock()

infos = []
infos_enable=[]

def on_image_received_left(data):
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
    global right_cache
    img = bridge.imgmsg_to_cv2(data, "bgr8")
    img = cv2.resize(img, (IM_WIDTH, IM_HEIGHT))
    # flip because the camera is flipped
    img = img[::-1, ::-1, :]
    right_lock.acquire()
    right_cache = img
    right_lock.release()


condition = "s"
def on_key_received(data):
    key = data.data
    global condition
    condition = key
    print("received key in pid ", key)

vehicle_real_speed_kmh = 0.0
def on_speed_received(data):
    global vehicle_real_speed_kmh
    speed = data.speed
    vehicle_real_speed_kmh = speed * 3.6

dbw_enable = False
def on_dbw_enable_changed(data):
    global dbw_enable
    dbw_enable = data.data

global pickle_path
# this is the center image
def on_image_received(data):
    # this would directly receive the raw image from the driver
    global bridge, vis_pub_full

    img = bridge.imgmsg_to_cv2(data, "bgr8")
    img = cv2.resize(img, (IM_WIDTH, IM_HEIGHT))
    # flip because the camera is flipped
    img = img[::-1, ::-1, :]

    if left_cache is None or right_cache is None:
        return
    left_lock.acquire()
    right_lock.acquire()
    sensors = [left_cache, img, right_cache]
    sensors = np.concatenate(sensors, axis=1)
    left_lock.release()
    right_lock.release()

    img_msg = bridge.cv2_to_imgmsg(sensors, "bgr8")
    vis_pub_full.publish(img_msg)
    # hashed image value to (speed and command)
    global vehicle_real_speed_kmh, condition

    infos.append([vehicle_real_speed_kmh/3.6, condition, vehicle_pos, vehicle_yaw, vehicle_pos_nmea])
    # pickle this out to a file
    with open(pickle_path+ ".pkl", 'wb') as f:
        pickle.dump(infos, f, protocol=2)

    # TODO: also optionally publish the enabled version
    if dbw_enable:
        global vis_pub_full_enable
        vis_pub_full_enable.publish(img_msg)
        infos_enable.append([vehicle_real_speed_kmh / 3.6, condition, vehicle_pos, vehicle_yaw, vehicle_pos_nmea])
        # pickle this out to a file
        with open(pickle_path+"_enable.pkl", 'wb') as f:
            pickle.dump(infos_enable, f, protocol=2)


vehicle_pos = [37.918355, -122.338461]
def on_gps_received(data):
    lat = data.latitude
    lng = data.longitude
    global vehicle_pos
    vehicle_pos = [lat, lng]

vehicle_pos_nmea = [37.918355, -122.338461]
def on_gps_received_nmea(data):
    if data.sentence[:4] != "[USB":
        #print("throw awaw sentence ", data.sentence)
        return
    else:
        pass
        #print("keep sentence ", data.sentence)

    msg = pynmea2.parse(data.sentence[6:])
    global vehicle_pos_nmea
    vehicle_pos_nmea=[float(msg.latitude), float(msg.longitude)]


vehicle_yaw = 0.0
def quaternion_to_yaw(msg):
    q = msg.orientation
    #yaw = math.atan2(2 * (q.x * q.w + q.y * q.z), 1 - 2 * (q.z ** 2 + q.w ** 2))
    pitch = math.atan2(2*(q.x*q.y + q.z*q.w), 1-2*(q.y**2 + q.z**2))
    #roll = math.asin(2*(q.x*q.z - q.y*q.w))
    return -pitch + np.pi / 2

def on_imu_received(data):
    global vehicle_yaw
    vehicle_yaw = quaternion_to_yaw(data)

if __name__ == "__main__":
    rospy.init_node('sync_3cam')

    global pickle_path
    pickle_path = sys.argv[1]

    # a global shared data structure, list of [forward speed in m/s, yaw rate in rad/s]
    global bridge
    bridge = CvBridge()

    global vis_pub_full
    vis_pub_full = rospy.Publisher('sync_3cam', sImage, queue_size=10)

    global vis_pub_full_enable
    vis_pub_full_enable = rospy.Publisher('sync_3cam_enable', sImage, queue_size=10)

    # subscribe to many topics
    rospy.Subscriber(INPUT_IMAGE_TOPIC, sImage, on_image_received, queue_size=1)
    rospy.Subscriber(INPUT_IMAGE_TOPIC_LEFT, sImage, on_image_received_left, queue_size=1)
    rospy.Subscriber(INPUT_IMAGE_TOPIC_RIGHT, sImage, on_image_received_right, queue_size=1)

    # add the commands and speed
    rospy.Subscriber("/vehicle/mkz_key_command", String, on_key_received, queue_size=10)
    rospy.Subscriber("/vehicle/steering_report", SteeringReport, on_speed_received, queue_size=1)
    rospy.Subscriber("/vehicle/dbw_enabled", Bool, on_dbw_enable_changed, queue_size=1)

    # the mapping related
    global vehicle_pos, vehicle_yaw
    rospy.Subscriber("/vehicle/gps/fix", NavSatFix, on_gps_received, queue_size=10)
    rospy.Subscriber("/xsens/imu/data", Imu, on_imu_received, queue_size=10)

    rospy.Subscriber("/nmea_sentence", Sentence, on_gps_received_nmea, queue_size=1)

    rospy.spin()
