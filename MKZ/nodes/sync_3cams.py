#!/usr/bin/env python

# This file put (1)driving model, (2)keyboard input (3)publish twist command (4)receive image together

# roslib related
import roslib, cv2
import rospy, threading

# messages related
from sensor_msgs.msg import Image as sImage
from cv_bridge import CvBridge
import numpy as np

INPUT_IMAGE_TOPIC = "/image_sender_0"
INPUT_IMAGE_TOPIC_LEFT = "/camera_array/cam1/image_raw"
INPUT_IMAGE_TOPIC_RIGHT = "/camera_array/cam2/image_raw"

global bridge, driving_model, vis_pub_full, vehicle_real_speed_kmh, direction, controller

IM_WIDTH = 768*2
IM_HEIGHT = 576*2

left_cache = None
left_lock = threading.Lock()

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

    vis_pub_full.publish(bridge.cv2_to_imgmsg(sensors, "bgr8"))


if __name__ == "__main__":
    rospy.init_node('sync_3cam')

    # a global shared data structure, list of [forward speed in m/s, yaw rate in rad/s]
    global bridge
    bridge = CvBridge()

    global vis_pub_full
    vis_pub_full = rospy.Publisher('sync_3cam', sImage, queue_size=10)

    # subscribe to many topics
    rospy.Subscriber(INPUT_IMAGE_TOPIC, sImage, on_image_received, queue_size=1)
    rospy.Subscriber(INPUT_IMAGE_TOPIC_LEFT, sImage, on_image_received_left, queue_size=1)
    rospy.Subscriber(INPUT_IMAGE_TOPIC_RIGHT, sImage, on_image_received_right, queue_size=1)

    rospy.spin()
