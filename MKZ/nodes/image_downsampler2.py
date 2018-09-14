#!/usr/bin/env python

import numpy as np

# ros related
import rospy
from sensor_msgs.msg import *
from sensor_msgs.msg import Image
from dbw_mkz_msgs.msg import *

from cv_bridge import CvBridge
import cv2, sys


image_pub = rospy.Publisher("/image_sender_0/H576W768", Image)
image_pub2 = rospy.Publisher("/image_sender_0/H288W384", Image)
image_raw = rospy.Publisher("/image_sender_0/raw", Image)

bridge = CvBridge()

def on_image_received(data):
    img = bridge.imgmsg_to_cv2(data, "rgb8")
    img = img[::-1, ::-1, :]

    image_raw.publish(bridge.cv2_to_imgmsg(img, "rgb8"))

    sz1 = cv2.resize(img, (768, 576))
    image_pub.publish(bridge.cv2_to_imgmsg(sz1, "rgb8"))

    sz2 = cv2.resize(sz1, (384, 288))
    image_pub2.publish(bridge.cv2_to_imgmsg(sz2, "rgb8"))

if __name__ == "__main__":
    rospy.init_node('image_downsampler')
    rospy.Subscriber("/camera_array/cam0/image_raw", Image, on_image_received, queue_size=10)

    rospy.spin()
