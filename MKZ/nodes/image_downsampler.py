#!/usr/bin/env python

import numpy as np

# ros related
import rospy
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2

image_pub = rospy.Publisher("/image_sender_0/H576W768", Image)
image_pub_flipped = rospy.Publisher("/image_sender_0/flipped", Image)
bridge = CvBridge()

def on_image_received(data):
    img = bridge.imgmsg_to_cv2(data, "rgb8")
    flipped = img[::-1, ::-1, :]
    img = cv2.resize(img,
               dsize=(768, 576),
               interpolation=cv2.INTER_AREA)
    img = img[::-1, ::-1, :]
    img = bridge.cv2_to_imgmsg(img, "rgb8")
    image_pub.publish(img)

    image_pub_flipped.publish(bridge.cv2_to_imgmsg(flipped, "rgb8"))

if __name__ == "__main__":
    rospy.init_node('image_downsampler')
    rospy.Subscriber("/image_sender_0",
                    Image, on_image_received, queue_size=10)

    rospy.spin()
