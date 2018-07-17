#!/usr/bin/env python

import numpy as np

# ros related
import rospy
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2

image_pub = rospy.Publisher("/image_sender_0/H576W768", Image)
bridge = CvBridge()

def on_image_received(data):
    img = bridge.imgmsg_to_cv2(data, "rgb8")
    img = cv2.resize(img,
               dsize=(768, 576),
               interpolation=cv2.INTER_AREA)
    img = bridge.cv2_to_imgmsg(img, "rgb8")
    image_pub.publish(img)

if __name__ == "__main__":
    rospy.init_node('image_downsampler')
    rospy.Subscriber("/image_sender_0",
                    Image, on_image_received, queue_size=10)

    rospy.spin()
