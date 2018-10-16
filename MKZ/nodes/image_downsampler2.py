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
image_raws=[ rospy.Publisher("/image_sender_"+str(i)+"/raw", Image) for i in range(3)]
p0=image_raws[0]
p1 = image_raws[1]
bridge = CvBridge()

def on_image_received(data, thisid):
    img = bridge.imgmsg_to_cv2(data, "rgb8")
    img = img[::-1, ::-1, :]

    image_raws[thisid].publish(bridge.cv2_to_imgmsg(img, "rgb8"))
    print("with in " + str(thisid))
    #sz1 = cv2.resize(img, (768, 576))
    #image_pub.publish(bridge.cv2_to_imgmsg(sz1, "rgb8"))

    #sz2 = cv2.resize(sz1, (384, 288))
    #image_pub2.publish(bridge.cv2_to_imgmsg(sz2, "rgb8"))


if __name__ == "__main__":
    rospy.init_node('image_downsampler')
    #for i in range(2): 

    rospy.Subscriber("/camera_array/cam"+str(0)+"/image_raw", Image, lambda x: on_image_received(x, 0), queue_size=1)
    rospy.Subscriber("/camera_array/cam"+str(1)+"/image_raw", Image, lambda x: on_image_received(x, 1), queue_size=1)
    rospy.Subscriber("/camera_array/cam" + str(2) + "/image_raw", Image, lambda x: on_image_received(x, 2),
                     queue_size=1)

    rospy.spin()
