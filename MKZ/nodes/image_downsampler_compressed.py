#!/usr/bin/env python

import numpy as np

# ros related
import rospy
from sensor_msgs.msg import *
from sensor_msgs.msg import Image, CompressedImage
from dbw_mkz_msgs.msg import *

from cv_bridge import CvBridge
import cv2

image_raws=[ rospy.Publisher("/compressed"+str(i), CompressedImage) for i in range(3)]

bridge = CvBridge()

def on_image_received(data, thisid):
    img = bridge.imgmsg_to_cv2(data, "bgr8")
    img = img[::-1, ::-1, :]

    img = cv2.resize(img, (768, 576))

    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = np.array(cv2.imencode('.jpg', img)[1]).tostring()

    image_raws[thisid].publish(msg)
    print("with in " + str(thisid))


if __name__ == "__main__":
    rospy.init_node('image_downsampler')

    rospy.Subscriber("/camera_array/cam0/image_raw", Image, lambda x: on_image_received(x, 0), queue_size=1)
    rospy.Subscriber("/camera_array/cam1/image_raw", Image, lambda x: on_image_received(x, 1), queue_size=1)
    rospy.Subscriber("/camera_array/cam2/image_raw", Image, lambda x: on_image_received(x, 2), queue_size=1)

    rospy.spin()
