#!/usr/bin/env python

import numpy as np

# ros related
import rospy
from sensor_msgs.msg import *
from dbw_mkz_msgs.msg import *

from cv_bridge import CvBridge
import cv2, sys
import core.yolo_detector as yolo

global count
image_pub = rospy.Publisher("temp_image_topic", Image)
bridge = CvBridge()

crop_car_hood = -1
global PREDICTION_HZ

cfg = "/home/bdd/intel/perceptionabstractionlayer/pal/data/yolov3.cfg"
meta = "/home/bdd/intel/perceptionabstractionlayer/pal/data/coco.data"
weight = "/home/bdd/intel/perceptionabstractionlayer/pal/data/COCO-yolov3.weights"
detector = yolo.YoloDetector(cfg, weight, meta)

def on_image_received(data):
    global count
    count += 1
    assert(30 % PREDICTION_HZ == 0)
    if count % (30 / PREDICTION_HZ) == 0:
        count = 0
        np_arr = np.fromstring(data.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, 1)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        if crop_car_hood > 0:
            cv_image = cv2.resize(cv_image, (360, 360))
            cv_image = cv_image[:-crop_car_hood, :, :]
        #cv_image = cv2.resize(cv_image, (320, 320))

        detections = detector.detect(cv_image)
        vis = detector.visualize_detections(cv_image, detections)

        image_pub.publish(bridge.cv2_to_imgmsg(vis, "rgb8"))

if __name__ == "__main__":
    global count
    count = 0
    global PREDICTION_HZ
    PREDICTION_HZ = int(sys.argv[1])

    rospy.init_node('real_image_publisher')
    rospy.Subscriber("/image_sender_0",
                    CompressedImage, on_image_received, queue_size=1)

    rospy.spin()
