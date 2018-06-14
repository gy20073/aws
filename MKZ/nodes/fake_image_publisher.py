#!/usr/bin/env python

# reading in the videos
import time, sys
from PIL import Image as PIL_Image
from cStringIO import StringIO
import numpy as np

# tensorflow related
import tensorflow as tf
from tensorflow.core.example import example_pb2

# ros related
import rospy

# messages related
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

if __name__ == "__main__":
    # the input tfrecord
    IN_FILE = sys.argv[1]

    # some global variables
    example = example_pb2.Example()
    image_pub = rospy.Publisher("temp_image_topic", Image)
    bridge = CvBridge()

    rospy.init_node('fake_image_publisher')

    for count, example_serialized in enumerate(tf.python_io.tf_record_iterator(IN_FILE)):
        example.ParseFromString(example_serialized)
        feature_map = example.features.feature
        encoded = feature_map['image/encoded'].bytes_list.value
        print "iterating the TFRecord file", count

    while True:
        for i in range(len(encoded)):
            if rospy.is_shutdown():
                print "rospy is shutdown"
                exit(0)

            if i % 5 == 0:
                file_jpgdata = StringIO(encoded[i])
                dt = PIL_Image.open(file_jpgdata)
                arr = np.asarray(dt)
                image_pub.publish(bridge.cv2_to_imgmsg(arr, "rgb8"))
                time.sleep(1.0/3)
                if i % 15 == 0:
                    print "publishing a fake image at 3Hz"
