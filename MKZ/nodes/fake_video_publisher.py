#!/usr/bin/env python

# reading in the videos
import time, cv2, sys

# ros related
import rospy

# messages related
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

if __name__ == "__main__":
    IN_FILE = sys.argv[1]
    print("publishing fake images with video", IN_FILE)

    image_pub = rospy.Publisher("/image_sender_0", Image)
    image_pub_left = rospy.Publisher("/camera_array/cam1/image_raw", Image)
    image_pub_right = rospy.Publisher("/camera_array/cam2/image_raw", Image)

    bridge = CvBridge()

    rospy.init_node('fake_image_publisher')

    cap = cv2.VideoCapture(IN_FILE)

    i = 0
    last_computation = time.time()
    while (cap.isOpened()):
        t0 = time.time()
        ret, frame = cap.read()
        if not ret or rospy.is_shutdown():
            break
        #frame = cv2.resize(frame, (2048, 1536))
        frame = cv2.resize(frame, (768, 576))

        image_pub.publish(bridge.cv2_to_imgmsg(frame[::-1, ::-1, ], "bgr8"))
        image_pub_left.publish(bridge.cv2_to_imgmsg(frame[::-1, ::-1, ], "bgr8"))
        image_pub_right.publish(bridge.cv2_to_imgmsg(frame[::-1, ::-1, ], "bgr8"))
        passed = time.time() - t0
        if 1.0/30 < passed:
            time_now = time.time()
            print("warning: fake video pub speed too slow, real_speed is ", 1.0 / (time_now - last_computation))
            last_computation = time_now
        else:
            time.sleep(1.0/30 - passed)
        i += 1
        if i % 30 == 0:
            print "publishing a fake image at 30Hz"
