#!/usr/bin/env python

# reading in the videos
import time, cv2

# ros related
import rospy

# messages related
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

if __name__ == "__main__":
    IN_FILE = video_path = "/root/video_lowres.mkv"

    image_pub = rospy.Publisher("image_sender_0", Image)
    bridge = CvBridge()

    rospy.init_node('fake_image_publisher')

    cap = cv2.VideoCapture(IN_FILE)

    i = 0
    batch_frames = []
    video_init = False
    i = 0
    while (cap.isOpened()):
        ret, frame = cap.read()
        if not ret:
            break
        image_pub.publish(bridge.cv2_to_imgmsg(frame, "rgb8"))
        time.sleep(1.0/3)
        i += 1
        if i % 15 == 0:
            print "publishing a fake image at 3Hz"
