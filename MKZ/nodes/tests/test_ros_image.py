import roslib
#roslib.load_manifest('dbw_mkz_msgs')
import rospy

# messages related
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np

bridge = CvBridge()
vis_pub_full = rospy.Publisher('vis_continuous_full', Image, queue_size=10)
DOWNSAMPLE_FACTOR = 6
counter = 0

from all_perceptions import Perceptions
perceptions = Perceptions(det_COCO=True,
                          det_TL=True,
                          det_TS=True,
                          seg=True,
                          depth=True,
                          batch_size=1,
                          gpu_assignment=[0, 1],
                          compute_methods={},
                          viz_methods={},
                          path_config="path_docker")

def on_image_received(data):
    global counter
    counter += 1
    if counter % DOWNSAMPLE_FACTOR == 0:
        img = bridge.imgmsg_to_cv2(data, "rgb8")
        img = np.expand_dims(img, axis=0)
        viz = perceptions.visualize(perceptions.compute(img), 0)
        viz = bridge.cv2_to_imgmsg(viz, "rgb8")
        vis_pub_full.publish(viz)

if __name__ == "__main__":
    rospy.init_node('test_ros_image')

    rospy.Subscriber("/image_sender_0/H576W768", Image, on_image_received, queue_size=10)

    rospy.spin()
