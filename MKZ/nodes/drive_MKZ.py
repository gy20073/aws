#!/usr/bin/env python

# This file put (1)driving model, (2)keyboard input (3)publish twist command (4)receive image together

# roslib related
import roslib
roslib.load_manifest('dbw_mkz_msgs')
import rospy, importlib, inspect

# messages related
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
from geometry_msgs.msg import Vector3

# standard system packages
import sys, os, time

# some constants
global PREDICTION_HZ, STOP_FUTURE_FRAMES
PREDICTION_HZ = 3
STOP_FUTURE_FRAMES = 0 # if the model predict the future,  instead of the immediate thing
SAFETY_SPEED = 10.0 # in m/s, cap the speed after time slow
TIME_SLOW_FACTOR = 1.0

KEYBOARD_TOPIC = "mkz_key_command"
TWIST_COMM_TOPIC = "twist_predicted_results"

global bridge, driving_model, vis_pub_full
twist_comm_pub = rospy.Publisher(TWIST_COMM_TOPIC, Vector3, queue_size=1)

def on_image_received(data):
    time0 = time.time()
    future_time = time.time()+1.0*STOP_FUTURE_FRAMES/PREDICTION_HZ * TIME_SLOW_FACTOR
    global bridge, driving_model, vis_pub_full

    img = bridge.imgmsg_to_cv2(data, "rgb8")

    # TODO: compute the yaw, and speed from the driving model
    yaw = 0.0
    speed = 0.0
    '''
    logits = driving_model.observe_a_frame(img)
    logits = logits[0]
    # we could not truncate the pdf distribution, otherwise the MAP would be quite unstable.
    #logits = add_prior_to_logit.prior_speed_limit(logits, TYPE, SAFETY_SPEED)

    MAP = driving_model.continuous_MAP([logits])
    yaw, speed = MAP[0]
    yaw = -yaw
    '''

    # apply the time slow factor
    speed /= TIME_SLOW_FACTOR
    yaw /= TIME_SLOW_FACTOR

    # safty control
    if speed > SAFETY_SPEED:
        print "speed still larger than safty speed, cropped. (It will affect performance)"
        speed = min(speed, SAFETY_SPEED)

    # the meaning of the predicted value
    # made by the relative_future_course_speed, with stop_future_frames=1
    # the future speed & yaw rate (between now and next frame, which is 1/3 second)

    # TODO: convert predicted measure to the required measure
    # the meaning of the required value: from -8.2rad to +8.2rad
    v3 = Vector3()
    v3.x = speed
    v3.y = yaw
    v3.z = future_time
    twist_comm_pub.publish(v3)

    # generate the visualization image
    # TODO: generate the visualization based on the driving model
    #rendered = driving_model.generate_visualization(img, logits, "vis_continuous")
    #vis_pub_full.publish(bridge.cv2_to_imgmsg(rendered, "rgb8"))

    print ("total time for on image receive is ", time.time()-time0)
    print "on image received"


def on_key_received(data):
    key = data.data
    if key == "s":
        # set to no prior function
        print("Prior: setting to no prior")
    elif key == "w":
        print("Prior: setting to straight prior")
    elif key == "a":
        print("Prior: setting to left prior")
    elif key == "d":
        print("Prior: setting to right prior")
    else:
        print("invalid command", key)

def get_file_real_path():
    abspath = os.path.abspath(inspect.getfile(inspect.currentframe()))
    return os.path.realpath(abspath)

if __name__ == "__main__":
    rospy.init_node('BDD_Driving_Model')
    DRIVING_MODEL_PATH = sys.argv[1]

    global PREDICTION_HZ, STOP_FUTURE_FRAMES
    if len(sys.argv) > 6:
        PREDICTION_HZ = int(sys.argv[5])
        STOP_FUTURE_FRAMES = int(sys.argv[6])

    # a global shared data structure, list of [forward speed in m/s, yaw rate in rad/s]
    global bridge, driving_model, vis_pub_full

    bridge = CvBridge()

    vis_pub_full = rospy.Publisher('vis_continuous_full', Image, queue_size=10)

    # BDD Driving model related
    # TODO: add in the CIL model here
    '''
    driving_model_code_path = os.path.join(os.path.dirname(get_file_real_path()), "../../BDD_Driving_Model")
    os.chdir(driving_model_code_path)
    sys.path.append(driving_model_code_path)
    from wrapper import Wrapper
    # TODO: using 1 only to speed up
    driving_model = Wrapper(config_name, DRIVING_MODEL_PATH, 1,
                            config_name="config_mkz",
                            config_path=".",
                            is_lstm=is_lstm,
                            post_config_func=post_config_func)
    '''
    rospy.Subscriber("temp_image_topic", Image, on_image_received, queue_size=1)

    rospy.Subscriber(KEYBOARD_TOPIC, String, on_key_received, queue_size=10)

    rospy.spin()
